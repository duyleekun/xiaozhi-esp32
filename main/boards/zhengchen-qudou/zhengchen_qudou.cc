#include "application.h"
#include "assets/lang_config.h"
#include "audio/codecs/box_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display/lcd_display.h"
#include "esp32_camera.h"
#include "i2c_device.h"
#include "settings.h"
#include "wifi_board.h"

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_check.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <iot_button.h>

#include <algorithm>
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>

#define TAG "Zhengchen_Qudou"

namespace {

class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t bus, uint8_t address)
        : I2cDevice(bus, address) {
        // Program the output latch before changing pin directions. This 0x03
        // value and the following direction mask match the factory-derived
        // Zhengchen sequence. Do not infer P0 polarity from the ST7789 signal
        // name: the board may invert it or use it as an LCD power gate.
        WriteReg(0x01, 0x03);
        // P0/P1/P2/P5 outputs; P3/P4/P6/P7 inputs.
        WriteReg(0x03, 0xd8);
    }

    void SetOutput(uint8_t bit, bool high) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint8_t value = ReadReg(0x01);
        value = (value & ~(1U << bit)) | (static_cast<uint8_t>(high) << bit);
        WriteReg(0x01, value);
    }

    bool GetInput(uint8_t bit) {
        std::lock_guard<std::mutex> lock(mutex_);
        return (ReadReg(0x00) & (1U << bit)) != 0;
    }

private:
    std::mutex mutex_;
};

struct Pca9557ButtonDriver {
    button_driver_t base;
    Pca9557* expander;
    uint8_t bit;
    bool active_low;
};

button_handle_t CreatePca9557Button(Pca9557* expander, uint8_t bit,
                                    bool active_low) {
    auto* driver = static_cast<Pca9557ButtonDriver*>(
        calloc(1, sizeof(Pca9557ButtonDriver)));
    ESP_ERROR_CHECK(driver == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
    driver->expander = expander;
    driver->bit = bit;
    driver->active_low = active_low;
    driver->base.enable_power_save = false;
    driver->base.get_key_level = [](button_driver_t* base) -> uint8_t {
        auto* self = reinterpret_cast<Pca9557ButtonDriver*>(base);
        bool level = self->expander->GetInput(self->bit);
        return self->active_low ? !level : level;
    };

    button_config_t config = {
        .long_press_time = 0,
        .short_press_time = 0,
    };
    button_handle_t handle = nullptr;
    ESP_ERROR_CHECK(iot_button_create(&config, &driver->base, &handle));
    return handle;
}

class Pca9557Button : public Button {
public:
    Pca9557Button(Pca9557* expander, uint8_t bit, bool active_low = true)
        : Button(CreatePca9557Button(expander, bit, active_low)) {}
};

class QudouPowerManager {
public:
    explicit QudouPowerManager(Pca9557* expander) : expander_(expander) {
        adc_oneshot_unit_init_cfg_t unit_config = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle_));

        adc_oneshot_chan_cfg_t channel_config = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(
            adc_handle_, BATTERY_LEVEL_ADC_CHANNEL, &channel_config));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(
            adc_handle_, BATTERY_REF_ADC_CHANNEL, &channel_config));

        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                static_cast<QudouPowerManager*>(arg)->CheckBatteryStatus();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "battery_check_timer",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(timer_, 1000000));
    }

    ~QudouPowerManager() {
        if (timer_ != nullptr) {
            esp_timer_stop(timer_);
            esp_timer_delete(timer_);
        }
        if (adc_handle_ != nullptr) {
            adc_oneshot_del_unit(adc_handle_);
        }
    }

    bool GetStatus(int& level, bool& charging, bool& discharging) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (battery_samples_.size() < kSampleCount) {
            return false;
        }
        level = battery_level_;
        charging = battery_level_ < 100 && is_charging_;
        discharging = !is_charging_;
        return true;
    }

private:
    static constexpr size_t kSampleCount = 3;
    static constexpr int kReadIntervalSeconds = 60;

    Pca9557* expander_;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    esp_timer_handle_t timer_ = nullptr;
    std::vector<uint16_t> ref_samples_;
    std::vector<uint16_t> battery_samples_;
    mutable std::mutex state_mutex_;
    int battery_level_ = 0;
    bool is_charging_ = false;
    int ticks_ = 0;

    void CheckBatteryStatus() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // The dumped factory image logs P6 level 1 as "Charging".
        bool charging = expander_->GetInput(PCA9557_CHARGE_STATUS);
        if (charging != is_charging_) {
            is_charging_ = charging;
            ESP_LOGI(TAG, "Charging status changed: %s (PCA9557 P6=%d)",
                     charging ? "Charging" : "Not charging", charging);
            ReadBatteryAdcData();
            return;
        }

        if (battery_samples_.size() < kSampleCount) {
            ReadBatteryAdcData();
            return;
        }

        if (++ticks_ % kReadIntervalSeconds == 0) {
            ReadBatteryAdcData();
        }
    }

    void ReadBatteryAdcData() {
        int ref_adc = 0;
        int battery_adc = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(
            adc_handle_, BATTERY_REF_ADC_CHANNEL, &ref_adc));
        ESP_ERROR_CHECK(adc_oneshot_read(
            adc_handle_, BATTERY_LEVEL_ADC_CHANNEL, &battery_adc));

        PushSample(ref_samples_, ref_adc);
        PushSample(battery_samples_, battery_adc);

        uint32_t average_ref = Average(ref_samples_);
        uint32_t average_battery = Average(battery_samples_);
        if (average_ref == 0) {
            ESP_LOGW(TAG, "Reference ADC value is zero; battery level unchanged");
            return;
        }

        float voltage = static_cast<float>(average_battery) /
                        static_cast<float>(average_ref) *
                        BATTERY_REFERENCE_VOLTAGE * BATTERY_DIVIDER_RATIO;
        if (voltage <= BATTERY_EMPTY_VOLTAGE) {
            battery_level_ = 0;
        } else if (voltage >= BATTERY_FULL_VOLTAGE) {
            battery_level_ = 100;
        } else {
            battery_level_ = static_cast<int>(
                (voltage - BATTERY_EMPTY_VOLTAGE) * 100.0f /
                (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE));
        }

        ESP_LOGI(TAG,
                 "Ref ADC: %lu, Battery ADC: %lu, Battery Voltage: %.2fV, "
                 "Level: %d%%",
                 static_cast<unsigned long>(average_ref),
                 static_cast<unsigned long>(average_battery), voltage,
                 battery_level_);
    }

    static void PushSample(std::vector<uint16_t>& samples, int value) {
        samples.push_back(static_cast<uint16_t>(value));
        if (samples.size() > kSampleCount) {
            samples.erase(samples.begin());
        }
    }

    static uint32_t Average(const std::vector<uint16_t>& samples) {
        uint32_t sum = 0;
        for (uint16_t value : samples) {
            sum += value;
        }
        return sum / samples.size();
    }
};

class QudouAudioCodec : public BoxAudioCodec {
public:
    QudouAudioCodec(i2c_master_bus_handle_t bus, Pca9557* expander)
        : BoxAudioCodec(bus, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                        AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK,
                        AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
                        AUDIO_I2S_GPIO_DIN, GPIO_NUM_NC,
                        AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR,
                        AUDIO_INPUT_REFERENCE),
          expander_(expander) {}

    void EnableOutput(bool enable) override {
        // Keep the external amplifier muted while the ES8311 changes state.
        if (!enable) {
            expander_->SetOutput(PCA9557_SPEAKER_ENABLE, false);
        }
        BoxAudioCodec::EnableOutput(enable);
        if (enable) {
            expander_->SetOutput(PCA9557_SPEAKER_ENABLE, true);
        }
    }

private:
    Pca9557* expander_;
};

}  // namespace

class Zhengchen_Qudou : public WifiBoard {
public:
    Zhengchen_Qudou()
        : camera_button_(CAMERA_BUTTON_GPIO),
          vibration_button_(VIBRATION_BUTTON_GPIO),
          power_button_(POWER_BUTTON_GPIO) {
        InitializeI2c();
        power_manager_ = new QudouPowerManager(expander_);
        InitializeSpi();
        InitializeSt7789Display();
        InitializeButtons();
        InitializeCamera();
        InitializeCameraOrientation();
        GetBacklight()->RestoreBrightness();
    }

    AudioCodec* GetAudioCodec() override {
        static QudouAudioCodec codec(i2c_bus_, expander_);
        return &codec;
    }

    Display* GetDisplay() override { return display_; }

    Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN,
                                      DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    Camera* GetCamera() override { return camera_; }

    bool GetBatteryLevel(int& level, bool& charging,
                         bool& discharging) override {
        return power_manager_->GetStatus(level, charging, discharging);
    }

private:
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Pca9557* expander_ = nullptr;
    QudouPowerManager* power_manager_ = nullptr;
    Pca9557Button* volume_up_button_ = nullptr;
    Pca9557Button* volume_down_button_ = nullptr;
    Button camera_button_;
    Button vibration_button_;
    Button power_button_;
    LcdDisplay* display_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    bool camera_is_front_ = false;

    void InitializeI2c() {
        i2c_master_bus_config_t config = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&config, &i2c_bus_));
        expander_ = new Pca9557(i2c_bus_, PCA9557_ADDRESS);
    }

    void InitializeSpi() {
        spi_bus_config_t config = {};
        config.mosi_io_num = GPIO_NUM_40;
        config.miso_io_num = GPIO_NUM_NC;
        config.sclk_io_num = GPIO_NUM_41;
        config.quadwp_io_num = GPIO_NUM_NC;
        config.quadhd_io_num = GPIO_NUM_NC;
        config.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &config, SPI_DMA_CH_AUTO));
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(
            esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(
            esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        // Preserve the ordering recovered from the related Zhengchen source and
        // supported by factory-image strings: the IDF panel reset sends ST7789
        // SWRESET because reset_gpio_num is NC, then P0 goes low before SLPOUT
        // and the remaining controller initialization. A conventional low/high
        // hardware-reset pulse is unsafe until P0's external polarity and
        // function have been verified on a scope.
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        expander_->SetOutput(PCA9557_LCD_CONTROL, false);

        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(
            esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        display_ = new SpiLcdDisplay(
            panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
            DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
            DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        volume_up_button_ =
            new Pca9557Button(expander_, PCA9557_VOLUME_UP, true);
        volume_down_button_ =
            new Pca9557Button(expander_, PCA9557_VOLUME_DOWN, true);

        // Physical testing identifies GPIO46 as the camera-icon button.
        camera_button_.OnClick([this]() {
            ESP_LOGI(TAG, "GPIO46 camera-button single-click callback");
            if (camera_ != nullptr && !camera_->Capture()) {
                ESP_LOGE(TAG, "Camera capture failed");
            }
        });
        camera_button_.OnDoubleClick([this]() {
            camera_is_front_ = !camera_is_front_;
            Settings settings("camera", true);
            settings.SetInt("is_front", camera_is_front_ ? 1 : 0);
            ESP_LOGI(TAG, "Camera orientation manually changed to %s",
                     camera_is_front_ ? "front" : "rear");
            ApplyCameraOrientation(camera_is_front_);
            GetDisplay()->ShowNotification(camera_is_front_
                                               ? "Camera: Front"
                                               : "Camera: Rear");
        });

        // GPIO48 is the power button. Its hardware long-press shutdown does
        // not depend on this software callback; a short press toggles chat.
        power_button_.OnClick([this]() {
            ESP_LOGI(TAG, "GPIO48 power-button single-click callback");
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        // The vendor image labels GPIO47's callback as the vibration button.
        // It remained high throughout camera-button and camera-rotation tests,
        // so leave it without an assigned user action for now.
        vibration_button_.OnPressDown([]() {
            ESP_LOGI(TAG, "GPIO47 input press-down callback");
        });
        vibration_button_.OnPressUp([]() {
            ESP_LOGI(TAG, "GPIO47 input press-up callback");
        });

        volume_up_button_->OnClick([this]() { ChangeVolume(10); });
        volume_up_button_->OnLongPress(
            [this]() { SetVolume(100, Lang::Strings::MAX_VOLUME); });
        volume_down_button_->OnClick([this]() { ChangeVolume(-10); });
        volume_down_button_->OnLongPress(
            [this]() { SetVolume(0, Lang::Strings::MUTED); });
    }

    void ChangeVolume(int delta) {
        auto* codec = GetAudioCodec();
        int volume = std::max(0, std::min(100, codec->output_volume() + delta));
        codec->SetOutputVolume(volume);
        GetDisplay()->ShowNotification(Lang::Strings::VOLUME +
                                       std::to_string(volume / 10));
    }

    void SetVolume(int volume, const std::string& message) {
        GetAudioCodec()->SetOutputVolume(volume);
        GetDisplay()->ShowNotification(message);
    }

    void InitializeCamera() {
        expander_->SetOutput(PCA9557_CAMERA_POWER, false);
        vTaskDelay(pdMS_TO_TICKS(20));

        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;
        config.ledc_timer = LEDC_TIMER_2;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = I2C_NUM_1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = CAMERA_XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 9;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
    }

    void InitializeCameraOrientation() {
        Settings settings("camera", false);
        camera_is_front_ = settings.GetInt("is_front", 0) != 0;
        ESP_LOGI(TAG, "Stored camera orientation: %s",
                 camera_is_front_ ? "front" : "rear");
        ApplyCameraOrientation(camera_is_front_);
    }

    void ApplyCameraOrientation(bool is_front) {
        if (camera_ == nullptr) {
            return;
        }
        // The dumped NVS had is_front=0 while the sensor ran mirrored and
        // vertically flipped, establishing this polarity for that position.
        camera_->SetHMirror(!is_front);
        camera_->SetVFlip(!is_front);
    }
};

DECLARE_BOARD(Zhengchen_Qudou);
