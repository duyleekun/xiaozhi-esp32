# Zhengchen Qudou

Reverse-engineered board support for the `zhengchen-qudou` ESP32-S3 device.
The pinout and peripheral choices come from two verified 16 MiB flash dumps,
boot logs, NVS inspection, and static analysis of the vendor OTA image.

Important validation status:

- ST7789 display, ES8311/ES7210 audio, GC2145 camera, and button pins match the
  vendor image.
- The ST7789 sequence preserves the ordering recovered from the related
  Zhengchen source and supported by strings in the factory image: PCA9557 P0
  starts high, IDF sends `SWRESET`, P0 goes low, then `SLPOUT`, MADCTL, COLMOD
  and RAMCTRL are sent by `esp_lcd_panel_init()`. Do not replace this with a
  normal active-low reset pulse until P0's external inversion/power-gate
  circuit has been measured.
- ES7210 uses four-slot TDM at 24 kHz; the application consumes microphone slot
  0 and playback-reference slot 1 for device-side AEC.
- Audio register programming is delegated to Espressif `esp_codec_dev` 1.5.11,
  the same ES7210/ES8311 driver family visible in the vendor binary. ES8311 runs
  DAC-only with a 6.144 MHz MCLK (256 x 24 kHz), 16-bit I2S, and external PA on
  PCA9557 P1. ES7210 runs as a slave in four-slot, 16-bit TDM; all four ADC
  inputs are enabled and channel 0 gain is set to 30 dB. No board-specific
  magic register writes are added on top of the codec driver.
- Battery measurement is recovered from the factory v1.0.8 image. ADC1 channel
  2 (GPIO3) measures a 1.24 V reference and ADC1 channel 9 (GPIO10) measures
  the divided battery voltage. The factory calculation averages three samples,
  applies a 2:1 divider ratio, and maps 3.40 V to 0% and 4.15 V to 100%.
- PCA9557 P6 is an active-high charge-status input. It drives the standard LVGL
  charging-bolt icon through `GetBatteryLevel()`; no MCU-controlled physical
  charge-LED output was found in the factory image or recovered pin map.
- GPIO46 is the active-low camera-icon button. GPIO48 is the active-low power
  button; its hardware long-press shutdown works independently of the firmware
  short-click action. Rotation produced no transition on GPIO46, GPIO47,
  GPIO48, or PCA9557 P7, so the hardware exposes no position signal. Matching
  the factory firmware's persisted `camera/is_front` behavior, double-clicking
  the camera button toggles front/rear orientation; a single click captures.

Build with:

```sh
python scripts/release.py zhengchen-qudou
```

Do not flash a reconstruction until the P0 waveform, SPI command trace, I2S
clocks, codec I2C register writes, both battery ADC inputs, and power-off
polarity have been verified against the original firmware.
