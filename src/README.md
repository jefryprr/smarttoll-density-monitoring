# SmartToll v3.2.1 — Firmware ESP32

> ⚠️ File firmware utama (`SmartToll_v3.2.1.ino`) perlu ditambahkan ke direktori ini.

## Prasyarat

- Arduino IDE ≥ 1.8.19 atau PlatformIO
- Board Manager: **ESP32 by Espressif Systems**
- Library:
  - `ESP32Servo` (by Kevin Harrington / John K. Bennett)
  - `LiquidCrystal_I2C` (by Frank de Brabander / Marco Schwartz)
  - `Wire` (built-in core ESP32)

## Upload

```bash
# Arduino IDE
# 1. Buka SmartToll_v3.2.1.ino
# 2. Tools → Board → ESP32 Arduino → ESP32 Dev Module
# 3. Pilih port serial
# 4. Upload (Ctrl+U)

# PlatformIO
pio run -e esp32dev -t upload
pio device monitor -b 115200
```
