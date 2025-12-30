# SHS-Z2M-Presence

Dual mmWave presence sensor firmware for ESP32-C6 with Zigbee2MQTT support.

## Hardware

- ESP32-C6
- LD2410C (presence detection)
- LD2450 (position tracking & zones)

## Features

- Dual sensor cross-validation for reduced false positives
- Zone-based detection (up to 3 zones)
- Real-time target position tracking
- Zigbee2MQTT compatible

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py flash
```

Full tutorial coming soon.

## Credits

Based on [SmartHomeScene/zigbee-esp32](https://github.com/SmartHomeScene/zigbee-esp32)

See the original firmware and flashing guide: [DIY Zigbee mmWave Presence Sensor with ESP32-C6 and LD2410](https://smarthomescene.com/guides/diy-zigbee-mmwave-presence-sensor-with-esp32-c6-and-ld2410/)
