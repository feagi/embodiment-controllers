# ESP32 FEAGI Firmware

ESP32 firmware templates for FEAGI deployment in both Standalone and Controller modes.

## Modes

### Standalone Mode
FEAGI brain runs directly on ESP32. The connectome is embedded in the firmware and neural processing happens on-device.

### Controller Mode
ESP32 acts as an I/O interface, communicating with FEAGI running on a separate device via Serial/WiFi/Bluetooth.

## Building

Firmware is built automatically via the FEAGI Desktop ESP32 Flasher tool. Configuration is injected at build time from the UI.

## Supported Devices

- ESP32-WROOM-32 (ESP32 DevKit)
- ESP32-S3 (ESP32-S3 DevKit)
- ESP32-C3 (coming soon)

## Directory Structure

```
esp32/
├── firmware/
│   ├── standalone/     # Standalone mode firmware
│   │   ├── Cargo.toml
│   │   ├── build.rs
│   │   └── src/
│   │       └── main.rs
│   └── controller/     # Controller mode firmware
│       ├── Cargo.toml
│       ├── build.rs
│       └── src/
│           └── main.rs
└── README.md
```

