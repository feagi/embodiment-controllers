# FEAGI ESP32 Standalone Firmware

Standalone firmware for ESP32 that runs a complete FEAGI neural network on-device.

## Features

- **On-device neural processing**: Complete FEAGI brain runs entirely on ESP32
- **Connectome embedding**: Serialized connectome is embedded in firmware at build time
- **GPIO configuration**: Configurable GPIO pins mapped to FEAGI cortical areas
- **Optimized for size**: Aggressive size optimization for embedded constraints

## Building

This firmware is built automatically by the FEAGI Desktop ESP32 Flasher tool. Configuration is injected at build time via `build.rs`.

## Configuration

Configuration is provided via `config.json` (generated from UI settings):

```json
{
  "mode": "standalone",
  "model": "esp32-devkit-v1",
  "burst_frequency": 100,
  "brain": {
    "path": "/path/to/connectome.connectome",
    "validated": true
  },
  "gpio": [
    {
      "pin": 25,
      "mode": "digital_output",
      "cortical_mapping": "ogpid00"
    }
  ]
}
```

## Connectome Format

The connectome must be in FEAGI's binary connectome format (`.connectome` file), serialized using `feagi-connectome-serialization`.

## Memory Constraints

- ESP32-WROOM-32: ~10,000 neurons, ~50,000 synapses
- ESP32-S3: ~15,000 neurons, ~75,000 synapses

These limits are enforced during validation in the FEAGI Desktop UI.

