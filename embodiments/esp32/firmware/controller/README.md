# FEAGI ESP32 Controller Firmware

Controller firmware for ESP32 that acts as an I/O interface for a FEAGI instance running on a separate device.

## Features

- **I/O Interface**: ESP32 handles sensors and actuators
- **Transport Support**: Serial/UART (WiFi and Bluetooth coming soon)
- **GPIO Configuration**: Configurable GPIO pins mapped to FEAGI cortical areas
- **Real-time communication**: Low-latency communication with remote FEAGI

## Building

This firmware is built automatically by the FEAGI Desktop ESP32 Flasher tool. Configuration is injected at build time via `build.rs`.

## Configuration

Configuration is provided via `config.json` (generated from UI settings):

```json
{
  "mode": "controller",
  "model": "esp32-devkit-v1",
  "transport": {
    "type": "serial",
    "config": {}
  },
  "burst_frequency": 100,
  "gpio": [
    {
      "pin": 36,
      "mode": "analog_input",
      "cortical_mapping": "igpia00"
    },
    {
      "pin": 25,
      "mode": "pwm_output",
      "cortical_mapping": "ogpia00"
    }
  ]
}
```

## Transport Types

### Serial/UART (Current)
- Baud rate: 115200
- Protocol: FEAGI message format
- Pins: UART0 (TX=1, RX=3 on ESP32)

### WiFi (Coming Soon)
- TCP/IP connection to FEAGI
- Configurable SSID/password

### Bluetooth (Coming Soon)
- Bluetooth Classic or BLE
- Configurable device name

## Operation

1. ESP32 reads sensor inputs from GPIO
2. Sends sensor data to FEAGI via transport
3. Receives motor commands from FEAGI via transport
4. Writes motor outputs to GPIO
5. Repeats at configured burst frequency

