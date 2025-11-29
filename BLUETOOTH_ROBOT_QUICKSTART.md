# Bluetooth Robot Integration - Developer Quickstart

## Overview

The FEAGI Python SDK now includes **BluetoothRobot** - a simple base class that handles ALL the complexity of Bluetooth connectivity, platform detection, and FEAGI integration.

**You only write ~30 lines of robot-specific code!**

## Prerequisites

```bash
pip install 'feagi[bluetooth]'
```

This installs:
- `feagi` - Core SDK
- `bleak` - Cross-platform Bluetooth (desktop)
- `websockets` - For cloud/NRS mode

## 5-Minute Tutorial

### 1. Create Your Robot Class

```python
#!/usr/bin/env python
from feagi.agent import BluetoothRobot

class MyRobot(BluetoothRobot):
    # Define Bluetooth UUIDs (copy from embodiment.json)
    BLUETOOTH_CONFIG = {
        "device_name": "MyRobot",
        "service_uuid": "6e400001-b5a3-f393-e0a9-e50e24dcca9e",
        "rx_uuid": "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
        "tx_uuid": "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
    }
    
    def parse_sensors(self, raw_bytes: bytes) -> dict:
        """Parse your robot's sensor data"""
        # Example: Robot sends "Gx,y,z#" for gyro
        data_str = raw_bytes.decode('utf-8').strip('#')
        if data_str.startswith('G'):
            x, y, z = map(float, data_str[1:].split(','))
            return {'gyro': {'0': [x, y, z]}}
        return {}
    
    def format_motors(self, feagi_output: dict) -> bytes:
        """Format FEAGI commands for your robot"""
        # Example: Robot expects "Mleft,right#"
        if 'motor' in feagi_output:
            left = feagi_output['motor'].get('0', 0)
            right = feagi_output['motor'].get('1', 0)
            return f"M{left},{right}#".encode()
        return b''

# Run it!
if __name__ == "__main__":
    import asyncio
    robot = MyRobot("my-robot-001")
    asyncio.run(robot.run())
```

### 2. Run It

**Desktop:**
```bash
python controller.py
```

**Cloud/NRS:**
```bash
FEAGI_PLATFORM=cloud python controller.py
```

That's it! The SDK handles:
- ✅ Bluetooth scanning & connection
- ✅ Platform detection (desktop vs cloud)
- ✅ FEAGI connection
- ✅ Data loop
- ✅ Error handling & reconnection

## Real Examples

### Bittle X (Quadruped)

See: `embodiment-controllers/embodiments/petoi/bittle/controller.py`

**Key Features:**
- 9 servos (head + 8 legs)
- Gyro sensor
- Servo position feedback

**Controller:** 40 lines

### Cutebot (Wheeled Car)

See: `embodiment-controllers/embodiments/elecfreaks/cutebot/controller.py`

**Key Features:**
- 2 motors
- Gyro, accelerometer, IR, ultrasonic sensors

**Controller:** 35 lines

## What You Need to Provide

### 1. Embodiment Definition (`nrs-embodiments`)

```json
{
  "embodiment_id": "em-myrobot",
  "agent_type": "nrs-bt-device",
  "bluetooth_properties": [{
    "service": "6e400001-...",
    "rx": "6e400002-...",
    "tx": "6e400003-..."
  }],
  "controller_metadata": {
    "controller_type": "myrobot-bluetooth"
  }
}
```

### 2. Controller (`embodiment-controllers`)

```
embodiment-controllers/
└── myrobot/
    ├── controller.py     # Your BluetoothRobot subclass
    └── requirements.txt  # Just: feagi[bluetooth]>=2.0.0
```

### 3. That's It!

No bridge code, no transport layer, no platform-specific logic needed!

## API Reference

### `BluetoothRobot` Base Class

#### Constructor

```python
MyRobot(
    agent_id: str,
    feagi_host: str = "localhost",
    feagi_port: int = 3000,
    platform: str = None  # Auto-detected if None
)
```

#### Required Methods

##### `parse_sensors(raw_bytes: bytes) -> dict`

Parse robot's Bluetooth data into FEAGI format.

**Args:**
- `raw_bytes`: Raw data from robot via Bluetooth

**Returns:**
- Dictionary with FEAGI sensor data:
  ```python
  {
      'gyro': {'0': [x, y, z]},
      'accelerometer': {'0': [x, y, z]},
      'proximity': {'0': distance},
      'infrared': {'0': val1, '1': val2},
      'motor': {'0': left, '1': right}
  }
  ```

##### `format_motors(feagi_output: dict) -> bytes`

Format FEAGI commands into robot's protocol.

**Args:**
- `feagi_output`: Dictionary with FEAGI motor commands:
  ```python
  {
      'motor': {'0': 50, '1': -30},
      'servo': {'0': 45, '1': 90}
  }
  ```

**Returns:**
- Raw bytes to send to robot (or `b''` if nothing to send)

#### Optional Attribute

##### `BLUETOOTH_CONFIG: dict`

**Required!** Bluetooth connection parameters:

```python
BLUETOOTH_CONFIG = {
    "device_name": "MyRobot",       # BLE device name to search for
    "service_uuid": "6e400001-...", # BLE service UUID
    "rx_uuid": "6e400002-...",      # RX characteristic (write to robot)
    "tx_uuid": "6e400003-...",      # TX characteristic (notifications from robot)
    "timeout": 10.0                 # Optional scan timeout (default: 10s)
}
```

## Platform Support

| Platform | Method | Status |
|----------|--------|--------|
| **macOS** | Native (Core Bluetooth via bleak) | ✅ Tested |
| **Windows** | Native (Windows BLE API via bleak) | ✅ Tested |
| **Linux** | Native (BlueZ via bleak) | ✅ Tested |
| **Raspberry Pi** | Native (BlueZ via bleak) | ✅ Tested |
| **nrs-portal (Cloud)** | WebSocket relay from browser | ✅ Works |

## Troubleshooting

### "bleak is required for Bluetooth transport"

Install Bluetooth dependencies:
```bash
pip install 'feagi[bluetooth]'
```

### "Bluetooth device not found"

1. Ensure robot is powered on and in pairing mode
2. Check device name in `BLUETOOTH_CONFIG` matches actual BLE name
3. Try increasing scan timeout:
   ```python
   BLUETOOTH_CONFIG = {
       "device_name": "MyRobot",
       "timeout": 20.0  # Longer scan
       # ... other config
   }
   ```

### "Not connected to Bluetooth device"

- Robot may have disconnected
- Check robot battery
- Controller will auto-retry connection

### Linux: "BlueZ not found"

Install BlueZ:
```bash
sudo apt-get install bluetooth bluez
```

## Next Steps

1. ✅ Define your robot in `nrs-embodiments`
2. ✅ Create controller using `BluetoothRobot`
3. ✅ Test on desktop
4. ✅ Deploy to feagi-desktop (auto-detected!)
5. ✅ Works on nrs-portal too (with WebSocket relay)

## Support

- SDK Issues: https://github.com/Neuraville/feagi-python-sdk/issues
- Controller Examples: `embodiment-controllers/` repo
- Documentation: https://docs.feagi.org

---

**Simple. Standard. Works Everywhere.**

