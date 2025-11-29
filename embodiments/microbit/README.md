# BBC micro:bit FEAGI Embodiment

Control BBC micro:bit devices with FEAGI neural networks.

## Components

### 1. Firmware (`firmware/`)
- **Language**: Rust (embedded)
- **Target**: ARM Cortex-M (nRF52/nRF51)
- **Output**: `.hex` file
- **Runs ON**: micro:bit hardware

Firmware that runs directly on the micro:bit. Handles sensors, GPIO, LED matrix, and Bluetooth communication.

**Build**: See `firmware/README.md`

### 2. Agent (`agent/`) - Coming Soon (Phase 3)
- **Language**: Python
- **Runs ON**: Desktop/Raspberry Pi
- **Connects**: micro:bit (BLE) ↔ FEAGI (ZMQ)

Python script that bridges Bluetooth LE communication from micro:bit to FEAGI Core.

## Quick Start

### Flash Firmware (Easiest)
1. Open **FEAGI Desktop**
2. Launch **micro:bit Flasher** tool
3. Configure sensors/GPIO
4. Build firmware
5. Flash to device

### Manual Build
```bash
cd firmware
./build-firmware.sh v2
cp firmware.hex /Volumes/MICROBIT/
```

### Connect to FEAGI (Phase 3)
```bash
cd agent
python agent.py --device "FEAGI-microbit"
```

## Architecture

```
micro:bit Hardware
  ↓ (Bluetooth LE)
Python Agent
  ↓ (ZMQ)
FEAGI Core
```

## FEAGI Cortical Area Mapping

### LED Matrix Output

The micro:bit LED matrix is mapped to a FEAGI OPU (Output Processing Unit) cortical area:

- **Cortical Type**: `omis` (Miscellaneous Motor)
- **Cortical Name**: "LED Matrix" or "Display Matrix"
- **Dimensions**: 5×5×1 (matches micro:bit LED matrix)
- **Coordinate Mapping**: Direct 1:1 mapping
  - Cortical area neuron at (x, y) → LED at (x, y)
  - Coordinates: x, y ∈ [0, 4]

**Usage**:
1. Create a cortical area in FEAGI with:
   - Type: `omis` (Miscellaneous Motor)
   - Name: "LED Matrix"
   - Dimensions: 5×5×1
2. Python agent subscribes to this cortical area
3. When neurons fire, extract (x, y) coordinates
4. Send coordinates to micro:bit via BLE
5. Firmware updates corresponding LEDs

**Example Cortical ID**: `_____10c-omis00-cx-__name-t` → "LED Matrix"

## Supported Features

**Sensors**:
- ✅ Accelerometer
- ✅ Magnetometer  
- ✅ Temperature
- ✅ Buttons (A, B)

**Outputs**:
- ✅ LED Matrix (5×5)
- ✅ GPIO Pins (digital, PWM)

**Communication**:
- ✅ Bluetooth LE UART
- 🚧 BLE service (Phase 3)

## Files

```
microbit/
  firmware/          # Rust embedded firmware
    src/
    Cargo.toml
    build-firmware.sh
  agent/             # Python BLE bridge (coming)
    agent.py
  README.md          # This file
```

## Development Status

- ✅ Phase 1: Desktop flasher UI
- ✅ Phase 2: Firmware scaffold (compiles, LED works)
- 🚧 Phase 3: BLE service + Python agent
- 🚧 Phase 4: Full FEAGI integration

## Documentation

- [Firmware README](firmware/README.md)
- [Quick Start](firmware/QUICKSTART.md)
- [Testing Guide](firmware/TESTING_GUIDE.md)

## License

MIT - See LICENSE

