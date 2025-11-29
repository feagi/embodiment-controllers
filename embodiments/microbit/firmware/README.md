# FEAGI micro:bit Controller

Embedded Rust firmware for BBC micro:bit devices that enables FEAGI neural network control via Bluetooth Low Energy.

## Features

- **Bluetooth LE Communication**: Custom FEAGI service for sensor data and GPIO control
- **Sensor Support**: Accelerometer, magnetometer, temperature, buttons
- **GPIO Control**: Digital/analog inputs, digital/PWM outputs
- **LED Matrix Display**: 5×5 LED grid for visual feedback
- **Configurable**: Build-time feature flags for device version and enabled features
- **Memory Efficient**: Optimized for micro:bit V1 (16KB RAM) and V2 (128KB RAM)

## Supported Devices

- **micro:bit V2** (nRF52833) - Recommended
  - 512KB Flash, 128KB RAM
  - 64MHz ARM Cortex-M4
  - Bluetooth 5.1

- **micro:bit V1** (nRF51822) - Limited support
  - 256KB Flash, 16KB RAM (very constrained!)
  - 16MHz ARM Cortex-M0
  - Bluetooth 4.1

## Prerequisites

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add ARM targets
rustup target add thumbv7em-none-eabihf  # For V2
rustup target add thumbv6m-none-eabi     # For V1

# Install probe-rs (for flashing and debugging)
cargo install probe-rs-cli --features cli

# OR install cargo-binutils (for generating .hex files)
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

## Building

### For micro:bit V2 (default):
```bash
cargo build-v2

# Generate .hex file for mass storage flashing
cargo objcopy --release --target thumbv7em-none-eabihf -- -O ihex target/thumbv7em-none-eabihf/release/feagi-microbit-controller.hex
```

### For micro:bit V1:
```bash
cargo build-v1

# Generate .hex file
cargo objcopy --release --target thumbv6m-none-eabi -- -O ihex target/thumbv6m-none-eabi/release/feagi-microbit-controller.hex
```

## Flashing

### Method 1: Mass Storage (Easiest)
1. Connect micro:bit via USB
2. Copy .hex file to `MICROBIT` drive
3. Device will flash and reboot automatically

```bash
# macOS
cp target/thumbv7em-none-eabihf/release/feagi-microbit-controller.hex /Volumes/MICROBIT/

# Linux
cp target/thumbv7em-none-eabihf/release/feagi-microbit-controller.hex /media/$USER/MICROBIT/

# Windows
copy target\thumbv7em-none-eabihf\release\feagi-microbit-controller.hex D:\
```

### Method 2: probe-rs (For Development)
```bash
# Flash and run with debugging
cargo flash-v2

# Or manually
probe-rs download --chip nRF52833_xxAA target/thumbv7em-none-eabihf/release/feagi-microbit-controller
```

## Bluetooth Service

**Service UUID**: `e95d0753-251d-470a-a062-fa1922dfa9a8`

### Characteristics:

1. **Sensor Data** (Read, Notify)
   - UUID: `e95d0754-251d-470a-a062-fa1922dfa9a8`
   - Format: JSON `{"accel":[x,y,z],"mag":[x,y,z],"temp":23.5,"buttons":{"a":false,"b":true}}`

2. **GPIO Control** (Write)
   - UUID: `e95d0755-251d-470a-a062-fa1922dfa9a8`
   - Format: Binary `[pin, mode, value, ...]`

3. **LED Matrix** (Write)
   - UUID: `e95d0756-251d-470a-a062-fa1922dfa9a8`
   - Format: 25 bytes (5×5 grid, brightness 0-255)

4. **Capabilities** (Read)
   - UUID: `e95d0757-251d-470a-a062-fa1922dfa9a8`
   - Format: JSON describing available sensors and GPIO

5. **Configuration** (Read, Write)
   - UUID: `e95d0758-251d-470a-a062-fa1922dfa9a8`
   - Format: JSON for runtime configuration

## Configuration

Create `config.json` in project root to customize build:

```json
{
  "version": "v2",
  "bluetooth_name": "FEAGI-microbit",
  "sampling_rate_hz": 10,
  "sensors": {
    "accelerometer": true,
    "magnetometer": true,
    "temperature": true,
    "buttons": true
  },
  "outputs": {
    "led_matrix": true
  },
  "gpio_mapping": {
    "0": {"mode": "analog_input", "cortical": "igpia00:neuron_0"},
    "8": {"mode": "digital_output", "cortical": "ogpid00:neuron_0"}
  }
}
```

## Project Structure

```
feagi-microbit-controller/
├── Cargo.toml              # Dependencies and build config
├── build.rs                # Build-time configuration generator
├── memory.x                # Memory layout for nRF52/nRF51
├── .cargo/
│   └── config.toml         # Target and runner configuration
├── src/
│   ├── main.rs             # Entry point and main loop
│   ├── sensors.rs          # Sensor reading (accel, mag, temp, buttons)
│   ├── bluetooth.rs        # BLE service implementation
│   ├── gpio_controller.rs  # GPIO pin control
│   └── led_display.rs      # 5×5 LED matrix driver
└── examples/
    ├── blink.rs            # Simple LED blink test
    └── sensor_test.rs      # Sensor reading test
```

## Development Status

### ✅ Implemented
- [x] Project structure
- [x] Build system with V1/V2 support
- [x] Module scaffolding
- [x] Configuration system

### 🚧 In Progress (Phase 2)
- [ ] Bluetooth LE UART service
- [ ] Sensor drivers (I2C communication)
- [ ] GPIO pin control
- [ ] LED matrix driver

### ⏳ TODO (Phase 3)
- [ ] Full BLE stack with SoftDevice
- [ ] OTA firmware updates
- [ ] Power management
- [ ] Multi-device support

## Integration with FEAGI

This firmware is designed to work with:
1. **FEAGI Desktop** - Configuration and flashing tool
2. **FEAGI Python SDK** - `feagi.sdk.microbit` module for BLE communication
3. **FEAGI Core** - Neural network processing

### Example Python Usage:
```python
from feagi.sdk import microbit

# Connect to micro:bit
mb = await microbit.connect("FEAGI-microbit")

# Read sensors
data = await mb.read_sensors()
print(f"Accel: {data['accelerometer']}")

# Control GPIO
await mb.set_gpio_digital(pin=8, value=True)
```

## Memory Constraints

### micro:bit V1 (16KB RAM - Very Limited!)
- Stack: ~4KB
- Heap: ~2KB
- Buffers: ~4KB
- Available: ~6KB
- **Use carefully!** Minimize allocations, use static buffers

### micro:bit V2 (128KB RAM - Comfortable)
- Stack: ~8KB
- Heap: ~16KB
- Buffers: ~32KB
- Available: ~72KB
- Much more flexibility for features

## Troubleshooting

### Build Errors
```bash
# Ensure targets are installed
rustup target list --installed

# Clean and rebuild
cargo clean
cargo build-v2
```

### Flashing Fails
- Ensure micro:bit appears as `MICROBIT` drive
- Try pressing reset button on back of device
- Check USB cable (must support data, not just power)

### Bluetooth Not Visible
- Firmware may not have BLE enabled yet (Phase 2)
- Check that device isn't already connected
- Try power cycling the micro:bit

## Contributing

See main FEAGI repository for contribution guidelines.

## License

MIT License - See LICENSE file

## Resources

- [micro:bit Hardware Documentation](https://tech.microbit.org/)
- [nRF52833 Datasheet](https://www.nordicsemi.com/products/nrf52833)
- [Embedded Rust Book](https://rust-embedded.github.io/book/)
- [probe-rs Documentation](https://probe.rs/)

