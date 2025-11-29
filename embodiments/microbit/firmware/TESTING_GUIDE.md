# micro:bit Firmware Testing Guide

Quick guide to test the FEAGI micro:bit firmware.

---

## Quick Build & Flash

```bash
# 1. Build firmware for micro:bit V2
cd /Users/nadji/code/FEAGI-2.0/feagi-microbit-controller
./build-firmware.sh v2

# 2. Connect micro:bit via USB (appears as MICROBIT drive)

# 3. Flash firmware
cp firmware.hex /Volumes/MICROBIT/

# 4. Wait for LED to stop flashing (auto-reboot)

# 5. Observe LED patterns:
#    - Heart pattern (startup)
#    - FEAGI "F" logo
#    - Center LED blinking (heartbeat)
```

---

## What to Expect

### ✅ Working Features:
- **LED Matrix**: Heart → FEAGI logo → heartbeat blink
- **Startup**: Quick boot (~2 seconds)
- **Stability**: No crashes, continuous running

### 🟡 Mock Features (Not Real Yet):
- **Sensors**: Returns mock accelerometer/magnetometer/temp data
- **Bluetooth**: No BLE service active yet
- **GPIO**: No pin control yet

---

## Troubleshooting

### Firmware Won't Flash
```bash
# Verify micro:bit is detected
ls -la /Volumes/MICROBIT/

# Check .hex file is valid
ls -lh firmware.hex
# Should be ~3-4KB

# Try pressing reset button on back of micro:bit
```

### LED Not Showing Patterns
```bash
# Rebuild with verbose output
cargo build --release --features v2 --target thumbv7em-none-eabihf --verbose

# Check for compilation warnings
# If it compiled successfully, the LED *should* work
```

### Build Fails
```bash
# Ensure Rust toolchain is installed
rustup show

# Verify target is installed
rustup target list --installed | grep thumbv7em

# Clean and rebuild
cargo clean
cargo build --release --features v2 --target thumbv7em-none-eabihf
```

---

## Development Testing

### Test LED Patterns
Edit `src/main.rs` to add more patterns:

```rust
// Show checkmark
led_display.show_checkmark();
led_display.show(&mut timer);
timer.delay_ms(2000_u32);
```

### Test Different Blink Rates
```rust
// Faster heartbeat
timer.delay_ms(250_u32);  // Instead of 1000
```

### Add Debug Output (requires probe-rs)
```rust
defmt::info!("LED pattern displayed");
defmt::debug!("Sensor data: {:?}", sensor_data);
```

---

## Memory Usage

Check how much memory the firmware uses:

```bash
cargo size --release --target thumbv7em-none-eabihf -- -A

# Expected output:
# .text:   ~3-4 KB (code)
# .rodata: ~500 bytes (constants)
# .data:   ~100 bytes (initialized data)
# .bss:    ~1 KB (uninitialized data)
```

---

## Next Steps

Once you verify the basic firmware works:

1. **Add Real Sensor Reading** (`src/sensors.rs`)
2. **Implement BLE Service** (`src/bluetooth.rs`)
3. **Add GPIO Control** (`src/gpio_controller.rs`)
4. **Create Python BLE Client** (FEAGI Python SDK)

---

## Quick Reference

### Build Commands:
```bash
# V2 (recommended)
cargo build --release --features v2 --target thumbv7em-none-eabihf

# V1 (limited RAM!)
cargo build --release --features v1 --target thumbv6m-none-eabi
```

### Generate .hex:
```bash
cargo objcopy --release --target thumbv7em-none-eabihf -- -O ihex firmware.hex
```

### Check Build Logs:
```bash
cargo build --release --features v2 --target thumbv7em-none-eabihf 2>&1 | tee build.log
```

---

**Status**: Ready for testing! 🚀

