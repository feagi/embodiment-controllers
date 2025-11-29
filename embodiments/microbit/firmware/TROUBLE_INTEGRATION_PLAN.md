# TrouBLE Integration Plan for micro:bit Firmware

## Current Status

**Issue**: Version conflict between `embassy-nrf 0.1.0` and `trouble-host 0.1.0`

- `embassy-nrf 0.1.0` requires `embassy-time 0.3`
- `trouble-host 0.1.0` requires `embassy-time 0.4`
- These are incompatible in the same dependency graph

## TrouBLE Overview

**What is TrouBLE?**
- Pure Rust BLE Host implementation from embassy-rs
- Manages upper BLE protocol layers (GATT, L2CAP, ATT, SMP)
- Dual licensed: MIT/Apache-2.0

**What it does:**
- Peripheral role: Advertise and accept connections
- Central role: Scan and connect to peripherals
- GATT server/client: Read, write, notifications
- L2CAP CoC: Connection-oriented channels

**Hardware Support:**
- nRF SoftDevice Controller
- UART HCI
- Raspberry Pi Pico W
- Apache NimBLE Controller
- ESP32
- Linux HCI Sockets

## Integration Options

### Option 1: Upgrade embassy-nrf to 0.8.x
**Pros:**
- Latest features and bug fixes
- Likely compatible with TrouBLE

**Cons:**
- Requires refactoring existing code
- May have breaking API changes
- Need to verify microbit-v2 compatibility

**Steps:**
1. Upgrade `embassy-nrf` from `0.1` to `0.8`
2. Update `embassy-executor` to compatible version
3. Update `embassy-time` to `0.4`
4. Refactor code for API changes
5. Add `trouble-host` dependency
6. Implement BLE stack using TrouBLE

### Option 2: Use microbit-bsp with trouble feature
**Pros:**
- Handles version conflicts automatically
- Pre-configured for micro:bit
- Well-tested integration

**Cons:**
- Requires switching from `microbit-v2` to `microbit-bsp`
- May need code refactoring
- Less control over HAL usage

**Steps:**
1. Replace `microbit-v2` with `microbit-bsp = { version = "0.4", features = ["trouble"] }`
2. Refactor code to use `microbit-bsp` APIs
3. Implement BLE using `microbit-bsp`'s TrouBLE integration

### Option 3: Wait for TrouBLE compatibility
**Pros:**
- No code changes needed
- Keep current stable setup

**Cons:**
- Unknown timeline
- May never happen if TrouBLE moves forward

### Option 4: Use nrf-sdc (SoftDevice Controller) with TrouBLE
**Pros:**
- Pure Rust Host (TrouBLE)
- Only Controller binary needed (smaller than full SoftDevice)

**Cons:**
- Still requires proprietary SoftDevice Controller binary
- More complex setup
- May have same version conflicts

## Recommended Approach

**Short-term**: Keep current blocking implementation, document TrouBLE integration as future work

**Long-term**: Option 1 (upgrade embassy-nrf) or Option 2 (use microbit-bsp)

## Implementation Notes

The BLE protocol layer (`bluetooth.rs`) is already implemented and ready for integration. Once TrouBLE is integrated, it will:

1. Initialize TrouBLE Host stack
2. Set up Nordic UART Service (NUS) for FEAGI communication
3. Handle advertising and connections
4. Process incoming BLE data and send sensor data
5. Support neuron firing data for LED matrix control

## References

- TrouBLE Documentation: https://embassy.dev/trouble/
- TrouBLE GitHub: https://github.com/embassy-rs/trouble
- microbit-bsp: https://docs.rs/microbit-bsp
- embassy-nrf: https://docs.embassy.dev/embassy-nrf

