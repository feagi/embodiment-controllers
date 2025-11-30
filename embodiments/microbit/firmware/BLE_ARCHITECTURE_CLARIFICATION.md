# BLE Architecture Clarification

## Current Setup

We're using **TrouBLE** (pure Rust BLE Host) with **nrf-sdc** (Nordic SoftDevice Controller).

### Architecture Layers

```
┌─────────────────────────────────────┐
│   trouble-host (Pure Rust)         │  ← BLE Host Stack (GATT, L2CAP, ATT)
│   MIT/Apache-2.0 License           │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│   nrf-sdc (SoftDevice Controller)   │  ← BLE Controller (Hardware Interface)
│   Uses Nordic Binary Blob            │  ⚠️ Proprietary component
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│   nRF52833 Hardware (Radio)         │
└─────────────────────────────────────┘
```

### What's Pure Rust vs Binary Blob

- ✅ **trouble-host**: Pure Rust BLE Host stack (GATT, L2CAP, ATT protocols)
- ⚠️ **nrf-sdc**: Rust wrapper around Nordic's SoftDevice Controller (binary blob)
- ✅ **Application Code**: Pure Rust

## The Issue

`nrf-sdc` provides the BLE Controller interface, but it wraps Nordic's proprietary SoftDevice Controller binary. This means:
- The **Host stack** (protocols) is pure Rust ✅
- The **Controller layer** (hardware interface) uses a binary blob ⚠️

## Options

### Option 1: Accept Hybrid Approach (Current)
- **Host**: Pure Rust (`trouble-host`)
- **Controller**: Nordic binary (`nrf-sdc`)
- **Status**: Standard approach for nRF chips with TrouBLE
- **License**: Host is MIT, Controller is proprietary

### Option 2: Pure Rust Controller (If Available)
- Need to find/implement a pure Rust BLE controller for nRF
- May not exist yet or may be experimental
- Would require significant development

### Option 3: Use Different Hardware
- Use a chip with pure Rust BLE support
- Not practical for micro:bit

## Recommendation

For the micro:bit, the hybrid approach (Option 1) is the standard way to use TrouBLE:
- `trouble-host` provides the pure Rust protocol stack
- `nrf-sdc` provides the hardware controller interface
- This is how TrouBLE is designed to work on nRF chips

The binary blob is only for the low-level radio controller, not the protocol stack.

## Next Steps

1. **Accept the hybrid approach** and proceed with implementation
2. **Document the license situation** clearly
3. **Implement BLE functionality** using `trouble-host` + `nrf-sdc`

The compatibility issue we hit (`ControllerCmdSync<LeReadBufferSize>`) is a trait implementation gap in `nrf-sdc`, not a fundamental architecture problem.


