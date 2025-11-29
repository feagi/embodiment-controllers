# BLE Version Mismatch Issue

## Problem

We upgraded `trouble-host` from `0.1.0` to `0.5.1` to resolve the `LeReadBufferSize` trait implementation issue. However, this introduced a new problem:

- **trouble-host@0.5.1** uses **bt-hci@0.6**
- **nrf-sdc@0.1.1** uses **bt-hci@0.3**

This version mismatch causes trait implementation issues where `SoftdeviceController` doesn't implement all traits required by `trouble_host::Controller` in the newer version.

## Current Status

The code compiles with a placeholder implementation. The BLE stack is initialized but not fully functional yet.

## Solutions

### Option 1: Downgrade trouble-host (Not Recommended)
- Go back to `trouble-host@0.1.0` which uses `bt-hci@0.2`
- Still has the `LeReadBufferSize` issue we were trying to fix
- Not a viable solution

### Option 2: Wait for nrf-sdc Update (Recommended)
- Wait for `nrf-sdc` to update to `bt-hci@0.6` or compatible version
- This is the cleanest solution but requires upstream changes

### Option 3: Use Different BLE Stack
- Consider using `nrf-softdevice` directly (but has license issues)
- Or find another pure Rust BLE controller implementation

### Option 4: Compatibility Layer (Complex)
- Create a compatibility layer that bridges bt-hci 0.3 and 0.6
- Very complex and error-prone

## Recommendation

**Option 2**: Wait for `nrf-sdc` to update, or check if there's a newer version that supports `bt-hci@0.6`.

In the meantime, we have a working placeholder that compiles and can be extended once compatibility is resolved.

## Next Steps

1. Check if there's a newer `nrf-sdc` version that supports `bt-hci@0.6`
2. Check if `microbit-bsp` has updates that resolve this
3. Monitor the `nrf-sdc` and `trouble-host` repositories for compatibility updates

