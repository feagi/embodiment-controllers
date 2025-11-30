# BLE Compatibility Layer Implementation Guide

## Overview

The compatibility layer bridges `bt-hci@0.3` (used by `nrf-sdc`) with `bt-hci@0.2` (used by `trouble-host@0.1.0`).

## Current Status

✅ **Structure Created**: `ble_compat.rs` with `BleCompatController` wrapper
⏳ **Implementation Pending**: ~30 trait methods need to be implemented

## Required Trait Implementations

The `trouble_host::Controller` trait requires implementing:

### Core Controller Trait
- `bt_hci::controller::Controller` (from bt-hci@0.2)

### Synchronous Commands (ControllerCmdSync)
1. `LeReadBufferSize`
2. `Disconnect`
3. `SetEventMask`
4. `SetEventMaskPage2`
5. `LeSetEventMask`
6. `LeSetRandomAddr`
7. `HostBufferSize`
8. `LeReadFilterAcceptListSize`
9. `SetControllerToHostFlowControl`
10. `Reset`
11. `ReadRssi`
12. `LeCreateConnCancel`
13. `LeSetScanEnable`
14. `LeSetExtScanEnable`
15. `LeClearFilterAcceptList`
16. `LeAddDeviceToFilterAcceptList`
17. `LeSetAdvEnable` (lifetime parameter)
18. `LeSetExtAdvEnable` (lifetime parameter)
19. `HostNumberOfCompletedPackets` (lifetime parameter)
20. `LeSetAdvData` (lifetime parameter)
21. `LeSetAdvParams`
22. `LeSetScanResponseData` (lifetime parameter)
23. `LeLongTermKeyRequestReply`
24. `ReadBdAddr`

### Asynchronous Commands (ControllerCmdAsync)
1. `LeConnUpdate`
2. `LeCreateConn`
3. `LeEnableEncryption`

## Implementation Pattern

For each command, the pattern is:

```rust
impl<'d> ControllerCmdSync<CommandV2> for BleCompatController<'d> {
    async fn exec(
        &self,
        cmd_v2: &CommandV2,
    ) -> Result<<CommandV2 as SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
        // 1. Convert v2 command to bytes
        let mut buf = [0u8; 256];
        let len = cmd_v2.write_hci(&mut buf)?;
        
        // 2. Deserialize as v3 command
        let cmd_v3 = CommandV3::from_hci_bytes(&buf[..len])?;
        
        // 3. Call underlying controller (uses bt-hci@0.3)
        let ret_v3 = self.inner.exec(&cmd_v3).await?;
        
        // 4. Convert v3 response to bytes
        let mut ret_buf = [0u8; 256];
        let ret_len = ret_v3.as_hci_bytes(&mut ret_buf)?;
        
        // 5. Deserialize as v2 response
        let ret_v2 = <CommandV2 as SyncCmd>::Return::from_hci_bytes(&ret_buf[..ret_len])?;
        
        Ok(ret_v2)
    }
}
```

## Binary Compatibility

HCI commands are binary-compatible between bt-hci versions because they follow the Bluetooth specification. The struct layouts are identical, so serialization/deserialization should work correctly.

## Reusability

This compatibility layer is **100% reusable** for:
- Any Nordic nRF device using `nrf-sdc` with `trouble-host`
- Not just micro:bit - works for nRF52833, nRF52840, etc.
- Any future device using the same stack combination

## Next Steps

1. Implement `bt_hci::controller::Controller` trait
2. Implement all `ControllerCmdSync` traits (24 commands)
3. Implement all `ControllerCmdAsync` traits (3 commands)
4. Test with actual BLE communication
5. Consider creating a macro to generate boilerplate

## Estimated Effort

- ~500-1000 lines of code
- Mostly repetitive (good candidate for macro generation)
- Straightforward but tedious
- Once complete, fully reusable across all nRF devices


