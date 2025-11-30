# BLE Communication Limitations

## Current Status

**Date:** 2024-12-19  
**Firmware Version:** 0.1.0  
**BLE Stack:** TrouBLE (trouble-host@0.1.0) via microbit-bsp@0.4.0

## ✅ What Works (One-Way Communication)

### Receiving Data (Client → micro:bit)
- **BLE Writes:** Fully functional
- **Neuron Firing Data:** Can receive neuron activation coordinates and display on LED matrix
- **GPIO Commands:** Can receive digital/PWM control commands
- **LED Matrix Commands:** Can receive direct LED matrix data
- **Connection Handling:** BLE advertising and connection acceptance works

### Implementation Status
- ✅ BLE stack initialization
- ✅ Advertising with device name
- ✅ Connection acceptance via `Advertiser::accept()`
- ✅ GATT write event processing
- ✅ Write data extraction and storage in `BLE_RX_BUFFER`
- ✅ Nordic UART Service (NUS) RX characteristic (Write)

## ❌ What Doesn't Work (Blocked by API)

### Sending Data (micro:bit → Client)
- **BLE Notifications:** **NOT FUNCTIONAL**
- **Sensor Data:** Cannot send accelerometer, magnetometer, temperature, button states
- **Status Updates:** Cannot send connection status, errors, or confirmations
- **Capabilities:** Cannot dynamically send capability data

### Root Cause
The `send_notify()` method in `ble_stack.rs` is incomplete due to API limitations:

1. **`Characteristic::notify()`** requires a `GattConnection` parameter
2. **`GattConnection::try_new()`** is `pub(crate)` - not accessible from our crate
3. **`Connection::alloc_tx()`** and **`Connection::send()`** are private methods

### Technical Details
```rust
// In ble_stack.rs, line ~281
pub async fn send_notify(&mut self, data: &[u8]) -> Result<(), &'static str> {
    // TODO: Implement proper notification sending
    // This requires GattConnection which we can't create directly
    // The proper implementation would use:
    //   tx_char.notify(&gatt_connection, &value).await
}
```

The infrastructure is in place:
- ✅ NUS TX characteristic is created with `Notify` property
- ✅ Characteristic handle is stored
- ✅ Data formatting is ready
- ❌ Cannot call `notify()` without `GattConnection`

## Impact on FEAGI Integration

### Current Capabilities
- **Motor Output:** ✅ FEAGI can send neuron activations → micro:bit LED matrix
- **Visual Feedback:** ✅ Neuron firing patterns display correctly
- **One-Way Control:** ✅ Can control micro:bit actuators (LEDs, GPIO)

### Missing Capabilities
- **Sensory Input:** ❌ FEAGI cannot receive sensor data from micro:bit
- **Closed Loop:** ❌ Sensorimotor loop is incomplete (motor → sensor feedback missing)
- **Adaptive Behavior:** ❌ FEAGI cannot adapt based on sensor feedback

### Workaround Options

#### Option 1: Write-Response Pattern (Not Implemented)
Instead of notifications, use write requests:
- Client writes a "request sensor data" command
- micro:bit responds via write response
- Less efficient (polling required) but would work

#### Option 2: Fix API Access (Future)
- Request trouble-host to expose `GattConnection::try_new()` as public
- Or use unsafe code to access private APIs (not recommended)
- Or restructure to use GattConnection properly (blocked by private constructor)

#### Option 3: Alternative BLE Stack (Future)
- Consider using a different BLE stack that has better API access
- Would require significant refactoring

## Testing One-Way Communication

### What You Can Test
1. **Flash firmware** to micro:bit
2. **Scan for device** - Should see "FEAGI-microbit" advertising
3. **Connect via BLE** - Connection should succeed
4. **Send neuron data** - Write to NUS RX characteristic
5. **Observe LED matrix** - Should display neuron activation patterns

### What Won't Work
1. **Receiving sensor data** - Notifications won't be sent
2. **Reading sensor characteristics** - No data will be available
3. **Bidirectional communication** - Only client → micro:bit works

### Python Agent Status
The Python agent (`agent.py`) expects to:
- ✅ Write neuron data to micro:bit (will work)
- ❌ Receive sensor data via notifications (will fail silently)

The agent will connect and send data, but won't receive any sensor feedback.

## Files Affected

- `src/ble_stack.rs` - `send_notify()` method incomplete (line ~281)
- `src/main.rs` - Sensor reading works, but `ble_stack.send_notify()` calls will fail (line ~315)
- `src/bluetooth.rs` - `send_sensor_data()` returns data, but cannot be sent
- `agent.py` - Will work for sending data, but won't receive sensor notifications

## Next Steps

1. **Test one-way communication** - Verify LED matrix control works
2. **Evaluate workaround options** - Decide on write-response pattern or API fix
3. **Document sensor data format** - Prepare for when notifications work
4. **Update agent.py** - Handle missing sensor data gracefully

## References

- TrouBLE documentation: https://github.com/trouble-rs/trouble
- microbit-bsp: https://crates.io/crates/microbit-bsp
- BLE GATT specification: Bluetooth Core Specification v5.3, Vol 3, Part G

