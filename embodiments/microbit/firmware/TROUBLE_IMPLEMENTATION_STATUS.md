# TrouBLE BLE Stack Implementation Status

## Overview

TrouBLE (pure Rust BLE Host) has been integrated into the firmware structure. The implementation provides the framework for BLE communication using Nordic UART Service (NUS).

## Current Status

### ✅ Completed

1. **BLE Stack Structure** (`src/ble_stack.rs`)
   - `BleStack` struct defined
   - Nordic UART Service UUIDs defined
   - Advertising data creation functions
   - Unit tests for advertising data and UUIDs

2. **Protocol Layer** (`src/bluetooth.rs`)
   - FEAGI protocol packet parsing
   - Command handling
   - Sensor data serialization
   - Neuron firing data handling

3. **Integration Points**
   - `BleStack` methods defined for:
     - Initialization
     - Advertising
     - Connection handling
     - Data transmission (notify)
     - Data reception (write)

### ⏳ Pending Implementation

#### 1. nrf-sdc Controller Initialization

**Issue:** nrf-sdc requires specific peripherals that conflict with `microbit-v2`:
- **RTC0** - Used by nrf-sdc for timing
- **TIMER0** - Used by nrf-sdc AND microbit-v2 (CONFLICT!)
- **PPI_CH17-29** - Used by nrf-sdc
- **TEMP** - Used by nrf-sdc

**Solution Options:**

**Option A: Coordinate Peripherals**
- Use `microbit-v2` for display/sensors/GPIO
- Use `embassy-nrf` peripherals for BLE (steal from Board)
- Initialize nrf-sdc with embassy-nrf peripherals
- Use different timer for microbit-v2 (TIMER1 instead of TIMER0)

**Option B: Use embassy-nrf BLE Directly**
- Skip TrouBLE/nrf-sdc
- Use `embassy-nrf`'s built-in BLE support (if available)
- Simpler but may have less features

**Option C: Use microbit-bsp with trouble feature**
- `microbit-bsp` may handle peripheral coordination
- Check if it supports TrouBLE integration

#### 2. TrouBLE Host Initialization

```rust
// TODO: In BleStack::new()
// 1. Initialize nrf-sdc controller
// 2. Create TrouBLE Host with controller
// 3. Set up GATT server
// 4. Register Nordic UART Service
```

#### 3. BLE Task Spawning

```rust
// TODO: In main()
// Spawn BLE task to:
// 1. Run TrouBLE host runner
// 2. Process BLE events
// 3. Handle connections
// 4. Route data to/from BluetoothService
```

#### 4. GATT Server Setup

```rust
// TODO: Set up GATT server with:
// - Nordic UART Service (NUS)
//   - TX Characteristic (notify) - for sending data
//   - RX Characteristic (write) - for receiving data
```

## Implementation Steps

### Step 1: Resolve Peripheral Conflicts

**Priority:** 🔥 CRITICAL

Choose one of the solution options above and implement peripheral coordination.

### Step 2: Initialize nrf-sdc

```rust
// Initialize MPSL (Multiprotocol Service Layer)
let mpsl = MultiprotocolServiceLayer::new(mpsl_peripherals, irqs, lfclk_cfg)?;

// Initialize nrf-sdc controller
let sdc = SoftdeviceController::builder()
    .support_adv()
    .support_peripheral()
    .build(sdc_peripherals, rng, mpsl, sdc_mem)?;

// Spawn MPSL and SDC tasks
spawner.must_spawn(mpsl_task(mpsl));
spawner.must_spawn(sdc_task(sdc));
```

### Step 3: Initialize TrouBLE Host

```rust
// Create TrouBLE Host with nrf-sdc controller
let host_resources = HostResources::new();
let stack = trouble_host::new(sdc, &mut host_resources);
let Host { peripheral, runner, .. } = stack.build();

// Spawn TrouBLE runner task
spawner.must_spawn(ble_runner_task(runner));
```

### Step 4: Set Up GATT Server

```rust
// Create GATT server
let gatt_server = peripheral.gatt_server();

// Register Nordic UART Service
let nus_service = gatt_server.add_service(NUS_SERVICE_UUID)?;
let tx_char = nus_service.add_characteristic(
    NUS_TX_CHAR_UUID,
    CharacteristicProperties::NOTIFY,
)?;
let rx_char = nus_service.add_characteristic(
    NUS_RX_CHAR_UUID,
    CharacteristicProperties::WRITE,
)?;
```

### Step 5: Start Advertising

```rust
// Create advertising data
let adv_data = create_advertising_data(device_name);

// Start advertising
let advertiser = peripheral.start_advertising(
    &adv_data,
    &create_scan_response(device_name),
    AdvertisingParams::default(),
)?;
```

### Step 6: Handle Connections

```rust
// In BLE task, handle connection events
match event {
    ConnectionEvent::Connected(conn) => {
        ble_stack.set_connected(true);
        // Store connection handle
    }
    ConnectionEvent::Disconnected => {
        ble_stack.set_connected(false);
        // Restart advertising
    }
    GattEvent::Write(handle, data) => {
        if handle == rx_char_handle {
            // Route to BluetoothService
            bluetooth_service.process_received_data(data);
        }
    }
}
```

### Step 7: Integrate with Main Loop

```rust
// In main loop:
// 1. Check for outgoing data from BluetoothService
// 2. Send via BLE notify
// 3. Process incoming BLE data
// 4. Route to BluetoothService
```

## Testing Checklist

- [ ] BLE advertising starts on boot
- [ ] Device appears in BLE scanner
- [ ] Connection from phone/computer succeeds
- [ ] Data transmission (sensor data) works
- [ ] Data reception (commands) works
- [ ] LED matrix updates from BLE neuron data
- [ ] GPIO control from BLE commands
- [ ] Reconnection after disconnect works

## Dependencies

- `trouble-host = "0.1"` ✅ Added
- `nrf-sdc = { version = "0.1", features = ["nrf52833"] }` ⚠️ Needs peripheral coordination
- `embassy-nrf = "0.8"` ✅ Upgraded
- `embassy-executor = "0.8"` ✅ Upgraded

## Next Steps

1. **Resolve peripheral conflicts** (Option A, B, or C)
2. **Implement nrf-sdc initialization** with coordinated peripherals
3. **Implement TrouBLE host initialization**
4. **Set up GATT server** with Nordic UART Service
5. **Spawn BLE task** in main
6. **Test on hardware**

## References

- [TrouBLE Documentation](https://embassy.dev/trouble)
- [nrf-sdc Documentation](https://github.com/alexmoon/nrf-sdc)
- [Nordic UART Service](https://infocenter.nordicsemi.com/topic/sdk_nrf5_v17.1.0/ble_sdk_app_nus_eval.html)


