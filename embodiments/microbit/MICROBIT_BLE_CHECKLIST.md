# micro:bit BLE Integration Checklist

## Prerequisites for Using micro:bit Bluetooth with FEAGI

### ✅ Phase 1: Firmware (micro:bit) - **IN PROGRESS**

#### 1.1 TrouBLE BLE Stack Implementation
- [ ] Initialize TrouBLE host with nrf-sdc controller
- [ ] Set up BLE advertising with device name
- [ ] Implement Nordic UART Service (NUS) GATT server
- [ ] Handle BLE connection/disconnection events
- [ ] Implement data transmission (notify) for sensor data
- [ ] Implement data reception (write) for commands/neuron data
- [ ] Integrate with existing `bluetooth.rs` protocol layer
- [ ] Test BLE advertising on hardware
- [ ] Test BLE connection from phone/computer
- [ ] Test data transmission/reception

#### 1.2 Protocol Integration
- [ ] Connect `BleStack` to `BluetoothService` methods
- [ ] Route incoming BLE data to `BluetoothService::process_received_data()`
- [ ] Route outgoing data from `BluetoothService` to `BleStack::send_notify()`
- [ ] Handle connection state updates
- [ ] Implement proper error handling and reconnection

#### 1.3 Main Loop Integration
- [ ] Spawn BLE task in embassy executor
- [ ] Coordinate BLE task with main control loop
- [ ] Ensure proper async/await usage
- [ ] Test LED matrix updates from BLE neuron data
- [ ] Test sensor data transmission via BLE

---

### ⏳ Phase 2: Python Agent - **NOT STARTED**

#### 2.1 BLE Connection
- [ ] Create Python agent using `bleak` or `bluepy` library
- [ ] Implement BLE device scanning for micro:bit
- [ ] Connect to micro:bit via BLE (Nordic UART Service)
- [ ] Handle connection/disconnection events
- [ ] Implement reconnection logic

#### 2.2 FEAGI Protocol Implementation
- [ ] Parse FEAGI protocol packets from micro:bit
- [ ] Send FEAGI protocol packets to micro:bit
- [ ] Map sensor data (accelerometer, magnetometer, temperature) to FEAGI sensory neurons
- [ ] Map FEAGI motor neurons to micro:bit GPIO/LED commands
- [ ] Map FEAGI `omis` cortical area to LED matrix (5x5x1)

#### 2.3 FEAGI SDK Integration
- [ ] Create `MicrobitAgent` class extending FEAGI Python SDK
- [ ] Register micro:bit capabilities with FEAGI Core
- [ ] Implement sensory data sending (sensors → FEAGI)
- [ ] Implement motor data receiving (FEAGI → micro:bit)
- [ ] Handle LED matrix updates from `omis` cortical area
- [ ] Handle GPIO control from FEAGI motor cortical areas

#### 2.4 Capability Discovery
- [ ] Request capabilities from micro:bit on connection
- [ ] Parse capabilities JSON from micro:bit
- [ ] Register capabilities with FEAGI Core dynamically
- [ ] Support different micro:bit configurations (sensors enabled/disabled)

---

### ⏳ Phase 3: FEAGI Core Integration - **NOT STARTED**

#### 3.1 Python SDK Support
- [ ] Add micro:bit device type to FEAGI Python SDK
- [ ] Create `MicrobitCapabilities` class
- [ ] Add helper methods for micro:bit-specific operations
- [ ] Document micro:bit integration in SDK docs

#### 3.2 Cortical Area Mapping
- [ ] Document LED matrix mapping to `omis` (5x5x1)
- [ ] Document GPIO mapping to motor cortical areas
- [ ] Document sensor mapping to sensory cortical areas
- [ ] Create example connectome for micro:bit

#### 3.3 Device Discovery
- [ ] Add micro:bit to device registry
- [ ] Support capability discovery from micro:bit
- [ ] Auto-configure cortical areas based on capabilities

---

### ⏳ Phase 4: Testing & Validation - **NOT STARTED**

#### 4.1 Hardware Testing
- [ ] Test BLE advertising on micro:bit V2
- [ ] Test BLE connection from Python agent
- [ ] Test sensor data transmission (accelerometer, magnetometer, temperature)
- [ ] Test LED matrix control from FEAGI
- [ ] Test GPIO control from FEAGI
- [ ] Test neuron firing → LED matrix mapping
- [ ] Test connection stability and reconnection

#### 4.2 End-to-End Testing
- [ ] Connect micro:bit → Python agent → FEAGI Core
- [ ] Send sensor data from micro:bit to FEAGI
- [ ] Receive motor commands from FEAGI to micro:bit
- [ ] Test LED matrix updates from FEAGI `omis` cortical area
- [ ] Test GPIO control from FEAGI motor cortical areas
- [ ] Test with example connectome

#### 4.3 Performance Testing
- [ ] Measure BLE latency
- [ ] Test data throughput (sensor data rate)
- [ ] Test with multiple micro:bits simultaneously
- [ ] Test power consumption

---

## Current Status

**Firmware:** ✅ Compiles, ⏳ BLE stack implementation in progress
**Python Agent:** ❌ Not started
**FEAGI Core Integration:** ❌ Not started
**Testing:** ❌ Not started

---

## Next Steps

1. **Complete TrouBLE BLE stack implementation** (Phase 1.1)
2. **Integrate BLE with protocol layer** (Phase 1.2-1.3)
3. **Create Python agent** (Phase 2)
4. **Test on hardware** (Phase 4.1)
5. **Integrate with FEAGI Core** (Phase 3)
6. **End-to-end testing** (Phase 4.2)

---

## Notes

- **License:** TrouBLE is MIT/Apache-2.0 (permissive)
- **BLE Service:** Nordic UART Service (NUS) for simple serial communication
- **Protocol:** Custom FEAGI protocol packets over NUS
- **LED Matrix:** Uses `omis` (Miscellaneous Motor) cortical area, 5x5x1 dimensions
- **GPIO:** Maps to standard FEAGI motor cortical areas

