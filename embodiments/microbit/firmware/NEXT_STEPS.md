# Next Steps - micro:bit BLE Implementation

## ✅ Completed

1. ✅ **Async refactor** - Embassy executor with async tasks
2. ✅ **Peripheral coordination** - Option C implemented (Board + embassy RADIO)
3. ✅ **BLE structure** - Protocol layer, packet parsing, data structures
4. ✅ **Communication channels** - BLE task ↔ Main loop via embassy-sync

---

## 🎯 Immediate Next Steps

### 1. **Implement BLE Stack** (Priority: HIGH)

**Current Status:** `ble_stack.rs` has stub implementation

**What needs to be done:**
- Implement actual BLE initialization using embassy-nrf or nrf-softdevice
- Implement BLE advertising
- Implement connection handling
- Implement characteristic read/write/notify

**Options:**
- **Option A:** Use `nrf-softdevice` (requires binary blob, but well-supported)
- **Option B:** Use `embassy-nrf` BLE (if available, pure Rust)
- **Option C:** Use `micro:bit BLE UART` crate (simplest, but limited)

**Recommendation:** Check if embassy-nrf has BLE support. If not, use nrf-softdevice with embassy executor.

**Files to modify:**
- `src/ble_stack.rs` - Implement actual BLE stack
- `Cargo.toml` - Add BLE dependencies

---

### 2. **Test Compilation** (Priority: HIGH)

**What to do:**
```bash
cd embodiment-controllers/embodiments/microbit/firmware
cargo check --target thumbv7em-none-eabihf
```

**Expected issues:**
- `steal()` method might not exist - need to verify embassy-nrf API
- BLE dependencies might need adjustment
- Type mismatches between Board and embassy types

---

### 3. **Verify Peripheral Coordination** (Priority: MEDIUM)

**What to verify:**
- Board initialization works
- RADIO access via `steal()` works
- No runtime conflicts

**If `steal()` doesn't exist:**
- Use unsafe to access RADIO from PAC directly
- Or restructure to not use Board::take()

---

### 4. **Implement BLE Service** (Priority: HIGH)

**Nordic UART Service (NUS) - Standard for micro:bit:**
- Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- TX (Notify): `6e400003-b5a3-f393-e0a9-e50e24dcca9e` (device → client)
- RX (Write): `6e400002-b5a3-f393-e0a9-e50e24dcca9e` (client → device)

**What to implement:**
- Create BLE service with NUS UUIDs
- Create TX characteristic (notify)
- Create RX characteristic (write)
- Handle characteristic writes (RX data)
- Send notifications (TX data)

---

### 5. **Integration Testing** (Priority: MEDIUM)

**What to test:**
- Firmware compiles and flashes
- Startup sequence displays "FEAGI"
- BLE advertising starts
- Device is discoverable
- Can connect from Python/Web Bluetooth
- Data can be sent/received

---

## 📋 Detailed Implementation Plan

### Phase 1: BLE Stack Selection & Setup

1. **Research embassy-nrf BLE support**
   - Check embassy-nrf documentation
   - Verify BLE API availability
   - Check license compatibility (must be MIT/Apache-2.0)

2. **Choose BLE stack**
   - If embassy-nrf has BLE → use it
   - If not → use nrf-softdevice with embassy executor
   - Fallback → micro:bit BLE UART crate

3. **Update dependencies**
   - Add chosen BLE stack to `Cargo.toml`
   - Update `ble_stack.rs with actual implementation

### Phase 2: BLE Service Implementation

1. **Initialize BLE stack**
   - Configure BLE parameters
   - Initialize SoftDevice (if using nrf-softdevice)
   - Set up event handlers

2. **Create NUS service**
   - Define service UUID
   - Create TX characteristic (notify)
   - Create RX characteristic (write)
   - Register service with BLE stack

3. **Implement advertising**
   - Create advertising data
   - Create scan response
   - Start advertising with device name

### Phase 3: Connection & Data Handling

1. **Handle connections**
   - Detect connection events
   - Update connection status
   - Notify main loop via channel

2. **Handle data reception**
   - Listen for RX characteristic writes
   - Parse incoming data
   - Send to main loop via `BLE_RX_CHANNEL`

3. **Handle data transmission**
   - Receive data from main loop via `BLE_TX_CHANNEL`
   - Send via TX characteristic notify
   - Handle MTU/notification limits

### Phase 4: Testing & Refinement

1. **Unit testing**
   - Test BLE initialization
   - Test advertising
   - Test connection handling

2. **Integration testing**
   - Test with Python agent
   - Test with Web Bluetooth
   - Test data round-trip

3. **Performance optimization**
   - Optimize packet sizes
   - Optimize notification frequency
   - Handle backpressure

---

## 🔍 Research Needed

1. **embassy-nrf BLE API**
   - Does it exist?
   - What's the API?
   - License compatibility?

2. **nrf-softdevice with embassy**
   - How to integrate?
   - Event handling?
   - License (binary blob - acceptable?)

3. **micro:bit BLE UART**
   - API documentation
   - Compatibility with embassy executor
   - Limitations

---

## 📝 Code Locations

**Files to implement:**
- `src/ble_stack.rs` - BLE stack implementation (stub → real)
- `src/bluetooth.rs` - Protocol layer (already done ✅)
- `src/main.rs` - Integration (already done ✅)

**Files to check:**
- `Cargo.toml` - Dependencies
- `build.rs` - Build configuration
- `memory.x` - Memory layout (might need SoftDevice space)

---

## 📊 Priority Matrix

| Task | Priority | Effort | Blocking |
|------|----------|--------|----------|
| BLE Stack Implementation | HIGH | High | Yes |
| Test Compilation | HIGH | Low | Yes |
| Verify Peripheral Coordination | MEDIUM | Low | No |
| BLE Service Implementation | HIGH | Medium | Yes |
| Integration Testing | MEDIUM | Medium | No |

---

## 🎯 Recommended Next Action

**Start with:** Research embassy-nrf BLE support and choose BLE stack

**Then:** Implement BLE stack initialization in `ble_stack.rs`

**Goal:** Get BLE advertising working and device discoverable



