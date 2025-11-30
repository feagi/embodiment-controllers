# Async Refactor Analysis for embassy-nrf BLE

## Impact Scope: **MICROBIT FIRMWARE ONLY** ✅

The async refactor is **completely isolated** to the micro:bit firmware. It does **NOT** impact any other parts of FEAGI.

---

## What Needs to Change

### 1. Main Entry Point (`src/main.rs`)

**Current (Blocking):**
```rust
#[entry]
fn main() -> ! {
    // Blocking initialization
    let board = Board::take().unwrap();
    let mut timer = Timer::new(board.TIMER0);
    
    // Blocking main loop
    loop {
        // Blocking operations
        timer.delay_ns(10_000_000);
    }
}
```

**Required (Async):**
```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Async initialization
    let p = embassy_nrf::init(Default::default());
    let board = Board::take().unwrap();
    
    // Spawn BLE task
    spawner.spawn(ble_task(p.RADIO, board)).unwrap();
    
    // Main control loop (can be async or blocking)
    // ...
}
```

**Impact**: Only `main.rs` entry point changes

---

### 2. BLE Stack (`src/ble_stack.rs`)

**Current**: Stub implementation with blocking interface

**Required**: Full async BLE implementation using embassy-nrf

**Impact**: Only `ble_stack.rs` needs async implementation

---

### 3. Other Modules

**Modules that DON'T need to change:**
- ✅ `src/bluetooth.rs` - Protocol layer (no async needed)
- ✅ `src/led_display.rs` - LED control (blocking is fine)
- ✅ `src/sensors.rs` - Sensor reading (blocking is fine)
- ✅ `src/gpio_controller.rs` - GPIO control (blocking is fine)

**Why**: These modules are called from the main loop, which can remain blocking. Only BLE operations need to be async.

---

## Architecture: Hybrid Approach

We can use a **hybrid architecture** where:
- **BLE operations**: Async (using embassy executor)
- **Main control loop**: Can remain blocking (or become async)
- **Other modules**: Remain blocking

### Option A: Full Async (Recommended)

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize
    let p = embassy_nrf::init(Default::default());
    let board = Board::take().unwrap();
    
    // Spawn BLE task (runs independently)
    spawner.spawn(ble_task(p.RADIO)).unwrap();
    
    // Main control loop (async)
    loop {
        // Read sensors, update LEDs, etc.
        // Can use async timers if needed
        embassy_time::Timer::after(Duration::from_millis(10)).await;
    }
}
```

### Option B: Hybrid (BLE Async, Main Loop Blocking)

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize
    let p = embassy_nrf::init(Default::default());
    let board = Board::take().unwrap();
    
    // Spawn BLE task (runs independently)
    spawner.spawn(ble_task(p.RADIO)).unwrap();
    
    // Spawn blocking main loop task
    spawner.spawn(blocking_main_loop(board)).unwrap();
    
    // Executor runs forever
}
```

---

## Files That Will Change

### Must Change:
1. ✅ `src/main.rs` - Entry point becomes async
2. ✅ `src/ble_stack.rs` - BLE implementation becomes async

### May Change (Optional):
3. ⚠️ `src/main.rs` - Main loop can become async (optional)
4. ⚠️ Timer usage - Can use async timers instead of blocking

### Won't Change:
- ✅ `src/bluetooth.rs` - Protocol layer (no async)
- ✅ `src/led_display.rs` - LED control (no async)
- ✅ `src/sensors.rs` - Sensor reading (no async)
- ✅ `src/gpio_controller.rs` - GPIO control (no async)
- ✅ `build.rs` - Build script (no change)
- ✅ `Cargo.toml` - Already has embassy dependencies
- ✅ `memory.x` - Linker script (no change)

---

## Impact on Other FEAGI Components

### ✅ feagi-desktop
**Impact**: **NONE**
- Only calls `cargo build` to compile firmware
- Flashes the resulting `.hex` file
- Doesn't care about internal code structure
- No changes needed

### ✅ feagi-core
**Impact**: **NONE**
- No references to micro:bit firmware
- Completely independent
- No changes needed

### ✅ Python Agent (Future)
**Impact**: **NONE**
- Connects via BLE (external protocol)
- Doesn't care about firmware internals
- No changes needed

### ✅ Build System
**Impact**: **NONE**
- `build-firmware.sh` just runs `cargo build`
- Works the same way
- No changes needed

---

## Refactor Steps

### Step 1: Update Dependencies
```toml
# Already done in Cargo.toml
embassy-nrf = { version = "0.1", features = ["nrf52833", "ble"] }
embassy-executor = { version = "0.1", features = ["arch-cortex-m"] }
```

### Step 2: Refactor main.rs Entry Point
- Change `#[entry]` to `#[embassy_executor::main]`
- Make `main()` async
- Initialize embassy peripherals

### Step 3: Implement BLE in ble_stack.rs
- Use embassy-nrf BLE API
- Create async BLE task
- Handle BLE events asynchronously

### Step 4: Spawn BLE Task
- Spawn BLE task from main
- BLE runs independently
- Main loop can remain blocking or become async

### Step 5: Communication Between Tasks
- Use embassy-sync channels or shared state
- BLE task receives data → updates shared buffer
- Main loop reads from buffer (blocking is fine)

---

## Communication Pattern

```
┌─────────────────┐
│  BLE Task       │  (Async)
│  (embassy-nrf)  │
└────────┬────────┘
         │
         │ Writes to shared buffer
         ▼
┌─────────────────┐
│  Shared Buffer  │  (heapless::Vec or embassy-sync channel)
│  (Thread-safe)  │
└────────┬────────┘
         │
         │ Main loop reads (blocking OK)
         ▼
┌─────────────────┐
│  Main Loop      │  (Can be blocking)
│  (Control)      │
└─────────────────┘
```

---

## Summary

### What Changes:
- ✅ `src/main.rs` - Entry point (async)
- ✅ `src/ble_stack.rs` - BLE implementation (async)

### What Doesn't Change:
- ✅ All other firmware modules
- ✅ feagi-desktop (build/flash process)
- ✅ feagi-core
- ✅ Python agent
- ✅ Build scripts

### Impact:
- **Scope**: Micro:bit firmware only
- **Risk**: Low (isolated changes)
- **Testing**: Can test BLE independently
- **Rollback**: Easy (just revert firmware changes)

---

## Recommendation

**Use Hybrid Approach:**
1. BLE task runs async (required for embassy-nrf)
2. Main control loop can remain blocking (simpler)
3. Use shared buffer/channel for communication
4. Minimal changes to existing code

This gives you:
- ✅ Working BLE (async)
- ✅ Simple main loop (blocking)
- ✅ Minimal refactoring
- ✅ Easy to understand



