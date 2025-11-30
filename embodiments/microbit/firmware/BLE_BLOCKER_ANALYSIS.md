# BLE Implementation Blocker - TrouBLE/Embassy Incompatibility

**Status:** BLOCKED - Bluetooth functionality non-operational due to executor deadlock

**Date:** 2024-01-XX (Current session)

---

## Problem Summary

The micro:bit firmware successfully initializes the BLE stack but **cannot advertise or accept connections** due to a fundamental architectural incompatibility between TrouBLE's event processing model and Embassy's single-threaded executor.

---

## What Works

- ✅ Firmware compilation and flashing
- ✅ Basic hardware initialization (micro:bit BSP, display, timers)
- ✅ FEAGI startup sequence (LED patterns)
- ✅ Main application loop execution
- ✅ BLE stack initialization (`BleStack::new()` completes successfully)
- ✅ GATT service definition (Nordic UART Service)
- ✅ Async task spawning and coordination

---

## What Doesn't Work (BLOCKER)

- ❌ **BLE Advertising** - `peripheral.advertise()` blocks indefinitely
- ❌ **BLE Connection Handling** - Can't accept connections
- ❌ **BLE Event Processing** - Runner tasks block executor
- ❌ **Device Discovery** - micro:bit is NOT discoverable via Bluetooth scan

---

## Root Cause Analysis

### TrouBLE Architecture Requirements

TrouBLE (the pure Rust BLE host stack) splits its event processing into three runner tasks:
1. **RxRunner** - Processes incoming BLE data from controller
2. **ControlRunner** - Handles connection events and state management  
3. **TxRunner** - Processes outgoing BLE data to controller

These runners expose a `.run().await` method that is designed to run continuously in the background.

### Embassy Executor Constraints

Embassy's `embassy-executor` provides a **single-threaded cooperative async runtime** for `no_std` environments:
- Tasks must periodically `await` to yield control back to the executor
- If a task never yields (blocks), the entire executor is blocked
- No preemptive multitasking or thread-based concurrency

### The Deadlock

```
peripheral.advertise()
  └─ Internally waits for BLE controller events
      └─ Requires RxRunner/ControlRunner to process events
          └─ runner.run().await blocks forever (doesn't yield)
              └─ Executor can't switch to other tasks
                  └─ advertise() never completes
                      └─ DEADLOCK
```

**Observed Behavior:**
- With runner tasks disabled: `BleStack::new()` succeeds, but `advertise()` hangs at Stage 12
- With runner tasks enabled: Firmware stuck at Stage 0 (top-left LED), main loop never runs
- With `advertise()` bypassed: Main loop runs (Stage 1), but no actual BLE functionality

---

## Attempted Solutions (All Failed)

### 1. Add Explicit Yields in Runner Loops ❌
```rust
loop {
    runner.run().await;
    Timer::after(Duration::from_micros(100)).await; // Yield
}
```
**Result:** Runner tasks still blocked executor completely. `runner.run()` itself doesn't yield internally.

### 2. Error Handling Loops ❌
```rust
loop {
    if let Err(_) = runner.run().await {
        Timer::after(Duration::from_millis(100)).await;
    }
}
```
**Result:** `runner.run()` never returns (neither Ok nor Err), so error handler never runs.

### 3. Drop Runners Entirely ❌
```rust
drop(host.runner); // Don't spawn runner tasks
```
**Result:** `advertise()` hangs indefinitely waiting for events that will never be processed.

### 4. Bypass Advertising ⚠️ (Current Workaround)
```rust
// Skip start_advertising() call, just set BLE_STAGE = 1
```
**Result:** Main loop runs, but Bluetooth is completely non-functional.

---

## Why This Wasn't Caught Earlier

1. **Limited TrouBLE Documentation:** TrouBLE examples assume multi-threaded environments or specific executor configurations not documented.
2. **microbit-bsp Integration:** The BSP provides BLE support, but doesn't include working examples with advertising/connections.
3. **Async Complexity:** The interaction between TrouBLE's blocking runners and Embassy's cooperative scheduler isn't obvious until runtime.
4. **Testing Gap:** We couldn't test actual BLE functionality until the full integration was complete.

---

## Options for Moving Forward

### Option 1: Use nrf-softdevice Instead of TrouBLE 🔴 LICENSE ISSUE
**Pros:**
- More mature, better documented
- Known to work with Embassy executor
- Full BLE 5.x feature support

**Cons:**
- **MAJOR BLOCKER:** Depends on Nordic's proprietary SoftDevice binary blob
- **LICENSE CONFLICT:** SoftDevice has restrictive license incompatible with FEAGI's MIT/Apache-2.0 requirements
- This was explicitly rejected in earlier architecture decisions

**Verdict:** ❌ Not viable due to licensing

---

### Option 2: Use nrf-sdc Directly (Without TrouBLE) ⚠️ COMPLEX
**Pros:**
- nrf-sdc (SoftDevice Controller) is MIT-licensed
- Lower-level control over BLE stack
- No dependency on TrouBLE's problematic runner architecture

**Cons:**
- Much more complex implementation (HCI command handling, GATT from scratch)
- Requires deep BLE protocol knowledge
- Significant development time (2-3 weeks minimum)
- May still have executor compatibility issues

**Effort:** High (estimated 40-60 hours)

**Verdict:** 🟡 Possible but risky

---

### Option 3: Use Interrupts to Drive Runners 🟢 POTENTIALLY VIABLE
**Approach:**
- Move runner task execution into interrupt handlers (e.g., RADIO_IRQHandler)
- Let hardware interrupts trigger BLE event processing
- Main async tasks remain non-blocking

**Pros:**
- Keeps TrouBLE's high-level API
- Leverages nRF52's hardware interrupts
- Main loop and BLE can coexist

**Cons:**
- Requires unsafe code and careful synchronization
- Embassy's async model may not play well with interrupt-driven code
- TrouBLE may not be designed for interrupt-based execution

**Effort:** Medium (estimated 20-30 hours)

**Verdict:** 🟡 Worth investigating, but uncertain success

---

### Option 4: Switch to Different Executor 🔴 BREAKS EMBASSY
**Options:**
- Use RTIC (Real-Time Interrupt-driven Concurrency) instead of Embassy
- Use bare-metal with custom event loop

**Pros:**
- RTIC has proven BLE examples for nRF52
- More control over task scheduling

**Cons:**
- Loses embassy-time, embassy-nrf, and other Embassy benefits
- Requires rewriting entire firmware from scratch
- microbit-bsp is Embassy-based, would lose BSP too

**Effort:** Very High (4-6 weeks)

**Verdict:** ❌ Too disruptive

---

### Option 5: Accept Limited Functionality (USB/Serial Only) 🟢 IMMEDIATE
**Approach:**
- Remove BLE entirely
- Implement USB CDC (USB Serial) communication instead
- FEAGI agent connects via USB cable instead of Bluetooth

**Pros:**
- USB CDC is well-supported by Embassy and nRF52833
- No executor conflicts
- Faster data transfer than BLE
- Can implement immediately

**Cons:**
- Requires USB cable (not wireless)
- Diverges from original "BLE embodiment" vision
- May still want BLE for production deployments

**Effort:** Low (estimated 8-12 hours)

**Verdict:** 🟢 Best short-term solution for unblocking testing

---

### Option 6: Hybrid Approach (USB Now, BLE Later) 🟢 RECOMMENDED
**Phase 1:** Implement USB CDC serial communication (Option 5)
- Get FEAGI integration working immediately
- Test neuron→LED mapping
- Validate sensor data flow
- Build out Python agent

**Phase 2:** Research BLE solution in parallel
- Investigate Option 3 (interrupt-driven runners)
- Test with minimal BLE example outside main firmware
- If successful, integrate back into main firmware
- If not, document as known limitation

**Pros:**
- Unblocks FEAGI integration testing NOW
- Provides fallback communication method
- Allows time for proper BLE research
- Both USB and BLE can coexist in final firmware

**Effort:** 
- Phase 1: Low (8-12 hours)
- Phase 2: Medium (20-30 hours, if pursued)

**Verdict:** 🟢🟢 STRONGLY RECOMMENDED

---

## Immediate Next Steps (Recommendation)

1. **Document Current BLE Blocker** ✅ (This document)
2. **Pivot to USB CDC Implementation**
   - Remove BLE initialization from main.rs
   - Add USB CDC device configuration
   - Implement Nordic UART Service protocol over USB serial
   - Update Python agent to use PySerial instead of Bleak
3. **Test FEAGI Integration via USB**
   - Flash firmware via USB
   - Run Python agent with `--transport usb` flag
   - Send neuron firing data to LED matrix
   - Validate display works correctly
4. **Update Documentation**
   - Update QUICKSTART.md to reflect USB-first approach
   - Update integration_notes.md with USB setup instructions
   - Mark BLE as "experimental/not yet functional"

**Estimated Time to Working USB Implementation:** 8-12 hours

---

## Long-Term Recommendations

1. **USB CDC as Primary Transport** for micro:bit v2
   - More reliable than BLE for development/testing
   - Faster data transfer
   - No pairing/discovery issues

2. **BLE as Optional Enhancement**
   - Only pursue after USB is stable
   - Consider it a "nice to have" feature
   - May require upstream fixes to TrouBLE or microbit-bsp

3. **Alternative Embodiments for Wireless**
   - ESP32-based boards have mature BLE stacks
   - ESP32 works well with esp-hal + Embassy
   - Could offer ESP32 as "wireless micro:bit alternative"

---

## Lessons Learned

1. **Validate Core Integration Early:** Should have tested BLE advertising in isolation before full integration
2. **Executor Compatibility is Critical:** Async libraries must be tested with target executor
3. **Have Fallback Plans:** USB should have been considered from the start
4. **Pure Rust != Production Ready:** TrouBLE is pure Rust but less mature than licensed alternatives

---

## Conclusion

**The micro:bit BLE implementation is BLOCKED by a fundamental TrouBLE/Embassy incompatibility.**

**Recommended Path:** Pivot to USB CDC serial communication for immediate functionality, with BLE as a future research project.

**Decision Needed:** Should we proceed with USB CDC implementation, or pursue one of the riskier BLE options?

