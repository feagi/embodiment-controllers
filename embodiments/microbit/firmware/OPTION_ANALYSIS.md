# BLE Implementation Option Analysis

## Option Comparison

### Option A: Coordinate Peripherals (Manual)
**Approach:** Keep `microbit-v2`, manually coordinate peripherals with nrf-sdc

**Pros:**
- ✅ Keep existing `microbit-v2` code (display, sensors, GPIO)
- ✅ Full control over peripheral allocation
- ✅ TrouBLE (pure Rust, MIT license)

**Cons:**
- ❌ Complex peripheral coordination
- ❌ TIMER0 conflict requires using TIMER1 for microbit-v2
- ❌ Need to carefully manage RTC0, PPI channels, TEMP
- ❌ Higher risk of runtime conflicts
- ❌ More code to maintain

**Effort:** High (2-3 days)
**Risk:** Medium-High (peripheral conflicts)

---

### Option B: embassy-nrf BLE Directly
**Approach:** Skip TrouBLE/nrf-sdc, use embassy-nrf's built-in BLE

**Pros:**
- ✅ Simpler (no nrf-sdc dependency)
- ✅ Already using embassy-nrf 0.8
- ✅ No peripheral conflicts

**Cons:**
- ❌ **embassy-nrf 0.8 does NOT have built-in BLE support**
- ❌ embassy-nrf is a HAL, not a BLE stack
- ❌ Would need to implement BLE from scratch or use different stack
- ❌ Not a viable option

**Effort:** N/A (not possible)
**Risk:** N/A

---

### Option C: microbit-bsp with trouble feature ⭐ **RECOMMENDED**
**Approach:** Replace `microbit-v2` with `microbit-bsp`, use its built-in TrouBLE support

**Pros:**
- ✅ **Designed specifically for micro:bit v2**
- ✅ **Handles peripheral coordination automatically**
- ✅ **Built-in BLE support via trouble-host**
- ✅ Integrates seamlessly with embassy-nrf
- ✅ Community-maintained and actively supported
- ✅ Includes LED display, motion sensors, BLE drivers
- ✅ Avoids all peripheral conflicts
- ✅ Cleaner architecture

**Cons:**
- ⚠️ Requires refactoring from `microbit-v2` to `microbit-bsp`
- ⚠️ Need to adapt display/sensor code to microbit-bsp API
- ⚠️ Slightly different API (but similar concepts)

**Effort:** Medium (1-2 days for refactoring)
**Risk:** Low (designed for this exact use case)

---

## Recommendation: **Option C (microbit-bsp)**

### Why Option C is Best:

1. **Purpose-Built:** `microbit-bsp` is specifically designed for micro:bit v2 with BLE support
2. **Conflict-Free:** Handles all peripheral coordination automatically
3. **TrouBLE Integration:** Built-in support for trouble-host (pure Rust, MIT license)
4. **Maintained:** Actively maintained by Rust embedded community
5. **Future-Proof:** Better long-term support and updates

### Migration Path:

1. Replace `microbit-v2 = "0.15"` with `microbit-bsp = { version = "0.4", features = ["trouble"] }`
2. Update imports: `use microbit_bsp::*` instead of `use microbit::*`
3. Adapt display code to microbit-bsp API (similar but may need adjustments)
4. Use microbit-bsp's BLE support (already integrated with trouble-host)
5. Keep existing protocol layer (`bluetooth.rs`) - no changes needed

### Code Changes Required:

**Before (microbit-v2):**
```rust
use microbit::board::Board;
let board = Board::take().expect("Failed to take Board");
let timer0 = board.TIMER0;
let display_pins = board.display_pins;
```

**After (microbit-bsp):**
```rust
use microbit_bsp::Board;
let board = Board::default();
// Peripherals are handled automatically
// BLE support is built-in
```

### Estimated Refactoring:

- **Display code:** ~30 minutes (API is similar)
- **Sensor code:** ~30 minutes (may need minor adjustments)
- **GPIO code:** ~30 minutes (similar API)
- **BLE integration:** ~2 hours (use microbit-bsp's built-in support)
- **Testing:** ~1 hour

**Total:** ~4-5 hours (1 day)

---

## Decision Matrix

| Criteria | Option A | Option B | Option C |
|----------|----------|----------|----------|
| **Complexity** | High | N/A | Medium |
| **Risk** | Medium-High | N/A | Low |
| **Maintenance** | High | N/A | Low |
| **Time to Implement** | 2-3 days | N/A | 1 day |
| **Future Support** | Medium | N/A | High |
| **Peripheral Conflicts** | Yes | N/A | No |
| **TrouBLE Support** | Yes | N/A | Yes (built-in) |

---

## Conclusion

**Choose Option C (microbit-bsp)** because:
- It's the right tool for the job
- Eliminates all peripheral conflicts
- Provides built-in BLE support
- Better long-term maintainability
- Lower risk and faster implementation

The refactoring effort is minimal compared to the complexity of manually coordinating peripherals, and the result will be cleaner and more maintainable.


