# BLE Stack Options Comparison

## Overview

This document compares the three main options for implementing BLE on the micro:bit V2, including license compatibility with FEAGI's MIT license.

---

## Option 1: nrf-softdevice

### Description
Official Nordic Semiconductor SoftDevice - a pre-compiled binary BLE stack provided by Nordic.

### License
- **SoftDevice Binary**: Proprietary license (Nordic's license)
- **nrf-softdevice Rust crate**: MIT/Apache-2.0 (permissive)
- **Issue**: The SoftDevice blob itself is proprietary, which may conflict with FEAGI's MIT-only policy

### Pros
✅ **Official & Stable**
- Official Nordic solution
- Production-tested and stable
- Full BLE 5.0 support
- Well-documented by Nordic

✅ **Feature Complete**
- All BLE features supported
- GATT server/client
- Advertising, scanning, connections
- Security features (pairing, bonding)

✅ **Performance**
- Optimized binary
- Low memory footprint
- Efficient power consumption

✅ **Rust Integration**
- `nrf-softdevice` crate provides good Rust bindings
- Active community support
- Good examples available

### Cons
❌ **License Conflict**
- **CRITICAL**: SoftDevice binary is proprietary
- Not compatible with MIT-only license policy
- Requires accepting Nordic's license terms
- May not be suitable for open-source projects requiring permissive-only licenses

❌ **Binary Blob Required**
- Must download and link proprietary binary
- Binary is large (~150KB)
- Version-specific (must match chip)

❌ **Complexity**
- Requires careful memory layout configuration
- SoftDevice reserves RAM/Flash
- More complex build process

❌ **Size**
- SoftDevice takes significant flash space
- Less room for application code

### Implementation Requirements
1. Download SoftDevice S140 for nRF52833 from Nordic
2. Add to project and link in build
3. Configure memory layout (already done in `memory.x`)
4. Use `nrf-softdevice` crate API
5. Handle async BLE events

### Code Example
```rust
// Cargo.toml
nrf-softdevice = "0.4"
nrf-softdevice-s140 = "0.4"

// Implementation
let config = nrf_softdevice::Config::default();
let sd = nrf_softdevice::Softdevice::enable(&config)?;
```

### Recommendation
⚠️ **Not Recommended** - License incompatibility with MIT-only policy

---

## Option 2: embassy-nrf BLE

### Description
Pure Rust BLE implementation using embassy async framework.

### License
- **embassy-nrf**: MIT/Apache-2.0 (permissive ✅)
- **embassy-executor**: MIT/Apache-2.0 (permissive ✅)
- **All dependencies**: Permissive licenses
- **Compatible**: Fully compatible with MIT license

### Pros
✅ **Pure Rust**
- No binary blobs required
- Full source code available
- Can audit and modify

✅ **Permissive License**
- MIT/Apache-2.0
- Compatible with FEAGI's MIT license
- No proprietary components

✅ **Modern Async**
- Uses async/await
- Clean, modern Rust code
- Good error handling

✅ **No Binary Blobs**
- Everything is Rust source
- Easier to build and deploy
- No external downloads needed

✅ **Active Development**
- Modern, actively maintained
- Good community support
- Regular updates

### Cons
❌ **Version Compatibility**
- BLE support may not be fully stable in all versions
- Need to verify embassy-nrf version with BLE support
- May require specific feature flags

❌ **Async Refactor Required**
- Current code is blocking
- Must refactor to async runtime
- More complex

❌ **Learning Curve**
- Async Rust can be complex
- Different programming model
- Requires understanding of embassy executor

❌ **Potential Stability**
- BLE support may be newer/less tested
- May have edge cases
- Community support may be limited

### Implementation Requirements
1. Add embassy-nrf with BLE features
2. Refactor main() to async
3. Use embassy executor
4. Implement BLE using embassy API
5. Handle async BLE events

### Code Example
```rust
// Cargo.toml
embassy-nrf = { version = "0.1", features = ["nrf52833", "ble"] }
embassy-executor = { version = "0.1", features = ["arch-cortex-m"] }

// Implementation
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let config = embassy_nrf::ble::Config::default();
    let (ble, _) = embassy_nrf::ble::init(p.RADIO, config).await;
    // ... BLE implementation
}
```

### Recommendation
✅ **Recommended** - If BLE support is stable in available version

---

## Option 3: micro:bit Built-in BLE UART

### Description
Use micro:bit's built-in BLE UART service (Nordic UART Service - NUS).

### License
- **microbit-v2 crate**: Check license (likely MIT/Apache-2.0)
- **Nordic UART Service**: Standard BLE service (no license issues)
- **Compatible**: Should be compatible with MIT license

### Pros
✅ **Simplest Implementation**
- Built into micro:bit-v2 crate
- Minimal code required
- Easy to use

✅ **Well-Tested**
- Standard Nordic UART Service
- Used by many micro:bit projects
- Stable and reliable

✅ **No Additional Dependencies**
- Uses existing microbit-v2 crate
- No new crates needed
- Smaller binary size

✅ **Quick to Implement**
- Less code to write
- Faster development
- Good for prototyping

### Cons
❌ **Limited Control**
- Can't use custom UUIDs directly
- Must use standard NUS UUIDs
- Less flexible than custom service

❌ **Protocol Overhead**
- Must send FEAGI protocol over UART
- Less efficient than direct BLE characteristics
- Additional parsing layer

❌ **Feature Limitations**
- Limited to UART-like interface
- No direct characteristic access
- May not support all BLE features

❌ **May Still Need BLE Stack**
- microbit-v2 may use SoftDevice internally
- Need to verify license compatibility
- May have same license issues as Option 1

### Implementation Requirements
1. Check if microbit-v2 has BLE UART support
2. Use micro:bit's BLE UART API
3. Send FEAGI packets over UART interface
4. Parse packets on receive

### Code Example
```rust
// If microbit-v2 has BLE UART:
use microbit::ble::BleUart;

let ble_uart = BleUart::new()?;
ble_uart.write(&packet)?;
```

### Recommendation
✅ **Recommended for Quick Start** - If license is compatible and meets needs

---

## License Compatibility Summary

| Option | License Status | Compatible with MIT? |
|--------|---------------|---------------------|
| nrf-softdevice | SoftDevice blob is proprietary | ❌ No |
| embassy-nrf | MIT/Apache-2.0 | ✅ Yes |
| micro:bit BLE UART | Depends on microbit-v2 crate | ⚠️ Check |

---

## Recommendation Matrix

### For FEAGI Project (MIT License Required)

**Best Choice: embassy-nrf BLE**
- ✅ Permissive license (MIT/Apache-2.0)
- ✅ Pure Rust, no blobs
- ✅ Full control
- ⚠️ Requires async refactor

**Alternative: micro:bit BLE UART**
- ✅ Simple implementation
- ⚠️ Verify microbit-v2 crate license
- ⚠️ Less flexible

**Avoid: nrf-softdevice**
- ❌ Proprietary SoftDevice blob
- ❌ License incompatibility

---

## Implementation Decision Tree

```
Start
  │
  ├─ Need permissive license only?
  │   ├─ Yes → embassy-nrf BLE ✅
  │   └─ No → Continue...
  │
  ├─ Need quick implementation?
  │   ├─ Yes → Check micro:bit BLE UART
  │   └─ No → Continue...
  │
  ├─ Need maximum stability?
  │   ├─ Yes → nrf-softdevice (if license OK)
  │   └─ No → embassy-nrf BLE
  │
  └─ Need simplest solution?
      ├─ Yes → micro:bit BLE UART
      └─ No → embassy-nrf BLE
```

---

## Next Steps

1. **Verify embassy-nrf BLE Support**
   - Check if BLE features are available in compatible version
   - Test with simple example
   - Verify stability

2. **If embassy-nrf BLE works:**
   - Refactor to async runtime
   - Implement BLE in `ble_stack.rs`
   - Test with nRF Connect

3. **If embassy-nrf BLE not available:**
   - Check micro:bit BLE UART license
   - If compatible, use that
   - Otherwise, consider alternative approaches

---

## License Verification Checklist

Before choosing an option, verify:

- [ ] All dependencies use permissive licenses (MIT, Apache-2.0, BSD)
- [ ] No proprietary binary blobs required
- [ ] Source code is available
- [ ] License is compatible with FEAGI's MIT license
- [ ] No copyleft licenses (GPL, LGPL) in dependency chain

---

## Conclusion

### Final Recommendation for FEAGI

**Primary Choice: embassy-nrf BLE**
- ✅ **License**: MIT/Apache-2.0 (fully permissive)
- ✅ **No Blobs**: Pure Rust implementation
- ✅ **Full Control**: Custom UUIDs and services
- ⚠️ **Requires**: Async refactor and version verification

**Fallback: micro:bit BLE UART**
- ⚠️ **License**: Need to verify microbit-v2 crate (likely MIT/Apache-2.0)
- ✅ **Simple**: Easiest to implement
- ⚠️ **Limitation**: Standard UART service only

**Not Recommended: nrf-softdevice**
- ❌ **License**: Proprietary SoftDevice blob
- ❌ **Incompatible**: With MIT-only policy

### License Compatibility Summary

| Component | License | MIT Compatible? |
|-----------|---------|----------------|
| embassy-nrf | MIT/Apache-2.0 | ✅ Yes |
| embassy-executor | MIT/Apache-2.0 | ✅ Yes |
| microbit-v2 crate | Likely MIT/Apache-2.0 | ⚠️ Verify |
| nrf-softdevice crate | MIT/Apache-2.0 | ✅ Yes |
| SoftDevice blob | Proprietary | ❌ No |

### Action Items

1. **Verify embassy-nrf BLE availability**
   - Check latest version with BLE support
   - Test simple BLE example
   - Verify stability

2. **If embassy-nrf works:**
   - Implement in `ble_stack.rs`
   - Refactor to async
   - Test with nRF Connect

3. **If embassy-nrf unavailable:**
   - Verify microbit-v2 crate license
   - Check if it has BLE UART support
   - Use if license compatible

The current implementation structure supports all three options, so you can switch between them by implementing the BLE stack code in `ble_stack.rs`.

