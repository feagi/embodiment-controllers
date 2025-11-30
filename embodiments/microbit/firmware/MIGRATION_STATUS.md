# microbit-bsp Migration Status

## Current Issue

We're migrating from `microbit-v2` to `microbit-bsp` but encountering API differences:

1. **Version Conflict:** microbit-bsp 0.4.0 requires embassy-nrf 0.4.x (we had 0.8.x)
2. **API Differences:** Need to understand microbit-bsp's Microbit struct fields
3. **Display API:** Different display API than microbit-v2

## Next Steps

1. Check microbit-bsp GitHub repo for examples
2. Understand Microbit struct fields (display, timer, etc.)
3. Update led_display.rs to use correct API
4. Update main.rs to use correct field access
5. Test compilation

## Resources

- microbit-bsp GitHub: https://github.com/lulf/microbit-bsp
- microbit-bsp docs: https://docs.rs/microbit-bsp


