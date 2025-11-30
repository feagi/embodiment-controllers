# Quick Flash Test

The firmware has been simplified to just show a heart pattern.

## To test:

1. **Rebuild firmware** in FEAGI Desktop:
   - Open micro:bit Flasher
   - Configure (any settings)
   - Build Firmware
   - Flash Device

2. **Expected result**:
   - Heart pattern ❤️ on LED matrix
   - Stays on forever

3. **If still no LEDs**:
   - micro:bit might be in bootloader mode
   - Try pressing reset button on back
   - Or unplug/replug USB

## Manual test:

```bash
cd /Users/nadji/code/FEAGI-2.0/embodiment-controllers/embodiments/microbit/firmware
cargo clean
cargo build --release --features v2 --target thumbv7em-none-eabihf
cargo objcopy --release --target thumbv7em-none-eabihf -- -O ihex test.hex
cp test.hex /Volumes/MICROBIT/
```

Wait 5 seconds, LEDs should show heart.



