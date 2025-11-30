#!/bin/bash
# Simple test - build minimal firmware

cd "$(dirname "$0")"

echo "🔨 Building minimal test firmware..."

# Create ultra-minimal test
cat > src/main_test.rs << 'EOF'
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;

#[cfg(feature = "v2")]
use microbit::{board::Board, display::blocking::Display, hal::Timer};

#[entry]
fn main() -> ! {
    if let Some(board) = Board::take() {
        let mut timer = Timer::new(board.TIMER0);
        let mut display = Display::new(board.display_pins);
        
        let heart = [
            [0, 1, 0, 1, 0],
            [1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1],
            [0, 1, 1, 1, 0],
            [0, 0, 1, 0, 0],
        ];
        
        loop {
            display.show(&mut timer, heart, 1000);
        }
    }
    
    loop {}
}
EOF

# Temporarily rename main.rs
mv src/main.rs src/main_backup.rs
mv src/main_test.rs src/main.rs

# Build
echo "⚙️  Compiling..."
cargo build --release --features v2 --target thumbv7em-none-eabihf

# Generate hex
echo "🔧 Converting to .hex..."
cargo objcopy --release --target thumbv7em-none-eabihf -- -O ihex test_firmware.hex

# Restore original main.rs
mv src/main.rs src/main_test_built.rs
mv src/main_backup.rs src/main.rs

echo "✅ Test firmware: test_firmware.hex"
echo ""
echo "To flash:"
echo "  cp test_firmware.hex /Volumes/MICROBIT/"



