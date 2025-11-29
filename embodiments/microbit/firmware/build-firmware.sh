#!/bin/bash
# Build FEAGI micro:bit firmware and generate .hex file

set -e

VERSION="${1:-v2}"  # Default to V2
CONFIG_FILE="${2:-}"  # Optional config.json from Desktop app

echo "🔨 Building FEAGI micro:bit controller for $VERSION..."

# Determine target (V2 only for now)
TARGET="thumbv7em-none-eabihf"
OUTPUT_NAME="feagi-microbit-v2.hex"

# If config file provided, copy it (build.rs can read it)
if [ -n "$CONFIG_FILE" ] && [ -f "$CONFIG_FILE" ]; then
    echo "📝 Using custom configuration: $CONFIG_FILE"
    # Get absolute paths to compare
    CONFIG_ABS=$(cd "$(dirname "$CONFIG_FILE")" && pwd)/$(basename "$CONFIG_FILE")
    TARGET_ABS=$(pwd)/config.json
    
    # Only copy if different files
    if [ "$CONFIG_ABS" != "$TARGET_ABS" ]; then
        cp "$CONFIG_FILE" config.json
    else
        echo "   Config already in place, skipping copy"
    fi
fi

# Build release firmware
echo "⚙️  Compiling Rust firmware (target: $TARGET)..."
cargo build --release --target "$TARGET"

# Convert to .hex format
echo "🔧 Converting to .hex format..."
BINARY_NAME="feagi-microbit-controller"
cargo objcopy --release --target "$TARGET" --bin "$BINARY_NAME" -- -O ihex "target/$TARGET/release/$OUTPUT_NAME"

# Get file size
SIZE=$(ls -lh "target/$TARGET/release/$OUTPUT_NAME" | awk '{print $5}')
echo "✅ Firmware built successfully: target/$TARGET/release/$OUTPUT_NAME ($SIZE)"

# Copy to easy location
cp "target/$TARGET/release/$OUTPUT_NAME" ./firmware.hex
echo "📦 Firmware ready: ./firmware.hex"
echo ""
echo "To flash:"
echo "  cp firmware.hex /Volumes/MICROBIT/"
echo ""

