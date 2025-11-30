#!/bin/bash
# Build FEAGI micro:bit firmware with selectable transport

set -e

VERSION="${1:-v2}"
CONFIG_FILE="${2:-}"
TRANSPORT="${3:-ble}"  # NEW: ble or usb

echo "🔨 Building FEAGI micro:bit controller for $VERSION (transport: $TRANSPORT)..."

# Determine feature flags
if [ "$TRANSPORT" = "usb" ]; then
    FEATURES="--no-default-features --features transport-usb"
    OUTPUT_NAME="feagi-microbit-usb-v2.hex"
else
    FEATURES="--features transport-ble"
    OUTPUT_NAME="feagi-microbit-ble-v2.hex"
fi

TARGET="thumbv7em-none-eabihf"

# Handle config file
if [ -n "$CONFIG_FILE" ] && [ -f "$CONFIG_FILE" ]; then
    echo "📝 Using custom configuration: $CONFIG_FILE"
    CONFIG_ABS=$(cd "$(dirname "$CONFIG_FILE")" && pwd)/$(basename "$CONFIG_FILE")
    TARGET_ABS=$(pwd)/config.json
    
    if [ "$CONFIG_ABS" != "$TARGET_ABS" ]; then
        cp "$CONFIG_FILE" config.json
    fi
fi

# Build firmware
echo "⚙️  Compiling Rust firmware (target: $TARGET, features: $FEATURES)..."
cargo build --release $FEATURES --target "$TARGET"

# Convert to .hex
echo "🔧 Converting to .hex format..."
BINARY_NAME="feagi-microbit-controller"
rust-objcopy --release --target "$TARGET" --bin "$BINARY_NAME" -- -O ihex --set-section-flags .text=alloc,code "target/$TARGET/release/$OUTPUT_NAME" || \
cargo objcopy --release --target "$TARGET" --bin "$BINARY_NAME" -- -O ihex "target/$TARGET/release/$OUTPUT_NAME"

SIZE=$(ls -lh "target/$TARGET/release/$OUTPUT_NAME" | awk '{print $5}')
echo "✅ Firmware built successfully: target/$TARGET/release/$OUTPUT_NAME ($SIZE)"

# Copy to easy location
cp "target/$TARGET/release/$OUTPUT_NAME" ./firmware.hex
echo "📦 Firmware ready: ./firmware.hex (transport: $TRANSPORT)"
echo ""
echo "To flash:"
echo "  cp firmware.hex /Volumes/MICROBIT/"
echo ""
