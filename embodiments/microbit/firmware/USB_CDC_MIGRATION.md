# micro:bit USB CDC Migration Guide

**Status**: ✅ Platform layer complete, 🔵 Firmware integration pending  
**Date**: Current session

---

## Overview

We're migrating the micro:bit from BLE (blocked by TrouBLE/Embassy issues) to USB CDC Serial for immediate functionality.

---

## What's Complete ✅

### 1. Shared Transport Infrastructure
- ✅ Transport-agnostic protocol layer (`feagi-embedded/src/transports/protocol.rs`)
- ✅ USB CDC HAL trait (`feagi-embedded/src/hal/usb_cdc.rs`)
- ✅ nRF52 USB CDC implementation (`feagi-embedded/src/platforms/nrf52_usb.rs`)

### 2. Architecture
```
micro:bit Hardware
      ↓
  nRF52833 USB Peripheral
      ↓
embassy-nrf USB Driver
      ↓
Nrf52UsbCdc (feagi-embedded)
      ↓
Protocol (transport-agnostic)
      ↓
  Application Logic
```

---

## What Needs To Be Done 🔵

### Step 1: Update Firmware Dependencies

**File**: `Cargo.toml`

```toml
[dependencies]
# Remove BLE dependencies
# trouble-host = { version = "0.1", features = ["peripheral", "gatt"] }
# bt-hci = { version = "0.2", features = ["uuid", "embassy-time"] }
# nrf-sdc = "0.1"

# Add USB CDC dependencies
feagi-embedded = { path = "../../../../../feagi-core/crates/feagi-embedded", features = ["bluetooth-nrf52"] }
embassy-usb = { version = "0.4", features = ["defmt"] }
```

### Step 2: Simplify main.rs

**File**: `src/main.rs`

```rust
#![no_std]
#![no_main]

use panic_halt as _;
use microbit_bsp::Microbit;
use feagi_embedded::transports::Protocol;
use feagi_embedded::platforms::Nrf52UsbCdc;
use feagi_embedded::hal::UsbCdcProvider;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let board = Microbit::default();
    let mut display = board.display;
    
    // Show startup sequence
    // ... (keep FEAGI letters) ...
    
    // Initialize USB CDC
    let mut usb = Nrf52UsbCdc::new(
        board.usb,
        "FEAGI-microbit",
        "Neuraville"
    );
    
    // Initialize protocol
    let mut protocol = Protocol::new("FEAGI-microbit");
    
    // Wait for USB connection
    usb.wait_for_connection().await;
    
    // Main loop
    let mut display_buffer = [[0u8; 5]; 5];
    
    loop {
        // Read from USB
        let mut buf = [0u8; 64];
        if let Ok(len) = usb.read_async(&mut buf).await {
            if len > 0 {
                // Process protocol
                protocol.process_received_data(&buf[..len]);
            }
        }
        
        // Handle commands
        if let Some(cmd) = protocol.receive_command() {
            match cmd {
                Command::NeuronFiring { coordinates } => {
                    display_buffer = [[0; 5]; 5];
                    for &(x, y) in coordinates.iter() {
                        if x < 5 && y < 5 {
                            display_buffer[y as usize][x as usize] = 255;
                        }
                    }
                }
                _ => {}
            }
        }
        
        // Update display
        let mut frame = Frame::<5, 5>::empty();
        for y in 0..5 {
            for x in 0..5 {
                if display_buffer[y][x] > 128 {
                    frame.set(x, y);
                }
            }
        }
        display.display(frame, Duration::from_millis(100)).await;
    }
}
```

### Step 3: Remove BLE-Specific Files

**Files to delete/archive**:
- `src/ble_compat.rs` (no longer needed)
- `src/ble_stack.rs` (no longer needed)
- Keep `src/bluetooth.rs` → rename to `src/protocol_handler.rs` (generic)

### Step 4: Update Python Agent

**File**: `nrs-embodiments/embodiments/bbc_microbit/controller/controller.py`

```python
#!/usr/bin/env python
"""
FEAGI micro:bit USB CDC Agent

Connects to micro:bit via USB Serial and bridges to FEAGI Core.
"""

import serial
import asyncio
from feagi.agent import BaseRobot

class MicrobitRobot(BaseRobot):
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        super().__init__()
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
    
    def format_motors(self, feagi_output: dict) -> bytes:
        """Convert FEAGI neuron data to micro:bit format"""
        neuron_coords = []
        if "LED Matrix" in feagi_output:
            data = feagi_output["LED Matrix"]
            if "x" in data and "y" in data:
                for i in range(len(data["x"])):
                    x, y = data["x"][i], data["y"][i]
                    if 0 <= x < 5 and 0 <= y < 5:
                        neuron_coords.append((x, y))
        
        # Format: [0x01] [count] [x1, y1, x2, y2, ...]
        packet = bytearray([0x01, len(neuron_coords)])
        for x, y in neuron_coords:
            packet.extend([x, y])
        return packet
    
    def send_to_device(self, data: bytes):
        """Send data via USB Serial"""
        self.ser.write(data)
    
    def receive_from_device(self) -> bytes:
        """Receive data via USB Serial"""
        return self.ser.read(64) or b""
```

---

## Migration Steps (In Order)

1. **Update `Cargo.toml`** - Add feagi-embedded, remove BLE deps
2. **Simplify `main.rs`** - Use USB CDC + Protocol
3. **Remove BLE files** - Clean up ble_compat.rs, ble_stack.rs
4. **Test compilation** - `cargo build --release`
5. **Flash firmware** - `./build-firmware.sh`
6. **Update Python agent** - Use PySerial instead of Bleak
7. **Test end-to-end** - Send neurons from FEAGI, see LEDs light up

---

## Testing

### Hardware Required
- micro:bit v2
- USB cable (micro-USB)
- Computer with Python 3.8+

### Test Procedure

1. **Flash firmware**:
   ```bash
   cd firmware
   ./build-firmware.sh
   cp firmware.hex /Volumes/MICROBIT/
   ```

2. **Identify USB port**:
   ```bash
   # Linux
   ls /dev/ttyACM*
   
   # macOS
   ls /dev/tty.usbmodem*
   
   # Windows
   # Check Device Manager
   ```

3. **Run Python agent**:
   ```bash
   cd ../controller
   python controller.py --port /dev/ttyACM0
   ```

4. **Send test data from FEAGI**:
   - Map cortical area to "LED Matrix" (5×5×1, omis type)
   - Trigger neurons
   - Verify LEDs light up on micro:bit

---

## Benefits of USB CDC

✅ **No BLE executor issues** - Embassy USB works reliably  
✅ **Faster data transfer** - 12 Mbps vs 2 Mbps  
✅ **Lower latency** - <1ms vs 7-30ms  
✅ **Simpler debugging** - Easy to monitor with serial terminal  
✅ **No pairing required** - Plug and play  

---

## Estimated Effort

| Task | Time | Status |
|------|------|--------|
| Platform layer (nRF52 USB CDC) | 2 hrs | ✅ Complete |
| Update firmware Cargo.toml | 15 min | 🔵 Pending |
| Simplify main.rs | 1-2 hrs | 🔵 Pending |
| Remove BLE files | 15 min | 🔵 Pending |
| Update Python agent | 1 hr | 🔵 Pending |
| Testing and debugging | 1-2 hrs | 🔵 Pending |
| **Total** | **5-7 hrs** | **30% Complete** |

---

## Next Actions

**Immediate**: Update firmware to use USB CDC  
**Priority**: High (unblocks micro:bit testing)  
**Owner**: TBD

Would you like me to proceed with updating the firmware now?

