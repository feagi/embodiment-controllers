# USB CDC Implementation Plan for micro:bit

**Goal:** Replace blocked BLE transport with USB CDC (USB Serial) for immediate FEAGI connectivity

**Estimated Effort:** 8-12 hours

---

## Overview

USB CDC (Communications Device Class) allows the micro:bit to appear as a virtual serial port on the host computer. This provides a reliable, high-speed alternative to BLE for FEAGI communication.

---

## Architecture

```
┌─────────────────┐
│   FEAGI Core    │
└────────┬────────┘
         │ WebSocket/ZMQ
         │
┌────────▼────────────┐
│  Python Agent       │
│  (controller.py)    │
└────────┬────────────┘
         │ PySerial
         │ (e.g., /dev/ttyACM0)
         │
┌────────▼────────────┐
│  micro:bit USB CDC  │
│  (Firmware)         │
└─────────────────────┘
         │
┌────────▼────────────┐
│   LED Matrix        │
│   (Display)         │
└─────────────────────┘
```

---

## Implementation Steps

### 1. Firmware: Add USB CDC Support

**File:** `Cargo.toml`
```toml
[dependencies]
# USB CDC support via Embassy
embassy-usb = { version = "0.4", features = ["defmt"] }
```

**File:** `src/usb_serial.rs` (new)
```rust
//! USB CDC Serial Transport
//! Provides USB serial communication for FEAGI protocol

use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config, UsbDevice};
use embassy_nrf::usb::Driver;
use static_cell::StaticCell;

pub struct UsbSerial {
    // USB CDC class for serial communication
}

impl UsbSerial {
    pub fn new(driver: Driver<'static>) -> Self {
        // Initialize USB CDC
    }
    
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        // Read from USB serial
    }
    
    pub async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        // Write to USB serial
    }
}
```

**File:** `src/main.rs` (modified)
```rust
// Remove BLE initialization
// Add USB initialization
let usb_driver = /* get from board */;
let mut usb_serial = usb_serial::UsbSerial::new(usb_driver);

// Main loop
loop {
    // Read commands from USB
    if let Ok(n) = usb_serial.read(&mut rx_buf).await {
        bluetooth.process_received_data(&rx_buf[..n]);
    }
    
    // Send sensor data via USB
    if let Some(data) = bluetooth.get_sensor_data() {
        let _ = usb_serial.write(&data).await;
    }
    
    // Update display...
}
```

---

### 2. Python Agent: Switch to PySerial

**File:** `controller/requirements.txt`
```
feagi[serial]>=2.0.0
# OR if feagi doesn't include serial extras:
feagi>=2.0.0
pyserial>=3.5
```

**File:** `controller/controller.py` (modified)
```python
from feagi.agent import SerialRobot  # Instead of BluetoothRobot
import serial

class MicrobitRobot(SerialRobot):
    SERIAL_CONFIG = {
        "port": "/dev/ttyACM0",  # Linux
        # "port": "COM3",  # Windows
        "baudrate": 115200,
        "timeout": 1.0,
    }
    
    # Same format_motors() and parse_sensors() as before
```

---

### 3. Update FEAGI Desktop Flasher

The flasher already supports USB flashing, no changes needed. Just update the description to mention "USB cable required for communication".

---

### 4. Testing Steps

1. **Flash firmware via USB**
   ```bash
   cd firmware
   ./build-firmware.sh
   cp firmware.hex /Volumes/MICROBIT/
   ```

2. **Identify USB serial port**
   ```bash
   # Linux
   ls /dev/ttyACM*
   
   # macOS
   ls /dev/tty.usbmodem*
   
   # Windows
   # Check Device Manager → Ports (COM & LPT)
   ```

3. **Run Python agent**
   ```bash
   cd nrs-embodiments/embodiments/bbc_microbit/controller
   python controller.py --transport usb --port /dev/ttyACM0
   ```

4. **Test LED matrix**
   - FEAGI sends neuron firing data
   - Python agent forwards via USB serial
   - micro:bit displays on LED matrix

---

## Advantages of USB CDC

✅ **No pairing/discovery** - Just plug in and go
✅ **Higher bandwidth** - USB 2.0 Full Speed (12 Mbps) vs BLE 5 (2 Mbps)
✅ **Lower latency** - No BLE connection intervals
✅ **More reliable** - No wireless interference
✅ **Simpler debugging** - Easy to monitor with serial terminal
✅ **Works with Embassy** - No executor conflicts

---

## Disadvantages

❌ **Requires cable** - Not wireless
❌ **Single connection** - One micro:bit per USB port
❌ **Mobility limited** - Tethered to computer

---

## Future: Hybrid USB + BLE

Once BLE blocker is resolved (if ever), we can support both:

```python
class MicrobitRobot:
    def __init__(self, transport="usb"):
        if transport == "usb":
            self._init_usb()
        elif transport == "bluetooth":
            self._init_ble()
```

This provides:
- **USB** for development/debugging
- **BLE** for demos/production (when fixed)

---

## Timeline

| Task | Estimated Time |
|------|----------------|
| Add embassy-usb dependency | 30 min |
| Implement usb_serial.rs | 3-4 hours |
| Update main.rs for USB | 1-2 hours |
| Update Python agent | 2-3 hours |
| Testing and debugging | 2-3 hours |
| Documentation updates | 1 hour |
| **Total** | **8-12 hours** |

---

## Decision Point

Should we proceed with USB CDC implementation to unblock FEAGI integration?

**Vote:**
- 👍 Yes, implement USB CDC now
- 👎 No, keep trying to fix BLE
- 🤔 Need more discussion

