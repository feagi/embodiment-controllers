# micro:bit LED Indicators

## USB Transport Firmware

### Startup Sequence
1. **"U" shape** (1 second) - Shows USB mode is active
   - Left vertical line: LEDs at (0,0), (0,1), (0,2), (0,3)
   - Bottom curve: LEDs at (1,4), (2,4)
   - Right vertical line: LEDs at (3,0), (3,1), (3,2), (3,3)

2. **Top-left LED** (blinking or solid) - **Waiting for USB connection**
   - Firmware is waiting for host to open serial port (DTR signal)
   - This is what you see when only one LED is lit at top-left
   - **Action needed**: Open a serial terminal or connect the Python controller

3. **Checkmark (✓)** (0.5 seconds) - USB connected successfully
   - LEDs at (4,0), (3,1), (2,2), (1,1), (0,0)
   - Host has opened serial port, communication ready

4. **Main loop** - Processing commands
   - LED matrix shows neuron firing data from FEAGI
   - Updates at ~20 Hz (50ms intervals)

## BLE Transport Firmware

### Startup Sequence
1. **"FEAGI" letters** - Shows firmware is running
   - Each letter displayed for 300ms

2. **Status indicators** (during BLE initialization):
   - **Top-left (solid)**: BLE stack initializing (Stage 0)
   - **Top-right**: BLE stack created (Stage 10)
   - **Bottom-left**: Runners spawned (Stage 11)
   - **Bottom-left + Bottom-right**: About to advertise (Stage 12)
   - **Center + expanding circles**: Advertising (waiting for connection)
   - **Checkmark**: Connected to BLE client

3. **Error indicators**:
   - **"X" pattern**: BLE error occurred

## Troubleshooting

### USB: Only top-left LED lit
**Problem**: Firmware is waiting for USB connection

**Solutions**:
1. Open a serial terminal at 115200 baud:
   ```bash
   # Linux/Mac
   screen /dev/ttyACM0 115200
   
   # Or use Python controller:
   python controller.py --transport usb
   ```

2. Check USB connection:
   ```bash
   # List USB ports
   python controller.py --list-ports
   ```

3. Verify firmware is USB variant:
   - Should have seen "U" shape on startup
   - If you saw "FEAGI" letters, you flashed BLE firmware instead

### BLE: Stuck at top-left LED
**Problem**: BLE initialization is blocking (known issue)

**Solutions**:
1. Use USB transport instead (recommended)
2. Wait longer - BLE may eventually initialize
3. Re-flash firmware

### No LEDs at all
**Problem**: Firmware not running or display issue

**Solutions**:
1. Check if firmware flashed successfully
2. Press reset button on micro:bit
3. Re-flash firmware
4. Check micro:bit battery/USB power

