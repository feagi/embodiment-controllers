# ESP32 LED Controller

Drive an LED on an ESP32 from a FEAGI brain. The brain runs in FEAGI on your computer. The ESP32 runs a
small Arduino sketch that sets LED brightness, and `controller.py` connects the two over USB serial.

```
FEAGI genome (opse output area) ──▶ controller.py ──USB serial──▶ ESP32 ──▶ LED
```

It is meant as a minimal, working starting point for ESP32 embodiments. The same pattern (FEAGI output area →
`controller.py` → a one-line serial protocol → firmware) extends to servos, motors, or sensors.

## Hardware/Software Requirements

- An ESP32 board (ESP32-WROOM-32 / DevKit V1 or similar) and a USB cable
- Optional: an LED and a 220–330 Ω resistor. Most DevKits have an on-board LED on GPIO2, which works with no wiring.
- [Arduino IDE](https://www.arduino.cc/en/software) or `arduino-cli`, with the **esp32 by Espressif** board package (2.x or 3.x)
- FEAGI, either the one bundled with Neurorobotics Studio or a standalone one (`pip install feagi`), plus Python 3.10+.
  The `feagi` Python SDK must match the FEAGI core you connect to (see *FEAGI versions* below).
- Windows, macOS or Linux. You may need the CP210x or CH340 USB driver for your board.

### Wiring

| ESP32 pin | Connects to |
|---|---|
| GPIO2 | 220–330 Ω resistor → LED anode (+, long leg) |
| GND | LED cathode (−, short leg) |

To use a different pin, change `LED_PIN` in `firmware/esp32_led/esp32_led.ino`.
- **Safe pins:** 4, 13, 16–19, 21–23, 25–27, 32, 33.
- **Avoid** 0, 12 and 15 (boot-strapping pins: these can stop the board from booting), 6–11 (flash), 1 and 3 (USB serial),
  and 34–39 (input only).
- A GPIO pin gives about 3.3 V at up to roughly 20 mA. For anything bigger than a single LED, use a transistor or MOSFET.

## Setup Instructions

### 1. Flash the firmware

Open `firmware/esp32_led/esp32_led.ino` in the Arduino IDE, select **ESP32 Dev Module** and your port, and upload.
If the upload stalls at `Connecting...`, hold the board's **BOOT** button until it starts. Or, with `arduino-cli`:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 -u -p <PORT> firmware/esp32_led
```

On boot the LED blinks three times and the board prints `READY esp32_led` at 115200 baud.

### 2. Local Environment

```bash
cd embodiment-controllers/embodiments/esp32
python3 -m venv venv
source venv/bin/activate          # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### Connection Methods

Start FEAGI and the Brain Visualizer (`feagi start`, then `feagi bv start`), then:

```bash
python controller.py --serial-port <PORT>              # e.g. COM5 or /dev/ttyUSB0
python controller.py                                   # dry run: FEAGI only, prints LED levels
python controller.py --feagi-host 192.168.1.100 --serial-port <PORT>   # remote FEAGI
```

Default settings live in `networking.json` (FEAGI connection) and `capabilities.json` (LED and serial). Any field can be
overridden with the matching `--flag`; see `python controller.py --help`.

## Capabilities

**Actuator:** one LED, exposed to FEAGI as a one-device PositionalServo (`opse`) output area: a 1×1×10 column of neurons.
If the loaded genome doesn't have this area, the controller creates it on startup.

- Any neuron in the column fires → LED on. Which Z level fired sets the brightness: FEAGI's decoder maps
  **z=0 → 100 %**, and each step up in z is dimmer. The exact scale depends on the FEAGI version: z=9 is about 10 % on
  FEAGI 2.1.43 and about 53 % on Neurorobotics Studio's bundled FEAGI.
- Nothing fires for `off_after_s` (default 0.15 s) → LED off. `--on-off` ignores brightness, so any firing gives 100 %.
- The controller re-registers by itself when FEAGI loads a new genome or restarts.

**Serial protocol** (115200 baud, one line per change), handy for testing in a serial monitor:

| Send | Effect |
|---|---|
| `0`–`100` | Set brightness in percent |
| `t` | Blink three times (self-test) |
| `?` | Print the current brightness |

### Example genome: ESP32 LED Blink (Brain Hub)

A self-blinking LED you can fully control from the Brain Visualizer. It runs at 20 Hz bursts.

**Get it from Brain Hub:** [ESP32 LED Blink](https://brainsforrobots.com/hub/g-1795193117.1198). In Neurorobotics
Studio, open **Brain Hub**, find *ESP32 LED Blink*, and click **Replace** to load it as the running brain. Or download
it and load it with `feagi start` / the Brain Visualizer. An offline copy is in `genome/esp32_led_blink.genome`.

```
LED Stop ──(inhibits)──▶ LED Enable ⟲ (self-latch) ──▶ Blink Clock ──(LED brightness)──▶ opse ──▶ LED
```

| To… | In the Brain Visualizer |
|---|---|
| Start blinking | Stimulate **LED Enable** once. It latches on. (It's off after the genome loads.) |
| Stop blinking | Stimulate **LED Stop** once |
| Change on time | **Blink Clock** → *Consecutive fire count* (bursts on; default 10 = 0.5 s) |
| Change off time | **Blink Clock** → *Snooze period* (bursts off; default 10 = 0.5 s) |
| Change brightness | Edit the **LED brightness** morphology vector `[0,0,z]` (z=0 = 100 %) |
| Fire the LED by hand | Stimulate the opse neuron at z=0 |

## Troubleshooting

- **No serial port shows up:** install the CP210x or CH340 driver, and use a USB cable that carries data.
- **Upload fails / port busy:** close any serial monitor, and stop `controller.py`. Only one program can hold the port.
- **Board keeps rebooting after wiring:** something is connected to a boot-strapping pin (0, 12 or 15). Move it.
- **LED never lights from FEAGI:** make sure something actually fires in the opse area. Running without
  `--serial-port` prints every LED level the controller would send.
- **`Device registration incomplete ... Positional Servo [output] (b3BzZQEBAQA=)`:** your `feagi` SDK is older than
  the FEAGI core it is talking to (for example, SDK 2.1.43 against Neurorobotics Studio's FEAGI). Run
  `pip install -U feagi` in the controller's venv.
- **Blink off time resets to 0 after you re-save the genome (standalone FEAGI 2.1.43 only):** the exporter writes
  `…-nx-snooze-i`, but the loader only reads `…-nx-snooze-f` (an integer). Add a matching
  `"_____10c-<cortical id>-nx-snooze-f": <n>` line. The shipped genome includes both.

## FEAGI versions

Tested with two setups:

| FEAGI core | `feagi` SDK | Notes |
|---|---|---|
| Neurorobotics Studio's bundled FEAGI (Nov 2026) | 2.16.18 (`pip install -U feagi`) | opse has 3 subunits; the controller and FEAGI create the extra ones automatically |
| Standalone `feagi start` 2.1.43 | 2.1.43 | opse has 2 subunits; see the snooze note under Troubleshooting |

The example genome loads and runs unchanged on both.

## Standalone firmware

`firmware/standalone/` is a separate, experimental Rust firmware that runs a FEAGI connectome on the ESP32 itself.
It is unrelated to this controller; see its own README.

## License

Apache 2.0. Dependencies: `feagi` (Apache-2.0), `pyserial` (BSD-3-Clause). The Arduino-ESP32 core (LGPL-2.1) is used
only as a library when compiling the sketch; none of its code is included here.
