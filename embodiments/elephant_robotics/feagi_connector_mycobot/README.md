# feagi_connector_mycobot

Connect an [Elephant Robotics MyCobot](https://www.elephantrobotics.com/en/mycobot-en/)
6-DOF robotic arm to FEAGI. FEAGI drives each joint as a positional servo and reads the
joint encoders back as proprioception, so a genome can both move and feel the arm.

The arm is controlled over USB serial through [`pymycobot`](https://pypi.org/project/pymycobot/) —
no ROS install is required. Works on Windows, macOS, and Linux.

# Requirements

- An Elephant Robotics MyCobot (e.g. MyCobot 280) connected over USB.
- Python 3.6+.
- `feagi-connector` and `pymycobot` (installed automatically on first run, or via
  `pip install feagi_connector_mycobot`).

# Install

```
pip3 install feagi_connector_mycobot
```

# Run

Plug in the arm, then start the controller. FEAGI auto-detects the serial port; if you
have multiple serial devices, pass the port explicitly with `--usb_address`.

**Neurorobotics Studio (cloud):**
```
python3 -m feagi_connector_mycobot --magic_link "<paste your magic link here>"
```

**Local FEAGI / Docker:**
```
python3 -m feagi_connector_mycobot --ip <feagi_ip> --port <feagi_zmq_port>
```

On Windows use `python` instead of `python3`.

# Flags

| Flag | Description |
|------|-------------|
| `--magic_link` | Magic link from the Neurorobotics Studio Embodiment tab. |
| `--ip` | FEAGI host IP (local/Docker). |
| `--port` | FEAGI ZMQ port (`30000` for Docker, `3000` for localhost). |
| `--usb_address` | Serial port of the arm, e.g. `/dev/ttyUSB0`, `/dev/cu.usbserial-XXXX`, or `COM3`. |

# Finding your USB port

- **Linux:** `ls /dev/ttyUSB*` (or `ls /dev/ttyACM*`).
- **macOS:** `ls /dev/cu.usbserial*`.
- **Windows:** check the port name (e.g. `COM3`) in Device Manager.

FEAGI will list the detected ports on startup if you do not pass `--usb_address`.

# Capabilities

- **Output — servo (6):** each FEAGI motor command sets a joint encoder position.
- **Input — servo_position (6):** live joint encoder positions stream back to FEAGI.

Edit `capabilities.json` to enable/disable joints or change per-joint min/max/default
encoder values.

# License

Apache 2.0
