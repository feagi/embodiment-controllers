# Elephant Robotics MyCobot Platform Controller

Canonical **controller_id**: `mycobot` (see `manifest.json` -> `bundle.id`).

Drives an Elephant Robotics **MyCobot 280** (6-DOF) by bridging the FEAGI engine to
the arm through the [`pymycobot`](https://pypi.org/project/pymycobot/) SDK over USB
serial. It sits under **`platforms/mycobot`** so it uses the same versioning /
Composer / feagi-core pipeline as the xArm, MuJoCo, and ROS 2 controllers — **not**
the legacy `feagi_connector` library (see "Why this replaces the old controller").

## Modules

| File | Responsibility |
| --- | --- |
| `controller.py` | Entry point: FEAGI SDK session, registration, and the receive/apply/stream loop. |
| `mycobot_device.py` | Typed adapter over `pymycobot.MyCobot` (joint angles, connect, disconnect). |
| `joint_map.py` | Parse/validate the desktop-generated joint map; angle normalization (shared with xArm). |
| `example_joint_map.json` | Sample joint map (MyCobot 280 limits) for local testing. |

## FEAGI mapping

- **Motor (FEAGI -> arm):** each joint registers as a positional `ServoMotor`; FEAGI
  joint targets are applied as absolute servo angles (degrees) via `send_angles`.
- **Sensory (arm -> FEAGI):** joint angles (`get_angles`) stream back as a `Servo`
  proprioception cortical area, one channel per joint.

The desktop app generates a **joint-map JSON** (passed via `--joint-map`) describing,
per joint: `motor_group`, `motor_channel`, `range_min_deg`, `range_max_deg`, and
`feedback_channel`, plus the shared `proprioception_group` and `z_neuron_resolution`.

## How it is launched

Like the other new-SDK controllers (`xarm`, `mujoco`), this controller is **launched
by Neurorobotics Studio / the desktop app**, which installs the bundle under
`~/.feagi/controllers/mycobot/<version>/` and spawns `controller.py` with the FEAGI
network ports, a base64 `--agent_id` (AgentDescriptor), an auth token, and the
generated `--joint-map`. The SDK runs in "safety mode" and refuses to start if any of
these are missing — there are no hardcoded defaults.

For local bring-up you can run it by hand against a running local FEAGI, supplying the
same values the app would (ZMQ ports from the engine, the sample joint map, and your
serial port):

```bash
python controller.py \
  --ip 127.0.0.1 --port 8000 \
  --feagi-zmq-motor-port <motor> --feagi-zmq-registration-port <reg> --feagi-zmq-sensory-port <sensory> \
  --feagi-zmq-connection-timeout-ms 2000 --feagi-zmq-registration-retries 5 \
  --feagi-zmq-heartbeat-interval-s 2.0 --feagi-http-timeout-s 5.0 \
  --agent_id "<base64 AgentDescriptor>" --auth-token-b64 "<base64 token>" \
  --usb-port /dev/cu.usbserial-XXXX \
  --joint-map example_joint_map.json \
  --feagi-burst-period-s 0.05 --motor-speed 50
```

Find your serial port with `ls /dev/cu.*` (macOS), `ls /dev/ttyUSB*` (Linux), or Device
Manager (Windows, e.g. `COM3`). The MyCobot 280 uses a CH340 USB-serial chip.

## Why this replaces the old controller

The previous MyCobot controllers under `embodiments/elephant_robotics/`
(`feagi_connector_mycobot`, `pure_python_mycobot`) use the legacy `feagi_connector`
library, which registers against FEAGI API endpoints (`/v1/network/network`,
`/v1/burst_engine/stimulation_period`) that the current `feagi-core` engine no longer
serves — so those controllers hang at registration against the current local desktop
engine and NRS. This controller targets the `feagi-core` SDK, matching every other
current embodiment.

## License

Apache 2.0
