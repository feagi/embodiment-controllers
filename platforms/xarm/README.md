# UFACTORY xArm Platform Controller

Canonical **controller_id**: `xarm` (see `manifest.json` -> `bundle.id`).

One controller drives any UFACTORY xArm-series device (Lite6 first) by bridging the
FEAGI engine to the arm through the [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
(`xarm-python-sdk`, BSD-3-Clause). It sits under **`controllers/platforms/xarm`** so it
uses the same versioning / Composer / GCS pipeline as the simulator and middleware
controllers (`controllers/README.md`).

## Source of truth

**Canonical file:** `nrs-embodiments/controllers/platforms/xarm/` (private, admin-gated).
The public mirror under `embodiment-controllers/platforms/xarm/` must stay byte-identical
when a release is cut. Neurorobotics Studio does **not** ship this controller inside
`feagi-desktop`; it downloads the Composer-installed bundle to
`~/.feagi(-staging)/controllers/xarm/<version>/` and spawns `controller.py`.

## Modules

| File | Responsibility |
| --- | --- |
| `controller.py` | Entry point: FEAGI SDK session, registration, and the receive/apply/stream loop. |
| `xarm_device.py` | Typed adapter over `XArmAPI` (joint angles, Cartesian, gripper, home, e-stop, DOF discovery). |
| `arbitration.py` | Single-owner control arbiter: e-stop wins, manual commands pause the FEAGI stream. |
| `control_server.py` | Loopback HTTP server for manual control + e-stop, delegating to the arbiter/device. |

## FEAGI mapping

- **Motor (FEAGI -> arm):** each joint registers as a positional `ServoMotor`; FEAGI
  joint targets are applied as absolute servo angles.
- **Sensory (arm -> FEAGI):** joint encoder angles stream back as a `Servo`
  proprioception cortical area, one channel per joint.

The desktop app generates a **joint-map JSON** (passed via `--joint-map`) describing,
per joint: `motor_group`, `motor_channel`, `range_min_deg`, `range_max_deg`, and
`feedback_channel`, plus the shared `proprioception_group` and `z_neuron_resolution`.

## Manual control + safety

The controller exposes a loopback HTTP control server (`--control-server-host/-port`).
Supported `POST /command` actions: `jog`, `move_cartesian`, `gripper`, `home`, `estop`,
`clear_estop`, `set_owner`. `GET /status` returns arbiter state plus live joint angles.

Arbitration rules (enforced in `arbitration.py`):

1. **Emergency stop always wins** - no FEAGI or manual motion until explicitly cleared.
2. **Manual override auto-pauses the FEAGI stream** for a settle window
   (`--manual-hold-settle-s`), re-armed on each manual command, then resumes.
3. Otherwise the configured streaming owner (FEAGI; ROS 2 in a later phase) drives the arm.

## Runtime arguments

All FEAGI network/timing values are passed explicitly by the launcher (no defaults),
matching the MuJoCo controller contract. See `controller.py` `_build_arg_parser()` for
the full list (`--ip`, `--port`, ZMQ ports/timeouts, `--agent_id`, `--xarm-ip`,
`--joint-map`, `--feagi-burst-period-s`, motor/manual speeds, `--manual-hold-settle-s`,
`--control-server-host/-port`).

## Launch surfaces

This controller process is spawned by **exactly one** surface: the **xARM Connector
app**. The user enters the arm IP and builds the joint map in its UI, then the
`start_xarm_connector_bridge` Tauri command spawns `controller.py` directly. The
generic embodiment launcher (`launcher.rs`) deliberately refuses to start the arm
(`controller_id "xarm"`), because the arm IP, joint mapping, and manual-control /
e-stop surface only exist in the connector app.

Launching the **Lite 6** embodiment (`arm_lite6`, `controller_metadata.controller_type
= "xarm"`) as an experiment therefore **opens the xARM Connector app** so the user
connects the robot first; the experiment's genome is already loaded into FEAGI, and
the connector spawns the controller on connect. The "Reconnect embodiment" action
behaves the same way for xARM.

## Local development

```bash
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Run `pytest` from this directory to exercise the arbiter, device adapter, control
service, and joint-map validation (no hardware required; the arm SDK is faked).

## See also

- `controllers/README.md` - controller categories, Composer/GCS upload pipeline
- `controllers/middleware/ros2/` - ROS 2 middleware controller (phase-2 ROS publishing target)
- [FEAGI docs](https://docs.feagi.org)
