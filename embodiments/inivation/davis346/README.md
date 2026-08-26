# iniVation DAVIS346 Event-Camera Controller

Streams DAVIS346 event-camera output into a FEAGI simple-vision cortical area
as XYZP voxels. Supports both live USB capture and AEDAT4 file playback.

## Design rationale

Event cameras report per-pixel brightness changes, not full frames. Each
event is `(x, y, polarity, timestamp)`. This controller maps every event
directly to a FEAGI voxel:

- `X`, `Y`: pixel column/row.
- `Z`: event polarity, `0` = OFF (brightness decreased), `1` = ON (brightness
  increased).
- `P`: a constant potential (`--event-potential`, default `200.0`) sufficient
  to cross the simple-vision IPU's default firing threshold.

DAVIS346 uses image coordinates (`(0, 0)` top-left, Y increases downward).
FEAGI / Brain Visualizer use cartesian voxels (`(0, 0)` bottom-left, Y
increases upward). `davis_events.events_to_xyzp` converts with
`feagi_y = height - 1 - sensor_y`.

Because event cameras already encode temporal change, this controller
injects voxels directly via `FeagiAgentClient.send_sensory_bytes` and does
**not** run FEAGI's RGB frame-difference pipeline against DAVIS APS frames.

## Licensing

All dependencies are permissively licensed, chosen specifically to avoid
copyleft/proprietary entanglement:

| Dependency             | License    | Notes                                                        |
|-------------------------|------------|---------------------------------------------------------------|
| `neuromorphic_drivers`  | MIT        | Implements DAVIS346 USB protocol in Rust; transitively links `libusb` (LGPL-2.1) via `rusb`. Acceptable here because this repository is open-source. |
| `aedat`                 | MIT        | Pure AEDAT4 file decoder, no USB dependency.                  |
| `feagi` (feagi-python-sdk) | Apache-2.0 | FEAGI client and byte-container serialization.             |

The iniVation `dv-processing` SDK is intentionally **not** used: its PyPI
wheels bundle an FFmpeg build with GPL-3 components, which is unacceptable
for downstream closed-source integrations of this controller.

## Installation

Requires Python 3.10-3.13. Precompiled wheels for `neuromorphic_drivers` and
`aedat` are not yet available for Python 3.14+; pip will fall back to a
source build via `maturin`, which currently fails against PyO3 0.23.4's
Python 3.13 ceiling. Use `python3.12` or `python3.13` explicitly if your
default `python3` resolves to 3.14:

```bash
python3.12 -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt
```

## One-time setup: `networking.json` and `capabilities.json`

The controller reads two bundled config files by default so routine runs
don't require restating every parameter on the command line:

- **`networking.json`**: FEAGI host/ports, timeouts, retries, and agent
  identity/auth. Ships pre-filled with common local-FEAGI defaults
  (`127.0.0.1:8000`, standard ZMQ ports) -- edit `auth_token_b64` to match
  your FEAGI instance if it enforces a specific token.
- **`capabilities.json`**: cortical targeting (`cortical_group_id`,
  `coordinates_3d`, `event_potential`) and default camera/playback source.
  No genome-specific ID needs to be filled in -- the controller derives and
  provisions the target cortical area itself (see "Cortical targeting"
  below).

Edit both files once for your setup; every field in them can still be
overridden per-run with a matching `--flag` (CLI always wins). Any field
left unset by both the config files and the CLI still fails loudly with a
clear error listing exactly what's missing -- nothing silently falls back
to a hardcoded value.

Point at alternate config files with `--networking-config` /
`--capabilities-config`, e.g. to keep separate profiles per FEAGI instance.

## Usage

### Discover connected DAVIS346 cameras

```bash
python3 controller.py --mode discover
```

Prints `{"cameras": [...]}` with serial numbers on stdout.

### Live USB capture

```bash
python3 controller.py --mode stream
```

Override anything per-run, e.g. to pin a specific camera serial:

```bash
python3 controller.py --mode stream --camera-serial <serial>
```

### AEDAT4 file playback

```bash
python3 controller.py --mode playback --aedat-path /path/to/recording.aedat4
```

Add `--loop` / `--no-loop` to override `capabilities.json`'s `playback.loop`.
Playback replays events at their original recorded pacing (see
`davis_events.playback_delay_s`).

## Cortical targeting

`vision.cortical_group_id` (in `capabilities.json`, or `--cortical-group-id`)
is the only cortical-targeting field you set: it is both the vision
capability group index advertised to FEAGI during registration, and the
unit index used to derive the target simple-vision ("isvi") cortical area's
ID, matching the same deterministic ID derivation `feagi-core` uses
server-side. It must be unique among this agent's registered vision units.

Before connecting, the controller (`cortical_area_provisioner.py`) calls
FEAGI's REST API directly to:

1. Check whether that cortical area already exists.
2. If missing, create it as an `isvi` IPU sized to the sensor/recording
   resolution (346x260x2 for a native DAVIS346, or the AEDAT4 file's
   recorded resolution for playback), positioned at `vision.coordinates_3d`.
3. If it exists but is the wrong size, resize it in place.

`vision.coordinates_3d` only matters the first time the area is created;
it has no effect once the area exists.

No parameter here has a hardcoded default baked into the code: every
network address, port, timeout, and retry count comes from `networking.json`
/ `capabilities.json` or an explicit CLI override.

## Known limitations

- **`FeagiAgentClient` capability registration is not used for provisioning.**
  `FeagiAgentClient.configure(vision_unit=...)` only records width/height/
  channels/group locally for outgoing sensory bytes; it is never transmitted
  to FEAGI during `connect()`, so FEAGI's own
  `auto_create_cortical_areas_from_device_registrations` path (triggered by
  the separate, legacy `JSONInputOutputDefinition`/`send_device_configuration()`
  message) never fires for this controller. This is why cortical-area
  provisioning here goes through FEAGI's REST API directly instead.
- **`feagi.genome.api.GenomeAPI` has no public method to create IPU/OPU areas.**
  Its only creation method, `add_custom_cortical_area`, creates a generic
  "custom" area with no sensory device encoder attached, so it cannot stand
  in for a real `isvi` vision area. `cortical_area_provisioner.py` documents
  and works around this by calling FEAGI's `POST /v1/cortical_area/cortical_area`
  through `GenomeAPI`'s internal request helper. This should move to a
  proper public `GenomeAPI` method once the SDK adds one.

## Files

- `controller.py`: CLI entry point (discover / stream / playback modes).
- `controller_config.py`: Loads and merges `networking.json` /
  `capabilities.json` with CLI overrides (pure functions, independently
  unit-tested).
- `davis_events.py`: Pure-function XYZP conversion and playback timing
  (no camera or FEAGI I/O; independently unit-tested).
- `cortical_area_provisioner.py`: Derives the target simple-vision cortical
  ID and creates/resizes that area over FEAGI's REST API before streaming
  (independently unit-tested against a fake `GenomeAPI`).
- `networking.json` / `capabilities.json`: Bundled config defaults (see
  "One-time setup" above).
- `test_davis_events.py`, `test_controller_config.py`,
  `test_cortical_area_provisioner.py`: `pytest` unit tests.
- `manifest.json`: Marketplace/Composer bundle metadata.
- `requirements.txt`: Python dependencies with license notes.

## Testing

```bash
pip install pytest
pytest test_davis_events.py test_controller_config.py test_cortical_area_provisioner.py -v
```
