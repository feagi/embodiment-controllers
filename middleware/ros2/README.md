# ROS 2 Middleware Controller

Canonical **controller_id**: `ros2` (see `manifest.json` → `bundle.id`).

This package sits under **`controllers/middleware/ros2`** so embodiments that depend on ROS 2 use the same pipeline as simulator/platform controllers:

- Semantic versioning and immutable publishes
- nrs-composer discovery and **GCS** layout: `middleware/ros2/<tarball>` (production bucket)
- **`embodiment.json`**: `controller_metadata.controller_type` = `"ros2"` must match `bundle.id`

## Source of truth (`ros_connector_bridge.py`)

**Canonical file:** `nrs-embodiments/controllers/middleware/ros2/ros_connector_bridge.py` (in this monorepo).

The copy in **this** directory must stay **byte-identical** to that file when you cut a release from **embodiment-controllers** (copy from nrs-embodiments before publish, or add CI `diff` gate).

Neurorobotics Studio does **not** ship the bridge under `feagi-desktop`; it executes the Composer-installed bundle under `~/.feagi(-staging)/controllers/ros2/<version>/`.

## Embodiments

Reference this middleware when your experiment requires ROS 2 (multiple robot stacks, DDS graphs).

## Runtime

**`ros_connector_bridge.py`** (manifest `installation.entry_point`) ships in this tarball. Neurorobotics Studio downloads it via the same controller registry/GCS pipeline as simulator bundles and installs under **`~/.feagi/controllers/ros2/<version>/`** (staging: **`~/.feagi-staging`**). Updating the semver in Composer upgrades the bridge without rebuilding the desktop app.

## See also

- [Controllers README](https://github.com/feagi/embodiment-controllers/tree/main/controllers) (once published at repo root layout)
- [FEAGI docs](https://docs.feagi.org)
