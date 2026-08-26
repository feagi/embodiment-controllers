"""Convert DAVIS346 event columns into FEAGI XYZP voxels.

This module is compute-only: it does not open cameras, read files, or talk to
FEAGI. Event cameras already encode temporal change, so callers must inject
these voxels directly into a simple-vision cortical area and must not run
FEAGI's RGB frame-difference pipeline on this data.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np
from numpy.typing import NDArray

# DAVIS 346 event array (iniVation hardware specification).
DAVIS346_WIDTH = 346
DAVIS346_HEIGHT = 260

# Simple-vision IPUs default to firing_threshold=150 with no charge accumulation.
# Event polarity is binary; P is a constant at or above that threshold.
EVENT_POTENTIAL = 200.0

POLARITY_OFF = 0
POLARITY_ON = 1

# AEDAT4 and neuromorphic_drivers timestamps are microseconds.
_USEC_PER_SEC = 1_000_000.0

# Shared structured-array dtype produced by both `neuromorphic_drivers`
# (live DAVIS346 capture) and `aedat` (AEDAT4 file playback):
#   [("t", "<u8"), ("x", "<u2"), ("y", "<u2"), ("on"/"p", "?")]
#
# The boolean polarity field's canonical numpy field name has changed across
# `aedat` releases ("on" vs "p"; both mean the same thing -- True/1 = ON,
# brightness increased), so callers must accept either name.
_POLARITY_FIELD_NAMES = ("on", "p")


def playback_delay_s(
    event_ts_us: int,
    origin_ts_us: int,
    origin_wall_s: float,
    now_wall_s: float,
) -> float:
    """Return how long to wait so file playback tracks recorded event time.

    Args:
        event_ts_us: Timestamp of the next event batch, microseconds.
        origin_ts_us: Timestamp of the first event in this playback pass.
        origin_wall_s: Wall clock when the first event was scheduled.
        now_wall_s: Current wall clock.

    Returns:
        Seconds to sleep (>= 0). Zero when playback is behind the recording.
    """
    if event_ts_us < origin_ts_us:
        raise ValueError(
            f"Event timestamp {event_ts_us} is before playback origin {origin_ts_us}"
        )
    recorded_s = (event_ts_us - origin_ts_us) / _USEC_PER_SEC
    elapsed_s = now_wall_s - origin_wall_s
    delay = recorded_s - elapsed_s
    return delay if delay > 0.0 else 0.0


def polarity_events_to_columns(
    polarity_events: NDArray,
) -> Tuple[NDArray[np.int32], NDArray[np.int32], NDArray[np.int32], NDArray[np.uint64]]:
    """Extract x, y, polarity, and timestamp columns from a polarity-events array.

    Accepts the structured numpy array shape shared by both `neuromorphic_drivers`
    packets (live capture) and `aedat` decoder packets (AEDAT4 playback):
    ``[("t", "<u8"), ("x", "<u2"), ("y", "<u2"), (<polarity>, "?")]``, where the
    boolean polarity field is named "on" or "p" depending on dependency version
    (see `_POLARITY_FIELD_NAMES`).

    Args:
        polarity_events: Structured array with fields "t", "x", "y", and a
            boolean polarity field named "on" or "p".

    Returns:
        Integer x, y, polarity (0/1) columns, and the raw timestamp column.
    """
    names = polarity_events.dtype.names
    if names is None:
        raise TypeError("polarity_events did not have a structured dtype")
    required = ("t", "x", "y")
    missing = [name for name in required if name not in names]
    polarity_field = next((name for name in _POLARITY_FIELD_NAMES if name in names), None)
    if polarity_field is None:
        missing.append("/".join(_POLARITY_FIELD_NAMES))
    if missing:
        raise TypeError(f"polarity_events missing fields {missing}; have {names}")
    xs = np.asarray(polarity_events["x"], dtype=np.int32)
    ys = np.asarray(polarity_events["y"], dtype=np.int32)
    polarities = np.where(
        np.asarray(polarity_events[polarity_field]), POLARITY_ON, POLARITY_OFF
    ).astype(np.int32)
    timestamps = np.asarray(polarity_events["t"], dtype=np.uint64)
    return xs, ys, polarities, timestamps


def events_to_xyzp(
    xs: NDArray[np.integer],
    ys: NDArray[np.integer],
    polarities: NDArray[np.integer],
    width: int,
    height: int,
    potential: float,
) -> Tuple[NDArray[np.uint32], NDArray[np.uint32], NDArray[np.uint32], NDArray[np.float32]]:
    """Map one burst of DVS events to unique XYZP voxels.

    Each event becomes voxel (x, y, z=polarity). Multiple events at the same
    voxel in one burst collapse to a single spike (last event wins). Events
    outside the sensor rectangle are dropped.

    DAVIS uses image coordinates: (0, 0) is top-left, y increases downward.
    FEAGI / Brain Visualizer use cartesian voxels: (0, 0) is bottom-left, y
    increases upward. This function converts Y with
    ``feagi_y = height - 1 - sensor_y``.

    Args:
        xs: Event x coordinates (sensor / image space).
        ys: Event y coordinates (sensor / image space, top-left origin).
        polarities: Event polarity (0=OFF, 1=ON). Boolean arrays are accepted.
        width: Sensor width in pixels.
        height: Sensor height in pixels.
        potential: Constant membrane injection for every kept voxel.

    Returns:
        Tuple of uint32 x/y/z arrays and a float32 potential array.
    """
    if width <= 0 or height <= 0:
        raise ValueError(f"Sensor dimensions must be positive, got {width}x{height}")
    if potential <= 0.0:
        raise ValueError(f"Event potential must be > 0, got {potential}")
    if not (len(xs) == len(ys) == len(polarities)):
        raise ValueError(
            "Event column lengths must match: "
            f"x={len(xs)} y={len(ys)} polarity={len(polarities)}"
        )

    if len(xs) == 0:
        return (
            np.zeros(0, dtype=np.uint32),
            np.zeros(0, dtype=np.uint32),
            np.zeros(0, dtype=np.uint32),
            np.zeros(0, dtype=np.float32),
        )

    x_arr = np.asarray(xs, dtype=np.int32)
    y_arr = np.asarray(ys, dtype=np.int32)
    z_arr = np.asarray(polarities, dtype=np.int32)
    z_arr = np.where(z_arr != 0, POLARITY_ON, POLARITY_OFF)

    in_bounds = (
        (x_arr >= 0)
        & (x_arr < width)
        & (y_arr >= 0)
        & (y_arr < height)
        & ((z_arr == POLARITY_OFF) | (z_arr == POLARITY_ON))
    )
    x_arr = x_arr[in_bounds]
    y_arr = y_arr[in_bounds]
    z_arr = z_arr[in_bounds]
    if x_arr.size == 0:
        return (
            np.zeros(0, dtype=np.uint32),
            np.zeros(0, dtype=np.uint32),
            np.zeros(0, dtype=np.uint32),
            np.zeros(0, dtype=np.float32),
        )

    # FEAGI cartesian Y (bottom-left) from sensor row (top-left).
    y_arr = (height - 1) - y_arr

    # Last event at a voxel wins. Unique on a packed key so one burst cannot
    # emit duplicate (x, y, z) rows into XYZP.
    voxel_key = (
        x_arr.astype(np.uint64) * np.uint64(height * 2)
        + y_arr.astype(np.uint64) * np.uint64(2)
        + z_arr.astype(np.uint64)
    )
    _, unique_indices = np.unique(voxel_key[::-1], return_index=True)
    keep = (len(voxel_key) - 1) - unique_indices
    keep.sort()

    x_out = x_arr[keep].astype(np.uint32)
    y_out = y_arr[keep].astype(np.uint32)
    z_out = z_arr[keep].astype(np.uint32)
    p_out = np.full(x_out.shape[0], potential, dtype=np.float32)
    return x_out, y_out, z_out, p_out
