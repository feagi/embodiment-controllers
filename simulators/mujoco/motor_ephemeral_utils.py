"""
Ephemeral motor command helpers for MuJoCo controller.

Kept dependency-free so unit tests can import without numpy/mujoco/feagi.
"""

from __future__ import annotations

from typing import Optional, Tuple


def motor_rx_is_new_packet(rx_seq: object, last_seen_rx_seq: int) -> bool:
    """
    True when the motor callback sequence advanced (new FEAGI motor packet).

    The MuJoCo controller uses this with incremental / effort-absolute modes to
    choose between refreshing SDK values and holding the last applied ctrl at
    physics rate when FEAGI does not send a new packet every simulation step.
    """
    return isinstance(rx_seq, int) and rx_seq != int(last_seen_rx_seq)


def _clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp ``value`` into ``[min_val, max_val]``."""
    if value < min_val:
        return min_val
    if value > max_val:
        return max_val
    return value


def resolve_servo_ctrl_command(
    *,
    snap_value: Optional[float],
    current_qpos: Optional[float],
    min_val: float,
    max_val: float,
) -> Tuple[float, str]:
    """
    Resolve one ServoMotor actuator command for the current physics tick.

    - When ``snap_value`` is present (fresh decode this tick), map the [0, 1]
      absolute/incremental position into the joint range and return mode
      ``\"command\"``.
    - When ``snap_value`` is absent, hold the current pose (clamped into range)
      until a new FEAGI command arrives; return mode ``\"hold_pose\"``.
    - If pose is unknown while holding, fall back to the range center.
    """
    if snap_value is not None:
        pos01 = _clamp(float(snap_value), 0.0, 1.0)
        ctrl = min_val + (max_val - min_val) * pos01
        return float(ctrl), "command"

    center = (min_val + max_val) / 2.0
    if current_qpos is None:
        return float(center), "hold_pose"
    try:
        qpos_f = float(current_qpos)
    except (TypeError, ValueError):
        return float(center), "hold_pose"
    if qpos_f != qpos_f:  # NaN
        return float(center), "hold_pose"
    return float(_clamp(qpos_f, min_val, max_val)), "hold_pose"
