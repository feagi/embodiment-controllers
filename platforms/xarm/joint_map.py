#!/usr/bin/env python
"""
Joint-to-FEAGI mapping model and validation for the xARM controller.

The desktop app produces a joint-map JSON describing how each arm joint maps to a
FEAGI servo motor (cortical group + channel) and how its proprioception feedback is
addressed. This module owns the parsing/validation and the angle normalization used
by the control loop. It deliberately has no FEAGI/hardware imports so it is portable
and unit-testable in isolation.

Copyright 2026 Neuraville Inc.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class JointMapping:
    """FEAGI cortical mapping for a single arm joint (one row of the desktop table)."""

    index: int
    motor_group: int
    motor_channel: int
    range_min_deg: float
    range_max_deg: float
    feedback_channel: int

    @property
    def span_deg(self) -> float:
        """Angular span of the joint range; used to normalize proprioception."""
        return self.range_max_deg - self.range_min_deg


@dataclass(frozen=True)
class JointMap:
    """Full joint-to-FEAGI mapping plus shared registration parameters."""

    proprioception_group: int
    z_neuron_resolution: int
    joints: Tuple[JointMapping, ...]


def _req_int(entry: Dict[str, object], key: str, *, minimum: int) -> int:
    value = entry.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
        raise ValueError(f"'{key}' must be an integer >= {minimum}.")
    return value


def _req_num(entry: Dict[str, object], key: str) -> float:
    value = entry.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"'{key}' must be a number.")
    return float(value)


def parse_joint_map(raw: object) -> JointMap:
    """
    Validate a decoded joint-map object and return a typed :class:`JointMap`.

    Raises:
        ValueError: If the structure is malformed or contains out-of-range values.
    """
    if not isinstance(raw, dict):
        raise ValueError("joint map must be a JSON object.")
    z_res = raw.get("z_neuron_resolution")
    if not isinstance(z_res, int) or isinstance(z_res, bool) or z_res <= 0:
        raise ValueError("z_neuron_resolution must be a positive integer.")
    proprio_group = raw.get("proprioception_group")
    if (
        not isinstance(proprio_group, int)
        or isinstance(proprio_group, bool)
        or proprio_group < 0
    ):
        raise ValueError("proprioception_group must be a non-negative integer.")
    joints_raw = raw.get("joints")
    if not isinstance(joints_raw, list) or not joints_raw:
        raise ValueError("joints must be a non-empty list.")

    joints: List[JointMapping] = []
    for entry in joints_raw:
        if not isinstance(entry, dict):
            raise ValueError("each joint entry must be a JSON object.")
        mapping = JointMapping(
            index=_req_int(entry, "index", minimum=0),
            motor_group=_req_int(entry, "motor_group", minimum=0),
            motor_channel=_req_int(entry, "motor_channel", minimum=0),
            range_min_deg=_req_num(entry, "range_min_deg"),
            range_max_deg=_req_num(entry, "range_max_deg"),
            feedback_channel=_req_int(entry, "feedback_channel", minimum=0),
        )
        if mapping.span_deg <= 0:
            raise ValueError(
                f"joint {mapping.index}: range_max_deg must exceed range_min_deg."
            )
        joints.append(mapping)

    indices = [j.index for j in joints]
    if len(set(indices)) != len(indices):
        raise ValueError("duplicate joint index in joint map.")
    return JointMap(
        proprioception_group=proprio_group,
        z_neuron_resolution=z_res,
        joints=tuple(sorted(joints, key=lambda j: j.index)),
    )


def load_joint_map(path: str) -> JointMap:
    """Read and validate the joint-mapping JSON file at ``path``."""
    with open(path, "r", encoding="utf-8") as handle:
        return parse_joint_map(json.load(handle))


def normalize_angle_to_unit(angle_deg: float, mapping: JointMapping) -> float:
    """Map a joint angle (deg) into ``[0.0, 1.0]`` across the joint's configured range."""
    fraction = (angle_deg - mapping.range_min_deg) / mapping.span_deg
    return max(0.0, min(1.0, fraction))
