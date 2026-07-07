#!/usr/bin/env python
"""
Hardware-free unit tests for the MyCobot controller's pure logic.

These exercise the device adapter (via an injected fake arm) and the joint-map
model without importing ``pymycobot``, the ``feagi`` SDK, or touching hardware —
mirroring the xArm controller's test approach. Runnable with ``pytest`` or directly
(``python test_mycobot_controller.py``).

Copyright 2026 Neuraville Inc.
"""
from __future__ import annotations

import json
import os
import tempfile

from joint_map import load_joint_map, normalize_angle_to_unit, parse_joint_map
from mycobot_device import MyCobotDevice, MyCobotDeviceError


class FakeMyCobot:
    """Minimal stand-in for ``pymycobot.MyCobot`` recording commanded moves."""

    def __init__(self, angles=None, moving=0):
        self._angles = list(angles if angles is not None else [0.0] * 6)
        self._moving = moving
        self.sent_angles = []
        self.sent_angle_calls = []
        self.released = False

    def get_angles(self):
        return list(self._angles)

    def send_angles(self, angles, speed):
        self.sent_angles.append((list(angles), speed))
        self._angles = list(angles)

    def send_angle(self, joint_id, angle, speed):
        self.sent_angle_calls.append((joint_id, angle, speed))

    def is_moving(self):
        return self._moving

    def release_all_servos(self):
        self.released = True

    def close(self):
        pass


def test_dof_and_angles():
    dev = MyCobotDevice(FakeMyCobot(angles=[1, 2, 3, 4, 5, 6]))
    assert dev.dof == 6
    assert dev.get_joint_angles() == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_speed_is_clamped_to_1_100():
    fake = FakeMyCobot()
    dev = MyCobotDevice(fake)
    dev.set_joint_angles([0, 0, 0, 0, 0, 0], speed=999)
    dev.set_joint_angles([0, 0, 0, 0, 0, 0], speed=-5)
    assert fake.sent_angles[0][1] == 100
    assert fake.sent_angles[1][1] == 1


def test_set_single_joint_is_1_indexed():
    fake = FakeMyCobot()
    dev = MyCobotDevice(fake)
    dev.set_joint_angle(0, 42.0, speed=50)
    assert fake.sent_angle_calls[0] == (1, 42.0, 50)  # 0-indexed -> servo id 1


def test_out_of_range_joint_raises():
    dev = MyCobotDevice(FakeMyCobot())
    try:
        dev.set_joint_angle(6, 0.0, speed=10)
    except MyCobotDeviceError:
        return
    raise AssertionError("expected MyCobotDeviceError for joint_index 6")


def test_is_moving_maps_states():
    assert MyCobotDevice(FakeMyCobot(moving=1)).is_moving() is True
    assert MyCobotDevice(FakeMyCobot(moving=0)).is_moving() is False
    assert MyCobotDevice(FakeMyCobot(moving=-1)).is_moving() is False  # read error -> not moving


def test_disconnect_releases_servos():
    fake = FakeMyCobot()
    MyCobotDevice(fake).disconnect()
    assert fake.released is True


def test_joint_map_parses_and_normalizes():
    raw = {
        "proprioception_group": 0,
        "z_neuron_resolution": 20,
        "joints": [
            {"index": 0, "motor_group": 0, "motor_channel": 0,
             "range_min_deg": -180.0, "range_max_deg": 180.0, "feedback_channel": 0},
        ],
    }
    jm = parse_joint_map(raw)
    assert len(jm.joints) == 1
    j = jm.joints[0]
    assert normalize_angle_to_unit(-180.0, j) == 0.0
    assert normalize_angle_to_unit(0.0, j) == 0.5
    assert normalize_angle_to_unit(180.0, j) == 1.0
    assert normalize_angle_to_unit(999.0, j) == 1.0  # clamped


def test_example_joint_map_file_is_valid():
    path = os.path.join(os.path.dirname(__file__), "example_joint_map.json")
    jm = load_joint_map(path)
    assert len(jm.joints) == 6


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"PASS {t.__name__}")
    print(f"\n{len(tests)} tests passed.")
