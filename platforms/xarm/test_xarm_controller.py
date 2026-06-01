#!/usr/bin/env python
"""
Unit tests for the FEAGI xARM platform controller modules.

These tests exercise the arbiter, joint-map validation, the hardware adapter return
handling, and the manual control service. The vendor xArm SDK is the only object
substituted (a fake), since it is outside the domain under test; all controller logic
runs for real. No FEAGI engine or physical arm is required.

Copyright 2026 Neuraville Inc.
"""
from __future__ import annotations

import threading
from typing import List, Optional, Tuple

import pytest

from arbitration import ArbiterStatus, ControlArbiter, ControlOwner
from control_server import ControlCommandError, ControlService
from joint_map import (
    JointMapping,
    normalize_angle_to_unit,
    parse_joint_map,
)
from xarm_device import GripperAction, XArmDevice, XArmDeviceError


# --------------------------------------------------------------------------- #
# Test doubles
# --------------------------------------------------------------------------- #
class FakeClock:
    """Manually advanced monotonic clock for deterministic arbiter timing tests."""

    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeArm:
    """Minimal stand-in for ``XArmAPI`` recording calls and returning SDK-shaped data."""

    def __init__(self, *, axis: int = 6) -> None:
        self.axis = axis
        self._angles: List[float] = [0.0] * axis
        self._position: List[float] = [200.0, 0.0, 150.0, 180.0, 0.0, 0.0]
        self.moving = False
        self.calls: List[Tuple[str, object]] = []
        self.fail_code: Optional[int] = None

    def motion_enable(self, *, enable: bool) -> int:
        self.calls.append(("motion_enable", enable))
        return 0

    def set_mode(self, mode: int) -> int:
        self.calls.append(("set_mode", mode))
        return 0

    def set_state(self, state: int) -> int:
        self.calls.append(("set_state", state))
        return 0

    def get_servo_angle(self) -> Tuple[int, List[float]]:
        return 0, list(self._angles)

    def set_servo_angle(self, *, angle: List[float], speed: float, wait: bool) -> int:
        self.calls.append(("set_servo_angle", (tuple(angle), speed)))
        if self.fail_code is not None:
            return self.fail_code
        self._angles = list(angle)
        return 0

    def get_position(self) -> Tuple[int, List[float]]:
        return 0, list(self._position)

    def set_position(self, **kwargs: float) -> int:
        self.calls.append(("set_position", kwargs))
        return 0

    def get_is_moving(self) -> bool:
        return self.moving

    def open_lite6_gripper(self) -> int:
        self.calls.append(("open_lite6_gripper", None))
        return 0

    def close_lite6_gripper(self) -> int:
        self.calls.append(("close_lite6_gripper", None))
        return 0

    def set_vacuum_gripper(self, *, on: bool) -> int:
        self.calls.append(("set_vacuum_gripper", on))
        return 0

    def reset(self, *, wait: bool) -> int:
        self.calls.append(("reset", wait))
        return 0

    def emergency_stop(self) -> None:
        self.calls.append(("emergency_stop", None))

    def disconnect(self) -> None:
        self.calls.append(("disconnect", None))


# --------------------------------------------------------------------------- #
# Arbiter
# --------------------------------------------------------------------------- #
def test_feagi_stream_allowed_by_default() -> None:
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=FakeClock())
    assert arbiter.feagi_stream_allowed() is True


def test_manual_command_pauses_then_resumes_feagi_stream() -> None:
    clock = FakeClock()
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=clock)
    assert arbiter.begin_manual_command() is True
    assert arbiter.feagi_stream_allowed() is False  # within hold window
    clock.advance(0.4)
    assert arbiter.feagi_stream_allowed() is False  # still holding
    clock.advance(0.2)
    assert arbiter.feagi_stream_allowed() is True  # settle window elapsed


def test_manual_command_rearms_hold_window() -> None:
    clock = FakeClock()
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=clock)
    arbiter.begin_manual_command()
    clock.advance(0.4)
    arbiter.begin_manual_command()  # re-arm
    clock.advance(0.3)
    assert arbiter.feagi_stream_allowed() is False  # extended by the second command


def test_estop_blocks_everything_until_cleared() -> None:
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=FakeClock())
    arbiter.engage_estop()
    assert arbiter.is_estopped() is True
    assert arbiter.feagi_stream_allowed() is False
    assert arbiter.begin_manual_command() is False  # manual blocked while e-stopped
    arbiter.clear_estop()
    assert arbiter.feagi_stream_allowed() is True


def test_non_feagi_owner_blocks_feagi_stream() -> None:
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=FakeClock())
    arbiter.set_streaming_owner(ControlOwner.ROS2)
    assert arbiter.feagi_stream_allowed() is False


def test_arbiter_rejects_non_positive_settle() -> None:
    with pytest.raises(ValueError):
        ControlArbiter(manual_hold_settle_s=0.0)


def test_status_snapshot_shape() -> None:
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=FakeClock())
    status = arbiter.status()
    assert isinstance(status, ArbiterStatus)
    assert status.owner is ControlOwner.FEAGI
    assert status.manual_hold_active is False


# --------------------------------------------------------------------------- #
# Joint map
# --------------------------------------------------------------------------- #
def _valid_joint_map_dict() -> dict:
    return {
        "proprioception_group": 0,
        "z_neuron_resolution": 20,
        "joints": [
            {
                "index": 1,
                "motor_group": 0,
                "motor_channel": 1,
                "range_min_deg": -180.0,
                "range_max_deg": 180.0,
                "feedback_channel": 1,
            },
            {
                "index": 0,
                "motor_group": 0,
                "motor_channel": 0,
                "range_min_deg": -90.0,
                "range_max_deg": 90.0,
                "feedback_channel": 0,
            },
        ],
    }


def test_parse_joint_map_sorts_joints_by_index() -> None:
    joint_map = parse_joint_map(_valid_joint_map_dict())
    assert [j.index for j in joint_map.joints] == [0, 1]
    assert joint_map.proprioception_group == 0
    assert joint_map.z_neuron_resolution == 20


def test_parse_joint_map_rejects_duplicate_index() -> None:
    bad = _valid_joint_map_dict()
    bad["joints"][1]["index"] = 1  # duplicate of the other joint
    with pytest.raises(ValueError, match="duplicate joint index"):
        parse_joint_map(bad)


def test_parse_joint_map_rejects_inverted_range() -> None:
    bad = _valid_joint_map_dict()
    bad["joints"][0]["range_min_deg"] = 200.0
    bad["joints"][0]["range_max_deg"] = 100.0
    with pytest.raises(ValueError, match="range_max_deg must exceed"):
        parse_joint_map(bad)


def test_parse_joint_map_rejects_bad_z_resolution() -> None:
    bad = _valid_joint_map_dict()
    bad["z_neuron_resolution"] = 0
    with pytest.raises(ValueError, match="z_neuron_resolution"):
        parse_joint_map(bad)


def test_normalize_angle_clamps_to_unit_interval() -> None:
    mapping = JointMapping(
        index=0,
        motor_group=0,
        motor_channel=0,
        range_min_deg=-90.0,
        range_max_deg=90.0,
        feedback_channel=0,
    )
    assert normalize_angle_to_unit(0.0, mapping) == pytest.approx(0.5)
    assert normalize_angle_to_unit(-90.0, mapping) == pytest.approx(0.0)
    assert normalize_angle_to_unit(90.0, mapping) == pytest.approx(1.0)
    assert normalize_angle_to_unit(180.0, mapping) == pytest.approx(1.0)  # clamped


# --------------------------------------------------------------------------- #
# Device adapter
# --------------------------------------------------------------------------- #
def test_device_discovers_dof_from_axis() -> None:
    device = XArmDevice(FakeArm(axis=6))
    assert device.dof == 6


def test_device_jog_applies_relative_delta() -> None:
    arm = FakeArm(axis=6)
    device = XArmDevice(arm)
    angles = device.jog_joint(2, 15.0, speed=30.0)
    assert angles[2] == pytest.approx(15.0)
    assert ("set_servo_angle", ((0.0, 0.0, 15.0, 0.0, 0.0, 0.0), 30.0)) in arm.calls


def test_device_jog_rejects_out_of_range_joint() -> None:
    device = XArmDevice(FakeArm(axis=6))
    with pytest.raises(XArmDeviceError, match="out of range"):
        device.jog_joint(9, 5.0, speed=30.0)


def test_device_raises_on_nonzero_code() -> None:
    arm = FakeArm(axis=6)
    arm.fail_code = 1
    device = XArmDevice(arm)
    with pytest.raises(XArmDeviceError, match="set_servo_angle failed"):
        device.set_joint_angles([0.0] * 6, speed=30.0)


# --------------------------------------------------------------------------- #
# Control service
# --------------------------------------------------------------------------- #
def _make_service(arm: FakeArm) -> Tuple[ControlService, ControlArbiter]:
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=FakeClock())
    service = ControlService(
        device=XArmDevice(arm),
        arbiter=arbiter,
        hardware_lock=threading.Lock(),
        manual_speed_deg_s=30.0,
        manual_cartesian_speed=100.0,
    )
    return service, arbiter


def test_service_jog_opens_manual_hold() -> None:
    arm = FakeArm()
    service, arbiter = _make_service(arm)
    result = service.handle("jog", {"joint_index": 0, "delta_deg": 10.0})
    assert result["action"] == "jog"
    assert arbiter.feagi_stream_allowed() is False  # manual hold engaged


def test_service_estop_engages_and_dispatches_hardware_stop() -> None:
    arm = FakeArm()
    service, arbiter = _make_service(arm)
    result = service.handle("estop", {})
    assert result["estopped"] is True
    assert arbiter.is_estopped() is True
    assert ("emergency_stop", None) in arm.calls


def test_service_blocks_manual_motion_while_estopped() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    service.handle("estop", {})
    with pytest.raises(ControlCommandError, match="emergency-stopped"):
        service.handle("jog", {"joint_index": 0, "delta_deg": 10.0})


def test_service_clear_estop_recovers() -> None:
    arm = FakeArm()
    service, arbiter = _make_service(arm)
    service.handle("estop", {})
    service.handle("clear_estop", {})
    assert arbiter.is_estopped() is False
    # set_state called again by recover_after_estop -> enable()
    assert ("set_state", 0) in arm.calls


def test_service_set_owner_validates_value() -> None:
    arm = FakeArm()
    service, arbiter = _make_service(arm)
    assert service.handle("set_owner", {"owner": "ros2"})["owner"] == "ros2"
    with pytest.raises(ControlCommandError):
        service.handle("set_owner", {"owner": "nonsense"})


def test_service_gripper_validates_action() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    service.handle("gripper", {"gripper_action": GripperAction.OPEN.value})
    assert ("open_lite6_gripper", None) in arm.calls
    with pytest.raises(ControlCommandError):
        service.handle("gripper", {"gripper_action": "squeeze"})


def test_service_cartesian_requires_six_components() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    service.handle("move_cartesian", {"pose": [200, 0, 150, 180, 0, 0]})
    with pytest.raises(ControlCommandError, match="6-element"):
        service.handle("move_cartesian", {"pose": [1, 2, 3]})


def test_service_jog_requires_numeric_args() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    with pytest.raises(ControlCommandError):
        service.handle("jog", {"joint_index": "two", "delta_deg": 10.0})


def test_service_unknown_action_rejected() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    with pytest.raises(ControlCommandError, match="Unknown action"):
        service.handle("teleport", {})
