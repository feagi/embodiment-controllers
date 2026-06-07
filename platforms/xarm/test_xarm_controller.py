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

import xarm_device
from arbitration import ArbiterStatus, ControlArbiter, ControlOwner
from control_server import ControlCommandError, ControlService
from joint_map import (
    JointMap,
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
        self.error_code: int = 0
        self.mode: int = 0

    def clean_error(self) -> int:
        self.calls.append(("clean_error", None))
        return 0

    def motion_enable(self, *, enable: bool) -> int:
        self.calls.append(("motion_enable", enable))
        return 0

    def set_mode(self, mode: int) -> int:
        self.calls.append(("set_mode", mode))
        self.mode = mode
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
        if self.fail_code is not None:
            return self.fail_code
        return 0

    def get_is_moving(self) -> bool:
        return self.moving

    def open_lite6_gripper(self) -> int:
        self.calls.append(("open_lite6_gripper", None))
        return 0

    def close_lite6_gripper(self) -> int:
        self.calls.append(("close_lite6_gripper", None))
        return 0

    def stop_lite6_gripper(self) -> int:
        self.calls.append(("stop_lite6_gripper", None))
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


def test_device_check_accepts_none_as_success() -> None:
    """reset(wait=True) returns None on some SDK versions; _check must not raise."""
    arm = FakeArm(axis=6)
    arm.reset = lambda *, wait: None  # type: ignore[assignment]
    device = XArmDevice(arm)
    device.home()


def test_device_enable_calls_clean_error() -> None:
    """enable() must call clean_error() before re-enabling to clear ControllerError state."""
    arm = FakeArm(axis=6)
    device = XArmDevice(arm)
    arm.calls.clear()
    device.enable()
    call_names = [c[0] for c in arm.calls]
    assert call_names[0] == "clean_error", "clean_error must be called first"
    assert "motion_enable" in call_names
    assert "set_mode" in call_names
    assert "set_state" in call_names


def test_device_socket_connect_failure_message_no_route() -> None:
    msg = XArmDevice._format_socket_connect_failure(
        ip="192.168.1.156",
        port=502,
        exc=OSError(65, "No route to host"),
    )
    assert "Unable to reach xArm at 192.168.1.156:502" in msg
    assert "Network path to the arm is unreachable" in msg


def test_device_preflight_socket_raises_xarm_error(monkeypatch: pytest.MonkeyPatch) -> None:
    def _raise_no_route(*_args: object, **_kwargs: object) -> object:
        raise OSError(65, "No route to host")

    monkeypatch.setattr(xarm_device.socket, "create_connection", _raise_no_route)
    with pytest.raises(XArmDeviceError, match="Unable to reach xArm"):
        XArmDevice._preflight_socket(ip="192.168.1.156", port=502)


# --------------------------------------------------------------------------- #
# Control service
# --------------------------------------------------------------------------- #
def _make_service(
    arm: FakeArm,
    manual_speed_deg_s: float = 30.0,
    manual_cartesian_speed: float = 100.0,
) -> Tuple[ControlService, ControlArbiter]:
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=FakeClock())
    service = ControlService(
        device=XArmDevice(arm),
        arbiter=arbiter,
        hardware_lock=threading.Lock(),
        manual_speed_deg_s=manual_speed_deg_s,
        manual_cartesian_speed=manual_cartesian_speed,
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


def test_service_gripper_stop_cuts_motor() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    service.handle("gripper", {"gripper_action": GripperAction.STOP.value})
    assert ("stop_lite6_gripper", None) in arm.calls


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


def test_service_move_joints_commands_absolute_angles() -> None:
    arm = FakeArm()
    service, arbiter = _make_service(arm)
    result = service.handle("move_joints", {"angles": [10.0, 20.0, 30.0, 0, 0, 0]})
    assert result["action"] == "move_joints"
    assert result["angles"] == [10.0, 20.0, 30.0, 0, 0, 0]
    assert arbiter.feagi_stream_allowed() is False  # manual hold engaged
    servo_calls = [c for c in arm.calls if c[0] == "set_servo_angle"]
    assert len(servo_calls) == 1
    assert servo_calls[0][1][0] == (10.0, 20.0, 30.0, 0, 0, 0)


def test_service_move_joints_rejects_empty_angles() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    with pytest.raises(ControlCommandError, match="non-empty"):
        service.handle("move_joints", {"angles": []})


def test_service_auto_recovers_after_motion_failure() -> None:
    """After a motion command fails, the service should auto-recover the arm."""
    arm = FakeArm()
    service, _ = _make_service(arm)
    arm.fail_code = 1
    with pytest.raises(XArmDeviceError):
        service.handle("move_cartesian", {"pose": [200, 0, 150, 180, 0, 0]})
    call_names = [c[0] for c in arm.calls]
    assert "clean_error" in call_names, "auto-recovery should call clean_error"
    assert "set_state" in call_names, "auto-recovery should call enable()"


def test_service_clear_error_re_enables_arm() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    arm.calls.clear()
    result = service.handle("clear_error", {})
    assert result["cleared"] is True
    call_names = [c[0] for c in arm.calls]
    assert "clean_error" in call_names
    assert "motion_enable" in call_names


def test_service_status_includes_error_code() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    result = service.handle("status", {})
    assert result["error_code"] == 0
    arm.error_code = 22
    result = service.handle("status", {})
    assert result["error_code"] == 22


def test_service_unknown_action_rejected() -> None:
    arm = FakeArm()
    service, _ = _make_service(arm)
    with pytest.raises(ControlCommandError, match="Unknown action"):
        service.handle("teleport", {})


# --------------------------------------------------------------------------- #
# Controller - FEAGI target application
# --------------------------------------------------------------------------- #
# The controller module transitively imports feagi.pns (Rust native libs).
# These tests skip gracefully when the Rust wheel is not installed.
_controller_import_error: Optional[str] = None
try:
    from controller import XArmFeagiController
except ImportError as _exc:
    _controller_import_error = str(_exc)
    XArmFeagiController = None  # type: ignore[assignment, misc]

_servo_import_error: Optional[str] = None
try:
    from feagi.pns.outputs import ServoMotor as _ServoMotor
except ImportError as _exc:
    _servo_import_error = str(_exc)
    _ServoMotor = None  # type: ignore[assignment, misc]

_needs_rust = pytest.mark.skipif(
    _controller_import_error is not None or _servo_import_error is not None,
    reason="Requires feagi_rust_py_libs native wheel",
)


def _make_controller(arm: FakeArm) -> "XArmFeagiController":
    """Build a controller with a 2-joint map for testing (no FEAGI connection)."""
    joint_map = JointMap(
        proprioception_group=0,
        z_neuron_resolution=20,
        joints=(
            JointMapping(
                index=0, motor_group=0, motor_channel=0,
                range_min_deg=-180.0, range_max_deg=180.0, feedback_channel=0,
            ),
            JointMapping(
                index=1, motor_group=0, motor_channel=1,
                range_min_deg=-90.0, range_max_deg=90.0, feedback_channel=1,
            ),
        ),
    )
    arbiter = ControlArbiter(manual_hold_settle_s=0.5, clock=FakeClock())
    device = XArmDevice(arm)
    ctrl = XArmFeagiController(
        device=device,
        joint_map=joint_map,
        arbiter=arbiter,
        burst_period_s=0.05,
        motor_speed_deg_s=30.0,
    )
    return ctrl


@_needs_rust
def test_apply_feagi_targets_skips_when_no_new_commands() -> None:
    """Arm should NOT be commanded when FEAGI has not sent any motor data."""
    arm = FakeArm()
    arm._angles = [45.0, 30.0, 0.0, 0.0, 0.0, 0.0]
    ctrl = _make_controller(arm)
    ctrl._servos = {
        0: _ServoMotor(range=(-180.0, 180.0), encoding="absolute", unit_id=0, channel_index=0),
        1: _ServoMotor(range=(-90.0, 90.0), encoding="absolute", unit_id=0, channel_index=1),
    }
    servo_calls_before = [c for c in arm.calls if c[0] == "set_servo_angle"]
    ctrl._apply_feagi_targets()
    servo_calls_after = [c for c in arm.calls if c[0] == "set_servo_angle"]
    assert len(servo_calls_after) == len(servo_calls_before), (
        "set_servo_angle should not be called when no FEAGI commands have arrived"
    )


def test_service_clamps_joint_speed_to_max() -> None:
    """Preset speed exceeding 180 deg/s should be clamped to the hardware max."""
    arm = FakeArm()
    service, _ = _make_service(arm)
    service.handle("move_joints", {"angles": [0, 0, 0, 0, 0, 0], "speed_deg_s": 999})
    servo_calls = [c for c in arm.calls if c[0] == "set_servo_angle"]
    assert len(servo_calls) == 1
    _angles, recorded_speed = servo_calls[0][1]
    assert recorded_speed <= 180.0, "Joint speed must be clamped to 180 deg/s"


def test_service_clamps_cartesian_speed_to_max() -> None:
    """Cartesian speed should be clamped to 500 mm/s regardless of config."""
    arm = FakeArm()
    service, _ = _make_service(arm, manual_cartesian_speed=9999.0)
    service.handle("move_cartesian", {"pose": [200, 0, 150, 180, 0, 0]})
    cart_calls = [c for c in arm.calls if c[0] == "set_position"]
    assert len(cart_calls) == 1
    kwargs = cart_calls[0][1]
    assert kwargs["speed"] <= 500.0, "Cartesian speed must be clamped to 500 mm/s"


def test_device_set_manual_mode() -> None:
    """set_manual_mode should switch the arm to mode 2 (teach/manual)."""
    arm = FakeArm()
    device = XArmDevice(arm)
    device.set_manual_mode()
    assert arm.mode == 2, "Arm should be in manual mode (2)"


def test_device_set_position_mode() -> None:
    """set_position_mode should switch the arm back to mode 0."""
    arm = FakeArm()
    device = XArmDevice(arm)
    device.set_manual_mode()
    device.set_position_mode()
    assert arm.mode == 0, "Arm should be back in position mode (0)"


def test_device_mode_property() -> None:
    """mode property should reflect the current arm mode."""
    arm = FakeArm()
    device = XArmDevice(arm)
    assert device.mode == 0
    device.set_manual_mode()
    assert device.mode == 2


def test_service_set_manual_mode_on() -> None:
    """set_manual_mode action with enabled=True should enter teach mode."""
    arm = FakeArm()
    service, _ = _make_service(arm)
    result = service.handle("set_manual_mode", {"enabled": True})
    assert result["manual_mode"] is True
    assert arm.mode == 2


def test_service_set_manual_mode_off() -> None:
    """set_manual_mode action with enabled=False should return to position mode."""
    arm = FakeArm()
    service, _ = _make_service(arm)
    service.handle("set_manual_mode", {"enabled": True})
    result = service.handle("set_manual_mode", {"enabled": False})
    assert result["manual_mode"] is False
    assert arm.mode == 0


def test_service_status_includes_arm_mode() -> None:
    """Status payload should include the arm_mode field."""
    arm = FakeArm()
    service, _ = _make_service(arm)
    status = service.handle("status", {})
    assert "arm_mode" in status
    assert status["arm_mode"] == 0
    service.handle("set_manual_mode", {"enabled": True})
    status = service.handle("status", {})
    assert status["arm_mode"] == 2


@_needs_rust
def test_apply_feagi_targets_commands_arm_after_feagi_update() -> None:
    """Arm should be commanded only for joints that received new FEAGI data."""
    arm = FakeArm()
    arm._angles = [45.0, 30.0, 0.0, 0.0, 0.0, 0.0]
    ctrl = _make_controller(arm)
    servo0 = _ServoMotor(range=(-180.0, 180.0), encoding="absolute", unit_id=0, channel_index=0)
    servo1 = _ServoMotor(range=(-90.0, 90.0), encoding="absolute", unit_id=0, channel_index=1)
    ctrl._servos = {0: servo0, 1: servo1}

    # Simulate FEAGI sending a command to servo 0 only.
    servo0._on_motor_command(0.75)

    ctrl._apply_feagi_targets()
    servo_calls = [c for c in arm.calls if c[0] == "set_servo_angle"]
    assert len(servo_calls) == 1, "set_servo_angle should be called once after FEAGI update"
    commanded_angles = servo_calls[0][1][0]
    # Joint 0 should have the new FEAGI-commanded angle, joint 1 keeps its physical angle.
    assert commanded_angles[0] != 45.0, "Joint 0 should be updated by FEAGI"
    assert commanded_angles[1] == 30.0, "Joint 1 should keep its physical angle (no FEAGI command)"
