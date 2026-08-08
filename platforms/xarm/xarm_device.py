#!/usr/bin/env python
"""
Thin hardware adapter around the UFACTORY xArm-Python-SDK (`XArmAPI`).

The adapter isolates every direct call into the vendor SDK behind a small, typed
surface so the FEAGI control loop, the manual HTTP control server, and the unit
tests never import or depend on `xArm` directly. This keeps the controller logic
hardware-agnostic and testable (the SDK is outside the domain under test, so it is
the one object tests are permitted to substitute).

Joint angles are expressed in degrees throughout, matching the xArm SDK servo-angle
convention. Cartesian poses are `[x, y, z, roll, pitch, yaw]` (mm and degrees).

Copyright 2026 Neuraville Inc.
"""
from __future__ import annotations

import errno
import logging
import socket
from enum import Enum
from typing import Final, List, Optional, Sequence

logger = logging.getLogger("xarm_controller.device")

#: xArm SDK success return code.
_XARM_CODE_OK: Final[int] = 0
#: Timeout (seconds) for preflight TCP check before SDK handshake.
_XARM_CONNECT_PREFLIGHT_TIMEOUT_S: Final[float] = 3.0


class XArmDeviceError(RuntimeError):
    """Raised when the xArm SDK reports a non-zero return code for an operation."""


class GripperAction(str, Enum):
    """Supported gripper actions, resolved per gripper hardware variant at call time."""

    OPEN = "open"
    CLOSE = "close"
    STOP = "stop"
    VACUUM_ON = "vacuum_on"
    VACUUM_OFF = "vacuum_off"


class XArmDevice:
    """
    Adapter exposing the arm operations the FEAGI controller and manual control need.

    Args:
        arm: A connected ``XArmAPI`` instance (dependency-injected so tests can pass a
            fake). Construct via :meth:`connect` in production.
    """

    def __init__(self, arm: object) -> None:
        self._arm = arm
        self._dof: Optional[int] = None

    @classmethod
    def connect(cls, ip: str) -> "XArmDevice":
        """
        Open a connection to the arm at ``ip`` and put it in servo-control ready state.

        Raises:
            XArmDeviceError: If the SDK cannot be imported or the arm rejects setup.
        """
        sdk_logger = None
        sdk_logger_level: Optional[int] = None
        try:
            from xarm.wrapper import XArmAPI
            from xarm.core.config.x_config import XCONF
            from xarm.core.utils.log import logger as xarm_sdk_logger
        except ImportError as exc:  # pragma: no cover - import guard
            raise XArmDeviceError(
                "xarm-python-sdk is not installed in the controller environment."
            ) from exc
        control_port = int(getattr(XCONF.SocketConf, "TCP_CONTROL_PORT", 502))
        cls._preflight_socket(ip=ip, port=control_port)
        try:
            # Keep startup logs focused: the SDK emits repeated per-attempt
            # "xArm is not connected" errors while probing firmware version.
            sdk_logger = xarm_sdk_logger
            sdk_logger_level = sdk_logger.level
            sdk_logger.setLevel(logging.CRITICAL)
            arm = XArmAPI(ip)
        except Exception as exc:
            raise XArmDeviceError(cls._format_connect_failure(ip=ip, exc=exc)) from exc
        finally:
            if sdk_logger is not None and sdk_logger_level is not None:
                sdk_logger.setLevel(sdk_logger_level)
        device = cls(arm)
        device.enable()
        set_approx = getattr(arm, "set_allow_approx_motion", None)
        if callable(set_approx):
            set_approx(True)
            logger.info("Singularity bypass (approx motion) enabled.")
        return device

    def enable(self) -> None:
        """Clear any lingering error and (re-)enable position-control mode.

        The xArm SDK requires ``clean_error`` before ``motion_enable`` when the
        arm is in a ControllerError state (codes 22, 24, etc.), otherwise all
        subsequent commands silently return code 1.
        """
        self._arm.clean_error()
        self._arm.motion_enable(enable=True)
        self._arm.set_mode(0)
        self._arm.set_state(state=0)

    @property
    def error_code(self) -> int:
        """Return the arm's current error code (0 means no error)."""
        return int(getattr(self._arm, "error_code", 0))

    @property
    def dof(self) -> int:
        """
        Number of actuated joints, discovered from the connected arm.

        Prefers the SDK ``axis`` attribute; falls back to the length of the reported
        servo-angle vector. Cached after first discovery.
        """
        if self._dof is None:
            axis = getattr(self._arm, "axis", None)
            if isinstance(axis, int) and axis > 0:
                self._dof = axis
            else:
                self._dof = len(self.get_joint_angles())
        return self._dof

    def get_joint_angles(self) -> List[float]:
        """Return current joint angles in degrees (length == dof)."""
        result = self._arm.get_servo_angle()
        angles = self._unwrap(result, "get_servo_angle")
        dof = getattr(self._arm, "axis", None)
        if isinstance(dof, int) and dof > 0:
            return [float(a) for a in angles[:dof]]
        return [float(a) for a in angles]

    def set_joint_angles(self, angles: Sequence[float], *, speed: float) -> None:
        """Command absolute joint angles (degrees) at the given joint speed (deg/s)."""
        code = self._arm.set_servo_angle(angle=list(angles), speed=speed, wait=False)
        self._check(code, "set_servo_angle")

    def jog_joint(self, joint_index: int, delta_deg: float, *, speed: float) -> List[float]:
        """
        Nudge a single joint by ``delta_deg`` degrees relative to its current angle.

        Returns the commanded angle vector. Raises if ``joint_index`` is out of range.
        """
        angles = self.get_joint_angles()
        if not 0 <= joint_index < len(angles):
            raise XArmDeviceError(
                f"joint_index {joint_index} out of range for {len(angles)}-DOF arm."
            )
        angles[joint_index] = angles[joint_index] + float(delta_deg)
        self.set_joint_angles(angles, speed=speed)
        return angles

    def get_cartesian(self) -> List[float]:
        """Return current tool pose ``[x, y, z, roll, pitch, yaw]`` (mm, degrees)."""
        result = self._arm.get_position()
        pose = self._unwrap(result, "get_position")
        return [float(v) for v in pose]

    def move_cartesian(self, pose: Sequence[float], *, speed: float) -> None:
        """Move the tool to absolute pose ``[x, y, z, roll, pitch, yaw]`` (mm, degrees)."""
        if len(pose) != 6:
            raise XArmDeviceError("Cartesian pose must have 6 components [x,y,z,r,p,y].")
        x, y, z, roll, pitch, yaw = (float(v) for v in pose)
        code = self._arm.set_position(
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=speed, wait=False
        )
        self._check(code, "set_position")

    def gripper(self, action: GripperAction) -> None:
        """Actuate the gripper for the given :class:`GripperAction`."""
        if action is GripperAction.OPEN:
            self._check(self._arm.open_lite6_gripper(), "open_lite6_gripper")
        elif action is GripperAction.CLOSE:
            self._check(self._arm.close_lite6_gripper(), "close_lite6_gripper")
        elif action is GripperAction.STOP:
            self._check(self._arm.stop_lite6_gripper(), "stop_lite6_gripper")
        elif action is GripperAction.VACUUM_ON:
            self._check(self._arm.set_vacuum_gripper(on=True), "set_vacuum_gripper")
        elif action is GripperAction.VACUUM_OFF:
            self._check(self._arm.set_vacuum_gripper(on=False), "set_vacuum_gripper")
        else:
            raise XArmDeviceError(f"Unsupported gripper action: {action!r}")

    def set_manual_mode(self) -> None:
        """Switch to manual/teach mode (mode 2) so the operator can move joints by hand."""
        self._arm.set_mode(2)
        self._arm.set_state(state=0)

    def set_position_mode(self) -> None:
        """Switch back to position-control mode (mode 0) from manual/teach mode."""
        self._arm.set_mode(0)
        self._arm.set_state(state=0)

    @property
    def mode(self) -> int:
        """Return the arm's current mode (0 = position, 2 = manual/teach)."""
        return int(getattr(self._arm, "mode", 0))

    def home(self) -> None:
        """Return the arm to its home/reset pose, blocking until complete."""
        self._check(self._arm.reset(wait=True), "reset")

    def is_moving(self) -> bool:
        """Return True while the arm reports an in-progress motion."""
        return bool(self._arm.get_is_moving())

    def emergency_stop(self) -> None:
        """Trigger an immediate hardware emergency stop."""
        self._arm.emergency_stop()

    def recover_after_estop(self) -> None:
        """Re-enable motion after an emergency stop has been cleared by the operator."""
        self.enable()

    def disconnect(self) -> None:
        """Release the connection to the arm."""
        self._arm.disconnect()

    @staticmethod
    def _preflight_socket(*, ip: str, port: int) -> None:
        """Fail fast when the controller IP/port is unreachable from this host."""
        try:
            with socket.create_connection(
                (ip, port), timeout=_XARM_CONNECT_PREFLIGHT_TIMEOUT_S
            ):
                return
        except OSError as exc:
            raise XArmDeviceError(
                XArmDevice._format_socket_connect_failure(ip=ip, port=port, exc=exc)
            ) from exc

    @staticmethod
    def _format_socket_connect_failure(*, ip: str, port: int, exc: OSError) -> str:
        """Build a focused diagnostic for preflight TCP socket failures."""
        raw = str(exc).strip() or exc.__class__.__name__
        err_no = getattr(exc, "errno", None)
        detail = ""
        if err_no in {errno.EHOSTUNREACH, errno.ENETUNREACH}:
            detail = (
                " Network path to the arm is unreachable from this machine. "
                "Verify subnet/routing, cable/Wi-Fi, and that the arm controller is powered on."
            )
        elif err_no == errno.ECONNREFUSED:
            detail = (
                " The arm host is reachable, but TCP control port is refusing connections. "
                "Verify remote API control is enabled on the arm."
            )
        elif (
            err_no == errno.ETIMEDOUT
            or isinstance(exc, TimeoutError)
            or "timed out" in raw.lower()
        ):
            detail = (
                " TCP connect timed out. "
                "Verify the arm IP, that the controller is powered on, "
                "and that this machine can reach it on the control port."
            )
        return f"Unable to reach xArm at {ip}:{port}: {raw}.{detail}"

    @staticmethod
    def _format_connect_failure(*, ip: str, exc: Exception) -> str:
        """Build a concise, user-facing diagnosis for xArm connect failures."""
        raw = str(exc).strip() or exc.__class__.__name__
        lowered = raw.lower()
        details = ""
        if (
            "failed to check version" in lowered
            or "connection reset by peer" in lowered
            or "xarm is not connected" in lowered
        ):
            details = (
                " The arm closed the TCP control session during version handshake. "
                "Another client (for example xArm Studio) may already hold the session, "
                "or the controller is rejecting this SDK connection."
            )
        return f"Unable to connect to xArm at {ip}: {raw}.{details}"

    @staticmethod
    def _unwrap(result: object, op: str) -> Sequence[float]:
        """
        Normalize an xArm ``(code, data)`` tuple, validating the return code.

        Several SDK getters return ``(code, payload)``; raise on non-zero code.
        """
        if isinstance(result, (tuple, list)) and len(result) == 2:
            code, payload = result
            XArmDevice._check(code, op)
            if not isinstance(payload, (tuple, list)):
                raise XArmDeviceError(f"{op} returned non-sequence payload: {payload!r}")
            return payload
        raise XArmDeviceError(f"{op} returned unexpected shape: {result!r}")

    @staticmethod
    def _check(code: object, op: str) -> None:
        """Raise :class:`XArmDeviceError` when an xArm return code is non-zero.

        Some SDK methods (notably ``reset(wait=True)``) return ``None`` on
        success rather than ``0``.  Treat ``None`` as success so callers
        do not raise spuriously.
        """
        if isinstance(code, (tuple, list)):
            code = code[0] if code else None
        if code is not None and code != _XARM_CODE_OK:
            raise XArmDeviceError(f"xArm {op} failed with code {code!r}")
