#!/usr/bin/env python
"""
Thin hardware adapter around the Elephant Robotics ``pymycobot`` SDK.

The adapter isolates every direct call into the vendor SDK behind a small, typed
surface so the FEAGI control loop and the unit tests never import or depend on
``pymycobot`` directly. This keeps the controller logic hardware-agnostic and
testable (the SDK is outside the domain under test, so it is the one object tests
are permitted to substitute).

Joint angles are expressed in degrees throughout, matching the MyCobot SDK
``get_angles``/``send_angles`` convention (the SDK also exposes raw encoder units;
this adapter deliberately works in degrees so the FEAGI mapping mirrors the xArm
controller).

Copyright 2026 Neuraville Inc.
"""
from __future__ import annotations

import logging
import time
from typing import Final, List, Optional, Sequence

logger = logging.getLogger("mycobot_controller.device")

#: MyCobot 280 has six actuated joints (servo ids 1..6 on the wire).
_MYCOBOT_DOF: Final[int] = 6
#: Default serial baud rate for the MyCobot 280 control board.
_MYCOBOT_BAUD: Final[int] = 115200
#: The MyCobot's first serial read(s) after connect frequently return empty; poll
#: up to this long during warm-up before declaring the arm unresponsive.
_WARMUP_TIMEOUT_S: Final[float] = 10.0
#: Delay between warm-up read attempts (seconds).
_WARMUP_POLL_S: Final[float] = 0.4


class MyCobotDeviceError(RuntimeError):
    """Raised when the MyCobot SDK cannot connect or returns an invalid reading."""


class MyCobotDevice:
    """
    Adapter exposing the arm operations the FEAGI controller needs.

    Args:
        arm: A connected ``MyCobot`` instance (dependency-injected so tests can pass
            a fake). Construct via :meth:`connect` in production.
    """

    def __init__(self, arm: object) -> None:
        self._arm = arm
        self._dof: Optional[int] = None
        self._last_angles: Optional[List[float]] = None

    @classmethod
    def connect(cls, usb_port: str, *, baud: int = _MYCOBOT_BAUD) -> "MyCobotDevice":
        """
        Open a serial connection to the arm on ``usb_port`` and ready it for control.

        Raises:
            MyCobotDeviceError: If the SDK cannot be imported or the arm rejects setup.
        """
        try:
            from pymycobot.mycobot import MyCobot
        except ImportError as exc:  # pragma: no cover - import guard
            raise MyCobotDeviceError(
                "pymycobot is not installed in the controller environment."
            ) from exc
        # pymycobot logs every serial frame at DEBUG; keep the console readable.
        logging.getLogger("pymycobot").setLevel(logging.WARNING)
        try:
            arm = MyCobot(usb_port, baud)
        except Exception as exc:
            raise MyCobotDeviceError(
                f"Unable to open MyCobot on {usb_port} @ {baud} baud: {exc}"
            ) from exc
        device = cls(arm)
        device.enable()
        return device

    def enable(self) -> None:
        """Power the servos and confirm the arm answers before control begins."""
        focus = getattr(self._arm, "power_on", None)
        if callable(focus):
            focus()
        # The MyCobot's first read(s) after connect/power_on routinely return empty
        # (empirically: attempt 0 == [], attempt 1 == real angles). Warm up with a
        # bounded poll until we get a full joint vector before trusting the arm.
        deadline = time.monotonic() + _WARMUP_TIMEOUT_S
        while time.monotonic() < deadline:
            try:
                if len(self.get_joint_angles()) >= _MYCOBOT_DOF:
                    return
            except MyCobotDeviceError:
                pass
            time.sleep(_WARMUP_POLL_S)
        raise MyCobotDeviceError(
            f"MyCobot returned no joint data within {_WARMUP_TIMEOUT_S:.0f}s. Check that "
            "the USB cable carries data (not power-only), the arm is powered on, and it "
            "is running Transponder (Basic) + AtomMain (Atom) firmware at 115200 baud."
        )

    @property
    def dof(self) -> int:
        """Number of actuated joints (fixed at 6 for the MyCobot 280 series)."""
        if self._dof is None:
            self._dof = len(self.get_joint_angles())
        return self._dof

    def get_joint_angles(self) -> List[float]:
        """Return current joint angles in degrees (length == dof).

        MyCobot serial reads intermittently return ``[]``; on a transient empty
        read we reuse the last good vector so the control loop never stalls, and
        only raise if we have never gotten a valid reading.
        """
        result = self._arm.get_angles()
        if isinstance(result, (list, tuple)) and len(result) >= _MYCOBOT_DOF:
            self._last_angles = [float(a) for a in result[:_MYCOBOT_DOF]]
            return list(self._last_angles)
        if self._last_angles is not None:
            return list(self._last_angles)
        raise MyCobotDeviceError(f"get_angles() returned no data: {result!r}")

    def set_joint_angles(self, angles: Sequence[float], *, speed: float) -> None:
        """Command absolute joint angles (degrees) at the given joint speed (1..100)."""
        self._arm.send_angles(list(angles), int(_clamp_speed(speed)))

    def set_joint_angle(self, joint_index: int, angle_deg: float, *, speed: float) -> None:
        """Command a single joint (0-indexed) to an absolute angle in degrees."""
        if not 0 <= joint_index < _MYCOBOT_DOF:
            raise MyCobotDeviceError(
                f"joint_index {joint_index} out of range for {_MYCOBOT_DOF}-DOF arm."
            )
        # pymycobot servo ids are 1-indexed.
        self._arm.send_angle(joint_index + 1, float(angle_deg), int(_clamp_speed(speed)))

    def is_moving(self) -> bool:
        """Return True while the arm reports an in-progress motion.

        ``is_moving`` returns 1 (moving), 0 (idle), or -1 (read error); treat the
        error sentinel as "not moving" so a transient read never blocks a command.
        """
        state = self._arm.is_moving()
        return state == 1

    def release(self) -> None:
        """Relax all servos (used on shutdown so the arm can be moved by hand)."""
        release = getattr(self._arm, "release_all_servos", None)
        if callable(release):
            release()

    def disconnect(self) -> None:
        """Release the servos and drop the serial connection."""
        self.release()
        close = getattr(self._arm, "close", None)
        if callable(close):
            close()


def _clamp_speed(speed: float) -> float:
    """MyCobot speeds are 1..100; clamp to keep the SDK from rejecting the command."""
    return max(1.0, min(100.0, float(speed)))
