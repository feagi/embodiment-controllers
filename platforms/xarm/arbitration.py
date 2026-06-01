#!/usr/bin/env python
"""
Control arbitration for the FEAGI xARM platform controller.

A physical arm must never be driven by two masters at once. This module owns the
single source of truth for "who may command the arm right now" and enforces three
rules that the main control loop and the HTTP control server both consult:

1. Emergency stop always wins. While engaged, no motion command (FEAGI or manual)
   is allowed until it is explicitly cleared.
2. A manual command (jog / Cartesian / gripper / home) opens a short "manual hold"
   window. While that window is open the FEAGI motor stream is paused so the two
   never fight; the window is refreshed on every manual command and the stream
   resumes automatically after a settle period with no manual activity.
3. When neither of the above applies, the configured streaming owner (FEAGI, and
   later ROS 2) drives the arm.

The arbiter holds no hardware references and performs no I/O, so it is trivially
unit-testable and portable to a future Rust/RTOS implementation (plain state +
monotonic-clock comparisons behind a single lock).

Copyright 2026 Neuraville Inc.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Final, Optional


class ControlOwner(str, Enum):
    """Streaming command source that drives the arm when no manual hold is active."""

    FEAGI = "feagi"
    ROS2 = "ros2"


@dataclass(frozen=True)
class ArbiterStatus:
    """Immutable snapshot of arbiter state for status endpoints and diagnostics."""

    owner: ControlOwner
    estopped: bool
    manual_hold_active: bool
    manual_hold_remaining_s: float


class ControlArbiter:
    """
    Thread-safe arbiter deciding whether FEAGI motor commands may reach the arm.

    Args:
        manual_hold_settle_s: Seconds the FEAGI stream stays paused after the most
            recent manual command. Must be > 0. The window is re-armed on each
            manual command, so continuous manual input keeps the stream paused.
        clock: Monotonic time source in seconds. Injectable for deterministic tests;
            defaults to ``time.monotonic``.
    """

    def __init__(
        self,
        *,
        manual_hold_settle_s: float,
        clock: Optional[Callable[[], float]] = None,
    ) -> None:
        if manual_hold_settle_s <= 0:
            raise ValueError("manual_hold_settle_s must be > 0.")
        import time

        self._settle_s: Final[float] = float(manual_hold_settle_s)
        self._clock: Final[Callable[[], float]] = clock or time.monotonic
        self._lock = threading.Lock()
        self._owner: ControlOwner = ControlOwner.FEAGI
        self._estopped: bool = False
        # Monotonic timestamp until which the FEAGI stream stays paused (manual hold).
        self._manual_hold_until: float = 0.0

    def set_streaming_owner(self, owner: ControlOwner) -> None:
        """Select which streaming source drives the arm when no manual hold is active."""
        with self._lock:
            self._owner = owner

    def engage_estop(self) -> None:
        """Engage emergency stop. Blocks all motion (FEAGI and manual) until cleared."""
        with self._lock:
            self._estopped = True
            # Drop any manual hold; e-stop supersedes everything.
            self._manual_hold_until = 0.0

    def clear_estop(self) -> None:
        """Release emergency stop so commands are honored again."""
        with self._lock:
            self._estopped = False

    def is_estopped(self) -> bool:
        """Return True while emergency stop is engaged."""
        with self._lock:
            return self._estopped

    def begin_manual_command(self) -> bool:
        """
        Register a manual command and (re)arm the manual-hold window.

        Returns:
            True if the manual command may proceed (not e-stopped), else False.
            E-stop does not arm a hold window because no motion is permitted.
        """
        with self._lock:
            if self._estopped:
                return False
            self._manual_hold_until = self._clock() + self._settle_s
            return True

    def feagi_stream_allowed(self) -> bool:
        """
        Return True when FEAGI motor commands may be applied to the arm.

        False while e-stopped, while a manual-hold window is open, or when the
        active streaming owner is not FEAGI.
        """
        with self._lock:
            if self._estopped or self._owner is not ControlOwner.FEAGI:
                return False
            return self._clock() >= self._manual_hold_until

    def status(self) -> ArbiterStatus:
        """Return an immutable snapshot for status/diagnostics endpoints."""
        with self._lock:
            now = self._clock()
            remaining = max(0.0, self._manual_hold_until - now)
            return ArbiterStatus(
                owner=self._owner,
                estopped=self._estopped,
                manual_hold_active=remaining > 0.0,
                manual_hold_remaining_s=remaining,
            )
