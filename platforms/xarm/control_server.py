#!/usr/bin/env python
"""
Local HTTP control server for manual xARM operation and emergency stop.

The FEAGI desktop app sends manual commands (jog, Cartesian move, gripper, home,
e-stop, streaming-owner selection) to the running controller through this small
loopback HTTP server, mirroring the introspection-endpoint pattern already used by
the MuJoCo controller. The desktop discovers the bound port from a descriptor the
launcher writes, exactly as it does for MuJoCo.

Design:
- :class:`ControlService` contains all command handling and arbitration logic and
  performs no networking, so it is unit-testable with a fake device.
- :class:`ControlServer` is a thin ``http.server`` wrapper (stdlib only, no extra
  dependencies) that decodes JSON requests and delegates to the service.
- All hardware access is serialized with the main loop via a shared lock, except
  emergency stop, which is dispatched immediately so it is never blocked by an
  in-flight motion command.

Copyright 2026 Neuraville Inc.
"""
from __future__ import annotations

import json
import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Final, Tuple

from arbitration import ControlArbiter, ControlOwner
from xarm_device import GripperAction, XArmDevice, XArmDeviceError

logger = logging.getLogger("xarm_controller.control_server")

#: Manual jog/move/gripper actions handled by the service.
_MANUAL_MOTION_ACTIONS: Final[frozenset[str]] = frozenset(
    {"jog", "move_joints", "move_cartesian", "gripper", "home"}
)

#: Lite6 hardware limits (deg/s and mm/s). Source: UFACTORY technical specs.
_MAX_JOINT_SPEED_DEG_S: Final[float] = 180.0
_MAX_TCP_SPEED_MM_S: Final[float] = 500.0


class ControlCommandError(ValueError):
    """Raised for malformed manual commands (maps to HTTP 400)."""


class ControlService:
    """
    Stateless-per-call manual command handler shared by the HTTP server and tests.

    Args:
        device: Hardware adapter for the arm.
        arbiter: Shared control arbiter; manual commands open the manual-hold window
            and emergency stop is enforced here.
        hardware_lock: Lock serializing hardware access against the FEAGI loop.
        manual_speed_deg_s: Joint speed (deg/s) used for jog moves.
        manual_cartesian_speed: Linear speed (mm/s) used for Cartesian moves.
    """

    def __init__(
        self,
        *,
        device: XArmDevice,
        arbiter: ControlArbiter,
        hardware_lock: threading.Lock,
        manual_speed_deg_s: float,
        manual_cartesian_speed: float,
    ) -> None:
        self._device = device
        self._arbiter = arbiter
        self._hw_lock = hardware_lock
        self._manual_speed = float(manual_speed_deg_s)
        self._cartesian_speed = float(manual_cartesian_speed)

    def handle(self, action: str, body: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute one manual command and return a JSON-serializable result payload.

        Raises:
            ControlCommandError: For unknown actions or invalid arguments.
            XArmDeviceError: For hardware-level failures.
        """
        if action == "estop":
            return self._handle_estop()
        if action == "clear_estop":
            self._arbiter.clear_estop()
            with self._hw_lock:
                self._device.recover_after_estop()
            return {"estopped": False}
        if action == "clear_error":
            with self._hw_lock:
                self._device.enable()
            return {"cleared": True}
        if action == "set_manual_mode":
            enabled = bool(body.get("enabled", False))
            with self._hw_lock:
                if enabled:
                    self._device.set_manual_mode()
                else:
                    self._device.set_position_mode()
            return {"manual_mode": enabled}
        if action == "set_owner":
            return self._handle_set_owner(body)
        if action == "status":
            return self._status_payload()
        if action in _MANUAL_MOTION_ACTIONS:
            return self._handle_manual_motion(action, body)
        raise ControlCommandError(f"Unknown action: {action!r}")

    def _handle_estop(self) -> Dict[str, Any]:
        """Engage e-stop immediately, bypassing the hardware lock for responsiveness."""
        self._arbiter.engage_estop()
        # Dispatch the hardware stop without acquiring the loop lock: the e-stop is a
        # dedicated SDK command and must not wait behind an in-flight motion command.
        self._device.emergency_stop()
        return {"estopped": True}

    def _handle_set_owner(self, body: Dict[str, Any]) -> Dict[str, Any]:
        owner_raw = body.get("owner")
        try:
            owner = ControlOwner(str(owner_raw))
        except ValueError as exc:
            raise ControlCommandError(
                f"owner must be one of {[o.value for o in ControlOwner]}"
            ) from exc
        self._arbiter.set_streaming_owner(owner)
        return {"owner": owner.value}

    def _handle_manual_motion(self, action: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if not self._arbiter.begin_manual_command():
            raise ControlCommandError("Arm is emergency-stopped; clear e-stop first.")
        with self._hw_lock:
            try:
                return self._dispatch_motion(action, body)
            except XArmDeviceError:
                try:
                    self._device.enable()
                    logger.info("[CONTROL] Auto-recovered arm after ControllerError.")
                except Exception:  # noqa: BLE001
                    logger.warning("[CONTROL] Auto-recovery failed.")
                raise

    def _dispatch_motion(self, action: str, body: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a single motion command (caller holds ``_hw_lock``)."""
        if action == "jog":
            joint_index = self._require_int(body, "joint_index")
            delta_deg = self._require_float(body, "delta_deg")
            angles = self._device.jog_joint(
                joint_index, delta_deg, speed=min(self._manual_speed, _MAX_JOINT_SPEED_DEG_S)
            )
            return {"action": action, "angles": angles}
        if action == "move_joints":
            angles = body.get("angles")
            if not isinstance(angles, (list, tuple)) or len(angles) == 0:
                raise ControlCommandError("angles must be a non-empty list of joint angles.")
            speed_override = body.get("speed_deg_s")
            speed = (
                float(speed_override)
                if isinstance(speed_override, (int, float)) and not isinstance(speed_override, bool) and speed_override > 0
                else self._manual_speed
            )
            speed = min(speed, _MAX_JOINT_SPEED_DEG_S)
            self._device.set_joint_angles(
                [float(a) for a in angles], speed=speed
            )
            return {"action": action, "angles": [float(a) for a in angles]}
        if action == "move_cartesian":
            pose = body.get("pose")
            if not isinstance(pose, (list, tuple)) or len(pose) != 6:
                raise ControlCommandError("pose must be a 6-element list.")
            self._device.move_cartesian(pose, speed=min(self._cartesian_speed, _MAX_TCP_SPEED_MM_S))
            return {"action": action, "pose": list(pose)}
        if action == "gripper":
            try:
                gripper_action = GripperAction(str(body.get("gripper_action", "")))
            except ValueError as exc:
                raise ControlCommandError(
                    "gripper_action must be open/close/stop/vacuum_on/vacuum_off."
                ) from exc
            self._device.gripper(gripper_action)
            return {"action": action, "gripper_action": gripper_action.value}
        # action == "home"
        self._device.home()
        return {"action": action}

    def _status_payload(self) -> Dict[str, Any]:
        status = self._arbiter.status()
        with self._hw_lock:
            angles = self._device.get_joint_angles()
            cartesian = self._device.get_cartesian()
            dof = self._device.dof
            error_code = self._device.error_code
            arm_mode = self._device.mode
        return {
            "owner": status.owner.value,
            "estopped": status.estopped,
            "manual_hold_active": status.manual_hold_active,
            "manual_hold_remaining_s": round(status.manual_hold_remaining_s, 3),
            "dof": dof,
            "joint_angles_deg": angles,
            "cartesian_pose": cartesian,
            "error_code": error_code,
            "arm_mode": arm_mode,
        }

    @staticmethod
    def _require_int(body: Dict[str, Any], key: str) -> int:
        value = body.get(key)
        if not isinstance(value, int) or isinstance(value, bool):
            raise ControlCommandError(f"{key} must be an integer.")
        return value

    @staticmethod
    def _require_float(body: Dict[str, Any], key: str) -> float:
        value = body.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ControlCommandError(f"{key} must be a number.")
        return float(value)


def _make_request_handler(
    service: ControlService,
) -> "type[BaseHTTPRequestHandler]":
    """Build a request handler class bound to ``service`` (one server per controller)."""

    class _Handler(BaseHTTPRequestHandler):
        # Silence default stderr request logging; controller uses structured logs.
        def log_message(self, *_args: Any) -> None:  # noqa: D401
            return

        def _send_json(self, status: int, payload: Dict[str, Any]) -> None:
            data = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:  # noqa: N802 - stdlib naming
            if self.path.rstrip("/") == "/status":
                self._dispatch("status", {})
            else:
                self._send_json(404, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802 - stdlib naming
            if self.path.rstrip("/") != "/command":
                self._send_json(404, {"error": "not found"})
                return
            length = int(self.headers.get("Content-Length", "0") or "0")
            raw = self.rfile.read(length) if length else b"{}"
            try:
                body = json.loads(raw or b"{}")
            except json.JSONDecodeError:
                self._send_json(400, {"error": "invalid JSON body"})
                return
            if not isinstance(body, dict):
                self._send_json(400, {"error": "body must be a JSON object"})
                return
            self._dispatch(str(body.get("action", "")), body)

        def _dispatch(self, action: str, body: Dict[str, Any]) -> None:
            try:
                result = service.handle(action, body)
                self._send_json(200, {"ok": True, "result": result})
            except ControlCommandError as exc:
                self._send_json(400, {"ok": False, "error": str(exc)})
            except XArmDeviceError as exc:
                logger.error("[CONTROL] Hardware command failed: %s", exc)
                self._send_json(502, {"ok": False, "error": str(exc)})

    return _Handler


class ControlServer:
    """
    Loopback HTTP server hosting :class:`ControlService` on a background thread.

    Args:
        service: Command handler to expose.
        host: Bind address (loopback only by default).
        port: Bind port; pass 0 to let the OS choose an ephemeral port.
    """

    def __init__(self, *, service: ControlService, host: str, port: int) -> None:
        handler_cls = _make_request_handler(service)
        self._httpd = ThreadingHTTPServer((host, port), handler_cls)
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="xarm-control-server",
            daemon=True,
        )

    @property
    def address(self) -> Tuple[str, int]:
        """Return the actual ``(host, port)`` the server is bound to."""
        host, port = self._httpd.server_address[:2]
        return str(host), int(port)

    def start(self) -> None:
        """Start serving requests on the background thread."""
        self._thread.start()

    def stop(self) -> None:
        """Stop the server and release the socket."""
        self._httpd.shutdown()
        self._httpd.server_close()
