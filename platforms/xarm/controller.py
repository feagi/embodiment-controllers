#!/usr/bin/env python
"""
FEAGI xARM Platform Controller - UFACTORY arms via the FEAGI Python SDK.

One controller drives any UFACTORY xArm-series device (Lite6 first) by bridging the
FEAGI engine to the arm through the xArm-Python-SDK:

- Motor (FEAGI -> arm): each joint is registered as a positional ``ServoMotor``;
  FEAGI joint targets are applied as absolute servo angles.
- Sensory (arm -> FEAGI): joint encoder angles are streamed back as a ``Servo``
  proprioception cortical area.
- Manual control + emergency stop: served over a local HTTP control server
  (:mod:`control_server`) and arbitrated against the FEAGI stream so the two never
  command the arm simultaneously (:mod:`arbitration`).

Network and timing parameters are passed explicitly by the launcher (no defaults),
matching the MuJoCo controller contract and the project's no-hardcoding rule.

Copyright 2026 Neuraville Inc.
"""
from __future__ import annotations

import argparse
import logging
import os
import threading
import time
from typing import Dict, Final

from feagi.pns import brain_output
from feagi.pns.outputs import ServoMotor

from arbitration import ControlArbiter
from control_server import ControlServer, ControlService
from joint_map import JointMap, load_joint_map, normalize_angle_to_unit
from xarm_device import XArmDevice

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("xarm_controller")

#: FEAGI sensory unit key for joint proprioception feedback.
_SERVO_SENSORY_UNIT: Final[str] = "Servo"
#: Minimum interval (seconds) between repeated tick-failure log lines.
_TICK_ERROR_LOG_INTERVAL_S: Final[float] = 5.0


class XArmFeagiController:
    """
    Owns the FEAGI SDK session, the arm device, arbiter, and control server.

    The main loop applies FEAGI joint targets (when the arbiter permits) and streams
    joint proprioception back to FEAGI every burst period.
    """

    def __init__(
        self,
        *,
        device: XArmDevice,
        joint_map: JointMap,
        arbiter: ControlArbiter,
        burst_period_s: float,
        motor_speed_deg_s: float,
    ) -> None:
        self._device = device
        self._joint_map = joint_map
        self._arbiter = arbiter
        self._burst_period_s = burst_period_s
        self._motor_speed = motor_speed_deg_s
        self._hardware_lock = threading.Lock()
        self._servos: Dict[int, ServoMotor] = {}
        self._last_applied_seq: Dict[int, int] = {}
        self._stop = threading.Event()
        self._tick_fail_count: int = 0
        self._tick_fail_last_log: float = 0.0

    @property
    def hardware_lock(self) -> threading.Lock:
        """Lock serializing device access between the loop and the control server."""
        return self._hardware_lock

    def register_devices(self) -> None:
        """Register one ServoMotor per joint and the proprioception sensory group."""
        for joint in self._joint_map.joints:
            self._servos[joint.index] = ServoMotor.register(
                range=(joint.range_min_deg, joint.range_max_deg),
                encoding="absolute",
                unit_id=joint.motor_group,
                channel_index=joint.motor_channel,
            )
        brain_output.register_sensor_units(
            {_SERVO_SENSORY_UNIT: len(self._joint_map.joints)},
            z_neuron_resolution=self._joint_map.z_neuron_resolution,
            group_index_start=self._joint_map.proprioception_group,
        )
        logger.info(
            "[REGISTER] %d ServoMotor joints + Servo proprioception group %d",
            len(self._servos),
            self._joint_map.proprioception_group,
        )

    def run(self) -> None:
        """Run the receive/apply/stream loop until stopped or interrupted."""
        logger.info("[RUN] Entering FEAGI <-> xARM control loop.")
        while not self._stop.is_set():
            cycle_start = time.monotonic()
            try:
                self._tick()
                if self._tick_fail_count:
                    logger.info(
                        "[LOOP] Recovered after %d consecutive tick failures.",
                        self._tick_fail_count,
                    )
                    self._tick_fail_count = 0
            except Exception as exc:  # noqa: BLE001 - loop must survive transient faults
                self._tick_fail_count += 1
                now = time.monotonic()
                if now - self._tick_fail_last_log >= _TICK_ERROR_LOG_INTERVAL_S:
                    logger.error(
                        "[LOOP] Tick failed (%d times): %s",
                        self._tick_fail_count,
                        exc,
                    )
                    self._tick_fail_last_log = now
                    self._tick_fail_count = 0
            elapsed = time.monotonic() - cycle_start
            remaining = self._burst_period_s - elapsed
            if remaining > 0:
                self._stop.wait(remaining)

    def _tick(self) -> None:
        """One control cycle: read FEAGI motor commands, apply, then stream sensory."""
        brain_output.receive()
        if self._arbiter.feagi_stream_allowed():
            self._apply_feagi_targets()
        self._stream_proprioception()

    def _apply_feagi_targets(self) -> None:
        """Apply FEAGI joint targets as absolute servo angles (when arm is idle).

        Only joints whose servo has received a new command since the last applied
        tick are updated; joints without fresh commands keep their current
        physical angle, preventing snap-back to the default midpoint.
        """
        has_new = False
        for joint in self._joint_map.joints:
            servo = self._servos[joint.index]
            if servo._rx_command_seq != self._last_applied_seq.get(joint.index, 0):
                has_new = True
                break
        if not has_new:
            return
        with self._hardware_lock:
            if self._device.is_moving():
                return
            angles = self._device.get_joint_angles()
            target = list(angles)
            for joint in self._joint_map.joints:
                servo = self._servos[joint.index]
                seq = servo._rx_command_seq
                if seq != self._last_applied_seq.get(joint.index, 0) and joint.index < len(target):
                    target[joint.index] = servo.get_angle()
                    self._last_applied_seq[joint.index] = seq
            self._device.set_joint_angles(target, speed=self._motor_speed)

    def _stream_proprioception(self) -> None:
        """Read joint encoders and push normalized proprioception scalars to FEAGI."""
        with self._hardware_lock:
            angles = self._device.get_joint_angles()
        for joint in self._joint_map.joints:
            if joint.index >= len(angles):
                continue
            scalar = normalize_angle_to_unit(angles[joint.index], joint)
            brain_output.write_sensor_scalar(
                unit_key=_SERVO_SENSORY_UNIT,
                group=self._joint_map.proprioception_group,
                channel_index=joint.feedback_channel,
                scalar_0_1=scalar,
            )
        brain_output.flush_sensory_bytes()

    def stop(self) -> None:
        """Signal the loop to exit at the next opportunity."""
        self._stop.set()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FEAGI xARM Platform Controller")
    # FEAGI network config - all explicit (no defaults), supplied by the launcher.
    parser.add_argument("--ip", required=True, help="FEAGI API host/IP")
    parser.add_argument("--port", type=int, required=True, help="FEAGI HTTP API port")
    parser.add_argument("--feagi-zmq-motor-port", type=int, required=True)
    parser.add_argument("--feagi-zmq-registration-port", type=int, required=True)
    parser.add_argument("--feagi-zmq-sensory-port", type=int, required=True)
    parser.add_argument("--feagi-zmq-connection-timeout-ms", type=int, required=True)
    parser.add_argument("--feagi-zmq-registration-retries", type=int, required=True)
    parser.add_argument("--feagi-zmq-heartbeat-interval-s", type=float, required=True)
    parser.add_argument("--feagi-http-timeout-s", type=float, required=True)
    parser.add_argument(
        "--agent_id",
        required=True,
        help="Base64 AgentDescriptor (48-byte payload) for FEAGI registration",
    )
    parser.add_argument(
        "--auth-token-b64",
        default=os.environ.get("FEAGI_AUTH_TOKEN_B64"),
        help="Base64 auth token (decodes to 32 bytes). Defaults to FEAGI_AUTH_TOKEN_B64.",
    )
    # Arm + control config.
    parser.add_argument("--xarm-ip", required=True, help="IP address of the xARM")
    parser.add_argument(
        "--joint-map",
        required=True,
        help="Path to the joint-mapping JSON generated by the desktop app",
    )
    parser.add_argument(
        "--feagi-burst-period-s",
        type=float,
        required=True,
        help="Control loop period in seconds (from FEAGI burst config)",
    )
    parser.add_argument(
        "--motor-speed-deg-s",
        type=float,
        required=True,
        help="Joint speed applied to FEAGI-driven moves (deg/s)",
    )
    parser.add_argument(
        "--manual-speed-deg-s",
        type=float,
        required=True,
        help="Joint speed applied to manual jog moves (deg/s)",
    )
    parser.add_argument(
        "--manual-cartesian-speed",
        type=float,
        required=True,
        help="Linear speed applied to manual Cartesian moves (mm/s)",
    )
    parser.add_argument(
        "--manual-hold-settle-s",
        type=float,
        required=True,
        help="Seconds the FEAGI stream stays paused after a manual command",
    )
    parser.add_argument(
        "--control-server-host",
        required=True,
        help="Bind address for the manual-control HTTP server (loopback)",
    )
    parser.add_argument(
        "--control-server-port",
        type=int,
        required=True,
        help="Bind port for the manual-control HTTP server (0 = ephemeral)",
    )
    return parser


def main() -> int:
    """Entry point: connect to FEAGI and the arm, then run the control loop."""
    args = _build_arg_parser().parse_args()
    if not args.auth_token_b64:
        raise RuntimeError(
            "Missing auth token. Provide --auth-token-b64 or set FEAGI_AUTH_TOKEN_B64."
        )

    joint_map = load_joint_map(args.joint_map)
    logger.info("[START] FEAGI xARM Controller")
    logger.info("[FEAGI] %s:%s", args.ip, args.port)
    logger.info("[XARM] %s", args.xarm_ip)

    device = XArmDevice.connect(args.xarm_ip)
    dof = device.dof
    logger.info("[ARM] Discovered %d-DOF arm", dof)
    if len(joint_map.joints) > dof:
        raise RuntimeError(
            f"joint map has {len(joint_map.joints)} joints but arm reports {dof} DOF."
        )

    brain_output.configure(
        agent_id=args.agent_id,
        feagi_host=args.ip,
        feagi_registration_port=args.feagi_zmq_registration_port,
        feagi_sensory_port=args.feagi_zmq_sensory_port,
        feagi_motor_port=args.feagi_zmq_motor_port,
        transport="zmq",
        feagi_connection_timeout_ms=args.feagi_zmq_connection_timeout_ms,
        feagi_registration_retries=args.feagi_zmq_registration_retries,
        feagi_heartbeat_interval_s=args.feagi_zmq_heartbeat_interval_s,
        feagi_api_port=args.port,
        feagi_http_timeout_s=args.feagi_http_timeout_s,
        auth_token_b64=args.auth_token_b64,
    )

    arbiter = ControlArbiter(manual_hold_settle_s=args.manual_hold_settle_s)
    controller = XArmFeagiController(
        device=device,
        joint_map=joint_map,
        arbiter=arbiter,
        burst_period_s=args.feagi_burst_period_s,
        motor_speed_deg_s=args.motor_speed_deg_s,
    )
    controller.register_devices()
    brain_output.connect()

    service = ControlService(
        device=device,
        arbiter=arbiter,
        hardware_lock=controller.hardware_lock,
        manual_speed_deg_s=args.manual_speed_deg_s,
        manual_cartesian_speed=args.manual_cartesian_speed,
    )
    control_server = ControlServer(
        service=service,
        host=args.control_server_host,
        port=args.control_server_port,
    )
    control_server.start()
    bound_host, bound_port = control_server.address
    logger.info("[CONTROL] Manual control server on http://%s:%d", bound_host, bound_port)

    exit_code = 0
    try:
        controller.run()
    except KeyboardInterrupt:
        logger.info("[STOP] Keyboard interrupt; shutting down.")
    except Exception as exc:  # noqa: BLE001 - top-level guard for clean teardown
        logger.error("[FATAL] %s", exc)
        exit_code = 1
    finally:
        controller.stop()
        control_server.stop()
        try:
            brain_output.disconnect()
        finally:
            device.disconnect()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
