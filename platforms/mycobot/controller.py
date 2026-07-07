#!/usr/bin/env python
"""
FEAGI MyCobot Platform Controller - Elephant Robotics arms via the FEAGI Python SDK.

Drives an Elephant Robotics MyCobot 280 (6-DOF) by bridging the FEAGI engine to the
arm through the ``pymycobot`` SDK:

- Motor (FEAGI -> arm): each joint is registered as a positional ``ServoMotor``;
  FEAGI joint targets are applied as absolute servo angles (degrees).
- Sensory (arm -> FEAGI): joint encoder angles are streamed back as a ``Servo``
  proprioception cortical area.

Network and timing parameters are passed explicitly by the launcher (no defaults),
matching the xArm / MuJoCo controller contract and the project's no-hardcoding rule.

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

from joint_map import JointMap, load_joint_map, normalize_angle_to_unit
from mycobot_device import MyCobotDevice

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("mycobot_controller")
# pymycobot logs every serial frame at DEBUG, which floods the console and hides the
# controller's own status. Keep only warnings/errors from the vendor SDK.
logging.getLogger("pymycobot").setLevel(logging.WARNING)

#: FEAGI sensory unit key for joint proprioception feedback.
_SERVO_SENSORY_UNIT: Final[str] = "Servo"
#: Minimum interval (seconds) between repeated tick-failure log lines.
_TICK_ERROR_LOG_INTERVAL_S: Final[float] = 5.0
#: Interval (seconds) between "still alive" heartbeat log lines.
_HEARTBEAT_INTERVAL_S: Final[float] = 5.0


class MyCobotFeagiController:
    """
    Owns the FEAGI SDK session and the arm device.

    The main loop applies FEAGI joint targets and streams joint proprioception back
    to FEAGI every burst period.
    """

    def __init__(
        self,
        *,
        device: MyCobotDevice,
        joint_map: JointMap,
        burst_period_s: float,
        motor_speed: float,
    ) -> None:
        self._device = device
        self._joint_map = joint_map
        self._burst_period_s = burst_period_s
        self._motor_speed = motor_speed
        self._hardware_lock = threading.Lock()
        self._servos: Dict[int, ServoMotor] = {}
        self._last_applied_seq: Dict[int, int] = {}
        self._stop = threading.Event()
        self._tick_fail_count: int = 0
        self._tick_fail_last_log: float = 0.0
        self._last_streamed: list[float] = []

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
        logger.info("[RUN] Entering FEAGI <-> MyCobot control loop.")
        last_heartbeat = time.monotonic()
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
            if cycle_start - last_heartbeat >= _HEARTBEAT_INTERVAL_S:
                logger.info(
                    "[ALIVE] streaming proprioception; joint angles=%s",
                    [round(a, 1) for a in self._last_streamed],
                )
                last_heartbeat = cycle_start
            elapsed = time.monotonic() - cycle_start
            remaining = self._burst_period_s - elapsed
            if remaining > 0:
                self._stop.wait(remaining)

    def _tick(self) -> None:
        """One control cycle: read FEAGI motor commands, apply, then stream sensory."""
        brain_output.receive()
        self._apply_feagi_targets()
        self._stream_proprioception()

    def _apply_feagi_targets(self) -> None:
        """Apply FEAGI joint targets as absolute servo angles (when arm is idle).

        Each mapped joint is commanded individually, so joints not in the joint map
        (e.g. a disabled or broken servo) are never driven. Only joints whose servo
        received a new command since the last applied tick are updated.
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
            for joint in self._joint_map.joints:
                servo = self._servos[joint.index]
                seq = servo._rx_command_seq
                if seq != self._last_applied_seq.get(joint.index, 0):
                    self._device.set_joint_angle(
                        joint.index, servo.get_angle(), speed=self._motor_speed
                    )
                    self._last_applied_seq[joint.index] = seq

    def _stream_proprioception(self) -> None:
        """Read joint encoders and push normalized proprioception scalars to FEAGI."""
        with self._hardware_lock:
            angles = self._device.get_joint_angles()
        self._last_streamed = angles
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
    parser = argparse.ArgumentParser(description="FEAGI MyCobot Platform Controller")
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
    parser.add_argument(
        "--usb-port",
        required=True,
        help="Serial port of the MyCobot (e.g. /dev/cu.usbserial-XXXX, /dev/ttyUSB0, COM3)",
    )
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
        "--motor-speed",
        type=float,
        required=True,
        help="Joint speed applied to FEAGI-driven moves (1..100)",
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
    logger.info("[START] FEAGI MyCobot Controller")
    logger.info("[FEAGI] %s:%s", args.ip, args.port)
    logger.info("[MYCOBOT] %s", args.usb_port)

    device = MyCobotDevice.connect(args.usb_port)
    dof = device.dof
    logger.info("[ARM] Connected to %d-DOF MyCobot", dof)
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

    controller = MyCobotFeagiController(
        device=device,
        joint_map=joint_map,
        burst_period_s=args.feagi_burst_period_s,
        motor_speed=args.motor_speed,
    )
    controller.register_devices()
    brain_output.connect()

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
        try:
            brain_output.disconnect()
        finally:
            device.disconnect()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
