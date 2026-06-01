#!/usr/bin/env python3
"""ROS 2 ⇄ FEAGI bridge subprocess (Neurorobotics Studio). JSON from Tauri (see RosConnector.tsx)."""

from __future__ import annotations

import argparse
import functools
import json
import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [ROS_BRIDGE] %(levelname)s: %(message)s",
)
logger = logging.getLogger(__name__)


@dataclass
class MappingRow:
    mapping_id: str
    direction: str
    ros_topic: str
    ros_message_type: Optional[str]
    feagi_io_template: str
    device_group_id: int
    channel_id: int
    motor_frame_mode: Optional[str] = None


def _parse_ros_motor_template(
    feagi_io_template: str,
    motor_frame_mode: Optional[str] = None,
) -> Optional[Tuple[str, str]]:
    """
    Map bridge config to (motor_kind, encoding) for the FEAGI Python SDK.

    Studio may send ``motorFrameMode: incremental`` with base template names (for legacy
    bridge scripts) or ``motor:*Incremental`` template strings; both are supported.
    """
    t = feagi_io_template.strip()
    inc_flag = (motor_frame_mode or "").strip().lower() == "incremental"
    if t == "motor:RotaryMotorIncremental":
        return ("rotary", "incremental")
    if t == "motor:PositionalServoIncremental":
        return ("servo", "incremental")
    if t == "motor:RotaryMotor":
        return ("rotary", "incremental" if inc_flag else "absolute")
    if t == "motor:PositionalServo":
        return ("servo", "incremental" if inc_flag else "absolute")
    return None


@dataclass
class VisionBinding:
    """One ROS camera → segmented vision cache (matches MuJoCo register_vision_groups + write path)."""

    mapping_id: str
    group_id: int
    width: int
    height: int
    lock: threading.Lock = field(default_factory=threading.Lock)
    latest_rgb: Any = None


@dataclass
class ProximityBinding:
    """One Float64 topic → Proximity cache (Infrared FEAGI template maps to scalar proximity IPU)."""

    mapping_id: str
    group_id: int
    channel_index: int
    lock: threading.Lock = field(default_factory=threading.Lock)
    latest_scalar_01: Optional[float] = None


@dataclass
class SmartImuBinding:
    mapping_id: str
    group_id: int
    channel_index: int
    lock: threading.Lock = field(default_factory=threading.Lock)
    latest_wxyz: Optional[Tuple[float, float, float, float]] = None


def parse_rows(cfg: Dict[str, Any]) -> List[MappingRow]:
    out: List[MappingRow] = []
    for o in cfg.get("mappings") or []:
        if not isinstance(o, dict):
            continue
        tpl_raw = o.get("feagiIoTemplate", o.get("feagi_io_template", ""))
        dg_raw = o.get("deviceGroupId", o.get("device_group_id", "0"))
        ch_raw = o.get("channelId", o.get("channel_id", "0"))
        mf_raw = o.get("motorFrameMode", o.get("motor_frame_mode"))
        motor_fm: Optional[str] = None
        if mf_raw is not None and str(mf_raw).strip() != "":
            motor_fm = str(mf_raw).strip()
        out.append(
            MappingRow(
                mapping_id=str(o.get("id", "")),
                direction=str(o.get("direction", "")).strip(),
                ros_topic=str(o.get("rosTopic", o.get("ros_topic", ""))).strip(),
                ros_message_type=o.get("rosMessageType", o.get("ros_message_type")),
                feagi_io_template=str(tpl_raw).strip(),
                device_group_id=int(dg_raw) if isinstance(dg_raw, int) else int(str(dg_raw).strip()),
                channel_id=int(ch_raw) if isinstance(ch_raw, int) else int(str(ch_raw).strip()),
                motor_frame_mode=motor_fm,
            ),
        )
    return out


def servo_publish_units_label() -> str:
    """
    How to publish positional servo angles on std_msgs/Float64.

    SDK ``ServoMotor`` uses **degrees** internally (default bridge range 0–180°). ROS 2 joint
    commands often expect **radians** (REP-103).

    Environment:
        FEAGI_ROS_SERVO_PUBLISH_UNITS: ``degrees`` (default) or ``radians`` / ``si``.
    """
    u = (os.environ.get("FEAGI_ROS_SERVO_PUBLISH_UNITS") or "degrees").strip().lower()
    if u in ("rad", "radian", "radians", "si"):
        return "radians"
    if u not in ("deg", "degree", "degrees", ""):
        logger.warning(
            "Unknown FEAGI_ROS_SERVO_PUBLISH_UNITS=%r; using degrees.",
            os.environ.get("FEAGI_ROS_SERVO_PUBLISH_UNITS"),
        )
    return "degrees"


def servo_angle_sdk_degrees_to_ros_scalar(angle_deg: float, units: str) -> float:
    """Map SDK angle (degrees) to the value published on ROS (degrees or radians)."""
    if units == "radians":
        return float(math.radians(angle_deg))
    return float(angle_deg)


def servo_publish_units_for_mapping(mapping: MappingRow, default_units: str) -> str:
    """
    Units for a specific servo mapping.

    When FEAGI_ROS_SERVO_PUBLISH_UNITS is explicitly set, keep it globally.
    Otherwise, default to radians for ros2_control-style command topics so
    controller inputs follow REP-103 expectations.
    """
    explicit_units = (os.environ.get("FEAGI_ROS_SERVO_PUBLISH_UNITS") or "").strip()
    if explicit_units != "":
        return default_units
    if _topic_suggests_ros2_control_joint_commands(mapping.ros_topic):
        return "radians"
    return default_units


def _topic_suggests_ros2_control_joint_commands(ros_topic: str) -> bool:
    """
    True if ``ros_topic`` looks like a ``ros2_control`` command interface.

    Those controllers typically subscribe with ``std_msgs/msg/Float64MultiArray`` on
    ``<controller>/commands``. Studio often leaves ``rosMessageType`` empty, which
    caused this bridge to default to ``Float64`` and produce no visible motion.
    """
    t = (ros_topic or "").strip().lower()
    if not t or "cmd_vel" in t:
        return False
    if "/state" in t or "joint_states" in t:
        return False
    if "/commands" in t:
        return True
    if t.endswith("/command"):
        return True
    if "position_command" in t or "joint_command" in t:
        return True
    return False


def servo_ros_command_msg_style(mapping: MappingRow) -> str:
    """
    Positional servo command message shape on ROS 2.

    ``ros2_control`` ``joint_group_position_controller`` (and similar) subscribes to
    ``std_msgs/msg/Float64MultiArray`` on ``<controller>/commands`` — one element per joint.
    A ``std_msgs/msg/Float64`` publisher will not match that subscription.

    Resolution (per mapping):
        1. ``rosMessageType`` from Studio topic scan: if it names Float64MultiArray, use it.
        2. Else ``FEAGI_ROS_SERVO_COMMAND_MSG=float64_multiarray`` (or ``float64``).
        3. Else if ``rosTopic`` looks like a ``ros2_control`` command topic, use MultiArray.
        4. Else ``float64`` (single-joint scalar topics).

    Returns:
        ``float64`` or ``float64_multiarray`` (str).
    """
    mt = (mapping.ros_message_type or "").strip().lower()
    compact = mt.replace(" ", "")
    if "float64multiarray" in compact:
        return "float64_multiarray"
    override = (os.environ.get("FEAGI_ROS_SERVO_COMMAND_MSG") or "").strip().lower()
    if override in ("float64_multiarray", "multiarray", "ros2_control"):
        return "float64_multiarray"
    if override in ("float64", "scalar"):
        return "float64"
    if override != "":
        logger.warning(
            "Unknown FEAGI_ROS_SERVO_COMMAND_MSG=%r; using float64 per mapping.",
            os.environ.get("FEAGI_ROS_SERVO_COMMAND_MSG"),
        )
    if _topic_suggests_ros2_control_joint_commands(mapping.ros_topic):
        logger.info(
            "Servo mapping %s: topic %r looks like ros2_control commands; "
            "using std_msgs/Float64MultiArray (set FEAGI_ROS_SERVO_COMMAND_MSG=float64 to force scalar).",
            mapping.mapping_id,
            mapping.ros_topic.strip(),
        )
        return "float64_multiarray"
    return "float64"


def numpy():
    try:
        import numpy as np
    except ImportError:
        logger.error("numpy required")
        sys.exit(1)
    return np


def image_to_rgb(msg: Any, np: Any) -> Any:
    """sensor_msgs/Image -> HWC RGB uint8."""
    h = int(getattr(msg, "height"))
    w = int(getattr(msg, "width"))
    st = int(getattr(msg, "step"))
    raw = bytes(memoryview(getattr(msg, "data")))
    flat = np.frombuffer(raw, dtype=np.uint8)
    k = st // w
    im = flat.reshape((h, w, k))
    enc = str(getattr(msg, "encoding", "")).lower()
    im = np.ascontiguousarray(im)
    if enc.startswith("rgb"):
        return im[..., :3]
    if enc.startswith("bgr"):
        o = np.empty((h, w, 3), dtype=np.uint8)
        o[..., 0], o[..., 1], o[..., 2] = im[..., 2], im[..., 1], im[..., 0]
        return o
    if "mono8" in enc:
        g = im[..., 0]
        return np.stack([g, g, g], axis=-1)
    g = im[..., 0]
    return np.stack([g, g, g], axis=-1)


def resize_nn(rgb: Any, nw: int, nh: int, np: Any) -> Any:
    hh, ww = rgb.shape[:2]
    if hh == nh and ww == nw:
        return rgb
    yi = np.linspace(0, hh - 1, nh).astype(np.int_)
    xi = np.linspace(0, ww - 1, nw).astype(np.int_)
    return rgb[np.ix_(yi, xi)]


def rotary_mapping_publishes_twist(mapping: MappingRow) -> bool:
    """
    geometry_msgs/msg/Twist on cmd_vel-style topics (differential-drive bases).

    RotaryMotor FEAGI output is normalized signed speed (−1..1); Twist.linear.x carries
    linear motion. Optional scale: FEAGI_ROS_CMD_VEL_LINEAR_SCALE (float).
    """
    mt = (mapping.ros_message_type or "").strip().lower()
    if mt != "" and "twist" in mt:
        return True
    t = mapping.ros_topic.strip()
    if not t:
        return False
    return t == "/cmd_vel" or t.endswith("/cmd_vel")


def _servo_angle_to_rotary_compatible_speed(sv: Any) -> float:
    """
    Convert positional-servo state to a normalized speed in [-1, 1].

    This provides rotary-compatible behavior when a positional-servo mapping is
    bound to a Twist/cmd_vel topic: centered angle => 0 speed, min/max =>
    full reverse/forward.
    """
    min_angle = float(getattr(sv, "min_angle", 0.0))
    max_angle = float(getattr(sv, "max_angle", 180.0))
    center = (min_angle + max_angle) / 2.0
    half = max(1e-9, (max_angle - min_angle) / 2.0)
    normalized = (float(sv.get_angle()) - center) / half
    return max(-1.0, min(1.0, normalized))


def _servo_command_to_rotary_compatible_speed(sv: Any) -> float:
    """
    Convert latest servo RX command to rotary-compatible speed in [-1, 1].

    This follows FEAGI motor semantics directly:
    - incremental/absolute values in [0, 1] use 0.5 neutral
    - signed values in [-1, 1] pass through
    Falls back to angle-derived speed when RX diagnostics are unavailable.
    """
    raw_value = getattr(sv, "_last_rx_value", None)
    mode = str(getattr(sv, "_last_rx_mode", "") or "").strip().lower()
    if raw_value is None:
        return _servo_angle_to_rotary_compatible_speed(sv)
    v = float(raw_value)
    if 0.0 <= v <= 1.0:
        speed = (v - 0.5) * 2.0
        return max(-1.0, min(1.0, speed))
    if -1.0 <= v <= 1.0:
        return v
    # Out-of-contract input: preserve deterministic bounded output.
    if mode in ("incremental", "absolute"):
        clamped = max(0.0, min(1.0, v))
        return max(-1.0, min(1.0, (clamped - 0.5) * 2.0))
    return _servo_angle_to_rotary_compatible_speed(sv)


def ros_message_is_nav_odometry(message_type: Optional[str]) -> bool:
    """True if ROS message type is nav_msgs/msg/Odometry (any common spelling)."""
    mt = (message_type or "").strip().lower()
    return "nav_msgs" in mt and "odometry" in mt


def twist_linear_scale_optional() -> Optional[float]:
    """If set and parseable, multiply FEAGI speed onto Twist.linear.x (e.g. m/s)."""
    raw = os.environ.get("FEAGI_ROS_CMD_VEL_LINEAR_SCALE")
    if raw is None:
        return None
    s = str(raw).strip()
    if s == "":
        return None
    try:
        return float(s)
    except ValueError:
        logger.warning(
            "Ignoring invalid FEAGI_ROS_CMD_VEL_LINEAR_SCALE=%r "
            "(using FEAGI speed as Twist.linear.x).",
            raw,
        )
        return None


def servo_incremental_step_ratio_optional() -> Optional[float]:
    """
    Optional override for ``ServoMotor.incremental_step_ratio`` (SDK default ``0.05``).

    Step degrees per neutral-normalized command ≈ ``(max-min)/2 * ratio * 2 * |value-0.5|``
    per FEAGI packet; default full excursion from neutral is ``90 * 0.05 * 2 * 0.5 = 4.5°``
    per edge. Missing or invalid env leaves SDK default.

    Environment:
        FEAGI_ROS_SERVO_INCREMENTAL_STEP_RATIO: positive float (e.g. ``0.1`` for larger steps).
    """
    raw = os.environ.get("FEAGI_ROS_SERVO_INCREMENTAL_STEP_RATIO")
    if raw is None:
        return None
    s = str(raw).strip()
    if s == "":
        return None
    try:
        v = float(s)
    except ValueError:
        logger.warning(
            "Ignoring invalid FEAGI_ROS_SERVO_INCREMENTAL_STEP_RATIO=%r.",
            raw,
        )
        return None
    if v < 0.0:
        logger.warning(
            "Ignoring negative FEAGI_ROS_SERVO_INCREMENTAL_STEP_RATIO=%r.",
            raw,
        )
        return None
    return v


def _normalize_quaternion_xyzw_to_wxyz(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[Tuple[float, float, float, float]]:
    """Match SmartImu ROS contract: return unit (w,x,y,z) with components in [-1,1]."""
    import math

    if not all(math.isfinite(v) for v in (x, y, z, w)):
        return None
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return None
    wn, xn, yn, zn = w / n, x / n, y / n, z / n
    return (
        max(-1.0, min(1.0, wn)),
        max(-1.0, min(1.0, xn)),
        max(-1.0, min(1.0, yn)),
        max(-1.0, min(1.0, zn)),
    )


def _assert_distinct_device_groups(
    vision_groups: List[int],
    proximity_groups: List[int],
    smart_groups: List[int],
) -> None:
    """Studio mappings must not collide: one FEAGI sensory group per modality instance."""
    all_g = [("vision", g) for g in vision_groups]
    all_g += [("proximity", g) for g in proximity_groups]
    all_g += [("smart_imu", g) for g in smart_groups]
    seen: Dict[int, str] = {}
    for label, g in all_g:
        if g in seen:
            raise RuntimeError(
                f"Duplicate device_group_id={g} ({label} conflicts with {seen[g]}). "
                "Use unique deviceGroupId per mapping."
            )
        seen[g] = label


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ROS2⇄FEAGI bridge agent")
    p.add_argument("--config-json", required=True)
    p.add_argument("--feagi-host", required=True)
    p.add_argument("--feagi-api-port", type=int, required=True)
    p.add_argument("--feagi-registration-port", type=int, required=True)
    p.add_argument("--feagi-sensory-port", type=int, required=True)
    p.add_argument("--feagi-motor-port", type=int, required=True)
    p.add_argument("--feagi-http-timeout-s", type=float, required=True)
    p.add_argument("--heartbeat-interval-s", type=float, required=True)
    p.add_argument("--heartbeat-join-timeout-s", type=float, required=True)
    p.add_argument("--connection-timeout-ms", type=int, required=True)
    p.add_argument("--registration-retries", type=int, required=True)
    p.add_argument("--agent-descriptor-manufacturer", required=True)
    p.add_argument("--agent-descriptor-version", type=int, required=True)
    p.add_argument("--agent-descriptor-name", required=True)
    p.add_argument("--agent-descriptor-b64", required=True)
    p.add_argument(
        "--auth-token-b64",
        default=os.environ.get("FEAGI_AUTH_TOKEN_B64"),
    )
    return p


def main() -> int:
    # Log line to confirm staged bundle matches repo (Studio uses
    # ~/.feagi(-staging)/controllers/ros2/<ver>/ or FEAGI_ROS_BRIDGE_SCRIPT).
    logger.info(
        "[ROS_BRIDGE] ros_connector_bridge.py build_id=servo-cmdtopic-20260504 "
        "(MultiArray heuristic for .../commands + device_registrations / ZMQ)"
    )
    args = build_parser().parse_args()
    with open(args.config_json, encoding="utf-8") as f:
        cfg = json.load(f)

    rows = parse_rows(cfg)
    if not rows:
        logger.error("No mappings.")
        return 1

    try:
        import rclpy
    except ImportError as e:
        logger.error(
            "Import failed (%s interpreter): cannot import rclpy: %s. "
            "The bridge must run with ROS 2 on PYTHONPATH — e.g. use the interpreter from "
            "your ROS install, or pip/conda ros packages into this environment per your OS.",
            sys.executable,
            e,
        )
        return 2
    try:
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float64 as FMsg
        from std_msgs.msg import Float64MultiArray as F64MultiMsg
        from geometry_msgs.msg import Twist as TwistMsg
        from nav_msgs.msg import Odometry as OdometryMsg
        from feagi.pns.outputs import RotaryMotor, ServoMotor
        from feagi.pns import brain_output
        from rclpy.executors import SingleThreadedExecutor
    except ImportError as e:
        logger.error(
            "Import failed (%s): %s. Ensure sensor_msgs/std_msgs/nav_msgs ROS packages are on PYTHONPATH "
            "(Studio derives this from your resolved ros2 CLI) and FEAGI PyO3 wheels: Neurorobotics Studio pip-installs "
            "feagi-rust-py-libs into your ROS interpreter on first start unless FEAGI_ROS_CONNECTOR_SKIP_AUTOPPIP=1. "
            "Manual: `%s -m pip install %s`. feagi-python-sdk must be on PYTHONPATH.",
            sys.executable,
            e,
            sys.executable,
            "feagi-rust-py-libs>=0.0.101",
        )
        return 2

    np = numpy()
    sens = [x for x in rows if x.direction == "ros_to_feagi_sensory"]
    mots = [x for x in rows if x.direction == "feagi_to_ros_motor"]

    os.environ.setdefault(
        "FEAGI_AGENT_DESCRIPTOR_B64",
        args.agent_descriptor_b64,
    )

    # vision_units tuples: (modality, w, h, channels, unit_str, group_id) — only for real Vision mappings.
    # SmartIMU / Infrared-only: leave empty so SegmentedVision is not exported in device_registrations
    # (no uncalled-for cortical areas). FeagiAgentClient still injects a minimal scalar_sensory capability
    # tuple for SENSORY agents when vision_units is empty (see feagi.pns.client.configure).
    vision_units: List[Tuple[str, int, int, int, str, int]] = []
    vision_bindings: List[VisionBinding] = []
    proximity_bindings: List[ProximityBinding] = []
    smart_bindings: List[SmartImuBinding] = []

    default_cam_wh = (640, 480)
    for r in sens:
        if r.feagi_io_template == "sensory:Vision":
            w, h = default_cam_wh
            vision_units.append(("camera", w, h, 3, "vision", int(r.device_group_id)))
            vision_bindings.append(
                VisionBinding(
                    mapping_id=r.mapping_id,
                    group_id=int(r.device_group_id),
                    width=w,
                    height=h,
                ),
            )
        elif r.feagi_io_template == "sensory:Infrared":
            proximity_bindings.append(
                ProximityBinding(
                    mapping_id=r.mapping_id,
                    group_id=int(r.device_group_id),
                    channel_index=int(r.channel_id),
                ),
            )
        elif r.feagi_io_template == "sensory:SmartIMU":
            if not ros_message_is_nav_odometry(r.ros_message_type):
                logger.error(
                    "sensory:SmartIMU mapping %s requires rosMessageType nav_msgs/msg/Odometry "
                    "(got %r).",
                    r.mapping_id,
                    r.ros_message_type,
                )
                return 1
            smart_bindings.append(
                SmartImuBinding(
                    mapping_id=r.mapping_id,
                    group_id=int(r.device_group_id),
                    channel_index=int(r.channel_id),
                ),
            )
            logger.info(
                "SmartIMU mapping %s: device_group_id=%s channel_id=%s "
                "(device_registrations use SmartIMU key; MuJoCo-style brain_output.register_sensor_units).",
                r.mapping_id,
                r.device_group_id,
                r.channel_id,
            )
        else:
            logger.error(
                "Sensory mapping %s skipped: feagiIoTemplate=%r is not supported by this "
                "bridge (expected sensory:Vision, sensory:Infrared, or sensory:SmartIMU).",
                r.mapping_id,
                r.feagi_io_template,
            )

    try:
        _assert_distinct_device_groups(
            [t[5] for t in vision_units],
            sorted({b.group_id for b in proximity_bindings}),
            sorted({b.group_id for b in smart_bindings}),
        )
    except RuntimeError as exc:
        logger.error("%s", exc)
        return 1

    rot_l: List[Tuple[MappingRow, Any]] = []
    servo_l: List[Tuple[MappingRow, Any]] = []
    for r in mots:
        parsed = _parse_ros_motor_template(r.feagi_io_template, r.motor_frame_mode)
        if parsed is None:
            logger.error(
                "Motor mapping %s skipped: feagiIoTemplate=%r is not supported by this "
                "bridge script. Re-copy ros_connector_bridge.py from the FEAGI repo "
                "(embodiment-controllers/middleware/ros2/) or re-install the ros2 controller "
                "from Composer so Incremental/Absolute motor variants match Studio.",
                r.mapping_id,
                r.feagi_io_template,
            )
            continue
        motor_kind, enc = parsed
        if motor_kind == "rotary":
            mv = RotaryMotor.register(
                encoding=enc,
                bidirectional=True,
                unit_id=int(r.device_group_id),
                channel_index=int(r.channel_id),
            )
            rot_l.append((r, mv))
        else:
            sv = ServoMotor.register(
                range=(0.0, 180.0),
                encoding=enc,
                unit_id=int(r.device_group_id),
                channel_index=int(r.channel_id),
            )
            step_ratio = servo_incremental_step_ratio_optional()
            if step_ratio is not None:
                sv.incremental_step_ratio = float(step_ratio)
                logger.info(
                    "Servo mapping %s: incremental_step_ratio=%s (FEAGI_ROS_SERVO_INCREMENTAL_STEP_RATIO).",
                    r.mapping_id,
                    sv.incremental_step_ratio,
                )
            servo_l.append((r, sv))

    sensory_parts = bool(vision_units or proximity_bindings or smart_bindings)
    has_motor_out = bool(rot_l or servo_l)

    if not sensory_parts and not has_motor_out:
        logger.error(
            "No sensory mappings and no motor mappings (parsed %d config row(s): "
            "%d ros_to_feagi_sensory, %d feagi_to_ros_motor). "
            "Check feagiIoTemplate values, direction, and that this script matches Studio "
            "(see motor mapping errors above).",
            len(rows),
            len(sens),
            len(mots),
        )
        return 1

    z_neuron = 10
    brain_output.configure(
        agent_id=args.agent_descriptor_b64,
        feagi_host=args.feagi_host,
        feagi_registration_port=int(args.feagi_registration_port),
        feagi_sensory_port=int(args.feagi_sensory_port),
        feagi_motor_port=int(args.feagi_motor_port),
        transport="zmq",
        feagi_connection_timeout_ms=int(args.connection_timeout_ms),
        feagi_registration_retries=int(args.registration_retries),
        feagi_heartbeat_interval_s=float(args.heartbeat_interval_s),
        feagi_api_port=int(args.feagi_api_port),
        feagi_http_timeout_s=float(args.feagi_http_timeout_s),
        auth_token_b64=getattr(args, "auth_token_b64", None),
        vision_units=vision_units,
    )

    if vision_units:
        reg_groups = brain_output.register_vision_groups(vision_units)
        logger.info(
            "[VISION] register_vision_groups -> %s (MuJoCo-style segmented vision cache).",
            reg_groups,
        )

    ir_by_group: Dict[int, List[ProximityBinding]] = {}
    for b in proximity_bindings:
        ir_by_group.setdefault(int(b.group_id), []).append(b)
    for gid in sorted(ir_by_group.keys()):
        binds = ir_by_group[gid]
        max_ch = max(int(x.channel_index) for x in binds) + 1
        gmap = brain_output.register_sensor_units(
            {"Proximity": max_ch},
            z_neuron_resolution=z_neuron,
            group_index_start=int(gid),
        )
        logger.info(
            "[SENSORY] Proximity (Infrared) group %s channels=%s -> cache group %s (%d topics)",
            gid,
            max_ch,
            gmap.get("Proximity"),
            len(binds),
        )

    smart_by_group: Dict[int, List[SmartImuBinding]] = {}
    for b in smart_bindings:
        smart_by_group.setdefault(b.group_id, []).append(b)
    for gid, binds in sorted(smart_by_group.items()):
        max_ch = max(int(x.channel_index) for x in binds) + 1
        gmap = brain_output.register_sensor_units(
            {"SmartIMU": max_ch},
            z_neuron_resolution=z_neuron,
            group_index_start=int(gid),
        )
        logger.info(
            "[SENSORY] SmartIMU group %s channels=%s -> cache group %s",
            gid,
            max_ch,
            gmap.get("SmartIMU"),
        )

    logger.info("[CONN] brain_output.connect() (single ZMQ agent + device_registrations)...")
    brain_output.connect()
    logger.info("   [OK] FEAGI connection established (brain_output).")

    rclpy.init()
    node = rclpy.create_node("feagi_ros_connector")

    for vb in vision_bindings:
        cb = functools.partial(on_image_vision_binding, vb, np)
        topic = next(
            (r.ros_topic for r in sens if r.mapping_id == vb.mapping_id),
            "",
        )
        if not topic.strip() and vb.mapping_id != "__dummy_vision__":
            logger.warning("Vision binding %s missing topic.", vb.mapping_id)
            continue
        if vb.mapping_id != "__dummy_vision__":
            node.create_subscription(Image, topic.strip(), cb, 10)

    for pb in proximity_bindings:
        topic = next(
            (r.ros_topic for r in sens if r.mapping_id == pb.mapping_id),
            "",
        ).strip()
        cb = functools.partial(on_float_proximity_binding, pb)
        node.create_subscription(FMsg, topic, cb, 10)

    for sb in smart_bindings:
        topic = next(
            (r.ros_topic for r in sens if r.mapping_id == sb.mapping_id),
            "",
        ).strip()
        cb = functools.partial(on_odometry_smart_imu_binding, sb)
        node.create_subscription(OdometryMsg, topic, cb, 10)
        logger.info(
            "SmartIMU mapping %s: nav_msgs/Odometry pose.orientation -> FEAGI cache (brain_output).",
            sb.mapping_id,
        )

    twist_lin_scale = twist_linear_scale_optional()
    if twist_lin_scale is not None:
        logger.info(
            "FEAGI_ROS_CMD_VEL_LINEAR_SCALE=%s (Twist.linear.x = FEAGI_speed * scale).",
            twist_lin_scale,
        )

    servo_ros_units_default = servo_publish_units_label()
    servo_units_by_mapping_id: Dict[str, str] = {}

    pubs: List[Tuple[MappingRow, Any, Any, str]] = []
    for r, mv in rot_l:
        topic_nm = r.ros_topic.strip()
        if rotary_mapping_publishes_twist(r):
            pubs.append(
                (
                    r,
                    mv,
                    node.create_publisher(TwistMsg, topic_nm, 10),
                    "rotary_twist",
                ),
            )
            logger.info(
                "Motor rotary %s publishes geometry_msgs/msg/Twist on %s "
                "(angular.z fixed 0 single-channel); set FEAGI_ROS_CMD_VEL_LINEAR_SCALE if needed.",
                r.mapping_id,
                topic_nm,
            )
        else:
            pubs.append(
                (
                    r,
                    mv,
                    node.create_publisher(FMsg, topic_nm, 10),
                    "rotary",
                ),
            )
    for r, sv in servo_l:
        cmd_style = servo_ros_command_msg_style(r)
        servo_units_by_mapping_id[r.mapping_id] = servo_publish_units_for_mapping(
            r,
            servo_ros_units_default,
        )
        topic_s = r.ros_topic.strip()
        if rotary_mapping_publishes_twist(r):
            pubs.append(
                (
                    r,
                    sv,
                    node.create_publisher(TwistMsg, topic_s, 10),
                    "servo_twist",
                ),
            )
            logger.info(
                "Motor positional servo %s publishes geometry_msgs/msg/Twist on %s "
                "(rotary-compatible speed from centered servo angle).",
                r.mapping_id,
                topic_s,
            )
            continue
        if cmd_style == "float64_multiarray":
            pubs.append(
                (
                    r,
                    sv,
                    node.create_publisher(F64MultiMsg, topic_s, 10),
                    "servo_multiarray",
                ),
            )
            logger.info(
                "Motor positional servo %s publishes std_msgs/Float64MultiArray on %s "
                "(one joint; ros2_control position controllers). rosMessageType=%r.",
                r.mapping_id,
                topic_s,
                r.ros_message_type,
            )
        else:
            pubs.append(
                (
                    r,
                    sv,
                    node.create_publisher(FMsg, topic_s, 10),
                    "servo",
                ),
            )
            logger.info(
                "Motor positional servo %s publishes std_msgs/Float64 on %s (rosMessageType=%r).",
                r.mapping_id,
                topic_s,
                r.ros_message_type,
            )
    if servo_l:
        default_units_effective = (
            "explicit via FEAGI_ROS_SERVO_PUBLISH_UNITS"
            if (os.environ.get("FEAGI_ROS_SERVO_PUBLISH_UNITS") or "").strip() != ""
            else "auto (radians on ros2_control-like command topics, degrees otherwise)"
        )
        logger.info(
            "Positional servo: default units mode = %s (%s). "
            "(SDK = degrees; FEAGI_ROS_SERVO_PUBLISH_UNITS forces global override). "
            "If commands are ignored, set FEAGI_ROS_SERVO_COMMAND_MSG=float64_multiarray when the "
            "topic expects std_msgs/Float64MultiArray.",
            servo_ros_units_default,
            default_units_effective,
        )

    # For servo->Twist mappings, publish non-zero only on new FEAGI RX events.
    # This preserves incremental "nudge then stop" semantics on velocity topics.
    servo_twist_last_rx_seq: Dict[str, int] = {}
    servo_twist_last_linear_x: Dict[str, float] = {}

    def spinner() -> None:
        exe = SingleThreadedExecutor()
        exe.add_node(node)
        try:
            while rclpy.ok():
                exe.spin_once(timeout_sec=0.02)
        finally:
            try:
                exe.remove_node(node)
            except Exception:
                pass
            node.destroy_node()

    threading.Thread(target=spinner, daemon=True).start()

    hz = 1.0 / 30.0
    nt = time.monotonic()
    logger.info("Entering bridge loop.")

    try:
        while rclpy.ok():
            now = time.monotonic()
            if sensory_parts and now >= nt:
                for vb in vision_bindings:
                    with vb.lock:
                        frame = vb.latest_rgb
                    if frame is not None:
                        brain_output.write_sensor_vision_frame(
                            group=vb.group_id,
                            channel_index=0,
                            frame_rgb=frame,
                        )
                for pb in proximity_bindings:
                    with pb.lock:
                        s01 = pb.latest_scalar_01
                    if s01 is not None:
                        brain_output.write_sensor_scalar(
                            unit_key="Proximity",
                            group=pb.group_id,
                            channel_index=int(pb.channel_index),
                            scalar_0_1=float(s01),
                        )
                for sb in smart_bindings:
                    with sb.lock:
                        wxyz = sb.latest_wxyz
                    if wxyz is not None:
                        brain_output.write_sensor_smart_imu(
                            group=sb.group_id,
                            channel_index=int(sb.channel_index),
                            quaternion_wxyz=wxyz,
                        )
                brain_output.flush_sensory_bytes()
                nt = now + hz
            if has_motor_out:
                brain_output.receive()
                for _r_row, mv, pub, kind in pubs:
                    if kind == "rotary_twist":
                        spd = float(mv.get_speed())
                        tw = TwistMsg()
                        if twist_lin_scale is not None:
                            tw.linear.x = spd * twist_lin_scale
                        else:
                            tw.linear.x = spd
                        tw.angular.z = 0.0
                        pub.publish(tw)
                    elif kind == "servo_multiarray":
                        units = servo_units_by_mapping_id.get(
                            _r_row.mapping_id,
                            servo_ros_units_default,
                        )
                        scalar = servo_angle_sdk_degrees_to_ros_scalar(
                            float(mv.get_angle()),
                            units,
                        )
                        ma = F64MultiMsg()
                        ma.data = [float(scalar)]
                        pub.publish(ma)
                    elif kind == "servo_twist":
                        mapping_id = _r_row.mapping_id
                        rx_seq = int(getattr(mv, "_rx_command_seq", 0) or 0)
                        prev_seq = servo_twist_last_rx_seq.get(mapping_id, 0)
                        if rx_seq > prev_seq:
                            spd = _servo_command_to_rotary_compatible_speed(mv)
                        else:
                            spd = 0.0
                        tw = TwistMsg()
                        if twist_lin_scale is not None:
                            tw.linear.x = spd * twist_lin_scale
                        else:
                            tw.linear.x = spd
                        tw.angular.z = 0.0
                        prev_linear_x = servo_twist_last_linear_x.get(mapping_id, float("nan"))
                        current_linear_x = float(tw.linear.x)
                        if rx_seq > prev_seq or abs(current_linear_x - prev_linear_x) > 1e-6:
                            logger.info(
                                "[SERVO->TWIST] map=%s mode=%s rx=%s seq=%s linear.x=%.6f",
                                mapping_id,
                                getattr(mv, "_last_rx_mode", None),
                                getattr(mv, "_last_rx_value", None),
                                rx_seq,
                                current_linear_x,
                            )
                        servo_twist_last_rx_seq[mapping_id] = max(prev_seq, rx_seq)
                        servo_twist_last_linear_x[mapping_id] = current_linear_x
                        pub.publish(tw)
                    else:
                        m = FMsg()
                        if kind == "rotary":
                            m.data = float(mv.get_speed())
                        else:
                            units = servo_units_by_mapping_id.get(
                                _r_row.mapping_id,
                                servo_ros_units_default,
                            )
                            m.data = servo_angle_sdk_degrees_to_ros_scalar(
                                float(mv.get_angle()),
                                units,
                            )
                        pub.publish(m)
            time.sleep(0.003)
    except KeyboardInterrupt:
        logger.info("Interrupt.")
    finally:
        try:
            brain_output.disconnect()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    return 0


def on_image_vision_binding(binding: VisionBinding, np: Any, msg: Any) -> None:
    rgb = image_to_rgb(msg, np)
    resized = resize_nn(rgb, binding.width, binding.height, np)
    with binding.lock:
        binding.latest_rgb = resized


def on_float_proximity_binding(binding: ProximityBinding, msg: Any) -> None:
    v = float(msg.data)
    max_cm = 400.0
    if 0.0 <= v <= 1.0:
        cm = v * max_cm
    else:
        cm = v
    scalar_01 = max(0.0, min(1.0, cm / max_cm))
    with binding.lock:
        binding.latest_scalar_01 = scalar_01


def on_odometry_smart_imu_binding(binding: SmartImuBinding, msg: Any) -> None:
    """nav_msgs/Odometry -> pose.pose.orientation (ROS x,y,z,w) -> cache wxyz."""
    pose = getattr(msg, "pose", None)
    if pose is None:
        return
    p = getattr(pose, "pose", None)
    if p is None:
        return
    ori = getattr(p, "orientation", None)
    if ori is None:
        return
    wxyz = _normalize_quaternion_xyzw_to_wxyz(
        float(ori.x),
        float(ori.y),
        float(ori.z),
        float(ori.w),
    )
    if wxyz is None:
        return
    with binding.lock:
        binding.latest_wxyz = wxyz


if __name__ == "__main__":
    raise SystemExit(main())
