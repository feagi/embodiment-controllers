#!/usr/bin/env python
"""
Generic MuJoCo Controller - Using FEAGI Python SDK
Supports any MuJoCo model by passing --model_xml argument
Copyright 2016-2025 Neuraville Inc.
"""
import os
import sys
import time
import argparse
import logging
from typing import Optional
import numpy as np
import mujoco
import mujoco.viewer
import xml.etree.ElementTree as ET
from feagi.pns.outputs import ServoMotor, RotaryMotor
from feagi.pns import brain_output


# Standard logger (keeps controller compatible with released feagi SDK wheels)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("mujoco_controller")

# Configuration
RUNTIME = float('inf')
SPEED = 120


def _resolve_include_path(base_path: str, include_path: str) -> str:
    """Resolve MuJoCo include path relative to the current XML file."""
    if os.path.isabs(include_path):
        return include_path
    base_dir = os.path.dirname(os.path.abspath(base_path))
    return os.path.abspath(os.path.join(base_dir, include_path))


def _iter_mujoco_xml_roots(xml_path: str, visited: set[str]) -> list[ET.Element]:
    """Collect XML roots from the entry file and its nested includes."""
    roots: list[ET.Element] = []
    resolved_path = os.path.abspath(xml_path)
    if resolved_path in visited:
        return roots
    visited.add(resolved_path)
    logger.info("[PARSE] Parsing MuJoCo XML: %s", resolved_path)

    try:
        tree = ET.parse(resolved_path)
    except Exception as e:
        logger.info(f"[WARN] Warning: Could not parse XML '{resolved_path}': {e}")
        return roots

    root = tree.getroot()
    roots.append(root)

    include_elements = root.findall(".//include")
    for include in include_elements:
        include_file = include.get("file")
        if not include_file:
            continue
        include_path = _resolve_include_path(resolved_path, include_file)
        logger.info("[INC] Resolved include: %s", include_path)
        if not os.path.exists(include_path):
            logger.info(
                "[WARN] Include file not found: %s (from %s)",
                include_path,
                resolved_path,
            )
            continue
        roots.extend(_iter_mujoco_xml_roots(include_path, visited))

    return roots


def _sanitize_name(name: str) -> str:
    """Normalize MuJoCo names for cross-platform stability."""
    return name.replace("/", "_").replace("\\", "_")


def _iter_body_elements(root: ET.Element) -> list[ET.Element]:
    """Return top-level body elements for a MuJoCo XML root."""
    if root.tag == "mujocoinclude":
        return root.findall("body")
    worldbody = root.find("worldbody")
    if worldbody is None:
        return []
    return worldbody.findall("body")


def _build_body_hierarchy(roots: list[ET.Element]) -> tuple[dict, dict, dict, dict]:
    """Build body hierarchy and joint mapping from XML roots."""
    body_children: dict[str, list[str]] = {}
    body_parent: dict[str, Optional[str]] = {}
    body_joints: dict[str, list[str]] = {}
    joint_to_body: dict[str, str] = {}

    def visit_body(body: ET.Element, parent_name: Optional[str]) -> None:
        body_name = body.get("name")
        if body_name:
            body_name = _sanitize_name(body_name)
            body_parent.setdefault(body_name, parent_name)
            body_children.setdefault(body_name, [])
            body_joints.setdefault(body_name, [])
        else:
            body_name = parent_name

        for joint in body.findall("joint"):
            joint_name = joint.get("name")
            if not joint_name or body_name is None:
                continue
            joint_name = _sanitize_name(joint_name)
            body_joints[body_name].append(joint_name)
            joint_to_body[joint_name] = body_name

        for child in body.findall("body"):
            child_name = child.get("name")
            if child_name:
                child_name = _sanitize_name(child_name)
                if body_name is not None:
                    body_children.setdefault(body_name, []).append(child_name)
            visit_body(child, body_name)

    for root in roots:
        for body in _iter_body_elements(root):
            visit_body(body, None)

    return body_children, body_parent, body_joints, joint_to_body


def _select_limb_roots(body_children: dict, body_parent: dict) -> list[str]:
    """Select limb root bodies for grouping."""
    root_bodies = sorted(name for name, parent in body_parent.items() if parent is None)
    if not root_bodies:
        return []
    if len(root_bodies) == 1:
        base_body = root_bodies[0]
        child_roots = body_children.get(base_body, [])
        return sorted(child_roots) if child_roots else [base_body]
    return root_bodies


def _map_joints_to_limbs(
    limb_roots: list[str],
    body_children: dict,
    body_joints: dict,
) -> tuple[dict, dict]:
    """Map joint names to limb roots and build ordered joint lists."""
    joint_to_limb: dict[str, str] = {}
    joint_order_by_limb: dict[str, list[str]] = {}

    for limb_root in limb_roots:
        queue = [(limb_root, 0)]
        joints_with_depth: list[tuple[int, str]] = []
        while queue:
            body_name, depth = queue.pop(0)
            for joint_name in body_joints.get(body_name, []):
                joint_to_limb[joint_name] = limb_root
                joints_with_depth.append((depth, joint_name))
            for child_name in body_children.get(body_name, []):
                queue.append((child_name, depth + 1))
        joints_with_depth.sort(key=lambda item: (item[0], item[1]))
        joint_order_by_limb[limb_root] = [name for _, name in joints_with_depth]

    return joint_to_limb, joint_order_by_limb


def parse_actuator_metadata_from_xml(xml_path: str) -> dict:
    """Parse MuJoCo XML (including nested includes) to extract actuator metadata."""
    actuator_metadata: dict[str, dict[str, Optional[str]]] = {}

    try:
        logger.info("Include parsing enabled. Entry XML: %s", os.path.abspath(xml_path))
        roots = _iter_mujoco_xml_roots(xml_path, visited=set())
        logger.info("[PARSE] Total XML roots parsed: %d", len(roots))
        for root in roots:
            logger.info("Root tag: %s", root.tag)
            actuator_section = root.find("actuator")
            if actuator_section is None:
                continue
            counter = 0
            for actuator in actuator_section:
                name = actuator.get("name")
                if not name:
                    name = f"actuator_{counter}"
                    counter += 1
                name = _sanitize_name(name)
                actuator_metadata[name] = {
                    "type": actuator.tag,
                    "joint": actuator.get("joint"),
                }

        logger.info("[PARSED] Parsed %d actuators from XML", len(actuator_metadata))
    except Exception as e:
        logger.info(f"[WARN] Warning: Could not parse XML for actuator metadata: {e}")

    return actuator_metadata


def register_mujoco_motors(model, xml_path, motor_gain: float = 1.0):
    """
    Register FEAGI motors for all MuJoCo actuators with limb grouping.

    Maps MuJoCo actuators to FEAGI ServoMotors or RotaryMotors based on:
    - Range: bounded → ServoMotor, unbounded → RotaryMotor
    - Type: position → absolute encoding, velocity/motor/general → incremental encoding

    Returns:
        tuple: (motors, group_names, group_channels)
    """
    actuator_metadata = parse_actuator_metadata_from_xml(xml_path)
    roots = _iter_mujoco_xml_roots(xml_path, visited=set())
    body_children, body_parent, body_joints, _ = _build_body_hierarchy(roots)
    limb_roots = _select_limb_roots(body_children, body_parent)
    joint_to_limb, joint_order_by_limb = _map_joints_to_limbs(
        limb_roots,
        body_children,
        body_joints,
    )

    actuators_by_joint: dict[str, list[str]] = {}
    ungrouped_actuators: list[str] = []
    for actuator_name, meta in actuator_metadata.items():
        joint_name = meta.get("joint")
        if joint_name:
            joint_name = _sanitize_name(joint_name)
        if joint_name and joint_name in joint_to_limb:
            actuators_by_joint.setdefault(joint_name, []).append(actuator_name)
        else:
            ungrouped_actuators.append(actuator_name)

    group_names = sorted(limb_roots)
    group_actuator_order: dict[str, list[str]] = {}
    for group_name in group_names:
        ordered_actuators: list[str] = []
        for joint_name in joint_order_by_limb.get(group_name, []):
            ordered_actuators.extend(sorted(actuators_by_joint.get(joint_name, [])))
        group_actuator_order[group_name] = ordered_actuators

    if ungrouped_actuators:
        group_names.append("ungrouped")
        group_actuator_order["ungrouped"] = sorted(ungrouped_actuators)

    actuator_details: dict[str, dict[str, object]] = {}
    counter = 0
    logger.info("\n[CFG] Registering %d actuators with FEAGI...", model.nu)
    for i in range(model.nu):
        actuator_name = model.actuator(i).name
        if actuator_name == "" or actuator_name is None:
            actuator_name = f"actuator_{counter}"
            counter += 1
        actuator_name = _sanitize_name(actuator_name)
        ctrl_range = model.actuator_ctrlrange[i]
        min_val, max_val = ctrl_range[0], ctrl_range[1]
        actuator_details[actuator_name] = {
            "index": i,
            "range": (float(min_val), float(max_val)),
        }
        if i < 3:
            logger.info(
                "   [ACT] Actuator '%s' ctrlrange: [%.4f, %.4f] (raw from MuJoCo)",
                actuator_name,
                min_val,
                max_val,
            )

    motors: list[tuple] = []
    group_channels: dict[int, dict[str, list[str]]] = {}

    for group_id, group_name in enumerate(group_names):
        group_channels[group_id] = {"positional_servo": [], "rotary_motor": []}
        for actuator_name in group_actuator_order.get(group_name, []):
            details = actuator_details.get(actuator_name)
            if details is None:
                logger.info(
                    "   [WARN] Actuator '%s' missing in MuJoCo model - skipping",
                    actuator_name,
                )
                continue

            actuator_type = actuator_metadata.get(actuator_name, {}).get("type")
            if actuator_type is None:
                logger.info(
                    "   [WARN] Actuator '%s' has unknown type - skipping",
                    actuator_name,
                )
                continue

            min_val, max_val = details["range"]
            is_bounded = not (np.isinf(min_val) or np.isinf(max_val))

            if actuator_type == "position":
                encoding = "absolute"
            elif actuator_type in ["velocity", "motor", "general"]:
                encoding = "incremental"
            else:
                logger.info(
                    "   [WARN] Actuator '%s' has unsupported type '%s' - skipping",
                    actuator_name,
                    actuator_type,
                )
                continue

            try:
                if is_bounded:
                    channel_index = len(group_channels[group_id]["positional_servo"])
                    motor = ServoMotor.register(
                        range=(float(min_val), float(max_val)),
                        encoding=encoding,
                        unit_id=group_id,
                        channel_index=channel_index,
                    )
                    group_channels[group_id]["positional_servo"].append(actuator_name)
                    device_type = "ServoMotor"
                else:
                    channel_index = len(group_channels[group_id]["rotary_motor"])
                    motor = RotaryMotor.register(
                        encoding=encoding,
                        bidirectional=True,
                        unit_id=group_id,
                        channel_index=channel_index,
                    )
                    group_channels[group_id]["rotary_motor"].append(actuator_name)
                    device_type = "RotaryMotor"

                motors.append(
                    (
                        motor,
                        actuator_name,
                        details["index"],
                        float(min_val),
                        float(max_val),
                        group_id,
                        channel_index,
                        device_type,
                        encoding,
                    )
                )
            except Exception as e:
                logger.info(f"   [FAIL] Failed to register '{actuator_name}': {e}")
                continue

    logger.info("[OK] Registered %d motors with FEAGI\n", len(motors))

    return motors, group_names, group_channels


def _signed_percentage_to_float(val) -> float:
    """Convert feagi_rust_py_libs SignedPercentage to a Python float in [-1.0, 1.0]."""
    if isinstance(val, (int, float)):
        return float(val)

    # Rust bindings expose getters (no implicit float conversion).
    for attr in ("get_as_m1_1", "get_as_m100_100"):
        if hasattr(val, attr):
            fn = getattr(val, attr)
            if callable(fn):
                out = fn()
                if attr == "get_as_m100_100":
                    return float(out) / 100.0
                return float(out)

    raise TypeError(f"Unsupported SignedPercentage type: {type(val)!r}")


def _clamp(value: float, low: float, high: float) -> float:
    """Clamp a numeric value to [low, high]."""
    return max(low, min(high, value))


def main():
    parser = argparse.ArgumentParser(description="Generic MuJoCo Controller for FEAGI")
    # Network config must be explicit (no defaults).
    parser.add_argument("--ip", required=True, help="FEAGI API host/IP")
    parser.add_argument("--port", type=int, required=True, help="FEAGI HTTP API port")
    parser.add_argument(
        "--feagi-zmq-motor-port",
        type=int,
        required=True,
        help="FEAGI ZMQ motor port (from FEAGI network config)",
    )
    parser.add_argument(
        "--feagi-zmq-registration-port",
        type=int,
        required=True,
        help="FEAGI ZMQ registration port (from FEAGI network config)",
    )
    parser.add_argument(
        "--feagi-zmq-sensory-port",
        type=int,
        required=True,
        help="FEAGI ZMQ sensory port (from FEAGI network config)",
    )
    parser.add_argument(
        "--feagi-zmq-connection-timeout-ms",
        type=int,
        required=True,
        help="ZMQ connection timeout in ms (from FEAGI network config)",
    )
    parser.add_argument(
        "--feagi-zmq-registration-retries",
        type=int,
        required=True,
        help="ZMQ registration retries (from FEAGI network config)",
    )
    parser.add_argument(
        "--feagi-zmq-heartbeat-interval-s",
        type=float,
        required=True,
        help="ZMQ heartbeat interval in seconds (from FEAGI network config)",
    )
    parser.add_argument(
        "--feagi-http-timeout-s",
        type=float,
        required=True,
        help="HTTP timeout in seconds for FEAGI API calls (from FEAGI network config)",
    )
    parser.add_argument(
        "--model_xml",
        required=True,
        help="Path to MuJoCo model XML file",
    )
    parser.add_argument(
        "--agent_id",
        required=True,
        help="Base64 AgentDescriptor (48-byte payload) for FEAGI registration",
    )
    parser.add_argument(
        "--auth-token-b64",
        default=os.environ.get("FEAGI_AUTH_TOKEN_B64"),
        help=(
            "Base64 auth token (must decode to 32 bytes). "
            "Defaults to FEAGI_AUTH_TOKEN_B64 env var."
        ),
    )
    parser.add_argument(
        "--cortical_input",
        default="iic400",
        help="Cortical area for sensory input",
    )
    parser.add_argument(
        "--cortical_output",
        default="o_motor",
        help="Cortical area for motor output",
    )
    parser.add_argument(
        '--motor_gain',
        type=float,
        default=10.0,
        help=(
            "Amplification factor for motor commands (default: 10.0). "
            "Use >1.0 for stronger movement."
        ),
    )
    args = parser.parse_args()

    logger.info("[START] Generic MuJoCo Controller (FEAGI Python SDK)")
    logger.info("[SRC] %s", os.path.abspath(__file__))
    logger.info(f"[FEAGI] {args.ip}:{args.port}")
    logger.info(f"[MODEL] {args.model_xml}")
    logger.info(f"[AGENT] {args.agent_id}")
    logger.info(f"[GAIN] Motor Gain: {args.motor_gain}x")
    if not args.auth_token_b64:
        raise RuntimeError(
            "Missing auth token. Provide --auth-token-b64 or set FEAGI_AUTH_TOKEN_B64."
        )

    # Load MuJoCo model from provided path
    try:
        logger.info(f"[LOAD] Loading model from: {args.model_xml}")
        model = mujoco.MjModel.from_xml_path(args.model_xml)
        data = mujoco.MjData(model)
        logger.info(f"[OK] Model loaded: {model.nq} DOF, {model.nu} actuators")
    except Exception as e:
        logger.info(f"[FAIL] Failed to load model '{args.model_xml}': {e}")
        return 1

    # Determine number of actuated joints (skip free joints)
    # Free joints have 7 DOF (3 position, 4 quaternion)
    free_joint_dofs = 0
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
            free_joint_dofs += 7
    
    actuated_joints = model.nu  # Number of actuators
    logger.info(
        "[STATS] Free joint DOFs: %d, Actuated joints: %d",
        free_joint_dofs,
        actuated_joints,
    )

    # Configure brain_output before registering motors. ServoMotor.register()
    # triggers brain_output._init_cache(), which requires agent_id to be set.
    logger.info(
        "[CFG] Configuring FEAGI connection "
        "(agent_id required for motor registration)..."
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

    # Register motors with FEAGI (uses brain_output cache; agent_id must already be set)
    motors, group_names, group_channels = register_mujoco_motors(
        model,
        args.model_xml,
        args.motor_gain,
    )
    _ = group_names
    logger.info("[MAP] Registered motor-channel mapping:")
    for (
        _motor,
        actuator_name,
        actuator_idx,
        min_val,
        max_val,
        group_id,
        channel_index,
        device_type,
        encoding,
    ) in motors:
        logger.info(
            "   [MAP] group=%d channel=%d actuator=%s idx=%d type=%s "
            "encoding=%s range=[%.4f, %.4f]",
            group_id,
            channel_index,
            actuator_name,
            actuator_idx,
            device_type,
            encoding,
            min_val,
            max_val,
        )

    if len(motors) == 0:
        logger.info("[FAIL] No motors registered - cannot connect to FEAGI")
        logger.info("   Aborting startup: FEAGI motor IO is required.")
        return 1
    else:
        feagi_enabled = True

        # brain_output already configured before register_mujoco_motors()
        try:
            # Ensure the motor unit is registered in the Rust ConnectorAgent with the
            # correct channel count BEFORE FEAGI registration.
            #
            # Otherwise FEAGI will create a default 1-channel motor OPU and
            # never reflect the true number of joints.
            cache = getattr(brain_output, "_cache", None)
            if cache is None:
                raise RuntimeError(
                    "brain_output._cache is not initialized after connect()."
                )

            try:
                # Register motor OPUs (PositionalServo / RotaryMotor) with the correct
                # counts.
                # NOTE: z_neuron_resolution=10 matches MotorCorticalUnit template
                # default.
                import feagi_rust_py_libs as frpl

                # NOTE:
                # Attribute access is used instead of dotted imports (e.g.,
                # `from feagi_rust_py_libs.data_structures...`) because some
                # platform builds do not register nested submodules into
                # Python's import system, even though they are accessible as
                # module attributes.
                frame_mode = (
                    frpl.data_structures.genomic.cortical_area
                    .FrameChangeHandling.Absolute()
                )
                positioning = (
                    frpl.data_structures.genomic.cortical_area
                    .PercentageNeuronPositioning.Linear()
                )
                z_neuron_resolution = 10

                for group_id, channels in sorted(group_channels.items()):
                    group_servo_count = len(channels.get("positional_servo", []))
                    group_rotary_count = len(channels.get("rotary_motor", []))
                    if group_servo_count > 0:
                        cache.motor_positional_servo_register(
                            group_id,
                            group_servo_count,
                            frame_mode,
                            z_neuron_resolution,
                            positioning,
                        )
                    if group_rotary_count > 0:
                        cache.motor_rotary_motor_register(
                            group_id,
                            group_rotary_count,
                            frame_mode,
                            z_neuron_resolution,
                            positioning,
                        )
            except Exception as e:
                raise RuntimeError(
                    f"Failed to register motor devices in ConnectorAgent: {e}"
                ) from e

            brain_output._motor_decoder_registered = True

            # Connect directly to FEAGI motor stream (ZMQ SUB) and decode bytes via
            # ConnectorAgent. We avoid brain_output.connect()/receive() because parts
            # of the Python SDK still reference removed Rust types (MotorCorticalType).
            logger.info("[CONN] Connecting motor stream (Rust SDK)...")
            brain_output.connect()
            logger.info("   [OK] Motor stream connected!")
            
        except Exception as e:
            logger.info(f"   [FAIL] FEAGI connection failed: {e}")
            logger.info(
                "   Continuing in standalone mode (viewer only, no FEAGI motor control)"
            )
            feagi_enabled = False

    # Launch MuJoCo viewer
    logger.info("\n[VIEW] Launching MuJoCo viewer...")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # Reset to initial pose
        # For models with keyframes, try to use standing pose (keyframe 4 for humanoid)
        if model.nkey > 4:
            mujoco.mj_resetDataKeyframe(model, data, 4)
        else:
            mujoco.mj_resetData(model, data)

        logger.info("[OK] Viewer running!")
        logger.info("   Press ESC in the viewer window to exit")
        logger.info("   You can manually move joints with the mouse")
        logger.info("   Physics simulation runs at 120 FPS")
        if feagi_enabled:
            logger.info("   FEAGI motor control: ACTIVE")
        else:
            logger.info("   FEAGI motor control: DISABLED (standalone mode)")

        start_time = time.time()
        frame_number = 0
        motor_telemetry_state: dict[tuple[int, int], dict[str, float]] = {}

        while viewer.is_running() and time.time() - start_time < RUNTIME:
            step_start = time.time()

            # Receive motor commands from FEAGI
            changed_in_tick = 0
            group_stats: dict[int, dict[str, float]] = {}
            if feagi_enabled:
                try:
                    brain_output.receive()

                    # Apply FEAGI commands to MuJoCo actuators
                    for motor_idx, (
                        motor,
                        actuator_name,
                        actuator_idx,
                        min_val,
                        max_val,
                        group_id,
                        channel_index,
                        device_type,
                        encoding,
                    ) in enumerate(motors):
                        norm_cmd = 0.0
                        applied_ctrl = 0.0
                        if isinstance(motor, ServoMotor):
                            angle = motor.get_angle()
                            center = (min_val + max_val) / 2.0
                            half_range = (max_val - min_val) / 2.0
                            if half_range > 0:
                                norm_cmd = _clamp(
                                    (angle - center) / half_range,
                                    -1.0,
                                    1.0,
                                )
                            # Apply gain in controller (SDK version compatibility:
                            # ServoMotor.register may not support gain)
                            if args.motor_gain != 1.0:
                                angle = center + ((angle - center) * args.motor_gain)
                                angle = max(min_val, min(max_val, angle))
                            data.ctrl[actuator_idx] = angle
                            applied_ctrl = angle
                        elif isinstance(motor, RotaryMotor):
                            speed = motor.get_speed()
                            norm_cmd = _clamp(float(speed), -1.0, 1.0)
                            if args.motor_gain != 1.0:
                                speed = max(-1.0, min(1.0, speed * args.motor_gain))
                            data.ctrl[actuator_idx] = speed
                            applied_ctrl = speed
                        else:
                            continue

                        state_key = (group_id, channel_index)
                        state = motor_telemetry_state.setdefault(
                            state_key,
                            {
                                "last_norm_cmd": norm_cmd,
                                "last_change_frame": 0.0,
                            },
                        )
                        was_changed = abs(norm_cmd - state["last_norm_cmd"]) > 1e-4
                        if was_changed:
                            changed_in_tick += 1
                            state["last_change_frame"] = float(frame_number)
                        state["last_norm_cmd"] = norm_cmd

                        stats = group_stats.setdefault(
                            group_id,
                            {
                                "count": 0.0,
                                "changed": 0.0,
                                "sum_abs": 0.0,
                                "max_abs": 0.0,
                            },
                        )
                        stats["count"] += 1.0
                        stats["sum_abs"] += abs(norm_cmd)
                        stats["max_abs"] = max(stats["max_abs"], abs(norm_cmd))
                        if was_changed:
                            stats["changed"] += 1.0

                        if frame_number % 120 == 0 and motor_idx < 4:
                            stale_frames = (
                                frame_number - int(state["last_change_frame"])
                            )
                            logger.info(
                                "   [MOTOR-CH] g=%d c=%d type=%s enc=%s act=%s "
                                "norm=%.4f ctrl=%.4f stale_frames=%d",
                                group_id,
                                channel_index,
                                device_type,
                                encoding,
                                actuator_name,
                                norm_cmd,
                                applied_ctrl,
                                stale_frames,
                            )
                except Exception as e:
                    if frame_number % 120 == 0:
                        logger.info(f"   [WARN] FEAGI receive error: {e}")
                        import traceback
                        traceback.print_exc()

            # Step simulation
            mujoco.mj_step(model, data)

            # Log every 120 frames (1 second at 120Hz)
            if frame_number % 120 == 0:
                elapsed = time.time() - start_time
                mode = "FEAGI" if feagi_enabled else "Standalone"
                logger.info(
                    "[FRAME] Frame %d | Time: %.1fs | Mode: %s",
                    frame_number,
                    elapsed,
                    mode,
                )
                if feagi_enabled:
                    global_max_abs = 0.0
                    for stats in group_stats.values():
                        global_max_abs = max(global_max_abs, stats["max_abs"])
                    logger.info(
                        "   [TELE] changed=%d/%d global_max_abs_norm=%.4f",
                        changed_in_tick,
                        len(motors),
                        global_max_abs,
                    )
                    for group_id, stats in sorted(group_stats.items()):
                        count = max(1.0, stats["count"])
                        logger.info(
                            (
                                "   [TELE][GROUP %d] changed=%d/%d "
                                "mean_abs=%.4f max_abs=%.4f"
                            ),
                            group_id,
                            int(stats["changed"]),
                            int(stats["count"]),
                            stats["sum_abs"] / count,
                            stats["max_abs"],
                        )
                    stale_sorted = sorted(
                        (
                            (
                                frame_number - int(state["last_change_frame"]),
                                group_id,
                                channel_index,
                                state["last_norm_cmd"],
                            )
                            for (group_id, channel_index), state in (
                                motor_telemetry_state.items()
                            )
                        ),
                        reverse=True,
                    )
                    if stale_sorted:
                        top_stale = stale_sorted[:4]
                        stale_text = ", ".join(
                            (
                                f"g{group_id}:c{channel_index}"
                                f"(stale={stale_frames},norm={norm:.4f})"
                            )
                            for stale_frames, group_id, channel_index, norm in top_stale
                        )
                        logger.info("   [TELE][STALE] %s", stale_text)

            # Sync viewer
            viewer.sync()

            # Maintain simulation speed
            elapsed = time.time() - step_start
            sleep_time = (1.0 / SPEED) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            frame_number += 1

        logger.info("\n[STOP] Simulation ended")
        logger.info(f"   Total frames: {frame_number}")
        logger.info(f"   Total time: {time.time() - start_time:.1f}s")

    # Cleanup FEAGI connection
    if feagi_enabled:
        try:
            brain_output.disconnect()
            logger.info("[OK] Disconnected from FEAGI motor stream")
        except Exception as e:
            logger.info(f"[WARN] Error disconnecting: {e}")

    logger.info("[DONE] MuJoCo controller shutdown complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())

