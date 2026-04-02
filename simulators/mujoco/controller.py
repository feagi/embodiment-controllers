#!/usr/bin/env python
"""
Generic MuJoCo Controller - Using FEAGI Python SDK
Supports any MuJoCo model by passing --model_xml argument
Copyright 2016-2025 Neuraville Inc.
"""
import hashlib
import base64
import json
import os
import shutil
import sys
import tempfile
import time
import argparse
import logging
import urllib.error
import urllib.request
import importlib.metadata
from dataclasses import dataclass
from typing import Any, Callable, Optional
import numpy as np
import mujoco
import mujoco.viewer
import xml.etree.ElementTree as ET
from feagi.pns.outputs import ServoMotor, RotaryMotor
from feagi.pns import brain_output
import inspect

from motor_ephemeral_utils import motor_rx_is_new_packet as _motor_rx_is_new_packet


# Standard logger (keeps controller compatible with released feagi SDK wheels)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("mujoco_controller")

# Configuration
RUNTIME = float('inf')
SPEED = 120
NAME_MAPPING_FILENAME = "mujoco_feagi_mappings.json"


def _resolve_controller_version_info() -> tuple[str, int | None]:
    """Resolve controller bundle version/build from local manifest when available."""
    manifest_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "manifest.json",
    )
    try:
        with open(manifest_path, "r", encoding="utf-8") as manifest_file:
            manifest_payload = json.load(manifest_file)
        bundle = manifest_payload.get("bundle", {})
        bundle_version = str(bundle.get("version", "unknown"))
        build_number_raw = bundle.get("build_number")
        build_number = int(build_number_raw) if build_number_raw is not None else None
        return bundle_version, build_number
    except Exception:
        return "unknown", None


def _resolve_feagi_sdk_version() -> str:
    """Resolve installed FEAGI Python SDK version."""
    try:
        return importlib.metadata.version("feagi-core")
    except importlib.metadata.PackageNotFoundError:
        try:
            return importlib.metadata.version("feagi")
        except importlib.metadata.PackageNotFoundError:
            return "unknown"
        except Exception:
            return "unknown"
    except Exception:
        return "unknown"


@dataclass(frozen=True)
class MujocoNameTranslator:
    """Translate MuJoCo names via explicit per-model mapping tables."""

    mapping_path: str
    model_joints: dict[str, str]
    model_actuators: dict[str, str]
    model_sensors: dict[str, str]
    source_entities: dict[str, str]
    incremental_step_ratios: dict[str, float]
    motor_control_contracts: dict[str, dict[str, Any]]
    vision_camera_overrides: list[str]
    vision_peripheral_resolution: Optional[tuple[int, int]]

    def translate_joint(self, joint_name: str) -> str:
        return self.model_joints.get(joint_name, joint_name)

    def translate_actuator(self, actuator_name: str) -> str:
        return self.model_actuators.get(actuator_name, actuator_name)

    def translate_sensor(self, sensor_name: str) -> str:
        return self.model_sensors.get(sensor_name, sensor_name)

    def translate_source_entity(self, name: str) -> str:
        if name in self.source_entities:
            return self.source_entities[name]
        if name in self.model_joints:
            return self.translate_joint(name)
        if name in self.model_actuators:
            return self.translate_actuator(name)
        return name

    def incremental_step_ratio_for(
        self,
        actuator_name: str,
        joint_name: str,
        source_entity: str,
    ) -> Optional[float]:
        """Resolve optional per-channel incremental step ratio from mapping table."""
        for key in (actuator_name, joint_name, source_entity):
            if key and key in self.incremental_step_ratios:
                return self.incremental_step_ratios[key]
        return None

    def motor_contract_for(
        self,
        actuator_name: str,
        joint_name: str,
        source_entity: str,
    ) -> dict[str, Any]:
        """Resolve optional per-channel motor control contract from mapping table."""
        for key in (actuator_name, joint_name, source_entity):
            if key and key in self.motor_control_contracts:
                contract = self.motor_control_contracts.get(key)
                if isinstance(contract, dict):
                    return contract
        return {}

    def get_vision_camera_overrides(self) -> list[str]:
        """Return configured explicit vision camera list (ordered, unique)."""
        return list(self.vision_camera_overrides)

    def get_vision_peripheral_resolution(self) -> Optional[tuple[int, int]]:
        """Return optional per-model peripheral segmented vision resolution."""
        if self.vision_peripheral_resolution is None:
            return None
        return tuple(self.vision_peripheral_resolution)


def _load_mujoco_name_translator(model_xml_path: str) -> MujocoNameTranslator:
    """Load optional per-model mapping table next to the model XML entry file."""
    model_entry_path = os.path.abspath(model_xml_path)
    mapping_path = os.path.join(os.path.dirname(model_entry_path), NAME_MAPPING_FILENAME)
    empty_translator = MujocoNameTranslator(
        mapping_path=mapping_path,
        model_joints={},
        model_actuators={},
        model_sensors={},
        source_entities={},
        incremental_step_ratios={},
        motor_control_contracts={},
        vision_camera_overrides=[],
        vision_peripheral_resolution=None,
    )
    if not os.path.isfile(mapping_path):
        logger.info("[NAME_MAP] No per-model mapping found at %s", mapping_path)
        return empty_translator
    try:
        with open(mapping_path, "r", encoding="utf-8") as file_obj:
            payload = json.load(file_obj)
    except Exception as exc:
        logger.info("[NAME_MAP] Failed to load mapping file '%s': %s", mapping_path, exc)
        return empty_translator

    model_joints = payload.get("joints", {})
    model_actuators = payload.get("actuators", {})
    model_sensors = payload.get("sensors", {})
    source_entities = payload.get("source_entities", {})
    raw_incremental_step_ratios = payload.get("incremental_step_ratios", {})
    raw_motor_control_contracts = payload.get("motor_control_contracts", {})
    raw_vision_camera_overrides = payload.get("vision_camera_overrides", [])
    raw_vision_peripheral_resolution = payload.get("vision_peripheral_resolution")
    incremental_step_ratios: dict[str, float] = {}
    if isinstance(raw_incremental_step_ratios, dict):
        for key, value in raw_incremental_step_ratios.items():
            if not isinstance(key, str):
                continue
            try:
                ratio = float(value)
            except (TypeError, ValueError):
                continue
            if not np.isfinite(ratio) or ratio <= 0.0:
                continue
            incremental_step_ratios[key] = ratio
    motor_control_contracts: dict[str, dict[str, Any]] = {}
    if isinstance(raw_motor_control_contracts, dict):
        for key, raw_contract in raw_motor_control_contracts.items():
            if not isinstance(key, str) or not isinstance(raw_contract, dict):
                continue
            parsed_contract: dict[str, Any] = {}
            semantics = raw_contract.get("control_semantics")
            if isinstance(semantics, str) and semantics:
                parsed_contract["control_semantics"] = semantics
            version = raw_contract.get("contract_version")
            if isinstance(version, str) and version:
                parsed_contract["contract_version"] = version
            absolute_scale = raw_contract.get("absolute_command_scale")
            if isinstance(absolute_scale, (int, float)):
                absolute_scale_f = float(absolute_scale)
                if np.isfinite(absolute_scale_f) and absolute_scale_f >= 0.0:
                    parsed_contract["absolute_command_scale"] = absolute_scale_f
            incremental_scale = raw_contract.get("incremental_command_scale")
            if isinstance(incremental_scale, (int, float)):
                incremental_scale_f = float(incremental_scale)
                if np.isfinite(incremental_scale_f) and incremental_scale_f >= 0.0:
                    parsed_contract["incremental_command_scale"] = incremental_scale_f
            ratio = raw_contract.get("incremental_step_ratio")
            if isinstance(ratio, (int, float)):
                ratio_f = float(ratio)
                if np.isfinite(ratio_f) and ratio_f > 0.0:
                    parsed_contract["incremental_step_ratio"] = ratio_f
            if parsed_contract:
                motor_control_contracts[key] = parsed_contract
    vision_camera_overrides: list[str] = []
    if isinstance(raw_vision_camera_overrides, list):
        seen_overrides: set[str] = set()
        for entry in raw_vision_camera_overrides:
            if not isinstance(entry, str):
                continue
            camera_name = _sanitize_name(entry)
            if not camera_name:
                continue
            lowered = camera_name.lower()
            if lowered in seen_overrides:
                continue
            seen_overrides.add(lowered)
            vision_camera_overrides.append(camera_name)
    vision_peripheral_resolution: Optional[tuple[int, int]] = None
    if (
        isinstance(raw_vision_peripheral_resolution, list)
        and len(raw_vision_peripheral_resolution) == 2
    ):
        width_raw, height_raw = raw_vision_peripheral_resolution
        if isinstance(width_raw, (int, float)) and isinstance(height_raw, (int, float)):
            width = int(width_raw)
            height = int(height_raw)
            if width > 0 and height > 0:
                vision_peripheral_resolution = (width, height)

    translator = MujocoNameTranslator(
        mapping_path=mapping_path,
        model_joints=model_joints if isinstance(model_joints, dict) else {},
        model_actuators=model_actuators if isinstance(model_actuators, dict) else {},
        model_sensors=model_sensors if isinstance(model_sensors, dict) else {},
        source_entities=source_entities if isinstance(source_entities, dict) else {},
        incremental_step_ratios=incremental_step_ratios,
        motor_control_contracts=motor_control_contracts,
        vision_camera_overrides=vision_camera_overrides,
        vision_peripheral_resolution=vision_peripheral_resolution,
    )
    logger.info(
        "[NAME_MAP] Loaded per-model mappings from %s (joints=%d actuators=%d sensors=%d sources=%d incremental_scales=%d contracts=%d vision_overrides=%d)",
        mapping_path,
        len(translator.model_joints),
        len(translator.model_actuators),
        len(translator.model_sensors),
        len(translator.source_entities),
        len(translator.incremental_step_ratios),
        len(translator.motor_control_contracts),
        len(translator.vision_camera_overrides),
    )
    return translator


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


def _iter_mujoco_xml_roots_with_paths(
    xml_path: str, visited: set[str]
) -> list[tuple[ET.Element, str, ET.ElementTree]]:
    """
    Collect (root, resolved_path, tree) from entry file and includes.
    Used for patching actuators; tree is needed to write back.
    """
    result: list[tuple[ET.Element, str, ET.ElementTree]] = []
    resolved_path = os.path.abspath(xml_path)
    if resolved_path in visited:
        return result
    visited.add(resolved_path)

    try:
        tree = ET.parse(resolved_path)
    except Exception as e:
        logger.info("[WARN] Could not parse XML for ctrlrange patch '%s': %s", resolved_path, e)
        return result

    root = tree.getroot()
    result.append((root, resolved_path, tree))

    for include in root.findall(".//include"):
        include_file = include.get("file")
        if not include_file:
            continue
        include_path = _resolve_include_path(resolved_path, include_file)
        if not os.path.exists(include_path):
            continue
        result.extend(_iter_mujoco_xml_roots_with_paths(include_path, visited))

    return result


def _build_joint_map_from_roots(roots: list[ET.Element]) -> dict[str, dict[str, str]]:
    """Build joint_name -> {range, actuatorfrcrange} from XML roots."""
    joint_map: dict[str, dict[str, str]] = {}

    def visit_body(body: ET.Element) -> None:
        for joint in body.findall("joint"):
            joint_name = joint.get("name")
            if not joint_name:
                continue
            joint_name = _sanitize_name(joint_name)
            joint_map[joint_name] = {
                "range": joint.get("range", ""),
                "actuatorfrcrange": joint.get("actuatorfrcrange", ""),
            }
        for child in body.findall("body"):
            visit_body(child)

    for root in roots:
        worldbody = root.find("worldbody")
        if worldbody is None:
            continue
        for body in worldbody.findall("body"):
            visit_body(body)

    return joint_map


def _infer_ctrlrange(
    actuator_type: str,
    joint_name: str,
    joint_map: dict[str, dict[str, str]],
) -> tuple[float, float]:
    """
    Infer ctrlrange when model reports [0,0].
    velocity: rad/s; position: joint range; motor/general: joint actuatorfrcrange.
    """
    joint_info = joint_map.get(joint_name, {}) if joint_name else {}
    range_str = joint_info.get("range", "")
    frc_str = joint_info.get("actuatorfrcrange", "")

    if actuator_type == "velocity":
        return (-5.0, 5.0)

    if actuator_type == "adhesion":
        # MuJoCo adhesion controls cannot be negative.
        if frc_str:
            parts = frc_str.split()
            if len(parts) >= 2:
                try:
                    lo = max(0.0, float(parts[0]))
                    hi = max(lo, float(parts[1]))
                    if hi == 0.0:
                        hi = 1.0
                    return (lo, hi)
                except ValueError:
                    pass
        return (0.0, 1.0)

    if actuator_type in ("motor", "general"):
        if frc_str:
            parts = frc_str.split()
            if len(parts) >= 2:
                try:
                    return (float(parts[0]), float(parts[1]))
                except ValueError:
                    pass
        return (-1.0, 1.0)

    if actuator_type == "position":
        if range_str:
            parts = range_str.split()
            if len(parts) >= 2:
                try:
                    return (float(parts[0]), float(parts[1]))
                except ValueError:
                    pass
        return (-3.14159, 3.14159)

    return (-1.0, 1.0)


def _needs_ctrlrange_patch(actuator: ET.Element) -> bool:
    """True if actuator has no ctrlrange or ctrlrange is effectively [0,0]."""
    ctrl = actuator.get("ctrlrange", "").strip()
    has_class_inheritance = bool((actuator.get("class") or "").strip())
    if not ctrl:
        # If actuator relies on a default class, ctrlrange may be inherited at
        # compile time. Do not patch these from XML fallback heuristics.
        return not has_class_inheritance
    parts = ctrl.split()
    if len(parts) < 2:
        return True
    try:
        lo, hi = float(parts[0]), float(parts[1])
        return abs(hi - lo) < 1e-9
    except ValueError:
        return True


def _prepare_model_load_path(entry_xml_path: str) -> tuple[str, Optional[str]]:
    """
    Prepare model path for loading. If any actuator has missing/zero ctrlrange,
    copy model dir to temp, patch XML, return (patched_entry_path, temp_dir).
    Otherwise return (entry_xml_path, None).
    On any error, fall back to original path (no patching).
    """
    entry_path = os.path.abspath(entry_xml_path)
    model_dir = os.path.dirname(entry_path)
    entry_basename = os.path.basename(entry_path)

    try:
        if not os.path.isfile(entry_path):
            logger.info("[WARN] Model path is not a file: %s", entry_path)
            return (entry_path, None)
        roots_with_trees = _iter_mujoco_xml_roots_with_paths(entry_path, set())
        if not roots_with_trees:
            return (entry_path, None)
        joint_map = _build_joint_map_from_roots([r for r, _, _ in roots_with_trees])

        needs_patch = False
        for root, _path, _tree in roots_with_trees:
            actuator_section = root.find("actuator")
            if actuator_section is None:
                continue
            for actuator in actuator_section:
                if _needs_ctrlrange_patch(actuator):
                    needs_patch = True
                    break
            if needs_patch:
                break

        if not needs_patch:
            return (entry_path, None)

        path_hash = hashlib.sha256(entry_path.encode()).hexdigest()[:12]
        temp_dir = os.path.join(tempfile.gettempdir(), f"feagi_mujoco_{path_hash}")
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir)
        shutil.copytree(model_dir, temp_dir)
        patched_entry = os.path.join(temp_dir, entry_basename)

        for root, path, tree in roots_with_trees:
            actuator_section = root.find("actuator")
            if actuator_section is None:
                continue
            modified = False
            for actuator in actuator_section:
                if not _needs_ctrlrange_patch(actuator):
                    continue
                joint_name = _sanitize_name(actuator.get("joint", "") or "")
                act_type = actuator.tag
                lo, hi = _infer_ctrlrange(act_type, joint_name, joint_map)
                actuator.set("ctrlrange", f"{lo} {hi}")
                if act_type == "position":
                    # Explicitly disable inheritrange to avoid conflicts with class-level
                    # defaults that may still define inheritrange during MJCF compilation.
                    actuator.set("inheritrange", "0")
                elif "inheritrange" in actuator.attrib:
                    del actuator.attrib["inheritrange"]
                modified = True
                logger.info(
                    "[PATCH] Inferred ctrlrange for '%s' (type=%s): [%.4f, %.4f]",
                    actuator.get("name", "?"),
                    act_type,
                    lo,
                    hi,
                )
            if modified:
                rel_path = os.path.relpath(path, model_dir)
                out_path = os.path.join(temp_dir, rel_path)
                out_dir = os.path.dirname(out_path)
                if out_dir:
                    os.makedirs(out_dir, exist_ok=True)
                tree.write(
                    out_path,
                    default_namespace="",
                    xml_declaration=True,
                    method="xml",
                    encoding="utf-8",
                )
        logger.info("[PATCH] Model with inferred ctrlrange at: %s", patched_entry)
        return (patched_entry, temp_dir)
    except Exception as e:
        logger.info(
            "[WARN] ctrlrange patching failed, using original model: %s",
            e,
            exc_info=True,
        )
        return (entry_path, None)


def _cleanup_mujoco_temp_dirs(
    temp_model_dir: Optional[str],
    keyframe_recovery_dir: Optional[str],
) -> None:
    """Remove temp directories used for ctrlrange patching and/or keyframe stripping."""
    for path in (temp_model_dir, keyframe_recovery_dir):
        if path and os.path.exists(path):
            try:
                shutil.rmtree(path, ignore_errors=True)
            except OSError as exc:
                logger.info("[WARN] Failed to remove temp model dir %s: %s", path, exc)


def _write_mjcf_without_keyframes(src_path: str, dst_path: str) -> int:
    """
    Remove top-level <keyframe> elements from an MJCF file and write to dst_path.

    Returns how many <keyframe> elements were removed. If zero, dst_path is not written.
    """
    tree = ET.parse(src_path)
    root = tree.getroot()
    removed = 0
    for keyframe in list(root.findall("keyframe")):
        root.remove(keyframe)
        removed += 1
    if removed == 0:
        return 0
    abs_dst = os.path.abspath(dst_path)
    parent = os.path.dirname(abs_dst)
    if parent:
        os.makedirs(parent, exist_ok=True)
    tree.write(
        abs_dst,
        default_namespace="",
        xml_declaration=True,
        method="xml",
        encoding="utf-8",
    )
    return removed


def _load_mujoco_model_resilient(
    model_load_path: str,
    temp_model_dir: Optional[str],
) -> tuple[Any, Any, Optional[str]]:
    """
    Load MuJoCo model from XML. On compile failure, retry once after stripping
    invalid <keyframe> sections from the entry MJCF (common when qpos length
    does not match nq). Keyframes in `<include>` files are not modified.

    Returns (MjModel, MjData, keyframe_recovery_temp_dir_or_none).
    """
    abs_load = os.path.abspath(model_load_path)
    logger.info("[LOAD] Loading model from: %s", abs_load)
    first_err: Optional[Exception] = None
    try:
        model = mujoco.MjModel.from_xml_path(abs_load)
        data = mujoco.MjData(model)
        logger.info("[OK] Model loaded: %d DOF, %d actuators", model.nq, model.nu)
        return model, data, None
    except Exception as exc:
        first_err = exc
        logger.warning(
            "[LOAD][WARN] MuJoCo could not compile the model (common causes: keyframe "
            "qpos length vs nq mismatch, bad includes, or schema errors). Error: %s",
            exc,
        )

    keyframe_recovery_dir: Optional[str] = None
    stripped_path: Optional[str] = None
    removed = 0

    if temp_model_dir:
        stripped_path = os.path.join(
            temp_model_dir,
            os.path.splitext(os.path.basename(abs_load))[0] + "__no_keyframes.xml",
        )
        removed = _write_mjcf_without_keyframes(abs_load, stripped_path)
    else:
        model_dir = os.path.dirname(abs_load)
        keyframe_recovery_dir = tempfile.mkdtemp(prefix="feagi_mujoco_kfstrip_")
        shutil.copytree(model_dir, keyframe_recovery_dir, dirs_exist_ok=True)
        stripped_path = os.path.join(keyframe_recovery_dir, os.path.basename(abs_load))
        removed = _write_mjcf_without_keyframes(stripped_path, stripped_path)

    if removed <= 0:
        if keyframe_recovery_dir and os.path.exists(keyframe_recovery_dir):
            shutil.rmtree(keyframe_recovery_dir, ignore_errors=True)
        logger.warning(
            "[LOAD][RECOVER] No <keyframe> in entry XML to remove. If keyframes live only "
            "in included MJCF files, edit those files or fix qpos lengths manually.",
        )
        assert first_err is not None
        raise first_err

    try:
        logger.warning(
            "[LOAD][RECOVER] Removed %d <keyframe> section(s). Named keyframe poses are "
            "unavailable; simulation will use default reset. Retrying load from: %s",
            removed,
            stripped_path,
        )
        assert stripped_path is not None
        model = mujoco.MjModel.from_xml_path(stripped_path)
        data = mujoco.MjData(model)
        logger.info(
            "[OK] Model loaded after keyframe recovery: %d DOF, %d actuators",
            model.nq,
            model.nu,
        )
        return model, data, keyframe_recovery_dir
    except Exception as second_err:
        if keyframe_recovery_dir and os.path.exists(keyframe_recovery_dir):
            shutil.rmtree(keyframe_recovery_dir, ignore_errors=True)
        logger.error(
            "[FAIL] Model load failed even after removing keyframes from entry XML: %s",
            second_err,
        )
        assert first_err is not None
        raise first_err from second_err


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
    actuator_metadata: dict[str, dict[str, Any]] = {}

    try:
        logger.info("Include parsing enabled. Entry XML: %s", os.path.abspath(xml_path))
        roots = _iter_mujoco_xml_roots(xml_path, visited=set())
        logger.info("[PARSE] Total XML roots parsed: %d", len(roots))
        tendon_to_joints: dict[str, list[str]] = {}
        for root in roots:
            tendon_section = root.find("tendon")
            if tendon_section is None:
                continue
            for tendon in tendon_section:
                tendon_name = tendon.get("name")
                if not tendon_name:
                    continue
                resolved_joints: list[str] = []
                for tendon_joint in tendon.findall(".//joint"):
                    joint_name = tendon_joint.get("joint")
                    if not joint_name:
                        continue
                    resolved_joints.append(_sanitize_name(joint_name))
                if resolved_joints:
                    tendon_to_joints[_sanitize_name(tendon_name)] = resolved_joints
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
                joint_name = actuator.get("joint")
                tendon_name = actuator.get("tendon")
                joint_names: list[str] = []
                if joint_name:
                    joint_names = [_sanitize_name(joint_name)]
                elif tendon_name:
                    joint_names = tendon_to_joints.get(_sanitize_name(tendon_name), [])
                source_entity = " + ".join(joint_names) if joint_names else name
                actuator_metadata[name] = {
                    "type": actuator.tag,
                    "joint": joint_names[0] if joint_names else None,
                    "joint_names": joint_names,
                    "tendon": _sanitize_name(tendon_name) if tendon_name else None,
                    "source_entity": source_entity,
                }

        logger.info("[PARSED] Parsed %d actuators from XML", len(actuator_metadata))
    except Exception as e:
        logger.info(f"[WARN] Warning: Could not parse XML for actuator metadata: {e}")

    return actuator_metadata


@dataclass(frozen=True)
class SensorRegistration:
    sensor_name: str
    display_name: str
    sensor_tag: str
    bundle_id: str
    bundle_type: str
    source_entity: str


@dataclass(frozen=True)
class RuntimeSensorChannel:
    sensor_name: str
    sensor_tag: str
    source_kind: str
    start_index: int
    length: int
    normalize_mode: str = "none"
    normalize_reference: float = 0.0
    normalize_min: float = 0.0
    normalize_max: float = 0.0


@dataclass(frozen=True)
class JointLimitGuard:
    joint_name: str
    qpos_addr: int
    qvel_addr: int
    lower: float
    upper: float


def parse_sensor_metadata_from_xml(xml_path: str) -> list[dict[str, str]]:
    """Parse MuJoCo XML sensor definitions with resolved names and references."""
    sensors: list[dict[str, str]] = []
    try:
        roots = _iter_mujoco_xml_roots(xml_path, visited=set())
        for root in roots:
            sensor_section = root.find("sensor")
            if sensor_section is None:
                continue
            counter = 0
            for sensor in sensor_section:
                sensor_name = sensor.get("name")
                if not sensor_name:
                    sensor_name = f"sensor_{counter}"
                    counter += 1
                sensor_name = _sanitize_name(sensor_name)
                source_entity = (
                    sensor.get("joint")
                    or sensor.get("site")
                    or sensor.get("objname")
                    or sensor_name
                )
                sensors.append(
                    {
                        "name": sensor_name,
                        "tag": sensor.tag,
                        "source_entity": _sanitize_name(source_entity),
                    }
                )
    except Exception as exc:
        logger.info("[WARN] Failed to parse sensor metadata from XML: %s", exc)
    return sensors


def _enum_name_or_value(enum_type: Any, value: int) -> str:
    """Return enum name when available, otherwise the numeric value."""
    try:
        return enum_type(value).name
    except Exception:
        return str(value)


def _sensor_type_to_tag(model, sensor_index: int) -> str:
    """Convert MuJoCo sensor type enum to normalized tag."""
    raw_value = int(model.sensor_type[sensor_index])
    enum_name = _enum_name_or_value(mujoco.mjtSensor, raw_value).lower()
    if enum_name.startswith("mjsens_"):
        enum_name = enum_name[len("mjsens_"):]
    return enum_name


def _resolve_sensor_source_entity(model, sensor_index: int) -> str:
    """Resolve sensor source entity name from MuJoCo object references."""
    try:
        obj_type_value = int(model.sensor_objtype[sensor_index])
        obj_type_name = _enum_name_or_value(mujoco.mjtObj, obj_type_value)
        obj_id = int(model.sensor_objid[sensor_index])
    except Exception:
        return f"sensor_{sensor_index}"

    if obj_id < 0:
        return f"sensor_{sensor_index}"

    try:
        if obj_type_name == "mjOBJ_JOINT":
            return _sanitize_name(model.joint(obj_id).name or f"joint_{obj_id}")
        if obj_type_name == "mjOBJ_GEOM":
            return _sanitize_name(model.geom(obj_id).name or f"geom_{obj_id}")
        if obj_type_name == "mjOBJ_SITE":
            return _sanitize_name(model.site(obj_id).name or f"site_{obj_id}")
        if obj_type_name == "mjOBJ_BODY":
            return _sanitize_name(model.body(obj_id).name or f"body_{obj_id}")
        if obj_type_name == "mjOBJ_CAMERA":
            return _sanitize_name(model.camera(obj_id).name or f"camera_{obj_id}")
        if obj_type_name == "mjOBJ_TENDON":
            return _sanitize_name(model.tendon(obj_id).name or f"tendon_{obj_id}")
        if obj_type_name == "mjOBJ_ACTUATOR":
            return _sanitize_name(model.actuator(obj_id).name or f"actuator_{obj_id}")
    except Exception:
        return f"sensor_{sensor_index}"

    return f"sensor_{sensor_index}"


def _sensor_tag_to_feagi_unit(sensor_tag: str) -> Optional[str]:
    normalized = sensor_tag.lower()
    if normalized in ("rangefinder", "camprojection", "camdistance"):
        return "Vision"
    if normalized in ("framequat", "gyro", "ballquat", "magnetometer"):
        return "Gyroscope"
    if normalized in (
        "distance",
        "geomdist",
        "proximity",
        "touch",
        "velocimeter",
        "jointpos",
        "jointvel",
        "tendonpos",
        "tendonvel",
        "actuatorpos",
        "actuatorvel",
        "framepos",
        "framelinvel",
        "frameangvel",
        "framelinacc",
        "frameangacc",
        "subtreecom",
        "subtreelinvel",
        "subtreeangmom",
    ):
        return "Proximity"
    if normalized in ("accelerometer",):
        return "Shock"
    if normalized in ("force", "torque", "actuatorfrc"):
        return "MiscData"
    return None


def _sensor_tag_to_signal_type(sensor_tag: str) -> str:
    normalized = sensor_tag.lower()
    if normalized in ("rangefinder", "camprojection", "camdistance"):
        return "vision"
    if normalized in ("framequat", "gyro", "magnetometer"):
        return "gyroscope"
    if normalized in (
        "distance",
        "geomdist",
        "proximity",
        "touch",
        "velocimeter",
        "jointpos",
        "jointvel",
        "tendonpos",
        "tendonvel",
        "actuatorpos",
        "actuatorvel",
        "framepos",
        "framelinvel",
        "frameangvel",
        "framelinacc",
        "frameangacc",
        "subtreecom",
        "subtreelinvel",
        "subtreeangmom",
    ):
        return "proximity"
    if normalized in ("accelerometer",):
        return "shock"
    if normalized in ("force", "torque", "actuatorfrc"):
        return "misc_data"
    return normalized


def _sensor_bundle_type(sensor_tag: str) -> str:
    normalized = sensor_tag.lower()
    if normalized in ("rangefinder", "camprojection", "camdistance"):
        return "camera_rig"
    if normalized in ("framequat", "gyro", "accelerometer", "magnetometer"):
        return "imu"
    if normalized in (
        "distance",
        "geomdist",
        "proximity",
        "touch",
        "velocimeter",
        "force",
        "torque",
        "jointpos",
        "jointvel",
        "tendonpos",
        "tendonvel",
        "actuatorpos",
        "actuatorvel",
        "actuatorfrc",
        "framepos",
        "framelinvel",
        "frameangvel",
        "framelinacc",
        "frameangacc",
        "subtreecom",
        "subtreelinvel",
        "subtreeangmom",
    ):
        return "sensor_array"
    return "custom"


def _build_sensor_registration_map(
    model,
    xml_path: str,
    strict_mode: bool = True,
    name_translator: Optional[MujocoNameTranslator] = None,
) -> tuple[dict[str, list[SensorRegistration]], list[RuntimeSensorChannel]]:
    """
    Build deterministic sensory registration entries and runtime channel layout.

    Uses XML metadata for semantic names and MuJoCo runtime metadata for dimensions.
    """
    by_unit: dict[str, list[SensorRegistration]] = {}
    runtime_channels: list[RuntimeSensorChannel] = []
    unsupported_sensor_types: dict[str, int] = {}

    # Runtime-discovered sensors from the compiled MuJoCo model.
    for sensor_index in range(model.nsensor):
        sensor = model.sensor(sensor_index)
        sensor_name_raw = sensor.name
        if not sensor_name_raw:
            sensor_name_raw = f"sensor_{sensor_index}"
        sensor_name = _sanitize_name(sensor_name_raw)
        display_name = sensor_name
        if name_translator is not None:
            display_name = name_translator.translate_sensor(sensor_name)
        sensor_tag = _sensor_type_to_tag(model, sensor_index)
        unit_key = _sensor_tag_to_feagi_unit(sensor_tag)
        if unit_key is None:
            unsupported_sensor_types[sensor_tag] = (
                unsupported_sensor_types.get(sensor_tag, 0) + 1
            )
            continue
        bundle_type = _sensor_bundle_type(sensor_tag)
        source_entity = _resolve_sensor_source_entity(model, sensor_index)
        if name_translator is not None:
            source_entity = name_translator.translate_source_entity(source_entity)
        registration = SensorRegistration(
            sensor_name=sensor_name,
            display_name=display_name,
            sensor_tag=sensor_tag,
            bundle_id=sensor_name,
            bundle_type=bundle_type,
            source_entity=source_entity,
        )
        by_unit.setdefault(unit_key, []).append(registration)

        sensor_dim = int(model.sensor_dim[sensor_index])
        sensor_addr = int(model.sensor_adr[sensor_index])
        runtime_channels.append(
            RuntimeSensorChannel(
                sensor_name=sensor_name,
                sensor_tag=sensor_tag,
                source_kind="sensordata",
                start_index=sensor_addr,
                length=max(sensor_dim, 1),
            )
        )

    # Deterministic derived channels for models that don't define explicit <sensor> blocks.
    # These channels are still sourced from MuJoCo runtime state (qpos/qvel/actuator_force).
    for joint_id in range(model.njnt):
        joint_type = int(model.jnt_type[joint_id])
        if joint_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        joint_name = _sanitize_name(model.joint(joint_id).name or f"joint_{joint_id}")
        display_joint_name = joint_name
        if name_translator is not None:
            display_joint_name = name_translator.translate_joint(joint_name)
        joint_lower, joint_upper = map(float, model.jnt_range[joint_id])
        has_finite_range = (
            np.isfinite(joint_lower)
            and np.isfinite(joint_upper)
            and joint_upper > joint_lower
        )
        if has_finite_range:
            range_span = joint_upper - joint_lower
        else:
            range_span = 0.0

        qpos_addr = int(model.jnt_qposadr[joint_id])
        joint_pos_name = f"jointpos_{joint_name}"
        joint_pos_display_name = f"joint_position_{display_joint_name}"
        if name_translator is not None:
            joint_pos_display_name = name_translator.translate_sensor(
                joint_pos_name
            ) if joint_pos_name in name_translator.model_sensors else joint_pos_display_name
        by_unit.setdefault("Proximity", []).append(
            SensorRegistration(
                sensor_name=joint_pos_name,
                display_name=joint_pos_display_name,
                sensor_tag="jointpos",
                bundle_id=joint_pos_name,
                bundle_type="sensor_array",
                source_entity=display_joint_name,
            )
        )
        runtime_channels.append(
            RuntimeSensorChannel(
                sensor_name=joint_pos_name,
                sensor_tag="jointpos",
                source_kind="qpos",
                start_index=qpos_addr,
                length=1,
                normalize_mode="range_0_1" if has_finite_range else "none",
                normalize_min=joint_lower if has_finite_range else 0.0,
                normalize_max=joint_upper if has_finite_range else 0.0,
            )
        )

        qvel_addr = int(model.jnt_dofadr[joint_id])
        joint_vel_name = f"jointvel_{joint_name}"
        joint_vel_display_name = f"joint_velocity_{display_joint_name}"
        if name_translator is not None:
            joint_vel_display_name = name_translator.translate_sensor(
                joint_vel_name
            ) if joint_vel_name in name_translator.model_sensors else joint_vel_display_name
        by_unit.setdefault("Proximity", []).append(
            SensorRegistration(
                sensor_name=joint_vel_name,
                display_name=joint_vel_display_name,
                sensor_tag="jointvel",
                bundle_id=joint_vel_name,
                bundle_type="sensor_array",
                source_entity=display_joint_name,
            )
        )
        runtime_channels.append(
            RuntimeSensorChannel(
                sensor_name=joint_vel_name,
                sensor_tag="jointvel",
                source_kind="qvel",
                start_index=qvel_addr,
                length=1,
                normalize_mode="signed_0_1" if has_finite_range else "none",
                normalize_reference=range_span if has_finite_range else 0.0,
            )
        )

    for actuator_index in range(model.nu):
        actuator_name = _sanitize_name(model.actuator(actuator_index).name or f"actuator_{actuator_index}")
        display_actuator_name = actuator_name
        if name_translator is not None:
            display_actuator_name = name_translator.translate_actuator(actuator_name)
        channel_name = f"actuatorfrc_{actuator_name}"
        channel_display_name = f"actuator_force_{display_actuator_name}"
        if name_translator is not None:
            channel_display_name = name_translator.translate_sensor(
                channel_name
            ) if channel_name in name_translator.model_sensors else channel_display_name
        by_unit.setdefault("MiscData", []).append(
            SensorRegistration(
                sensor_name=channel_name,
                display_name=channel_display_name,
                sensor_tag="actuatorfrc",
                bundle_id=channel_name,
                bundle_type="sensor_array",
                source_entity=display_actuator_name,
            )
        )
        runtime_channels.append(
            RuntimeSensorChannel(
                sensor_name=channel_name,
                sensor_tag="actuatorfrc",
                source_kind="actuator_force",
                start_index=actuator_index,
                length=1,
                normalize_mode="none",
            )
        )

    if strict_mode and unsupported_sensor_types:
        unsupported_parts = ", ".join(
            f"{tag}({count})"
            for tag, count in sorted(unsupported_sensor_types.items())
        )
        raise RuntimeError(
            "Unsupported MuJoCo sensor types discovered during strict capability parsing: "
            f"{unsupported_parts}"
        )

    for unit_key, entries in by_unit.items():
        by_unit[unit_key] = sorted(entries, key=lambda entry: entry.sensor_name)
    runtime_channels.sort(key=lambda channel: (channel.start_index, channel.sensor_name))
    return by_unit, runtime_channels


def _infer_bundle_type(bundle_name: str, labels: list[str]) -> str:
    """
    Infer deterministic bundle type from model labels.

    The taxonomy stays simulator-agnostic and is used by registration metadata.
    """
    text = f"{bundle_name} {' '.join(labels)}".lower()
    if "wheel" in text or "steer" in text or "drive" in text:
        return "wheel_cluster"
    if "leg" in text or any(token in text for token in ("hip", "knee", "ankle", "thigh", "shin")):
        return "leg"
    if any(token in text for token in ("arm", "shoulder", "elbow", "wrist")):
        return "arm"
    if any(token in text for token in ("gripper", "finger", "claw", "hand")):
        return "gripper"
    if any(token in text for token in ("camera", "imu", "gyro", "lidar", "sensor")):
        return "sensor_array"
    return "custom"


def _canonical_motor_unit_key(unit_key: str) -> Optional[str]:
    lowered = unit_key.lower()
    if "positional" in lowered and "servo" in lowered:
        return "positional_servo"
    if "rotary" in lowered and "motor" in lowered:
        return "rotary_motor"
    return None


def _ensure_motor_registration_io_flags(unit_def: dict) -> None:
    """
    Ensure motor unit registration includes explicit IO config flags.

    FEAGI derives motor cortical IDs from device registrations using
    frame_change_handling and percentage_neuron_positioning fields.
    Some SDK/runtime combinations omit these fields for motor units, which can
    prevent motor cortical areas from auto-creating and force standalone mode.
    """
    io_flags = unit_def.get("io_configuration_flags")
    if not isinstance(io_flags, dict):
        io_flags = {}
    io_flags.setdefault("frame_change_handling", "Absolute")
    io_flags.setdefault("percentage_neuron_positioning", "Linear")
    unit_def["io_configuration_flags"] = io_flags
    unit_def.setdefault("frame_change_handling", io_flags["frame_change_handling"])
    unit_def.setdefault(
        "percentage_neuron_positioning",
        io_flags["percentage_neuron_positioning"],
    )


def _build_motor_registration_enricher(
    model_xml: str,
    group_names: list[str],
    group_channel_metadata: dict[int, dict[str, list[dict[str, str]]]],
    sensory_registration_metadata: dict[str, list[SensorRegistration]],
) -> Callable[[dict], dict]:
    source_model = os.path.abspath(model_xml)

    def enrich(registrations: dict) -> dict:
        output_units = registrations.get("output_units_and_decoder_properties")
        if not isinstance(output_units, dict):
            return registrations

        for unit_key, entries in output_units.items():
            canonical_unit = _canonical_motor_unit_key(str(unit_key))
            if canonical_unit is None or not isinstance(entries, list):
                continue

            for entry in entries:
                if not isinstance(entry, list) or not entry:
                    continue
                unit_def = entry[0]
                if not isinstance(unit_def, dict):
                    continue
                group_id = unit_def.get("cortical_unit_index")
                if not isinstance(group_id, int):
                    continue
                if group_id < 0 or group_id >= len(group_names):
                    raise RuntimeError(
                        f"Invalid cortical_unit_index {group_id} for motor registration"
                    )

                bundle_name = _sanitize_name(group_names[group_id])
                channel_metadata = (
                    group_channel_metadata
                    .get(group_id, {})
                    .get(canonical_unit, [])
                )
                _ensure_motor_registration_io_flags(unit_def)
                unit_def["friendly_name"] = bundle_name
                device_grouping = unit_def.get("device_grouping")
                if not isinstance(device_grouping, list) or not device_grouping:
                    raise RuntimeError(
                        f"Missing device_grouping for motor unit group {group_id}"
                    )
                if len(channel_metadata) < len(device_grouping):
                    raise RuntimeError(
                        "Insufficient channel metadata for group "
                        f"{group_id} unit {canonical_unit}: "
                        f"expected {len(device_grouping)}, got {len(channel_metadata)}"
                    )

                for channel_index, channel in enumerate(device_grouping):
                    if not isinstance(channel, dict):
                        raise RuntimeError(
                            f"Invalid device_grouping entry at group {group_id} channel {channel_index}"
                        )
                    metadata = channel_metadata[channel_index]
                    channel_name = metadata["channel_name"]
                    channel["friendly_name"] = channel_name
                    device_properties = channel.get("device_properties")
                    if not isinstance(device_properties, dict):
                        device_properties = {}
                    device_properties.update(
                        {
                            "bundle_type": metadata["bundle_type"],
                            "bundle_id": metadata["bundle_id"],
                            "modality": "motor",
                            "signal_type": canonical_unit,
                            "source_model": source_model,
                            "source_entity": metadata["source_entity"],
                            "joint_name": metadata["joint_name"],
                            "link_name": metadata["link_name"],
                            "actuator_name": metadata["actuator_name"],
                            "naming_schema_version": "1",
                        }
                    )
                    incremental_step_ratio = metadata.get("incremental_step_ratio")
                    if isinstance(incremental_step_ratio, (int, float)):
                        device_properties["incremental_step_ratio"] = float(
                            incremental_step_ratio
                        )
                    control_semantics = metadata.get("control_semantics")
                    if isinstance(control_semantics, str) and control_semantics:
                        device_properties["control_semantics"] = control_semantics
                    contract_version = metadata.get("contract_version")
                    if isinstance(contract_version, str) and contract_version:
                        device_properties["contract_version"] = contract_version
                    absolute_command_scale = metadata.get("absolute_command_scale")
                    if isinstance(absolute_command_scale, (int, float)):
                        device_properties["absolute_command_scale"] = float(
                            absolute_command_scale
                        )
                    incremental_command_scale = metadata.get("incremental_command_scale")
                    if isinstance(incremental_command_scale, (int, float)):
                        device_properties["incremental_command_scale"] = float(
                            incremental_command_scale
                        )
                    channel["device_properties"] = device_properties

        input_units = registrations.get("input_units_and_encoder_properties")
        if isinstance(input_units, dict):
            for unit_key, entries in sorted(input_units.items()):
                if not isinstance(entries, list) or not entries:
                    continue
                unit_registrations = list(
                    sensory_registration_metadata.get(str(unit_key), [])
                )
                metadata_cursor = 0
                for unit_idx, entry in enumerate(entries):
                    if not isinstance(entry, list) or not entry:
                        continue
                    unit_def = entry[0]
                    if not isinstance(unit_def, dict):
                        continue
                    grouping = unit_def.get("device_grouping")
                    if not isinstance(grouping, list):
                        continue
                    unit_friendly_name = str(unit_key).lower()
                    if metadata_cursor < len(unit_registrations):
                        unit_friendly_name = unit_registrations[metadata_cursor].display_name
                    elif len(entries) > 1:
                        unit_friendly_name = f"{str(unit_key).lower()}_{unit_idx}"
                    unit_def["friendly_name"] = unit_friendly_name
                    for channel_idx, channel in enumerate(grouping):
                        if not isinstance(channel, dict):
                            continue
                        sensor: Optional[SensorRegistration] = None
                        if metadata_cursor < len(unit_registrations):
                            sensor = unit_registrations[metadata_cursor]
                        metadata_cursor += 1
                        if sensor is None:
                            sensor = SensorRegistration(
                                sensor_name=f"{str(unit_key).lower()}_{unit_idx}_{channel_idx}",
                                display_name=f"{str(unit_key).lower()}_{unit_idx}_{channel_idx}",
                                sensor_tag=str(unit_key).lower(),
                                bundle_id=f"{str(unit_key).lower()}_{unit_idx}",
                                bundle_type="sensor_array",
                                source_entity=f"{str(unit_key).lower()}_{unit_idx}",
                            )
                        channel["friendly_name"] = sensor.display_name
                        device_properties = channel.get("device_properties")
                        if not isinstance(device_properties, dict):
                            device_properties = {}
                        device_properties.update(
                            {
                                "bundle_type": str(sensor.bundle_type),
                                "bundle_id": str(sensor.bundle_id),
                                "modality": "sensory",
                                "signal_type": str(_sensor_tag_to_signal_type(sensor.sensor_tag)),
                                "source_model": str(source_model),
                                "source_entity": str(sensor.source_entity),
                                "sensor_name": str(sensor.sensor_name),
                                "sensor_tag": str(sensor.sensor_tag),
                                "naming_schema_version": "1",
                            }
                        )
                        channel["device_properties"] = device_properties
        return registrations

    return enrich


def register_mujoco_sensors_in_cache(
    sensor_registration_map: dict[str, list[SensorRegistration]],
    group_index_start: int = 0,
) -> dict[str, int]:
    """Register MuJoCo sensors in cache via Python SDK abstractions."""
    unit_counts = {
        unit_key: len(registrations)
        for unit_key, registrations in sorted(sensor_registration_map.items())
    }
    return brain_output.register_sensor_units(
        unit_counts,
        z_neuron_resolution=10,
        group_index_start=group_index_start,
    )


def _log_discovered_capability_summary(
    sensor_registration_map: dict[str, list[SensorRegistration]],
    runtime_sensor_channels: list[RuntimeSensorChannel],
    motors: list[tuple],
    group_channels: dict[int, dict[str, list[str]]],
) -> None:
    """Emit deterministic capability summary for registration troubleshooting."""
    logger.info("[CAPS] FEAGI registration capability summary:")

    input_unit_counts = {
        unit_key: len(sensor_registration_map[unit_key])
        for unit_key in sorted(sensor_registration_map.keys())
    }
    output_group_counts = {
        str(group_id): {
            "positional_servo": len(group_channels[group_id].get("positional_servo", [])),
            "rotary_motor": len(group_channels[group_id].get("rotary_motor", [])),
        }
        for group_id in sorted(group_channels.keys())
    }

    # Machine-parseable single-line payload for FEAGI Desktop/controller logs.
    compact_payload = {
        "input_units": input_unit_counts,
        "output_groups": output_group_counts,
        "totals": {
            "input_unit_count": len(sensor_registration_map),
            "input_channel_count": sum(input_unit_counts.values()),
            "runtime_sensor_channel_count": len(runtime_sensor_channels),
            "motor_channel_count": len(motors),
            "motor_group_count": len(group_channels),
        },
    }
    logger.info("[CAPS][JSON] %s", json.dumps(compact_payload, sort_keys=True))

    total_sensor_channels = sum(input_unit_counts.values())
    logger.info(
        "[CAPS][INPUT] units=%d channels=%d runtime_channels=%d",
        len(sensor_registration_map),
        total_sensor_channels,
        len(runtime_sensor_channels),
    )
    for unit_key in sorted(input_unit_counts.keys()):
        logger.info(
            "[CAPS][INPUT][%s] channels=%d",
            unit_key,
            input_unit_counts[unit_key],
        )

    logger.info(
        "[CAPS][OUTPUT] motors=%d groups=%d",
        len(motors),
        len(group_channels),
    )
    for group_id in sorted(group_channels.keys()):
        channels = output_group_counts[str(group_id)]
        logger.info(
            "[CAPS][OUTPUT][group=%d] positional_servo=%d rotary_motor=%d",
            group_id,
            channels["positional_servo"],
            channels["rotary_motor"],
        )


def register_mujoco_motors(
    model,
    xml_path,
    motor_gain: float = 1.0,
    name_translator: Optional[MujocoNameTranslator] = None,
):
    """
    Register FEAGI motors for all MuJoCo actuators with limb grouping.

    Maps MuJoCo actuators to FEAGI ServoMotors or RotaryMotors based on:
    - Range: bounded → ServoMotor, unbounded → RotaryMotor
    - Type: bounded → absolute-safe ServoMotor, unbounded → incremental RotaryMotor

    Returns:
        tuple: (motors, group_names, group_channels, group_channel_metadata)
    """
    actuator_metadata = parse_actuator_metadata_from_xml(xml_path)
    roots = _iter_mujoco_xml_roots(xml_path, visited=set())
    body_children, body_parent, body_joints, joint_to_body = _build_body_hierarchy(roots)
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
    group_channel_metadata: dict[int, dict[str, list[dict[str, str]]]] = {}
    # Use compact FEAGI group indexing (0..N-1 for non-empty actuator groups).
    # This keeps motor cortical IDs deterministic across models where parsed
    # group labels may start at non-zero indices.
    compact_group_names = [
        name for name in group_names if group_actuator_order.get(name)
    ]
    if not compact_group_names:
        compact_group_names = list(group_names)

    for group_id, group_name in enumerate(compact_group_names):
        group_channels[group_id] = {"positional_servo": [], "rotary_motor": []}
        group_channel_metadata[group_id] = {"positional_servo": [], "rotary_motor": []}
        ordered_labels = [
            _sanitize_name(act_name)
            for act_name in group_actuator_order.get(group_name, [])
        ]
        bundle_type = _infer_bundle_type(group_name, ordered_labels)
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

            if actuator_type not in ["position", "velocity", "motor", "general", "adhesion"]:
                raise RuntimeError(
                    f"Unsupported MuJoCo actuator type '{actuator_type}' for actuator '{actuator_name}'"
                )

            # Use absolute-safe default for bounded channels (ServoMotor).
            # Runtime command_mode from FEAGI still switches to incremental when
            # incremental cortical packets are present, but this prevents
            # unintended accumulation if a packet arrives without explicit mode.
            encoding = "absolute" if is_bounded else "incremental"

            try:
                actuator_info = actuator_metadata.get(actuator_name, {})
                joint_name = actuator_info.get("joint")
                if isinstance(joint_name, str):
                    joint_name = _sanitize_name(joint_name)
                else:
                    joint_name = ""
                joint_names = actuator_info.get("joint_names", [])
                if not isinstance(joint_names, list):
                    joint_names = []
                resolved_joint_names = [
                    _sanitize_name(name)
                    for name in joint_names
                    if isinstance(name, str) and name
                ]
                if not joint_name and resolved_joint_names:
                    joint_name = resolved_joint_names[0]
                link_name = joint_to_body.get(joint_name, "") if joint_name else ""
                # Use one canonical source label per FEAGI channel.
                # Tendon actuators may reference multiple joints internally, but
                # one cortical channel should map to one display identity in BV.
                source_entity = joint_name or actuator_name
                motor_contract = (
                    name_translator.motor_contract_for(
                        actuator_name=actuator_name,
                        joint_name=joint_name,
                        source_entity=source_entity,
                    )
                    if name_translator is not None
                    else {}
                )
                incremental_step_ratio = (
                    name_translator.incremental_step_ratio_for(
                        actuator_name=actuator_name,
                        joint_name=joint_name,
                        source_entity=source_entity,
                    )
                    if name_translator is not None
                    else None
                )
                contract_incremental_ratio = motor_contract.get("incremental_step_ratio")
                if isinstance(contract_incremental_ratio, (int, float)):
                    incremental_step_ratio = float(contract_incremental_ratio)
                control_semantics = motor_contract.get("control_semantics")
                contract_version = motor_contract.get("contract_version")
                absolute_command_scale = motor_contract.get("absolute_command_scale")
                incremental_command_scale = motor_contract.get("incremental_command_scale")
                display_source_entity = source_entity
                if name_translator is not None:
                    if joint_name:
                        display_source_entity = name_translator.translate_joint(joint_name)
                    else:
                        display_source_entity = name_translator.translate_actuator(actuator_name)
                if is_bounded:
                    channel_index = len(group_channels[group_id]["positional_servo"])
                    motor = ServoMotor.register(
                        range=(float(min_val), float(max_val)),
                        encoding=encoding,
                        unit_id=group_id,
                        channel_index=channel_index,
                    )
                    if incremental_step_ratio is not None and hasattr(motor, "incremental_step_ratio"):
                        setattr(motor, "incremental_step_ratio", incremental_step_ratio)
                    if isinstance(control_semantics, str) and control_semantics and hasattr(motor, "control_semantics"):
                        setattr(motor, "control_semantics", control_semantics)
                    if isinstance(absolute_command_scale, (int, float)) and hasattr(motor, "absolute_command_scale"):
                        setattr(motor, "absolute_command_scale", float(absolute_command_scale))
                    if isinstance(incremental_command_scale, (int, float)) and hasattr(motor, "incremental_command_scale"):
                        setattr(motor, "incremental_command_scale", float(incremental_command_scale))
                    group_channels[group_id]["positional_servo"].append(actuator_name)
                    group_channel_metadata[group_id]["positional_servo"].append(
                        {
                            "bundle_type": bundle_type,
                            "bundle_id": _sanitize_name(group_name),
                            "channel_name": display_source_entity,
                            "source_entity": source_entity,
                            "joint_name": joint_name,
                            "link_name": link_name,
                            "actuator_name": actuator_name,
                            "incremental_step_ratio": incremental_step_ratio,
                            "control_semantics": control_semantics,
                            "contract_version": contract_version,
                            "absolute_command_scale": absolute_command_scale,
                            "incremental_command_scale": incremental_command_scale,
                        }
                    )
                    device_type = "ServoMotor"
                else:
                    channel_index = len(group_channels[group_id]["rotary_motor"])
                    motor = RotaryMotor.register(
                        encoding=encoding,
                        bidirectional=True,
                        unit_id=group_id,
                        channel_index=channel_index,
                    )
                    if isinstance(control_semantics, str) and control_semantics and hasattr(motor, "control_semantics"):
                        setattr(motor, "control_semantics", control_semantics)
                    if isinstance(absolute_command_scale, (int, float)) and hasattr(motor, "absolute_command_scale"):
                        setattr(motor, "absolute_command_scale", float(absolute_command_scale))
                    if isinstance(incremental_command_scale, (int, float)) and hasattr(motor, "incremental_command_scale"):
                        setattr(motor, "incremental_command_scale", float(incremental_command_scale))
                    group_channels[group_id]["rotary_motor"].append(actuator_name)
                    group_channel_metadata[group_id]["rotary_motor"].append(
                        {
                            "bundle_type": bundle_type,
                            "bundle_id": _sanitize_name(group_name),
                            "channel_name": display_source_entity,
                            "source_entity": source_entity,
                            "joint_name": joint_name,
                            "link_name": link_name,
                            "actuator_name": actuator_name,
                            "incremental_step_ratio": incremental_step_ratio,
                            "control_semantics": control_semantics,
                            "contract_version": contract_version,
                            "absolute_command_scale": absolute_command_scale,
                            "incremental_command_scale": incremental_command_scale,
                        }
                    )
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

    return motors, compact_group_names, group_channels, group_channel_metadata


def _normalize_channel_value(raw_value: float, channel: RuntimeSensorChannel) -> float:
    """Normalize a runtime channel value for FEAGI sensory encoding."""
    if channel.normalize_mode == "range_0_1":
        min_v = channel.normalize_min
        max_v = channel.normalize_max
        if max_v <= min_v:
            return raw_value
        return _clamp((raw_value - min_v) / (max_v - min_v), 0.0, 1.0)
    if channel.normalize_mode == "signed_0_1":
        reference = abs(channel.normalize_reference)
        if reference <= 0.0:
            return raw_value
        return _clamp((raw_value / reference) + 0.5, 0.0, 1.0)
    return raw_value


def _to_feagi_potential(value: float, channel: RuntimeSensorChannel) -> float:
    """
    Convert normalized/raw runtime values to FEAGI neuron potential scale [0, 100].

    Percentage-based channels use explicit [0,1] normalization.
    Raw channels (e.g., actuator forces) use bounded magnitude mapping so small
    movements still produce visible activity without exploding on spikes.
    """
    if not np.isfinite(value):
        return 0.0
    if channel.normalize_mode in ("range_0_1", "signed_0_1"):
        return _clamp(value, 0.0, 1.0) * 100.0
    return _clamp(np.tanh(abs(value)) * 100.0, 0.0, 100.0)


def _to_unit_scalar_0_1(value: float, channel: RuntimeSensorChannel) -> float:
    """Convert runtime value to unit scalar in [0,1] for typed cache writes."""
    if not np.isfinite(value):
        return 0.0
    if channel.normalize_mode in ("range_0_1", "signed_0_1"):
        return _clamp(value, 0.0, 1.0)
    return _clamp(float(np.tanh(abs(value))), 0.0, 1.0)


def _read_runtime_channel_sample(data, sensor_channel: RuntimeSensorChannel, dim: int) -> Optional[float]:
    """Read one scalar sample from MuJoCo runtime buffers."""
    sample_index = sensor_channel.start_index + dim
    if sensor_channel.source_kind == "sensordata":
        if sample_index < len(data.sensordata):
            return float(data.sensordata[sample_index])
    elif sensor_channel.source_kind == "qpos":
        if sample_index < len(data.qpos):
            return float(data.qpos[sample_index])
    elif sensor_channel.source_kind == "qvel":
        if sample_index < len(data.qvel):
            return float(data.qvel[sample_index])
    elif sensor_channel.source_kind == "actuator_force":
        if sample_index < len(data.actuator_force):
            return float(data.actuator_force[sample_index])
    return None


def _clamp(value: float, low: float, high: float) -> float:
    """Clamp a numeric value to [low, high]."""
    return max(low, min(high, value))


def _build_joint_limit_guards(model) -> list[JointLimitGuard]:
    """
    Build 1-DOF joint limit guards from MuJoCo model metadata.

    This is intentionally generic across all actuator categories because
    enforcement happens at the joint state level after physics integration.
    """
    guards: list[JointLimitGuard] = []
    valid_joint_types = {
        mujoco.mjtJoint.mjJNT_HINGE,
        mujoco.mjtJoint.mjJNT_SLIDE,
    }

    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] not in valid_joint_types:
            continue
        if hasattr(model, "jnt_limited") and not bool(model.jnt_limited[joint_id]):
            continue

        lower, upper = map(float, model.jnt_range[joint_id])
        if not np.isfinite(lower) or not np.isfinite(upper) or upper <= lower:
            continue

        qpos_addr = int(model.jnt_qposadr[joint_id])
        qvel_addr = int(model.jnt_dofadr[joint_id])
        if qpos_addr < 0:
            continue

        joint_name = model.joint(joint_id).name or f"joint_{joint_id}"
        guards.append(
            JointLimitGuard(
                joint_name=_sanitize_name(joint_name),
                qpos_addr=qpos_addr,
                qvel_addr=qvel_addr,
                lower=lower,
                upper=upper,
            )
        )

    return guards


def _enforce_joint_limits(
    model,
    data,
    guards: list[JointLimitGuard],
) -> list[tuple[str, float, float]]:
    """Clamp joint positions to limits and cancel outward velocity at limits."""
    corrections: list[tuple[str, float, float]] = []

    for guard in guards:
        current_qpos = float(data.qpos[guard.qpos_addr])
        corrected_qpos = current_qpos
        if current_qpos < guard.lower:
            corrected_qpos = guard.lower
        elif current_qpos > guard.upper:
            corrected_qpos = guard.upper

        if corrected_qpos == current_qpos:
            continue

        data.qpos[guard.qpos_addr] = corrected_qpos
        if 0 <= guard.qvel_addr < len(data.qvel):
            current_qvel = float(data.qvel[guard.qvel_addr])
            if (
                corrected_qpos == guard.lower and current_qvel < 0.0
            ) or (
                corrected_qpos == guard.upper and current_qvel > 0.0
            ):
                data.qvel[guard.qvel_addr] = 0.0

        corrections.append((guard.joint_name, current_qpos, corrected_qpos))

    if corrections:
        mujoco.mj_forward(model, data)

    return corrections


def _build_expected_motor_cortical_ids(motors: list[tuple]) -> list[str]:
    """
    Build deterministic expected motor cortical IDs used for SDK registration verification.

    This keeps controller behavior stable even when Python SDK/venv variants differ in
    internal expected-ID derivation logic.
    """
    expected: set[str] = set()
    for motor_entry in motors:
        motor = motor_entry[0]
        group_id = int(motor_entry[5])
        encoding = str(motor_entry[8])
        if isinstance(motor, ServoMotor):
            # PositionalServo emits absolute + incremental percentage cortical areas.
            # [o,p,s,e, config_lo, config_hi, sub_unit, group]
            expected.add(
                base64.b64encode(
                    bytes([111, 112, 115, 101, 1, 0, 0, group_id & 0xFF])
                ).decode()
            )
            expected.add(
                base64.b64encode(
                    bytes([111, 112, 115, 101, 1, 1, 1, group_id & 0xFF])
                ).decode()
            )
        elif isinstance(motor, RotaryMotor):
            # RotaryMotor uses signed-percentage ("mot"). Respect runtime encoding mode.
            frame_bit = 1 if encoding == "incremental" else 0
            config_lo = 5
            config_hi = frame_bit
            sub_unit = 0
            expected.add(
                base64.b64encode(
                    bytes([111, 109, 111, 116, config_lo, config_hi, sub_unit, group_id & 0xFF])
                ).decode()
            )
    return sorted(expected)


def _health_check_url(feagi_host: str, feagi_api_port: int) -> str:
    """Build FEAGI health check endpoint URL."""
    return f"http://{feagi_host}:{feagi_api_port}/v1/system/health_check"


def _simulation_timestep_url(feagi_host: str, feagi_api_port: int) -> str:
    """Build FEAGI simulation timestep endpoint URL."""
    return f"http://{feagi_host}:{feagi_api_port}/v1/burst_engine/simulation_timestep"


def _read_feagi_simulation_rate_hz(
    feagi_host: str,
    feagi_api_port: int,
    feagi_http_timeout_s: float,
) -> float:
    """Read current FEAGI simulation rate from health check."""
    endpoint = _health_check_url(feagi_host, feagi_api_port)
    request = urllib.request.Request(endpoint, method="GET")
    try:
        with urllib.request.urlopen(request, timeout=feagi_http_timeout_s) as response:
            body = response.read().decode("utf-8")
    except urllib.error.URLError as exc:
        raise RuntimeError(f"FEAGI health_check request failed: {exc}") from exc

    try:
        payload = json.loads(body)
    except json.JSONDecodeError as exc:
        raise RuntimeError("FEAGI health_check returned invalid JSON.") from exc
    if not isinstance(payload, dict):
        raise RuntimeError("FEAGI health_check response must be a JSON object.")

    timestep = payload.get("simulation_timestep")
    if not isinstance(timestep, (int, float)):
        raise RuntimeError("FEAGI health_check missing numeric simulation_timestep.")
    timestep = float(timestep)
    if not np.isfinite(timestep) or timestep <= 0.0:
        raise RuntimeError(f"FEAGI health_check returned invalid simulation_timestep={timestep}")
    return 1.0 / timestep


def _set_feagi_simulation_rate_hz(
    feagi_host: str,
    feagi_api_port: int,
    feagi_http_timeout_s: float,
    requested_rate_hz: float,
) -> None:
    """Request FEAGI simulation timestep update for target rate."""
    if not np.isfinite(requested_rate_hz) or requested_rate_hz <= 0.0:
        raise RuntimeError(f"Requested rate must be > 0 Hz, got {requested_rate_hz}")
    timestep = 1.0 / float(requested_rate_hz)
    endpoint = _simulation_timestep_url(feagi_host, feagi_api_port)
    body = json.dumps({"simulation_timestep": timestep}).encode("utf-8")
    request = urllib.request.Request(
        endpoint,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=feagi_http_timeout_s):
            pass
    except urllib.error.URLError as exc:
        raise RuntimeError(f"FEAGI simulation_timestep update failed: {exc}") from exc


def _enforce_feagi_model_rate_strict(
    model,
    feagi_host: str,
    feagi_api_port: int,
    feagi_http_timeout_s: float,
) -> None:
    """
    Strictly enforce FEAGI burst-engine timestep from MuJoCo model timestep.

    Fails startup if FEAGI cannot be updated to the model-requested rate.
    """
    model_timestep = float(model.opt.timestep)
    if not np.isfinite(model_timestep) or model_timestep <= 0.0:
        raise RuntimeError(
            f"MuJoCo model has invalid timestep={model_timestep}; cannot negotiate FEAGI rate."
        )
    requested_rate_hz = 1.0 / model_timestep
    logger.info(
        "[RATE] Model timestep=%.8f requested_rate_hz=%.2f",
        model_timestep,
        requested_rate_hz,
    )

    current_rate_hz = _read_feagi_simulation_rate_hz(
        feagi_host,
        feagi_api_port,
        feagi_http_timeout_s,
    )
    logger.info("[RATE] FEAGI current effective rate=%.2f Hz", current_rate_hz)
    rate_epsilon_hz = 0.01
    if abs(current_rate_hz - requested_rate_hz) <= rate_epsilon_hz:
        logger.info("[RATE] FEAGI rate already matches model request.")
        return

    logger.info("[RATE] Updating FEAGI rate to %.2f Hz...", requested_rate_hz)
    _set_feagi_simulation_rate_hz(
        feagi_host,
        feagi_api_port,
        feagi_http_timeout_s,
        requested_rate_hz,
    )
    updated_rate_hz = _read_feagi_simulation_rate_hz(
        feagi_host,
        feagi_api_port,
        feagi_http_timeout_s,
    )
    if abs(updated_rate_hz - requested_rate_hz) > rate_epsilon_hz:
        raise RuntimeError(
            "FEAGI rate mismatch after update: "
            f"requested={requested_rate_hz:.2f}Hz actual={updated_rate_hz:.2f}Hz"
        )
    logger.info(
        "[RATE] FEAGI simulation rate enforced at %.2f Hz (timestep=%.8f).",
        updated_rate_hz,
        1.0 / updated_rate_hz,
    )


def _build_vision_units_from_model(
    model,
    name_translator: Optional[MujocoNameTranslator] = None,
) -> tuple[list[tuple[str, int, int, int, str, int]], list[tuple[str, int, int]]]:
    """
    Build deterministic FEAGI vision units from MuJoCo camera metadata.

    Register every camera mounted on the robot body subtree (not scene/world cameras)
    as a separate FEAGI vision group.

    Robot scope is normally the common ancestor of all non-free joint attachment
    bodies. For jointless rigs (static meshes, camera-only assets with no explicit
    ``<joint>``), the same ancestor is inferred from bodies that mount fixed cameras
    so vision still registers.
    """
    if not (hasattr(model, "vis") and hasattr(model.vis, "global_")):
        return [], []
    # Standard FEAGI segmented vision input resolution.
    # Segmentation topology is configured in SDK registration:
    # center=128x128x3, peripheral=32x32x1.
    render_width = 128
    render_height = 128
    nbody = int(getattr(model, "nbody", 0) or 0)
    if nbody <= 1:
        return [], []

    body_parent = [int(model.body_parentid[i]) for i in range(nbody)]
    ncam = int(getattr(model, "ncam", 0) or 0)
    fixed_camera_mode = int(getattr(mujoco.mjtCamLight, "mjCAMLIGHT_FIXED", 0))

    def _ancestors(body_id: int) -> list[int]:
        lineage: list[int] = []
        cursor = int(body_id)
        while cursor >= 0:
            lineage.append(cursor)
            if cursor == 0:
                break
            cursor = body_parent[cursor]
        return lineage

    candidate_body_ids = [
        int(model.jnt_bodyid[joint_index])
        for joint_index in range(int(getattr(model, "njnt", 0) or 0))
        if int(model.jnt_type[joint_index]) != int(mujoco.mjtJoint.mjJNT_FREE)
    ]
    if not candidate_body_ids:
        candidate_body_ids = [
            int(model.jnt_bodyid[joint_index])
            for joint_index in range(int(getattr(model, "njnt", 0) or 0))
        ]
    if not candidate_body_ids:
        # Jointless MJCF (e.g. static camera housing): infer robot subtree from fixed
        # camera mount bodies so LCA and camera filtering still apply.
        mount_body_ids: list[int] = []
        for camera_index in range(ncam):
            if int(model.cam_mode[camera_index]) != fixed_camera_mode:
                continue
            cam_body_id = int(model.cam_bodyid[camera_index])
            if cam_body_id > 0:
                mount_body_ids.append(cam_body_id)
        candidate_body_ids = sorted(set(mount_body_ids))
        if candidate_body_ids:
            logger.info(
                "[VISION] Robot root inferred from fixed camera mounts "
                "(no joint bodies): mount_body_ids=%s",
                candidate_body_ids,
            )
    if not candidate_body_ids:
        return [], []

    first_lineage = _ancestors(candidate_body_ids[0])
    common_ancestors = set(first_lineage)
    for body_id in candidate_body_ids[1:]:
        common_ancestors &= set(_ancestors(body_id))
    if not common_ancestors:
        return [], []
    robot_root_body_id = next(
        (body_id for body_id in first_lineage if body_id in common_ancestors),
        0,
    )

    def _is_descendant(candidate_body_id: int, ancestor_body_id: int) -> bool:
        cursor = int(candidate_body_id)
        while cursor >= 0:
            if cursor == ancestor_body_id:
                return True
            if cursor == 0:
                break
            cursor = body_parent[cursor]
        return False

    selected_cameras: list[tuple[str, int]] = []
    for camera_index in range(ncam):
        camera_mode = int(model.cam_mode[camera_index])
        # Mounted robot sensors should be fixed to their mount body; tracking
        # cameras are scene/navigation views and must be excluded.
        if camera_mode != fixed_camera_mode:
            continue
        camera_body_id = int(model.cam_bodyid[camera_index])
        if camera_body_id <= 0:
            continue
        if not _is_descendant(camera_body_id, robot_root_body_id):
            continue
        try:
            camera_name = _sanitize_name(
                model.camera(camera_index).name or f"camera_{camera_index}"
            )
        except Exception:
            camera_name = f"camera_{camera_index}"
        selected_cameras.append((camera_name, camera_index))

    selected_cameras.sort(key=lambda item: item[0])
    if name_translator is not None:
        override_names = [
            name.lower() for name in name_translator.get_vision_camera_overrides()
        ]
        if override_names:
            by_name = {name.lower(): (name, idx) for name, idx in selected_cameras}
            override_selected: list[tuple[str, int]] = []
            missing_override_names: list[str] = []
            for override_name in override_names:
                camera_entry = by_name.get(override_name)
                if camera_entry is not None:
                    override_selected.append(camera_entry)
                else:
                    missing_override_names.append(override_name)
            if missing_override_names:
                logger.warning(
                    "[VISION][OVERRIDE] Requested cameras not detected on robot: %s",
                    missing_override_names,
                )
            selected_cameras = override_selected
    if not selected_cameras:
        logger.warning(
            "[VISION] No robot-mounted cameras selected for FEAGI vision registration."
        )
        return [], []
    selected_camera_groups = [
        (camera_name, camera_index, group_index)
        for group_index, (camera_name, camera_index) in enumerate(selected_cameras)
    ]
    logger.info(
        "[VISION] Selected cameras for FEAGI segmented vision: %s",
        [
            {
                "name": camera_name,
                "camera_index": camera_index,
                "group": group_index,
            }
            for camera_name, camera_index, group_index in selected_camera_groups
        ],
    )
    vision_units = [
        ("camera", render_width, render_height, 3, "vision", group_index)
        for group_index, _ in enumerate(selected_cameras)
    ]
    return vision_units, selected_camera_groups


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
        default=1.0,
        help=(
            "Amplification factor for motor commands (default: 1.0). "
            "Use >1.0 for stronger movement."
        ),
    )
    parser.add_argument(
        "--enable-telemetry",
        action="store_true",
        help=(
            "Enable periodic [FRAME]/[TELE] diagnostics. "
            "Disabled by default."
        ),
    )
    args = parser.parse_args()

    logger.info("[START] Generic MuJoCo Controller (FEAGI Python SDK)")
    logger.info("[SRC] %s", os.path.abspath(__file__))
    runtime_controller_version = os.path.basename(
        os.path.dirname(os.path.abspath(__file__))
    )
    manifest_controller_version, manifest_build_number = _resolve_controller_version_info()
    feagi_sdk_version = _resolve_feagi_sdk_version()
    mujoco_version = getattr(mujoco, "__version__", "unknown")
    logger.info(
        "[VERSIONS] controller_runtime=%s controller_bundle=%s build=%s "
        "python_sdk=%s mujoco=%s python=%s",
        runtime_controller_version,
        manifest_controller_version,
        manifest_build_number if manifest_build_number is not None else "unknown",
        feagi_sdk_version,
        mujoco_version,
        sys.version.split()[0],
    )
    try:
        import feagi.pns.client as feagi_client_module
        import feagi.pns.xyzp_decoders as feagi_xyzp_module
        logger.info("[SDK] feagi.pns.client=%s", inspect.getsourcefile(feagi_client_module))
        logger.info("[SDK] feagi.pns.xyzp_decoders=%s", inspect.getsourcefile(feagi_xyzp_module))
    except Exception as sdk_path_err:
        logger.info("[SDK] Failed to resolve SDK source paths: %s", sdk_path_err)
    logger.info(f"[FEAGI] {args.ip}:{args.port}")
    logger.info(f"[MODEL] {args.model_xml}")
    logger.info(f"[AGENT] {args.agent_id}")
    logger.info(f"[GAIN] Motor Gain: {args.motor_gain}x")
    name_translator = _load_mujoco_name_translator(args.model_xml)
    if not args.auth_token_b64:
        raise RuntimeError(
            "Missing auth token. Provide --auth-token-b64 or set FEAGI_AUTH_TOKEN_B64."
        )

    # Prepare model path: patch missing/zero ctrlrange in temp copy if needed
    model_load_path, temp_model_dir = _prepare_model_load_path(args.model_xml)
    keyframe_recovery_dir: Optional[str] = None

    # Load MuJoCo model; on keyframe/qpos mismatch, strip <keyframe> from entry MJCF and retry
    try:
        model, data, keyframe_recovery_dir = _load_mujoco_model_resilient(
            model_load_path,
            temp_model_dir,
        )
    except Exception:
        _cleanup_mujoco_temp_dirs(temp_model_dir, None)
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
    joint_limit_guards = _build_joint_limit_guards(model)
    logger.info(
        "[SAFETY] Joint-limit guards active for %d joints",
        len(joint_limit_guards),
    )
    vision_units, selected_vision_cameras = _build_vision_units_from_model(
        model,
        name_translator,
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
        vision_units=vision_units,
    )

    # Register motors with FEAGI (uses brain_output cache; agent_id must already be set)
    sensor_registration_map, runtime_sensor_channels = _build_sensor_registration_map(
        model,
        model_load_path,
        name_translator=name_translator,
    )
    supported_scalar_sensor_units = {"Proximity", "Shock", "MiscData"}
    unsupported_registration_details: dict[str, list[str]] = {}
    for unit_key, registrations in sensor_registration_map.items():
        if unit_key in supported_scalar_sensor_units:
            continue
        unsupported_registration_details[unit_key] = sorted(
            registration.sensor_name for registration in registrations
        )
    unsupported_registered_units = sorted(unsupported_registration_details.keys())
    if unsupported_registered_units:
        logger.warning(
            "[SENSORY][UNSUPPORTED] Excluding unsupported FEAGI sensory units "
            "from cache registration. Supported units=%s Unsupported units=%s",
            sorted(supported_scalar_sensor_units),
            unsupported_registered_units,
        )
        for unit_key in unsupported_registered_units:
            skipped_sensors = unsupported_registration_details[unit_key]
            logger.warning(
                "[SENSORY][UNSUPPORTED][%s] skipped_channels=%d skipped_sensors=%s",
                unit_key,
                len(skipped_sensors),
                skipped_sensors,
            )
    sensor_registration_map = {
        unit_key: registrations
        for unit_key, registrations in sensor_registration_map.items()
        if unit_key in supported_scalar_sensor_units
    }
    sensory_registration_metadata = {
        unit_key: list(registrations)
        for unit_key, registrations in sensor_registration_map.items()
    }
    if selected_vision_cameras:
        sensory_registration_metadata["Vision"] = [
            SensorRegistration(
                sensor_name=camera_name,
                display_name=camera_name,
                sensor_tag="camprojection",
                bundle_id=camera_name,
                bundle_type="camera_rig",
                source_entity=camera_name,
            )
            for camera_name, _camera_index, _group_index in selected_vision_cameras
        ]
    supported_sensor_names = {
        registration.sensor_name
        for registrations in sensor_registration_map.values()
        for registration in registrations
    }
    runtime_sensor_channels = [
        channel
        for channel in runtime_sensor_channels
        if channel.sensor_name in supported_sensor_names
    ]
    logger.info(
        "[SENSORY][SUPPORTED] Registered FEAGI sensory cache units=%s total_channels=%d",
        sorted(sensor_registration_map.keys()),
        sum(len(registrations) for registrations in sensor_registration_map.values()),
    )
    runtime_channel_by_name = {
        channel.sensor_name: channel for channel in runtime_sensor_channels
    }
    sensor_group_index_start = len(vision_units)
    sensor_cache_channel_layout: list[tuple[str, int, int, RuntimeSensorChannel]] = []
    for group_offset, unit_key in enumerate(sorted(sensor_registration_map.keys())):
        group_index = sensor_group_index_start + group_offset
        registrations = sensor_registration_map[unit_key]
        for channel_index, registration in enumerate(registrations):
            channel = runtime_channel_by_name.get(registration.sensor_name)
            if channel is None:
                raise RuntimeError(
                    "Missing runtime channel for registered sensor "
                    f"'{registration.sensor_name}' ({unit_key})"
                )
            sensor_cache_channel_layout.append(
                (unit_key, group_index, channel_index, channel)
            )
    logger.info(
        "[SENSORS] Parsed %d runtime sensor channels across %d FEAGI sensory units",
        len(runtime_sensor_channels),
        len(sensor_registration_map),
    )

    motors, group_names, group_channels, group_channel_metadata = register_mujoco_motors(
        model,
        model_load_path,
        args.motor_gain,
        name_translator,
    )
    _log_discovered_capability_summary(
        sensor_registration_map,
        runtime_sensor_channels,
        motors,
        group_channels,
    )
    brain_output.set_device_registration_enricher(
        _build_motor_registration_enricher(
            model_load_path,
            group_names,
            group_channel_metadata,
            sensory_registration_metadata,
        )
    )
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

    sensors_only_feagi = len(motors) == 0
    has_feagi_sensory = bool(vision_units) or bool(sensor_registration_map)
    if sensors_only_feagi and not has_feagi_sensory:
        logger.info(
            "[FAIL] No motors and no FEAGI sensory outputs (vision/scalar cache); "
            "nothing to stream. Add cameras/sensors to the model or use a robot with actuators."
        )
        _cleanup_mujoco_temp_dirs(temp_model_dir, keyframe_recovery_dir)
        return 1
    if sensors_only_feagi:
        logger.info(
            "[CFG] Sensors-only FEAGI mode: 0 actuators; motor ZMQ receive disabled, "
            "vision/scalar sensory streaming enabled."
        )

    feagi_enabled = True

    # brain_output already configured before register_mujoco_motors()
    try:
        # Ensure the motor unit is registered in the Rust ConnectorAgent with the
        # correct channel count BEFORE FEAGI registration.
        #
        # Otherwise FEAGI will create a default 1-channel motor OPU and
        # never reflect the true number of joints.
        try:
            # Register sensory and motor layouts via SDK wrappers.
            if vision_units:
                peripheral_resolution = None
                if name_translator is not None:
                    peripheral_resolution = (
                        name_translator.get_vision_peripheral_resolution()
                    )
                vision_groups = brain_output.register_vision_groups(
                    vision_units,
                    peripheral_resolution=peripheral_resolution,
                )
                logger.info(
                    "[VISION] Registered %d vision cache group(s): %s",
                    len(vision_groups),
                    vision_groups,
                )
            scalar_sensor_groups = register_mujoco_sensors_in_cache(
                sensor_registration_map,
                group_index_start=sensor_group_index_start,
            )
            logger.info(
                "[SENSORY] Registered scalar sensory cache groups with offset=%d: %s",
                sensor_group_index_start,
                scalar_sensor_groups,
            )
            brain_output.register_motor_groups(
                group_channels,
                z_neuron_resolution=10,
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to register motor devices in ConnectorAgent: {e}"
            ) from e

        if not sensors_only_feagi:
            logger.info("[CONN] Connecting motor stream (Python SDK)...")
            expected_motor_ids = _build_expected_motor_cortical_ids(motors)
            if hasattr(brain_output, "_collect_motor_cortical_ids"):
                brain_output._collect_motor_cortical_ids = lambda: list(expected_motor_ids)
        else:
            logger.info("[CONN] Connecting FEAGI (sensory-only agent; no motor commands)...")

        brain_output.connect()
        logger.info("   [OK] FEAGI connection established!")
        try:
            _enforce_feagi_model_rate_strict(
                model,
                args.ip,
                args.port,
                args.feagi_http_timeout_s,
            )
        except Exception as rate_error:
            logger.info("[FAIL] FEAGI simulation rate negotiation failed: %s", rate_error)
            _cleanup_mujoco_temp_dirs(temp_model_dir, keyframe_recovery_dir)
            return 1

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
            if sensors_only_feagi:
                logger.info("   FEAGI sensory streaming: ACTIVE (no motors)")
            else:
                logger.info("   FEAGI motor control: ACTIVE")
        else:
            logger.info("   FEAGI motor control: DISABLED (standalone mode)")

        start_time = time.time()
        frame_number = 0
        last_sim_time = float(data.time)
        motor_telemetry_state: dict[tuple[int, int], dict[str, float]] = {}
        last_motor_snapshot: dict[str, float] = {}
        motor_channel_labels: dict[tuple[int, int], str] = {
            (group_id, channel_index): actuator_name
            for (
                _motor,
                actuator_name,
                _actuator_idx,
                _min_val,
                _max_val,
                group_id,
                channel_index,
                _device_type,
                _encoding,
            ) in motors
        }
        telemetry_window_frames = 120
        recent_change_window_frames = SPEED * 5
        window_changed_samples = 0
        window_changed_channels: set[tuple[int, int]] = set()
        window_group_stats: dict[int, dict[str, float]] = {}
        runtime_unsupported_unit_write_failures: dict[str, int] = {}
        vision_renderer = None
        if vision_units and selected_vision_cameras:
            _modality, vision_width, vision_height, _channels, _unit, _group = vision_units[0]
            try:
                vision_renderer = mujoco.Renderer(
                    model,
                    int(vision_height),
                    int(vision_width),
                )
                logger.info(
                    "[VISION] Vision frame rendering active: cameras=%d resolution=%dx%d",
                    len(selected_vision_cameras),
                    int(vision_width),
                    int(vision_height),
                )
            except Exception as vision_renderer_error:
                logger.warning(
                    "[VISION][DISABLED] Failed to initialize renderer: %s",
                    vision_renderer_error,
                )
        window_joint_limit_corrections = 0
        window_joint_limit_joints: set[str] = set()

        while viewer.is_running() and time.time() - start_time < RUNTIME:
            step_start = time.time()
            sim_reset_detected = float(data.time) + 1e-12 < last_sim_time
            if sim_reset_detected:
                logger.info("[VIEW] Simulation reset detected; clearing motor runtime state")
                for (
                    motor,
                    _actuator_name,
                    actuator_idx,
                    min_val,
                    max_val,
                    _group_id,
                    _channel_index,
                    _device_type,
                    _encoding,
                ) in motors:
                    neutral_ctrl = 0.0
                    if isinstance(motor, ServoMotor):
                        neutral_ctrl = (min_val + max_val) / 2.0
                        if hasattr(motor, "_current_angle"):
                            setattr(motor, "_current_angle", neutral_ctrl)
                    elif isinstance(motor, RotaryMotor):
                        neutral_ctrl = 0.0
                        if hasattr(motor, "_current_speed"):
                            setattr(motor, "_current_speed", 0.0)
                    data.ctrl[actuator_idx] = neutral_ctrl
                    if hasattr(motor, "_last_rx_mode"):
                        setattr(motor, "_last_rx_mode", None)
                motor_telemetry_state.clear()
                last_motor_snapshot.clear()

            # Receive motor commands from FEAGI
            changed_in_tick = 0
            frame_group_stats: dict[int, dict[str, float]] = {}
            changed_channels_this_tick: list[tuple[int, int, str, float, float]] = []
            if feagi_enabled:
                try:
                    brain_output.receive()
                    motor_snapshot = getattr(brain_output, "_motor_data", {}) or {}
                    if isinstance(motor_snapshot, dict):
                        # Log only on snapshot changes to avoid spam.
                        changed = {}
                        for k, v in motor_snapshot.items():
                            try:
                                vf = float(v)
                            except Exception:
                                continue
                            pv = last_motor_snapshot.get(str(k))
                            if pv is None or abs(vf - pv) > 1e-6:
                                changed[str(k)] = vf
                        if changed:
                            logger.info("[MOTOR-SNAPSHOT] %s", changed)
                            last_motor_snapshot = {
                                str(k): float(v)
                                for k, v in motor_snapshot.items()
                                if isinstance(v, (int, float))
                            }

                    # Apply FEAGI commands to MuJoCo actuators
                    for (
                        motor,
                        actuator_name,
                        actuator_idx,
                        min_val,
                        max_val,
                        group_id,
                        channel_index,
                        _device_type,
                        _encoding,
                    ) in motors:
                        state_key = (group_id, channel_index)
                        prev_state = motor_telemetry_state.get(state_key, {})
                        last_seen_rx_seq = getattr(motor, "_last_seen_rx_seq", -1)
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

                            rx_mode = getattr(motor, "_last_rx_mode", None)
                            rx_seq = getattr(motor, "_rx_command_seq", None)
                            control_semantics = getattr(
                                motor,
                                "control_semantics",
                                "normalized_position",
                            )
                            is_incremental = rx_mode == "incremental"
                            is_effort_absolute = (
                                rx_mode == "absolute"
                                and control_semantics == "normalized_effort_drive"
                            )
                            has_new_packet = _motor_rx_is_new_packet(
                                rx_seq, int(last_seen_rx_seq)
                            )
                            if (is_incremental and not has_new_packet) or (
                                is_effort_absolute and not has_new_packet
                            ):
                                # Sparse command handling: hold the last applied target until
                                # a new packet arrives. This keeps both absolute and incremental
                                # control responsive under low-rate command streams.
                                held_ctrl = float(
                                    getattr(
                                        motor,
                                        "_latched_ctrl",
                                        prev_state.get("last_applied_ctrl", center),
                                    )
                                )
                                held_ctrl = max(min_val, min(max_val, held_ctrl))
                                data.ctrl[actuator_idx] = held_ctrl
                                applied_ctrl = held_ctrl
                                if half_range > 0:
                                    norm_cmd = _clamp(
                                        (held_ctrl - center) / half_range,
                                        -1.0,
                                        1.0,
                                    )
                                else:
                                    norm_cmd = 0.0
                            else:
                                data.ctrl[actuator_idx] = angle
                                applied_ctrl = angle
                                setattr(motor, "_latched_ctrl", float(angle))
                                if isinstance(rx_seq, int):
                                    setattr(motor, "_last_seen_rx_seq", int(rx_seq))
                        elif isinstance(motor, RotaryMotor):
                            speed = motor.get_speed()
                            norm_cmd = _clamp(float(speed), -1.0, 1.0)
                            if args.motor_gain != 1.0:
                                speed = max(-1.0, min(1.0, speed * args.motor_gain))

                            rx_mode = getattr(motor, "_last_rx_mode", None)
                            rx_seq = getattr(motor, "_rx_command_seq", None)
                            control_semantics = getattr(
                                motor,
                                "control_semantics",
                                "normalized_velocity",
                            )
                            is_incremental = rx_mode == "incremental"
                            is_effort_absolute = (
                                rx_mode == "absolute"
                                and control_semantics == "normalized_effort_drive"
                            )
                            has_new_packet = _motor_rx_is_new_packet(
                                rx_seq, int(last_seen_rx_seq)
                            )
                            if (is_incremental and not has_new_packet) or (
                                is_effort_absolute and not has_new_packet
                            ):
                                # Sparse command handling: keep previous commanded speed.
                                held_speed = _clamp(
                                    float(
                                        getattr(
                                            motor,
                                            "_latched_ctrl",
                                            prev_state.get("last_applied_ctrl", 0.0),
                                        )
                                    ),
                                    -1.0,
                                    1.0,
                                )
                                data.ctrl[actuator_idx] = held_speed
                                applied_ctrl = held_speed
                                norm_cmd = held_speed
                            else:
                                data.ctrl[actuator_idx] = speed
                                applied_ctrl = speed
                                setattr(motor, "_latched_ctrl", float(speed))
                                if isinstance(rx_seq, int):
                                    setattr(motor, "_last_seen_rx_seq", int(rx_seq))
                        else:
                            continue
                        state = motor_telemetry_state.setdefault(
                            state_key,
                            {
                                "last_norm_cmd": norm_cmd,
                                "last_applied_ctrl": applied_ctrl,
                                "last_change_frame": 0.0,
                                "seen_change": False,
                                "last_rx_seq": -1.0,
                            },
                        )
                        rx_seq_for_state = getattr(motor, "_rx_command_seq", None)
                        if isinstance(rx_seq_for_state, int):
                            state["last_rx_seq"] = float(rx_seq_for_state)
                        was_changed = (
                            abs(norm_cmd - state["last_norm_cmd"]) > 1e-6
                            or abs(applied_ctrl - state["last_applied_ctrl"]) > 1e-6
                        )
                        if was_changed:
                            changed_in_tick += 1
                            state["last_change_frame"] = float(frame_number)
                            state["seen_change"] = True
                            changed_channels_this_tick.append(
                                (
                                    group_id,
                                    channel_index,
                                    actuator_name,
                                    norm_cmd,
                                    applied_ctrl,
                                )
                            )
                            rx_value = getattr(motor, "_last_rx_value", None)
                            rx_raw = getattr(motor, "_last_rx_raw_value", None)
                            rx_mode = getattr(motor, "_last_rx_mode", None)
                            logger.info(
                                "[JOINT-MOVE] group=%d channel=%d actuator=%s rx=%s raw=%s mode=%s "
                                "norm=%.6f ctrl=%.6f delta_ctrl=%.6f motor_cls=%s.%s has_on_cb=%s has_rx_attr=%s",
                                group_id,
                                channel_index,
                                actuator_name,
                                "None" if rx_value is None else f"{rx_value:.6f}",
                                "None" if rx_raw is None else f"{rx_raw:.6f}",
                                "None" if rx_mode is None else str(rx_mode),
                                norm_cmd,
                                applied_ctrl,
                                applied_ctrl - state["last_applied_ctrl"],
                                motor.__class__.__module__,
                                motor.__class__.__name__,
                                hasattr(motor, "_on_motor_command"),
                                hasattr(motor, "_last_rx_value"),
                            )
                        state["last_norm_cmd"] = norm_cmd
                        state["last_applied_ctrl"] = applied_ctrl

                        stats = frame_group_stats.setdefault(
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

                    sensory_samples_written = 0

                    if vision_renderer is not None:
                        for (
                            camera_name,
                            camera_index,
                            group_index,
                        ) in selected_vision_cameras:
                            vision_frame = None
                            try:
                                vision_renderer.update_scene(data, camera=int(camera_index))
                                vision_frame = vision_renderer.render()
                            except Exception as vision_frame_error:
                                if frame_number % 240 == 0:
                                    logger.warning(
                                        "[VISION][SKIP][%s] render_failed=%s",
                                        camera_name,
                                        vision_frame_error,
                                    )
                                continue
                            try:
                                brain_output.write_sensor_vision_frame(
                                    group=group_index,
                                    channel_index=0,
                                    frame_rgb=vision_frame,
                                )
                                sensory_samples_written += 1
                            except Exception as vision_write_error:
                                if frame_number % 240 == 0:
                                    logger.warning(
                                        "[VISION][SKIP][%s] cache_write_failed=%s",
                                        camera_name,
                                        vision_write_error,
                                    )

                    if runtime_sensor_channels:
                        unsupported_units: set[str] = set()
                        writable_units = supported_scalar_sensor_units

                        for unit_key, group_index, channel_index, sensor_channel in (
                            sensor_cache_channel_layout
                        ):
                            if unit_key not in writable_units:
                                unsupported_units.add(unit_key)
                                continue
                            raw_value = _read_runtime_channel_sample(
                                data,
                                sensor_channel,
                                0,
                            )
                            if raw_value is None:
                                continue
                            normalized = _normalize_channel_value(
                                raw_value,
                                sensor_channel,
                            )
                            scalar_0_1 = _to_unit_scalar_0_1(
                                normalized,
                                sensor_channel,
                            )
                            try:
                                brain_output.write_sensor_scalar(
                                    unit_key=unit_key,
                                    group=group_index,
                                    channel_index=channel_index,
                                    scalar_0_1=scalar_0_1,
                                )
                                sensory_samples_written += 1
                            except ValueError:
                                unsupported_units.add(unit_key)
                                runtime_unsupported_unit_write_failures[unit_key] = (
                                    runtime_unsupported_unit_write_failures.get(unit_key, 0)
                                    + 1
                                )

                        if unsupported_units:
                            for unit_key in sorted(unsupported_units):
                                logger.warning(
                                    "[SENSORY][SKIP-WRITE][%s] skipped_writes=%d "
                                    "reason='unit not supported by scalar sensory cache'",
                                    unit_key,
                                    runtime_unsupported_unit_write_failures.get(unit_key, 0),
                                )

                    if sensory_samples_written > 0:
                        brain_output.flush_sensory_bytes()
                except Exception as e:
                    if frame_number % 120 == 0:
                        logger.info(f"   [WARN] FEAGI receive error: {e}")
                        import traceback
                        traceback.print_exc()

            if feagi_enabled and args.enable_telemetry:
                window_changed_samples += changed_in_tick
                for key in changed_channels_this_tick:
                    window_changed_channels.add((key[0], key[1]))
                for group_id, stats in frame_group_stats.items():
                    window_stats = window_group_stats.setdefault(
                        group_id,
                        {
                            "count": 0.0,
                            "changed": 0.0,
                            "sum_abs": 0.0,
                            "max_abs": 0.0,
                        },
                    )
                    window_stats["count"] += stats["count"]
                    window_stats["changed"] += stats["changed"]
                    window_stats["sum_abs"] += stats["sum_abs"]
                    window_stats["max_abs"] = max(
                        window_stats["max_abs"],
                        stats["max_abs"],
                    )

            # Step simulation
            mujoco.mj_step(model, data)
            limit_corrections = _enforce_joint_limits(model, data, joint_limit_guards)
            if limit_corrections and args.enable_telemetry:
                window_joint_limit_corrections += len(limit_corrections)
                for joint_name, _, _ in limit_corrections:
                    window_joint_limit_joints.add(joint_name)

            # Log every telemetry window (1 second at 120Hz), but throttle
            # output during completely idle periods.
            if args.enable_telemetry and frame_number % telemetry_window_frames == 0:
                elapsed = time.time() - start_time
                mode = "FEAGI" if feagi_enabled else "Standalone"
                if feagi_enabled:
                    global_max_abs = 0.0
                    for stats in window_group_stats.values():
                        global_max_abs = max(global_max_abs, stats["max_abs"])
                    recently_changed_channels = [
                        (group_id, channel_index, state)
                        for (group_id, channel_index), state in (
                            motor_telemetry_state.items()
                        )
                        if state.get("seen_change")
                        and (
                            frame_number - int(state["last_change_frame"])
                            <= recent_change_window_frames
                        )
                    ]
                    no_delta_observed_channels = [
                        (group_id, channel_index)
                        for (group_id, channel_index), state in (
                            motor_telemetry_state.items()
                        )
                        if not state.get("seen_change")
                    ]
                    has_recent_activity = (
                        window_changed_samples > 0
                        or len(recently_changed_channels) > 0
                        or global_max_abs > 0.0
                    )
                    emit_window = has_recent_activity
                    if emit_window:
                        logger.info(
                            "[FRAME] Frame %d | Time: %.1fs | Mode: %s",
                            frame_number,
                            elapsed,
                            mode,
                        )
                        logger.info(
                            (
                                "   [TELE] changed_samples=%d changed_channels=%d/%d "
                                "active_change_5s=%d no_delta_observed=%d global_max_abs_norm=%.4f"
                            ),
                            window_changed_samples,
                            len(window_changed_channels),
                            len(motors),
                            len(recently_changed_channels),
                            len(no_delta_observed_channels),
                            global_max_abs,
                        )
                        for group_id, stats in sorted(window_group_stats.items()):
                            count = max(1.0, stats["count"])
                            should_log_group = (
                                stats["changed"] > 0
                                or stats["max_abs"] > 0.0
                            )
                            if not should_log_group:
                                continue
                            logger.info(
                                (
                                    "   [TELE][GROUP %d] changed_samples=%d/%d "
                                    "mean_abs=%.4f max_abs=%.4f"
                                ),
                                group_id,
                                int(stats["changed"]),
                                int(stats["count"]),
                                stats["sum_abs"] / count,
                                stats["max_abs"],
                            )
                        if window_changed_channels:
                            sorted_changes = sorted(window_changed_channels)[:8]
                            change_text = ", ".join(
                                (
                                    f"g{group_id}:c{channel_index}"
                                    f"({motor_channel_labels.get((group_id, channel_index), 'unknown')})"
                                )
                                for group_id, channel_index in sorted_changes
                            )
                            logger.info("   [TELE][CHANGED] %s", change_text)

                        unchanged_duration_sorted = sorted(
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
                                if state.get("seen_change")
                            ),
                            reverse=True,
                        )
                        if unchanged_duration_sorted:
                            top_unchanged = unchanged_duration_sorted[:6]
                            unchanged_text = ", ".join(
                                (
                                    f"g{group_id}:c{channel_index}"
                                    f"({motor_channel_labels.get((group_id, channel_index), 'unknown')})"
                                    f"(unchanged_for_frames={stale_frames},norm={norm:.4f})"
                                )
                                for stale_frames, group_id, channel_index, norm in top_unchanged
                            )
                            logger.info(
                                "   [TELE][UNCHANGED_FOR_FRAMES] %s",
                                unchanged_text,
                            )
                        if no_delta_observed_channels:
                            no_delta_text = ", ".join(
                                (
                                    f"g{group_id}:c{channel_index}"
                                    f"({motor_channel_labels.get((group_id, channel_index), 'unknown')})"
                                )
                                for group_id, channel_index in no_delta_observed_channels[:6]
                            )
                            logger.info(
                                "   [TELE][NO_CHANGE_OBSERVED] %s",
                                no_delta_text,
                            )
                        if window_joint_limit_corrections > 0:
                            corrected_preview = ", ".join(
                                sorted(window_joint_limit_joints)[:8]
                            )
                            logger.info(
                                (
                                    "   [TELE][JOINT_LIMIT_GUARD] corrections=%d "
                                    "unique_joints=%d joints=%s"
                                ),
                                window_joint_limit_corrections,
                                len(window_joint_limit_joints),
                                corrected_preview,
                            )

                    window_changed_samples = 0
                    window_changed_channels.clear()
                    window_group_stats.clear()
                    window_joint_limit_corrections = 0
                    window_joint_limit_joints.clear()
                else:
                    logger.info(
                        "[FRAME] Frame %d | Time: %.1fs | Mode: %s",
                        frame_number,
                        elapsed,
                        mode,
                    )
                    if window_joint_limit_corrections > 0:
                        corrected_preview = ", ".join(
                            sorted(window_joint_limit_joints)[:8]
                        )
                        logger.info(
                            (
                                "   [TELE][JOINT_LIMIT_GUARD] corrections=%d "
                                "unique_joints=%d joints=%s"
                            ),
                            window_joint_limit_corrections,
                            len(window_joint_limit_joints),
                            corrected_preview,
                        )
                        window_joint_limit_corrections = 0
                        window_joint_limit_joints.clear()

            # Sync viewer
            viewer.sync()

            # Maintain simulation speed
            elapsed = time.time() - step_start
            sleep_time = (1.0 / SPEED) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            last_sim_time = float(data.time)
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

    _cleanup_mujoco_temp_dirs(temp_model_dir, keyframe_recovery_dir)

    logger.info("[DONE] MuJoCo controller shutdown complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())

