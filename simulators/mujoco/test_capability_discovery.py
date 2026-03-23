#!/usr/bin/env python3
"""
Tests for MuJoCo capability discovery and strict-mode sensor validation.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import mujoco
import pytest


def _load_controller_module():
    """Load controller module with lightweight FEAGI stubs."""
    outputs_mod = types.ModuleType("feagi.pns.outputs")
    outputs_mod.ServoMotor = type("ServoMotor", (), {})
    outputs_mod.RotaryMotor = type("RotaryMotor", (), {})

    brain_output_mod = types.SimpleNamespace()
    pns_mod = types.ModuleType("feagi.pns")
    pns_mod.outputs = outputs_mod
    pns_mod.brain_output = brain_output_mod

    feagi_mod = types.ModuleType("feagi")
    feagi_mod.pns = pns_mod

    sys.modules.setdefault("feagi", feagi_mod)
    sys.modules.setdefault("feagi.pns", pns_mod)
    sys.modules.setdefault("feagi.pns.outputs", outputs_mod)

    controller_path = (
        Path(__file__).resolve().parent / "controller.py"
    )
    spec = importlib.util.spec_from_file_location("mujoco_controller", controller_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_spot_discovers_derived_channels_strict_mode():
    """Spot model should expose deterministic derived channels in strict mode."""
    controller = _load_controller_module()
    spot_xml = (
        Path(__file__).resolve().parents[3]
        / "embodiments"
        / "mujoco_boston_dynamics_spot"
        / "model"
        / "spot.xml"
    )
    model = mujoco.MjModel.from_xml_path(str(spot_xml))
    sensor_map, runtime_channels = controller._build_sensor_registration_map(
        model,
        str(spot_xml),
        strict_mode=True,
    )

    assert "Proximity" in sensor_map
    assert "MiscData" in sensor_map
    assert any(ch.source_kind == "qpos" for ch in runtime_channels)
    assert any(ch.source_kind == "qvel" for ch in runtime_channels)
    assert any(ch.source_kind == "actuator_force" for ch in runtime_channels)
    assert any(
        ch.source_kind == "qpos" and ch.normalize_mode == "range_0_1"
        for ch in runtime_channels
    )
    assert any(
        ch.source_kind == "qvel" and ch.normalize_mode == "signed_0_1"
        for ch in runtime_channels
    )


def test_strict_mode_rejects_unsupported_sensor_type(tmp_path: Path):
    """Unsupported MuJoCo sensor tags must fail fast in strict mode."""
    controller = _load_controller_module()
    xml_path = tmp_path / "unsupported_sensor.xml"
    xml_path.write_text(
        """
<mujoco model="unsupported_sensor">
  <worldbody>
    <body name="base">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.2" size="0.02"/>
    </body>
  </worldbody>
  <sensor>
    <clock name="sim_clock"/>
  </sensor>
</mujoco>
        """.strip(),
        encoding="utf-8",
    )

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    with pytest.raises(RuntimeError, match="Unsupported MuJoCo sensor types"):
        controller._build_sensor_registration_map(
            model,
            str(xml_path),
            strict_mode=True,
        )


def test_strict_mode_accepts_geomdist_sensor(tmp_path: Path):
    """geomdist sensor should be treated as supported proximity data."""
    controller = _load_controller_module()
    xml_path = tmp_path / "geomdist_sensor.xml"
    xml_path.write_text(
        """
<mujoco model="geomdist_sensor">
  <worldbody>
    <body name="base">
      <geom name="g1" type="sphere" pos="0 0 0" size="0.02"/>
      <geom name="g2" type="sphere" pos="0 0 0.1" size="0.02"/>
    </body>
  </worldbody>
  <sensor>
    <geomdist name="gap_distance" geom1="g1" geom2="g2"/>
  </sensor>
</mujoco>
        """.strip(),
        encoding="utf-8",
    )

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    sensor_map, runtime_channels = controller._build_sensor_registration_map(
        model,
        str(xml_path),
        strict_mode=True,
    )

    assert "Proximity" in sensor_map
    assert any(entry.sensor_tag == "geomdist" for entry in sensor_map["Proximity"])
    assert any(channel.sensor_tag == "geomdist" for channel in runtime_channels)


def test_strict_mode_accepts_velocimeter_sensor(tmp_path: Path):
    """velocimeter sensor should be treated as supported proximity data."""
    controller = _load_controller_module()
    xml_path = tmp_path / "velocimeter_sensor.xml"
    xml_path.write_text(
        """
<mujoco model="velocimeter_sensor">
  <worldbody>
    <body name="base">
      <geom type="sphere" pos="0 0 0" size="0.02"/>
      <site name="imu_site" pos="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <velocimeter name="linvel" site="imu_site"/>
  </sensor>
</mujoco>
        """.strip(),
        encoding="utf-8",
    )

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    sensor_map, runtime_channels = controller._build_sensor_registration_map(
        model,
        str(xml_path),
        strict_mode=True,
    )

    assert "Proximity" in sensor_map
    assert any(entry.sensor_tag == "velocimeter" for entry in sensor_map["Proximity"])
    assert any(channel.sensor_tag == "velocimeter" for channel in runtime_channels)


def test_spot_name_mapping_translates_motor_and_sensor_labels():
    """Name mapping table should translate Spot abbreviations to readable labels."""
    controller = _load_controller_module()
    spot_xml = (
        Path(__file__).resolve().parents[3]
        / "embodiments"
        / "mujoco_boston_dynamics_spot"
        / "model"
        / "spot.xml"
    )
    model = mujoco.MjModel.from_xml_path(str(spot_xml))
    translator = controller._load_mujoco_name_translator(str(spot_xml))

    motors, _group_names, _group_channels, group_channel_metadata = (
        controller.register_mujoco_motors(
            model,
            str(spot_xml),
            1.0,
            translator,
        )
    )
    assert motors, "Expected Spot model to register motors"
    positional_channels = [
        channel_meta
        for per_group in group_channel_metadata.values()
        for channel_meta in per_group.get("positional_servo", [])
    ]
    assert any(
        entry["channel_name"] == "front_right_hip_roll"
        for entry in positional_channels
    )

    sensor_map, _runtime_channels = controller._build_sensor_registration_map(
        model,
        str(spot_xml),
        strict_mode=True,
        name_translator=translator,
    )
    proximity_labels = [entry.display_name for entry in sensor_map["Proximity"]]
    assert any(
        label == "joint_position_front_right_hip_roll"
        for label in proximity_labels
    )


def test_name_mapping_translator_uses_per_model_tables():
    """Translator should load exact mappings from each model folder."""
    controller = _load_controller_module()

    anymal_path = (
        Path(__file__).resolve().parents[3]
        / "embodiments"
        / "mujoco_anybotics_anymal_b"
        / "model"
        / "scene.xml"
    )
    anymal_translator = controller._load_mujoco_name_translator(str(anymal_path))
    assert (
        anymal_translator.translate_joint("LF_HAA")
        == "left_front_hip_abduction_adduction"
    )

    leap_path = (
        Path(__file__).resolve().parents[3]
        / "embodiments"
        / "mujoco_leap_hand"
        / "model"
        / "scene_right.xml"
    )
    leap_translator = controller._load_mujoco_name_translator(str(leap_path))
    assert (
        leap_translator.translate_joint("rf_mcp")
        == "ring_finger_metacarpophalangeal"
    )
