"""Load and merge bundled JSON configuration for the DAVIS346 controller.

This module is compute-only: it reads JSON files and produces plain dicts.
It does not talk to FEAGI or a camera. Values loaded here are defaults only
-- CLI flags always take precedence, and any field left unset by both the
config files and the CLI still fails the controller's explicit
required-argument check (no silent fallback).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

DEFAULT_NETWORKING_FILENAME = "networking.json"
DEFAULT_CAPABILITIES_FILENAME = "capabilities.json"


def load_json_config(path: Path) -> Dict[str, Any]:
    """Load a JSON config file, or return {} if it does not exist.

    Args:
        path: Path to a JSON config file.

    Returns:
        The parsed JSON object, or an empty dict if the file is absent.

    Raises:
        ValueError: If the file exists but is not valid JSON.
    """
    if not path.is_file():
        return {}
    try:
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in config file {path}: {exc}") from exc


def flatten_networking_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten networking.json's nested sections into CLI-argument-name keys.

    Args:
        config: Parsed networking.json content (or {} if absent).

    Returns:
        Flat dict keyed by the same names as the controller's CLI arguments
        (e.g. "feagi_host", "agent_descriptor_manufacturer").
    """
    feagi_settings = config.get("feagi_settings", {})
    agent_settings = config.get("agent_settings", {})
    return {
        "feagi_host": feagi_settings.get("feagi_host"),
        "feagi_api_port": feagi_settings.get("feagi_api_port"),
        "feagi_registration_port": feagi_settings.get("feagi_registration_port"),
        "feagi_sensory_port": feagi_settings.get("feagi_sensory_port"),
        "feagi_motor_port": feagi_settings.get("feagi_motor_port"),
        "feagi_http_timeout_s": feagi_settings.get("feagi_http_timeout_s"),
        "heartbeat_interval_s": feagi_settings.get("heartbeat_interval_s"),
        "connection_timeout_ms": feagi_settings.get("connection_timeout_ms"),
        "registration_retries": feagi_settings.get("registration_retries"),
        "agent_id": agent_settings.get("agent_id"),
        "agent_descriptor_manufacturer": agent_settings.get("agent_descriptor_manufacturer"),
        "agent_descriptor_name": agent_settings.get("agent_descriptor_name"),
        "agent_descriptor_version": agent_settings.get("agent_descriptor_version"),
        "auth_token_b64": agent_settings.get("auth_token_b64") or None,
    }


def flatten_capabilities_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten capabilities.json's nested sections into CLI-argument-name keys.

    Args:
        config: Parsed capabilities.json content (or {} if absent).

    Returns:
        Flat dict keyed by the same names as the controller's CLI arguments
        (e.g. "cortical_id", "aedat_path").
    """
    vision = config.get("vision", {})
    camera = config.get("camera", {})
    playback = config.get("playback", {})
    return {
        "cortical_group_id": vision.get("cortical_group_id"),
        "cortical_coordinates_3d": vision.get("coordinates_3d"),
        "event_potential": vision.get("event_potential"),
        "camera_serial": camera.get("camera_serial") or None,
        "aedat_path": playback.get("aedat_path") or None,
        "loop": playback.get("loop"),
    }


def apply_config_defaults(
    args_dict: Dict[str, Any], defaults: Dict[str, Optional[Any]]
) -> Dict[str, Any]:
    """Fill unset CLI arguments (None) from bundled config defaults.

    CLI-provided values always win: a key is only replaced when its current
    value is None. Keys not present in `args_dict` are ignored.

    Args:
        args_dict: Parsed CLI arguments as a dict (e.g. vars(argparse.Namespace)).
        defaults: Flattened config values, e.g. from flatten_networking_config.

    Returns:
        A new dict with unset fields filled from defaults.
    """
    merged = dict(args_dict)
    for key, default_value in defaults.items():
        if key in merged and merged[key] is None and default_value is not None:
            merged[key] = default_value
    return merged
