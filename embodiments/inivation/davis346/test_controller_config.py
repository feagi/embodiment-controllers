"""Unit tests for controller_config.py JSON loading and CLI/config merging.

No mocking: exercises the real JSON parsing and dict-merge logic against
real temp files. No camera or FEAGI connection is required.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from controller_config import (
    apply_config_defaults,
    flatten_capabilities_config,
    flatten_networking_config,
    load_json_config,
)


class TestLoadJsonConfig:
    def test_missing_file_returns_empty_dict(self, tmp_path: Path) -> None:
        assert load_json_config(tmp_path / "does_not_exist.json") == {}

    def test_valid_file_is_parsed(self, tmp_path: Path) -> None:
        path = tmp_path / "config.json"
        path.write_text(json.dumps({"a": 1}), encoding="utf-8")
        assert load_json_config(path) == {"a": 1}

    def test_malformed_json_raises_value_error(self, tmp_path: Path) -> None:
        path = tmp_path / "bad.json"
        path.write_text("{not valid json", encoding="utf-8")
        with pytest.raises(ValueError):
            load_json_config(path)


class TestFlattenNetworkingConfig:
    def test_flattens_nested_sections(self) -> None:
        config = {
            "feagi_settings": {
                "feagi_host": "127.0.0.1",
                "feagi_api_port": 8000,
            },
            "agent_settings": {
                "agent_id": "davis346_event_camera",
                "auth_token_b64": "AAAA",
            },
        }
        flat = flatten_networking_config(config)
        assert flat["feagi_host"] == "127.0.0.1"
        assert flat["feagi_api_port"] == 8000
        assert flat["agent_id"] == "davis346_event_camera"
        assert flat["auth_token_b64"] == "AAAA"

    def test_empty_config_yields_all_none(self) -> None:
        flat = flatten_networking_config({})
        assert all(value is None for value in flat.values())

    def test_empty_auth_token_string_is_normalized_to_none(self) -> None:
        flat = flatten_networking_config({"agent_settings": {"auth_token_b64": ""}})
        assert flat["auth_token_b64"] is None


class TestFlattenCapabilitiesConfig:
    def test_flattens_nested_sections(self) -> None:
        config = {
            "vision": {
                "cortical_group_id": 0,
                "coordinates_3d": [-100, 30, 0],
                "event_potential": 200.0,
            },
            "camera": {"camera_serial": "12345"},
            "playback": {"aedat_path": "/tmp/rec.aedat4", "loop": True},
        }
        flat = flatten_capabilities_config(config)
        assert flat["cortical_group_id"] == 0
        assert flat["cortical_coordinates_3d"] == [-100, 30, 0]
        assert flat["event_potential"] == 200.0
        assert flat["camera_serial"] == "12345"
        assert flat["aedat_path"] == "/tmp/rec.aedat4"
        assert flat["loop"] is True

    def test_empty_camera_serial_string_is_normalized_to_none(self) -> None:
        flat = flatten_capabilities_config({"camera": {"camera_serial": ""}})
        assert flat["camera_serial"] is None


class TestApplyConfigDefaults:
    def test_fills_unset_cli_args_from_defaults(self) -> None:
        args_dict = {"feagi_host": None, "feagi_api_port": None}
        defaults = {"feagi_host": "127.0.0.1", "feagi_api_port": 8000}
        merged = apply_config_defaults(args_dict, defaults)
        assert merged == {"feagi_host": "127.0.0.1", "feagi_api_port": 8000}

    def test_cli_provided_value_is_not_overridden(self) -> None:
        args_dict = {"feagi_host": "192.168.1.50"}
        defaults = {"feagi_host": "127.0.0.1"}
        merged = apply_config_defaults(args_dict, defaults)
        assert merged["feagi_host"] == "192.168.1.50"

    def test_cli_provided_falsy_but_not_none_value_is_preserved(self) -> None:
        # 0 and False are meaningful values, not "unset".
        args_dict = {"cortical_group_id": 0, "loop": False}
        defaults = {"cortical_group_id": 5, "loop": True}
        merged = apply_config_defaults(args_dict, defaults)
        assert merged["cortical_group_id"] == 0
        assert merged["loop"] is False

    def test_missing_default_leaves_value_unset(self) -> None:
        args_dict = {"cortical_group_id": None}
        defaults = {"cortical_group_id": None}
        merged = apply_config_defaults(args_dict, defaults)
        assert merged["cortical_group_id"] is None

    def test_keys_absent_from_args_dict_are_ignored(self) -> None:
        merged = apply_config_defaults({}, {"feagi_host": "127.0.0.1"})
        assert merged == {}
