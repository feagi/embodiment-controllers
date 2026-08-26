"""Unit tests for cortical_area_provisioner.py.

No mocking library: FEAGI's REST API is outside the domain of this unit
(it is exercised by exactly one manual integration path, `_create_simple_vision_cortical_area`'s
`genome_api._request` call), so these tests use a small in-memory fake standing in for
`feagi.genome.api.GenomeAPI` to exercise the create/resize/no-op decision logic, which is the
actual subject under test.
"""

from __future__ import annotations

from typing import Any, Dict, List

import pytest

from cortical_area_provisioner import (
    compute_isvi_cortical_id,
    ensure_simple_vision_cortical_area,
)


class FakeGenomeAPI:
    """In-memory stand-in for feagi.genome.api.GenomeAPI's used surface."""

    def __init__(self, existing_areas: Dict[str, Dict[str, Any]] | None = None) -> None:
        self.areas: Dict[str, Dict[str, Any]] = dict(existing_areas or {})
        self.created_payloads: List[Dict[str, Any]] = []
        self.update_calls: List[Any] = []

    def list_cortical_area_ids(self) -> List[str]:
        return list(self.areas.keys())

    def get_cortical_area_properties(self, cortical_id: str) -> Dict[str, Any]:
        return self.areas[cortical_id]

    def update_cortical_area(self, cortical_id: str, changes: Dict[str, Any]) -> Dict[str, Any]:
        self.update_calls.append((cortical_id, changes))
        self.areas[cortical_id]["cortical_dimensions"] = changes["cortical_dimensions"]
        return {"cortical_id": cortical_id}

    def _request(self, method: str, path: str, json_data: Dict[str, Any]) -> Dict[str, Any]:
        self.created_payloads.append(json_data)
        cortical_id = compute_isvi_cortical_id(json_data["group_id"])
        self.areas[cortical_id] = {
            "cortical_id": cortical_id,
            "cortical_dimensions": list(json_data["per_device_dimensions"]),
        }
        return {"cortical_id": cortical_id}


class TestComputeIsviCorticalId:
    def test_matches_known_live_genome_id_for_group_zero(self) -> None:
        # Confirmed against a live FEAGI genome's default simple-vision area (group 0).
        assert compute_isvi_cortical_id(0) == "aXN2aQkAAAA="

    def test_group_index_changes_last_byte_only(self) -> None:
        # Confirmed against a live FEAGI genome's default simple-vision areas.
        assert compute_isvi_cortical_id(1) == "aXN2aQkAAAE="
        assert compute_isvi_cortical_id(9) == "aXN2aQkAAAk="

    def test_rejects_out_of_range_unit_index(self) -> None:
        with pytest.raises(ValueError):
            compute_isvi_cortical_id(256)
        with pytest.raises(ValueError):
            compute_isvi_cortical_id(-1)

    def test_rejects_out_of_range_subunit_index(self) -> None:
        with pytest.raises(ValueError):
            compute_isvi_cortical_id(0, subunit_index=256)


class TestEnsureSimpleVisionCorticalArea:
    def test_creates_area_when_missing(self) -> None:
        fake = FakeGenomeAPI()
        cortical_id = ensure_simple_vision_cortical_area(
            fake, unit_index=0, width=346, height=260, depth=2, coordinates_3d=(-100, 30, 0)
        )
        assert cortical_id == compute_isvi_cortical_id(0)
        assert len(fake.created_payloads) == 1
        payload = fake.created_payloads[0]
        assert payload["cortical_type"] == "IPU"
        assert payload["group_id"] == 0
        assert payload["per_device_dimensions"] == [346, 260, 2]
        assert payload["coordinates_3d"] == [-100, 30, 0]
        assert fake.update_calls == []

    def test_resizes_area_when_dimensions_mismatch(self) -> None:
        cortical_id = compute_isvi_cortical_id(0)
        fake = FakeGenomeAPI(
            existing_areas={
                cortical_id: {"cortical_id": cortical_id, "cortical_dimensions": [64, 64, 3]}
            }
        )
        result = ensure_simple_vision_cortical_area(
            fake, unit_index=0, width=346, height=260, depth=2, coordinates_3d=(-100, 30, 0)
        )
        assert result == cortical_id
        assert fake.created_payloads == []
        assert fake.update_calls == [(cortical_id, {"cortical_dimensions": [346, 260, 2]})]
        assert fake.areas[cortical_id]["cortical_dimensions"] == [346, 260, 2]

    def test_no_op_when_dimensions_already_match(self) -> None:
        cortical_id = compute_isvi_cortical_id(0)
        fake = FakeGenomeAPI(
            existing_areas={
                cortical_id: {"cortical_id": cortical_id, "cortical_dimensions": [346, 260, 2]}
            }
        )
        ensure_simple_vision_cortical_area(
            fake, unit_index=0, width=346, height=260, depth=2, coordinates_3d=(-100, 30, 0)
        )
        assert fake.created_payloads == []
        assert fake.update_calls == []
