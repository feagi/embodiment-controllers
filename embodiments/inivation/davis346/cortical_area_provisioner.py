"""Ensure the DAVIS346 target simple-vision cortical area exists and is sized
correctly before the controller connects and starts streaming.

Why this exists: feagi-python-sdk's `FeagiAgentClient.configure(vision_unit=...)`
only records capability metadata client-side. It is never transmitted to FEAGI
during registration (`RegistrationRequest` on the wire has no vision-capability
fields), so FEAGI cannot auto-create or resize the target area from that call
alone. This module instead manages the cortical area directly over FEAGI's
REST API via `feagi.genome.api.GenomeAPI`, mirroring what a human would
otherwise do once via Brain Visualizer before running the controller.

This module is DAVIS346-specific: it only manages a single-device simple
vision ("isvi") cortical area addressed by group/unit index.
"""

from __future__ import annotations

import logging
from typing import Any, List, Sequence

logger = logging.getLogger(__name__)

# feagi-structures: 4-char cortical_type_key for the simple-vision sensory unit.
_ISVI_CORTICAL_TYPE_KEY = "isvi"

# feagi-structures IOCorticalAreaConfigurationFlag: variant 9 = CartesianPlane,
# frame-change bit 0 = Absolute. This is the framing FEAGI's default genome
# templates use for simple vision (confirmed against a live genome's isvi
# cortical IDs), so a controller-created area must use the same encoding to
# be indistinguishable from one created by the standard template.
_CARTESIAN_PLANE_ABSOLUTE_CONFIG_FLAG = 9

# DAVIS346 registers as one physical camera per simple-vision cortical group.
_SIMPLE_VISION_DEVICE_COUNT = 1


def compute_isvi_cortical_id(unit_index: int, subunit_index: int = 0) -> str:
    """Compute the deterministic simple-vision cortical ID FEAGI derives server-side.

    Mirrors feagi-core's `IOCorticalAreaConfigurationFlag::as_io_cortical_id` for
    the "isvi" unit with CartesianPlane(Absolute) framing. This lets the caller
    target an area with only a group/unit index instead of a separately
    configured, genome-specific cortical ID.

    Args:
        unit_index: Cortical group/unit index (wire byte 7); this is
            capabilities.json's "cortical_group_id".
        subunit_index: Sub-area index within the unit (wire byte 6). Simple
            vision units have exactly one subunit, so this is 0.

    Returns:
        Base64-encoded 8-byte cortical ID.
    """
    if not (0 <= unit_index <= 255):
        raise ValueError(f"unit_index must be 0-255, got {unit_index}")
    if not (0 <= subunit_index <= 255):
        raise ValueError(f"subunit_index must be 0-255, got {subunit_index}")

    config_bytes = _CARTESIAN_PLANE_ABSOLUTE_CONFIG_FLAG.to_bytes(2, byteorder="little")
    raw = (
        _ISVI_CORTICAL_TYPE_KEY.encode("ascii")
        + config_bytes
        + bytes([subunit_index, unit_index])
    )
    import base64

    return base64.b64encode(raw).decode("ascii")


def ensure_simple_vision_cortical_area(
    genome_api: Any,
    unit_index: int,
    width: int,
    height: int,
    depth: int,
    coordinates_3d: Sequence[int],
) -> str:
    """Create or resize the target simple-vision cortical area to match the sensor.

    Args:
        genome_api: A `feagi.genome.api.GenomeAPI` pointed at the target FEAGI
            instance's REST API.
        unit_index: Cortical group/unit index for this vision agent
            (capabilities.json "cortical_group_id").
        width: Required cortical area width (sensor width in pixels).
        height: Required cortical area height (sensor height in pixels).
        depth: Required cortical area depth (2 for DAVIS ON/OFF polarity planes).
        coordinates_3d: (x, y, z) position to place the area at if it must be
            created. Ignored if the area already exists.

    Returns:
        The area's base64 cortical ID, usable directly as the XYZP injection
        target.
    """
    cortical_id = compute_isvi_cortical_id(unit_index)
    desired_dimensions = [int(width), int(height), int(depth)]

    existing_ids = set(genome_api.list_cortical_area_ids())
    if cortical_id not in existing_ids:
        logger.info(
            "Simple-vision cortical area %s (group %s) not found; creating at %sx%sx%s",
            cortical_id,
            unit_index,
            *desired_dimensions,
        )
        _create_simple_vision_cortical_area(
            genome_api, unit_index, desired_dimensions, coordinates_3d
        )
        return cortical_id

    properties = genome_api.get_cortical_area_properties(cortical_id)
    current_dimensions = list(properties.get("cortical_dimensions") or [])
    if current_dimensions != desired_dimensions:
        logger.info(
            "Resizing simple-vision cortical area %s from %s to %s",
            cortical_id,
            current_dimensions,
            desired_dimensions,
        )
        genome_api.update_cortical_area(cortical_id, {"cortical_dimensions": desired_dimensions})
    else:
        logger.info(
            "Simple-vision cortical area %s already at %s; no resize needed",
            cortical_id,
            desired_dimensions,
        )
    return cortical_id


def _create_simple_vision_cortical_area(
    genome_api: Any,
    unit_index: int,
    dimensions: List[int],
    coordinates_3d: Sequence[int],
) -> None:
    """POST a new simple-vision (isvi) IPU cortical area.

    NOTE - feagi-python-sdk gap: `feagi.genome.api.GenomeAPI` has no public
    method for creating IPU/OPU areas. Its only creation method,
    `add_custom_cortical_area`, creates a plain "custom" area with no sensory
    device encoder attached -- not a real IPU -- so it cannot be used here.
    This function calls FEAGI's `POST /v1/cortical_area/cortical_area`
    directly through `GenomeAPI`'s internal request helper as a documented
    workaround until the SDK grows a proper
    `create_ipu_cortical_area`/`create_opu_cortical_area` method.
    """
    payload = {
        "cortical_id": _ISVI_CORTICAL_TYPE_KEY,  # this field is the cortical_type_key, not an ID
        "cortical_type": "IPU",
        "group_id": unit_index,
        "device_count": _SIMPLE_VISION_DEVICE_COUNT,
        "coordinates_3d": list(coordinates_3d),
        "per_device_dimensions": list(dimensions),
        "data_type_configs_by_subunit": {"0": _CARTESIAN_PLANE_ABSOLUTE_CONFIG_FLAG},
    }
    genome_api._request("POST", "/cortical_area/cortical_area", json_data=payload)
