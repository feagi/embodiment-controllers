#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Auto-provision the PositionalServo (opse) output cortical area on a running FEAGI.

FEAGI must have a matching cortical area before a motor agent can register, and a
barebones genome does not include one. This module creates it idempotently, so the user
never has to touch the REST API by hand -- the controller calls
``ensure_servo_cortical_area()`` on startup.

A PositionalServo OPU is several cortical areas (subunits) created together: an
absolute one (subunit 0, config 1), an incremental one (subunit 1, config 257) and,
on newer FEAGI cores, a third subunit. FEAGI's registration step verifies they all
exist, so one POST creates the whole set. The subunit count is read from FEAGI, and an
existing area is found by its properties, because the cortical-ID byte layout differs
between FEAGI versions. The LED is driven by the absolute area.
"""

from __future__ import annotations

import base64
import logging

from feagi.genome.api import GenomeAPI, GenomeAPIError

logger = logging.getLogger("esp32_led.provisioner")

# Z resolution (steps) per servo. Matches the FEAGI PositionalServo template default.
_Z_RESOLUTION = 10


def _find_absolute_opse(api: GenomeAPI, group_id: int):
    """
    Return the cortical ID of the absolute opse area for ``group_id``, or None.

    Matched by properties, not by a hardcoded ID, because the ID byte layout differs
    between FEAGI versions.
    """
    opse_prefix = base64.b64encode(b"opse")[:5].decode("ascii")  # "b3BzZ"
    for cid in api.list_cortical_area_ids():
        if not cid.startswith(opse_prefix):
            continue
        try:
            props = api._request("POST", "/cortical_area/cortical_area_properties",
                                 json_data={"cortical_id": cid})
        except GenomeAPIError:
            continue
        if (props.get("cortical_subtype") == "opse"
                and int(props.get("group_id", -1)) == int(group_id)
                and props.get("encoding_type", props.get("coding_behavior")) == "Absolute"
                and int(props.get("subunit_id", 0)) == 0):
            return cid
    return None


def _subunit_configs(api: GenomeAPI) -> dict:
    """
    data_type_configs_by_subunit for every opse subunit this FEAGI defines.

    Older FEAGI cores define 2 subunits (absolute, incremental); newer ones define 3
    and reject a create request that leaves one out. Subunit 0 = absolute (1),
    subunit 1 = incremental (257); any further subunit gets the absolute config.
    """
    units = 2
    try:
        types = api._request("GET", "/cortical_area/opu/types")
        units = int((types.get("opse") or {}).get("units", 2))
    except Exception as exc:  # noqa: BLE001 - fall back to the 2-subunit layout
        logger.debug("Could not read opse unit count (%s); assuming 2.", exc)
    configs = {"0": 1, "1": 257}
    for subunit in range(2, units):
        configs[str(subunit)] = 1
    return configs


def ensure_servo_cortical_area(base_url: str, servo_count: int,
                               group_id: int = 0, timeout: float = 10.0) -> bool:
    """
    Ensure a PositionalServo OPU area sized for ``servo_count`` servos exists.

    Returns True if the area was created by this call, False if it already existed.
    Raises GenomeAPIError if FEAGI is unreachable or rejects the request.
    """
    api = GenomeAPI(base_url, timeout=timeout)

    found = _find_absolute_opse(api, group_id)
    if found:
        logger.info("PositionalServo area already present (%s); leaving it as-is.", found)
        return False

    payload = {
        "cortical_id": "opse",            # cortical *type* key, not the final ID
        "cortical_type": "OPU",
        "group_id": int(group_id),
        "device_count": int(servo_count),
        "coordinates_3d": [70, 0, 0],
        "per_device_dimensions": [1, 1, _Z_RESOLUTION],
        "data_type_configs_by_subunit": _subunit_configs(api),
    }

    logger.info("Creating PositionalServo OPU area for %d servo(s)...", servo_count)
    result = api._request("POST", "/cortical_area/cortical_area", json_data=payload)
    created = result.get("cortical_ids") if isinstance(result, dict) else result
    logger.info("Created servo cortical area(s): %s", created)
    return True


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    import sys
    url = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8000"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    try:
        made = ensure_servo_cortical_area(url, n)
        print("created" if made else "already-existed")
    except GenomeAPIError as e:
        print("ERROR:", e)
        sys.exit(1)
