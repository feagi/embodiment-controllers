#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Auto-provision the PositionalServo (opse) output cortical area on a running FEAGI.

FEAGI must have a matching cortical area before a motor agent can register, and a
barebones genome does not include one. This module creates it idempotently, so the user
never has to touch the REST API by hand -- the controller calls
``ensure_servo_cortical_area()`` on startup.

A PositionalServo OPU is TWO cortical areas: an absolute one and an incremental one.
FEAGI's registration step verifies both exist, so we always create the pair. A single
POST with ``device_count = N`` and ``data_type_configs_by_subunit = {"0": 1, "1": 257}``
creates both:

    subunit 0, config 1   -> absolute     opse area  (id b3BzZQEAAAA=)
    subunit 1, config 257 -> incremental  opse area  (id b3BzZQEBAQA=)
"""

from __future__ import annotations

import base64
import logging

from feagi.genome.api import GenomeAPI, GenomeAPIError

logger = logging.getLogger("esp32_led.provisioner")

# Cortical IDs FEAGI derives for a group-0 PositionalServo OPU. We only need the
# absolute one to detect "already provisioned"; both are created together.
_ABSOLUTE_OPSE_ID = "b3BzZQEAAAA="      # base64 of b"opse\x01\x00\x00\x00"
_INCREMENTAL_OPSE_ID = "b3BzZQEBAQA="   # base64 of b"opse\x01\x01\x01\x00"

# Z resolution (steps) per servo. Matches the FEAGI PositionalServo template default.
_Z_RESOLUTION = 10


def ensure_servo_cortical_area(base_url: str, servo_count: int,
                               group_id: int = 0, timeout: float = 10.0) -> bool:
    """
    Ensure a PositionalServo OPU area sized for ``servo_count`` servos exists.

    Returns True if the area was created by this call, False if it already existed.
    Raises GenomeAPIError if FEAGI is unreachable or rejects the request.
    """
    api = GenomeAPI(base_url, timeout=timeout)

    existing = set(api.list_cortical_area_ids())
    if _ABSOLUTE_OPSE_ID in existing:
        logger.info("PositionalServo area already present (%s); leaving it as-is.",
                    _ABSOLUTE_OPSE_ID)
        return False

    payload = {
        "cortical_id": "opse",            # cortical *type* key, not the final ID
        "cortical_type": "OPU",
        "group_id": int(group_id),
        "device_count": int(servo_count),
        "coordinates_3d": [70, 0, 0],
        "per_device_dimensions": [1, 1, _Z_RESOLUTION],
        "data_type_configs_by_subunit": {"0": 1, "1": 257},
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
