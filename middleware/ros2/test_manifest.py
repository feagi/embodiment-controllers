#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Smoke tests for the ros2 middleware manifest (no network, no ROS runtime)."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

_MANIFEST = Path(__file__).resolve().parent / "manifest.json"


class Ros2ManifestTests(unittest.TestCase):
    """Validate manifest invariants required by Composer and embodiments."""

    def test_manifest_bundle_id_ros2(self) -> None:
        """bundle.id must stay the canonical controller_id for Composer and embodiments."""
        data = json.loads(_MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(data["bundle"]["id"], "ros2")
        self.assertTrue(data["bundle"]["version"])
        self.assertEqual(data["installation"]["entry_point"], "ros_connector_bridge.py")
        self.assertEqual(data["controller"]["category"], "middleware")

    def test_manifest_source_path_matches_tree(self) -> None:
        data = json.loads(_MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(data["source"]["path"], "controllers/middleware/ros2")


if __name__ == "__main__":
    unittest.main()
