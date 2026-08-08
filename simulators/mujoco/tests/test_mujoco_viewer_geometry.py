"""Tests for MuJoCo viewer geometry env parsing and file persistence."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco_viewer_geometry as viewer_geometry


def test_target_rect_from_env_parses_launch_variables(monkeypatch) -> None:
    monkeypatch.setenv("FEAGI_WINDOW_X", "100")
    monkeypatch.setenv("FEAGI_WINDOW_Y", "200")
    monkeypatch.setenv("FEAGI_WINDOW_WIDTH", "640")
    monkeypatch.setenv("FEAGI_WINDOW_HEIGHT", "480")
    assert viewer_geometry._target_rect_from_env() == (100, 200, 640, 480)


def test_target_rect_from_env_returns_none_when_missing(monkeypatch) -> None:
    for key in ("FEAGI_WINDOW_X", "FEAGI_WINDOW_Y", "FEAGI_WINDOW_WIDTH", "FEAGI_WINDOW_HEIGHT"):
        monkeypatch.delenv(key, raising=False)
    assert viewer_geometry._target_rect_from_env() is None


def test_write_geometry_file_roundtrip(tmp_path: Path, monkeypatch) -> None:
    out = tmp_path / "geom.json"
    monkeypatch.setenv("FEAGI_CONTROLLER_GEOMETRY_FILE", str(out))
    viewer_geometry._write_geometry_file((10, 20, 300, 400))
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["x"] == 10
    assert payload["y"] == 20
    assert payload["width"] == 300
    assert payload["height"] == 400


def test_write_geometry_file_atomic_noop_when_no_env(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.delenv("FEAGI_CONTROLLER_GEOMETRY_FILE", raising=False)
    # Should not raise even with no path configured
    viewer_geometry._write_geometry_file((0, 0, 100, 100))


def test_apply_initial_geometry_macos_spawns_background_thread(monkeypatch) -> None:
    """On macOS the apply function starts a daemon thread — no GLFW calls."""
    monkeypatch.setattr(viewer_geometry, "_is_macos", lambda: True)
    monkeypatch.setenv("FEAGI_WINDOW_X", "100")
    monkeypatch.setenv("FEAGI_WINDOW_Y", "50")
    monkeypatch.setenv("FEAGI_WINDOW_WIDTH", "400")
    monkeypatch.setenv("FEAGI_WINDOW_HEIGHT", "300")

    threads_started: list[str] = []

    import threading
    original_start = threading.Thread.start

    def mock_start(self: threading.Thread) -> None:
        threads_started.append(self.name)

    monkeypatch.setattr(threading.Thread, "start", mock_start)
    viewer_geometry.apply_initial_geometry(object())  # No GLFW viewer needed
    assert any("mujoco-geom" in n for n in threads_started)


def test_apply_initial_geometry_noop_when_no_env(monkeypatch) -> None:
    """No thread or GLFW call when env vars are absent."""
    for key in ("FEAGI_WINDOW_X", "FEAGI_WINDOW_Y", "FEAGI_WINDOW_WIDTH", "FEAGI_WINDOW_HEIGHT"):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setattr(viewer_geometry, "_is_macos", lambda: True)
    # Should return without touching anything
    viewer_geometry.apply_initial_geometry(object())


def test_macos_do_set_returns_false_when_no_window(monkeypatch) -> None:
    """_macos_do_set returns False gracefully when no NSWindow is found."""
    monkeypatch.setattr(viewer_geometry, "_macos_get_window", lambda _lib: None)
    monkeypatch.setattr(viewer_geometry, "_macos_get_screen_height", lambda _lib: 900.0)
    assert viewer_geometry._macos_do_set(100, 0, 400, 300) is False


def test_viewer_geometry_recorder_tick_throttled(monkeypatch) -> None:
    """tick() only persists geometry after the configured interval."""
    calls: list[object] = []
    monkeypatch.setattr(viewer_geometry, "persist_geometry", lambda v: calls.append(v))
    recorder = viewer_geometry.ViewerGeometryRecorder(interval_s=60.0)
    fake_viewer = object()
    recorder.tick(fake_viewer)  # First call — should persist
    recorder.tick(fake_viewer)  # Immediate second call — throttled
    assert len(calls) == 1


def test_viewer_geometry_recorder_flush_always_persists(monkeypatch) -> None:
    calls: list[object] = []
    monkeypatch.setattr(viewer_geometry, "persist_geometry", lambda v: calls.append(v))
    recorder = viewer_geometry.ViewerGeometryRecorder(interval_s=60.0)
    fake_viewer = object()
    recorder.flush(fake_viewer)
    recorder.flush(fake_viewer)
    assert len(calls) == 2
