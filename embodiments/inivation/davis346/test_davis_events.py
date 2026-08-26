"""Unit tests for davis_events.py XYZP conversion and playback timing.

No mocking: these tests exercise the real numpy conversion logic directly.
No camera hardware or FEAGI connection is required.
"""

from __future__ import annotations

import numpy as np
import pytest

from davis_events import (
    DAVIS346_HEIGHT,
    DAVIS346_WIDTH,
    POLARITY_OFF,
    POLARITY_ON,
    events_to_xyzp,
    playback_delay_s,
    polarity_events_to_columns,
)


class TestEventsToXyzp:
    def test_empty_input_returns_empty_arrays(self) -> None:
        xs, ys, zs, ps = events_to_xyzp(
            np.array([], dtype=np.int32),
            np.array([], dtype=np.int32),
            np.array([], dtype=np.int32),
            width=DAVIS346_WIDTH,
            height=DAVIS346_HEIGHT,
            potential=200.0,
        )
        assert xs.size == 0 and ys.size == 0 and zs.size == 0 and ps.size == 0

    def test_single_event_flips_y_to_cartesian(self) -> None:
        width, height = 346, 260
        xs, ys, zs, ps = events_to_xyzp(
            np.array([10], dtype=np.int32),
            np.array([0], dtype=np.int32),  # top row in sensor space
            np.array([POLARITY_ON], dtype=np.int32),
            width=width,
            height=height,
            potential=200.0,
        )
        assert xs.tolist() == [10]
        assert ys.tolist() == [height - 1]  # top row maps to top of cartesian voxels
        assert zs.tolist() == [POLARITY_ON]
        assert ps.tolist() == [200.0]

    def test_polarity_is_binarized(self) -> None:
        xs, ys, zs, ps = events_to_xyzp(
            np.array([1, 2], dtype=np.int32),
            np.array([1, 2], dtype=np.int32),
            np.array([5, 0], dtype=np.int32),  # any nonzero collapses to ON
            width=346,
            height=260,
            potential=200.0,
        )
        assert set(zs.tolist()) <= {POLARITY_OFF, POLARITY_ON}
        assert zs.tolist() == [POLARITY_ON, POLARITY_OFF]

    def test_out_of_bounds_events_are_dropped(self) -> None:
        xs, ys, zs, ps = events_to_xyzp(
            np.array([-1, 5, 400], dtype=np.int32),
            np.array([5, -1, 5], dtype=np.int32),
            np.array([1, 1, 1], dtype=np.int32),
            width=346,
            height=260,
            potential=200.0,
        )
        assert xs.size == 0

    def test_duplicate_voxel_in_one_burst_collapses_to_single_spike(self) -> None:
        # Same (x, y, polarity) voxel hit twice in one burst must not double-spike.
        xs, ys, zs, ps = events_to_xyzp(
            np.array([10, 10], dtype=np.int32),
            np.array([20, 20], dtype=np.int32),
            np.array([POLARITY_ON, POLARITY_ON], dtype=np.int32),
            width=346,
            height=260,
            potential=200.0,
        )
        assert xs.size == 1
        assert zs.tolist() == [POLARITY_ON]

    def test_same_pixel_different_polarity_yields_two_distinct_voxels(self) -> None:
        # Polarity is part of the voxel address (z axis), so OFF and ON at the
        # same pixel in one burst are two separate voxels, not a collision.
        xs, ys, zs, ps = events_to_xyzp(
            np.array([10, 10], dtype=np.int32),
            np.array([20, 20], dtype=np.int32),
            np.array([POLARITY_OFF, POLARITY_ON], dtype=np.int32),
            width=346,
            height=260,
            potential=200.0,
        )
        assert xs.size == 2
        assert set(zs.tolist()) == {POLARITY_OFF, POLARITY_ON}

    def test_mismatched_column_lengths_raise(self) -> None:
        with pytest.raises(ValueError):
            events_to_xyzp(
                np.array([1, 2], dtype=np.int32),
                np.array([1], dtype=np.int32),
                np.array([1, 1], dtype=np.int32),
                width=346,
                height=260,
                potential=200.0,
            )

    def test_non_positive_dimensions_raise(self) -> None:
        with pytest.raises(ValueError):
            events_to_xyzp(
                np.array([1], dtype=np.int32),
                np.array([1], dtype=np.int32),
                np.array([1], dtype=np.int32),
                width=0,
                height=260,
                potential=200.0,
            )

    def test_non_positive_potential_raises(self) -> None:
        with pytest.raises(ValueError):
            events_to_xyzp(
                np.array([1], dtype=np.int32),
                np.array([1], dtype=np.int32),
                np.array([1], dtype=np.int32),
                width=346,
                height=260,
                potential=0.0,
            )


class TestPolarityEventsToColumns:
    def _make_events(
        self, rows: list[tuple[int, int, int, bool]], polarity_field: str = "on"
    ) -> np.ndarray:
        dtype = [("t", "<u8"), ("x", "<u2"), ("y", "<u2"), (polarity_field, "?")]
        return np.array(rows, dtype=dtype)

    def test_extracts_columns_from_shared_dtype(self) -> None:
        events = self._make_events([(100, 5, 6, True), (200, 7, 8, False)])
        xs, ys, polarities, timestamps = polarity_events_to_columns(events)
        assert xs.tolist() == [5, 7]
        assert ys.tolist() == [6, 8]
        assert polarities.tolist() == [POLARITY_ON, POLARITY_OFF]
        assert timestamps.tolist() == [100, 200]

    def test_extracts_columns_when_polarity_field_is_named_p(self) -> None:
        # Some `aedat` releases expose the boolean polarity field as "p"
        # instead of "on"; both mean True/1 = ON.
        events = self._make_events([(100, 5, 6, True), (200, 7, 8, False)], polarity_field="p")
        xs, ys, polarities, timestamps = polarity_events_to_columns(events)
        assert xs.tolist() == [5, 7]
        assert ys.tolist() == [6, 8]
        assert polarities.tolist() == [POLARITY_ON, POLARITY_OFF]
        assert timestamps.tolist() == [100, 200]

    def test_missing_fields_raise_type_error(self) -> None:
        bad = np.zeros(1, dtype=[("t", "<u8"), ("x", "<u2")])
        with pytest.raises(TypeError):
            polarity_events_to_columns(bad)

    def test_missing_polarity_field_raises_type_error(self) -> None:
        bad = np.zeros(1, dtype=[("t", "<u8"), ("x", "<u2"), ("y", "<u2")])
        with pytest.raises(TypeError):
            polarity_events_to_columns(bad)

    def test_unstructured_array_raises_type_error(self) -> None:
        with pytest.raises(TypeError):
            polarity_events_to_columns(np.array([1, 2, 3]))


class TestPlaybackDelay:
    def test_zero_delay_when_playback_is_behind_schedule(self) -> None:
        delay = playback_delay_s(
            event_ts_us=2_000_000, origin_ts_us=0, origin_wall_s=0.0, now_wall_s=5.0
        )
        assert delay == 0.0

    def test_positive_delay_when_playback_is_ahead_of_schedule(self) -> None:
        delay = playback_delay_s(
            event_ts_us=2_000_000, origin_ts_us=0, origin_wall_s=0.0, now_wall_s=0.5
        )
        assert delay == pytest.approx(1.5)

    def test_event_before_origin_raises(self) -> None:
        with pytest.raises(ValueError):
            playback_delay_s(
                event_ts_us=0, origin_ts_us=100, origin_wall_s=0.0, now_wall_s=0.0
            )
