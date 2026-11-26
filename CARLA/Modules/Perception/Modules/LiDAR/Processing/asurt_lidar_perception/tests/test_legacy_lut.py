"""Regression checks for the original stateful LUT callback and its safety bounds."""

import numpy as np
import pytest

from asurt_lidar_perception.legacy_lut_pipeline import (
    LegacyLUTParams, LegacyLUTPipeline,
)
from asurt_lidar_perception.legacy_cone_classifier import ConeClassifier


def _track_scan():
    """Flat road with two geometrically matching cones at x=5, y=+/-1."""
    xx, yy = np.meshgrid(np.linspace(0.8, 12.0, 45),
                         np.linspace(-5.0, 5.0, 27))
    ground = np.column_stack(
        (xx.ravel(), yy.ravel(), np.full(xx.size, -1.1))
    )
    cones = []
    for side in (-1.0, 1.0):
        for dz in np.linspace(0.025, 0.285, 9):
            radius = 0.11 * (1.0 - dz / 0.325)
            for angle in np.linspace(0, 2*np.pi, 14, endpoint=False):
                cones.append([
                    5.0 + radius*np.cos(angle),
                    side + radius*np.sin(angle),
                    -1.1 + dz,
                ])
    return np.vstack((ground, cones)).astype(np.float32)


def test_empty_lut_falls_back_to_original_distance_and_z_bounds():
    processor = LegacyLUTPipeline()
    assert processor.lut_centerline == {}
    assert isinstance(processor.lut_classifier, ConeClassifier)
    assert isinstance(processor.legacy, ConeClassifier)
    assert processor.lut_classifier is not processor.legacy
    points = np.array([
        [-3, 0, 0], [5, 1, 1], [21, 0, 0], [0, 0, 3],
        [float("nan"), 0, 0],
    ], dtype=np.float32)
    actual = processor.filter_with_lut(points)
    np.testing.assert_array_equal(actual, points[:2])


def test_build_paired_corridor_and_apply_nearest_cell_filter():
    processor = LegacyLUTPipeline(
        LegacyLUTParams(lut_filter_margin=0.1)
    )
    processor._begin_frame(11.0, "velodyne")
    pairs = [[4.03, 1.1, -0.4], [4.03, -0.9, -0.4]]
    assert processor.build_lut(pairs) == 1
    assert len(processor.lut_centerline) == 1
    key = next(iter(processor.lut_centerline))
    assert processor.lut_centerline[key] == pytest.approx(0.1)
    assert processor.lut_width[key] == pytest.approx(1.0)
    cloud = np.array([
        [4.04, 0.0, 0.0],     # in corridor
        [4.04, 1.8, 0.0],     # outside corridor
        [6.2, 0.0, 0.0],      # outside x margin
        [4.04, 0.0, 3.0],     # outside z bounds
    ])
    filtered = processor.filter_with_lut(cloud)
    np.testing.assert_allclose(filtered, cloud[:1], atol=1e-6)


def test_failed_candidates_cannot_seed_or_poison_lut():
    processor = LegacyLUTPipeline(
        LegacyLUTParams(lut_max_width_change=0.1)
    )
    processor._begin_frame(3.0, "velodyne")
    assert processor.build_lut([[5, 1, 0], [5.05, 1.2, 0]]) == 0
    assert processor.build_lut([[5, 3.1, 0], [5, -1, 0]]) == 0
    assert processor.build_lut([[5, 2.2, 0], [5, -2.2, 0]]) == 0
    assert not processor.lut_centerline
    assert processor.build_lut([[5.0, 1.0, 0], [5.0, -1.0, 0]]) == 1
    key = next(iter(processor.lut_centerline))
    first_seen = processor._seen_frame[key]
    processor._begin_frame(3.1, "velodyne")
    assert processor.build_lut([[5, 1.8, 0], [5, -1.8, 0]]) == 0
    assert processor.lut_width[key] == pytest.approx(1.0)
    assert processor._seen_frame[key] == first_seen


def test_lut_ttl_bound_and_clock_or_frame_reset():
    params = LegacyLUTParams(
        lut_max_entries=8, lut_ttl_frames=2, lut_ttl_seconds=0.5
    )
    processor = LegacyLUTPipeline(params)
    processor._begin_frame(10.0, "velodyne")
    assert processor.build_lut([[5, 1, 0], [5, -1, 0]]) == 1
    processor._begin_frame(10.1, "velodyne")
    assert processor.lut_centerline
    processor._begin_frame(10.6, "velodyne")
    assert processor.lut_centerline == {}
    processor.build_lut([[4.0, 1, 0], [4.0, -1, 0]])
    processor._begin_frame(10.7, "velodyne")
    processor._begin_frame(10.8, "velodyne")
    processor._begin_frame(10.9, "velodyne")
    assert not processor.lut_centerline  # frame TTL independently active
    processor.build_lut([[5, 1, 0], [5, -1, 0]])
    processor._begin_frame(1.0, "velodyne")
    assert not processor.lut_centerline
    assert processor.resets == 1
    processor.build_lut([[5, 1, 0], [5, -1, 0]])
    processor._begin_frame(1.1, "other_sensor")
    assert not processor.lut_centerline
    assert processor.resets == 2
    # A long strip of valid cone pairs cannot grow the grid without bound.
    processor._begin_frame(2.0, "other_sensor")
    for x in np.arange(1.0, 8.0, 0.3):
        processor.build_lut([[x, 1.0, 0], [x, -1.0, 0]])
        assert len(processor.lut_centerline) <= params.lut_max_entries


def test_full_order_initializes_then_uses_corridor_and_updates_after_clustering():
    processor = LegacyLUTPipeline()
    scan = _track_scan()
    first, cones, before = processor.process(
        scan, stamp_s=100.0, frame_id="velodyne"
    )
    assert before["after_roi"] == len(scan)  # initial radial/z fallback
    assert before["after_ground"] < before["after_car"]
    assert before["filtered"] == len(first)
    assert before["clusters"] == 2
    assert before["lut_candidates"] == 2
    assert before["lut_updated"] > 0
    assert before["lut_entries"] > 0
    assert len(cones) == 2
    assert sorted(round(float(c[0][1])) for c in cones) == [-1, 1]
    second, after_cones, after = processor.process(
        scan, stamp_s=100.1, frame_id="velodyne"
    )
    assert after["after_roi"] < before["after_roi"]
    assert len(second) > 0
    assert after["lut_updated"] > 0
    assert len(after_cones) == 2


def test_no_accepted_strict_cones_means_no_corridor_update():
    class RejectFit:
        def isCone(self, points):
            return [False], None

    processor = LegacyLUTPipeline()
    processor.lut_classifier = RejectFit()
    filtered, cones, metrics = processor.process(
        _track_scan(), stamp_s=11.0, frame_id="velodyne"
    )
    assert len(filtered) > 0
    assert len(cones) == 2  # independent final classifier still passes
    assert metrics["lut_candidates"] == 0
    assert metrics["lut_updated"] == 0
    assert metrics["lut_fallback"] is True
    assert processor.lut_centerline == {}


def test_malformed_lut_settings_are_rejected():
    for overrides in (
        {"lut_resolution": 0},
        {"lut_ema_alpha": -0.1},
        {"lut_max_entries": 0},
        {"lut_ttl_frames": 0},
        {"lut_ttl_seconds": 0},
    ):
        with pytest.raises(ValueError):
            LegacyLUTParams(**overrides)
