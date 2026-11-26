"""Semantic checks for the independent LiDAR pipeline and its original cone fit."""

import numpy as np

from asurt_lidar_perception.fast_pipeline import (
    FastConePipeline, Params, _components,
)
from asurt_lidar_perception.legacy_cone_classifier import ConeClassifier
from asurt_lidar_perception.legacy_box_pipeline import (
    LegacyBoxPipeline, LegacyBoxParams,
)


def _single_cone_scan():
    """A small isolated 30 cm cone over a densely sampled horizontal road."""
    ground_x, ground_y = np.meshgrid(
        np.linspace(0.8, 12.0, 45), np.linspace(-5.0, 5.0, 27)
    )
    road = np.column_stack((
        ground_x.ravel(), ground_y.ravel(),
        np.full(ground_x.size, -1.1),
    ))
    returns = []
    for dz in np.linspace(0.025, 0.285, 9):
        radius = 0.11 * (1.0 - dz / 0.325)
        for angle in np.linspace(0.0, 2 * np.pi, 14, endpoint=False):
            returns.append((
                5.0 + radius * np.cos(angle),
                1.0 + radius * np.sin(angle),
                -1.1 + dz,
            ))
    return np.vstack((road, np.asarray(returns))).astype(np.float32)


def test_cone_is_detected_from_returns_and_ground_is_removed():
    processor = FastConePipeline()
    assert isinstance(processor.legacy, ConeClassifier)
    retained, cones, metrics = processor.process(_single_cone_scan())
    assert metrics["legacy_fit"] is True
    assert 15 <= len(retained) < 80  # Road is removed; cone survives.
    assert len(cones) == 1
    center, confidence = cones[0]
    assert np.linalg.norm(center[:2] - [5.0, 1.0]) < 0.15
    assert -1.1 < center[2] < -0.75
    assert confidence == 0.75


def test_original_fit_rejection_cannot_become_a_cone_detection():
    class RejectingFit:
        def isCone(self, points):
            return [False], None

    processor = FastConePipeline()
    processor.legacy = RejectingFit()
    retained, cones, metrics = processor.process(_single_cone_scan())
    assert retained.shape[0] > 0
    assert metrics["clusters"] == 1
    assert cones == []


def test_euclidean_clustering_connects_neighbor_chains():
    # Endpoints are outside the radius, but the middle point joins them.
    points = np.array([
        [0.0, 0.0, 0.0], [0.15, 0.0, 0.0], [0.3, 0.0, 0.0],
        [2.0, 0.0, 0.0], [2.15, 0.0, 0.0],
    ])
    groups = _components(points, radius=0.20)
    assert sorted(sorted(int(i) for i in group) for group in groups) == [
        [0, 1, 2], [3, 4],
    ]
    assert _components(points, radius=0.05) == []


def test_invalid_returns_and_empty_roi_never_create_a_cone():
    processor = FastConePipeline(Params(max_forward_m=10.0))
    cloud = np.array([
        [np.nan, 0, 0], [np.inf, 0, 0], [-5, 0, 0],
        [20, 0, 0], [0.3, 0, 0],
    ], dtype=np.float32)
    retained, cones, metrics = processor.process(cloud)
    assert retained.shape == (0, 3)
    assert cones == []
    assert metrics["after_roi"] == 0


def test_legacy_box_keeps_original_roi_car_filter_and_cone_decision():
    processor = LegacyBoxPipeline()
    assert processor.params == LegacyBoxParams()
    retained, cones, metrics = processor.process(_single_cone_scan())
    assert metrics["legacy_mode"] == "box"
    assert metrics["after_roi"] < metrics["input"]
    assert metrics["after_car"] < metrics["after_roi"]
    assert metrics["after_ground"] < metrics["after_car"]
    assert 15 < len(retained) < 100
    assert len(cones) == 1
    assert np.linalg.norm(cones[0][0][:2] - [5.0, 1.0]) < 0.15


def test_legacy_box_preserves_original_classifier_rejection():
    class RejectingFit:
        def isCone(self, points):
            return [False], None

    processor = LegacyBoxPipeline()
    processor.legacy = RejectingFit()
    retained, cones, metrics = processor.process(_single_cone_scan())
    assert len(retained) > 0
    assert metrics["clusters"] > 0
    assert cones == []
