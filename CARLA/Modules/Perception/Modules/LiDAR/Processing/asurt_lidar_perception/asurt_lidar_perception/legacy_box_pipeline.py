"""CPU port of the original ASURT LiDAR box processing path.

See oldstack/lidarpipeline/{Filter,EuclideanClustering,ConeClassifier}.py.
The legacy LUT corridor is a separate stateful algorithm and is not ported.
Pandas and Open3D operations are expressed in NumPy/SciPy without importing
the old stack or requiring unavailable runtime dependencies.
"""

from dataclasses import dataclass
from time import perf_counter

import numpy as np

from .fast_pipeline import _components
from .legacy_cone_classifier import ConeClassifier


@dataclass(frozen=True)
class LegacyBoxParams:
    # Original settings from Perception_Lidar/config/params.yaml.
    x_min: float = 0.0
    x_max: float = 8.0
    y_min: float = -4.0
    y_max: float = 8.0
    z_min: float = -2.0
    z_max: float = 2.0
    car_x_min: float = -1.0
    car_x_max: float = 1.0
    car_y_min: float = -0.5
    car_y_max: float = 0.5
    ground_level: float = 0.2
    point_num: int = 10
    distance_threshold: float = 0.06
    num_iterations: int = 200
    horizontal_plane_gradient: float = 0.8
    voxel_size: float = 0.03
    cluster_distance_threshold: float = 0.2
    cluster_min_size: int = 2
    cluster_max_size: int = 100
    max_input_points: int = 85000

    def __post_init__(self):
        if not (self.x_min < self.x_max and self.y_min < self.y_max and
                self.z_min < self.z_max):
            raise ValueError("Invalid legacy box bounds")
        if not (self.car_x_min < self.car_x_max and
                self.car_y_min < self.car_y_max):
            raise ValueError("Invalid car exclusion bounds")
        if not (0 < self.distance_threshold <= 0.5 and
                3 <= self.num_iterations <= 5000 and 0 < self.voxel_size <= 0.2):
            raise ValueError("Invalid ground/voxel settings")
        if not (0 < self.cluster_distance_threshold <= 1 and
                1 <= self.cluster_min_size <= self.cluster_max_size):
            raise ValueError("Invalid clustering settings")
        if self.max_input_points < 1000:
            raise ValueError("Invalid maximum input size")


def _remove_ground_ransac(points, params):
    """Delete the RANSAC ground inliers when the plane is horizontal.

    Filter.removeGround segments only candidate points below ground_level
    and removes those original indices if abs(normal.z) exceeds the threshold.
    The highest-consensus three-point RANSAC fit is refined by SVD.
    """
    candidate_idx = np.flatnonzero(points[:, 2] < params.ground_level)
    if len(candidate_idx) <= params.point_num:
        return points
    candidates = np.asarray(points[candidate_idx], dtype=np.float64)
    rng = np.random.default_rng(42)
    best = np.empty(0, dtype=np.int32)
    best_normal = None
    n = len(candidates)
    for trial in range(params.num_iterations):
        sample = candidates[rng.choice(n, 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = np.linalg.norm(normal)
        if length <= 1e-9:
            continue
        normal /= length
        offset = -float(normal @ sample[0])
        inliers = np.flatnonzero(
            np.abs(candidates @ normal + offset) <= params.distance_threshold
        )
        if len(inliers) > len(best):
            best = inliers
            best_normal = normal
        # When 97% of candidates fit one plane, ten independent hypotheses
        # already give very high confidence; avoid 190 redundant traversals.
        if trial >= 9 and len(best) >= int(n * 0.97):
            break
    if best_normal is None or len(best) < 3:
        return points
    inlier_points = candidates[best]
    center = inlier_points.mean(axis=0)
    _, _, axes = np.linalg.svd(inlier_points - center, full_matrices=False)
    normal = axes[-1]
    if not np.isfinite(normal).all():
        normal = best_normal
        center = inlier_points[0]
    if abs(normal[2]) <= params.horizontal_plane_gradient:
        return points
    refined = (
        np.abs((candidates - center) @ normal) <= params.distance_threshold
    )
    mask = np.ones(len(points), dtype=bool)
    mask[candidate_idx[refined]] = False
    return points[mask]


def _voxel_centroid(points, voxel_size):
    """Open3D voxel_down_sample equivalent: mean returns in each voxel."""
    if not len(points):
        return points
    origin = np.min(points, axis=0) - 0.5 * voxel_size
    bins = np.floor((points - origin) / voxel_size).astype(np.int32)
    _, inverse = np.unique(bins, axis=0, return_inverse=True)
    total = np.zeros((int(inverse.max()) + 1, 3), dtype=np.float64)
    np.add.at(total, inverse, points)
    sizes = np.bincount(inverse)
    return np.asarray(total / sizes[:, None], dtype=np.float32)


class LegacyBoxPipeline:
    def __init__(self, params=None):
        self.params = params or LegacyBoxParams()
        # Exact historical ConeClassifier.py, vendored in this package.
        self.legacy = ConeClassifier(
            radius=0.1, height=0.3, minPoints=1,
            l2LossTh=0.1, linLossPerc=0.2,
        )

    def process(self, cloud):
        """Return filtered cloud, cone centers, and timing in sensor frame."""
        t0 = perf_counter()
        p = self.params
        if cloud.ndim != 2 or cloud.shape[1] < 3:
            raise ValueError("LiDAR cloud must be N x 3/4")
        total = len(cloud)
        if total > p.max_input_points:
            cloud = cloud[::(total + p.max_input_points - 1) // p.max_input_points]
        cloud = np.asarray(cloud[:, :3], dtype=np.float32)
        bounds = (
            np.isfinite(cloud).all(axis=1) &
            (cloud[:, 0] >= p.x_min) & (cloud[:, 0] <= p.x_max) &
            (cloud[:, 1] >= p.y_min) & (cloud[:, 1] <= p.y_max) &
            (cloud[:, 2] >= p.z_min) & (cloud[:, 2] <= p.z_max)
        )
        roi = cloud[bounds]
        # Filter.removeCar uses inclusive xy exclusion at any z.
        car = (
            (roi[:, 0] >= p.car_x_min) & (roi[:, 0] <= p.car_x_max) &
            (roi[:, 1] >= p.car_y_min) & (roi[:, 1] <= p.car_y_max)
        )
        without_car = roi[~car]
        without_ground = _remove_ground_ransac(without_car, p)
        points = _voxel_centroid(without_ground, p.voxel_size)
        groups = _components(points, p.cluster_distance_threshold)
        cones = []
        for group in groups:
            if not p.cluster_min_size <= len(group) <= p.cluster_max_size:
                continue
            cluster = np.asarray(points[group], dtype=np.float64)
            predicted, _ = self.legacy.isCone(cluster)
            if predicted[0]:
                lower = np.min(cluster, axis=0)
                upper = np.max(cluster, axis=0)
                # Old node used AABB midpoint for x/y of marker center.
                center = np.array(
                    [(lower[0] + upper[0]) * 0.5,
                     (lower[1] + upper[1]) * 0.5, lower[2]],
                    dtype=np.float32,
                )
                cones.append((center, 0.75))
        return points, cones, {
            "input": total,
            "after_roi": len(roi),
            "after_car": len(without_car),
            "after_ground": len(without_ground),
            "filtered": len(points),
            "clusters": len(groups),
            "cones": len(cones),
            "legacy_mode": "box",
            "legacy_fit": True,
            "ms": (perf_counter() - t0) * 1000,
        }
