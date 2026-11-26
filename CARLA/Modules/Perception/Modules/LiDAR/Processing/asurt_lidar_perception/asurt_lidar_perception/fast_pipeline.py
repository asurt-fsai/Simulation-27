"""Efficient, ROS-independent adaptation of the old ASURT LiDAR cone pipeline.

Source algorithm: fs-system-26-oldstack/Perception_Lidar/src/lidarpipeline.
The original cone conic-fit routine is reused when present; expensive
Python KdTree, pandas DataFrame and Open3D RANSAC per packet are replaced
with NumPy, SciPy cKDTree and a bounded robust local-ground estimate.

Output is strictly geometry inferred from LiDAR returns, never CARLA actor
positions, cone labels or a ground-truth race-course map. Blue/yellow colour
must come from camera perception in a later fusion stage.
"""
from dataclasses import dataclass
from time import perf_counter

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from .legacy_cone_classifier import ConeClassifier


@dataclass(frozen=True)
class Params:
    max_forward_m: float = 24.0
    max_lateral_m: float = 10.0
    min_z_m: float = -2.0
    max_z_m: float = 1.8
    voxel_size_m: float = 0.055
    cluster_radius_m: float = 0.25
    min_cluster_points: int = 4
    max_cluster_points: int = 220
    min_cone_height_m: float = 0.065
    max_cone_height_m: float = 0.46
    max_cone_width_m: float = 0.44
    ground_margin_m: float = 0.055
    use_legacy_cone_fit: bool = True
    max_input_points: int = 85000

    def __post_init__(self):
        if not (2 <= self.max_forward_m <= 150 and 1 <= self.max_lateral_m <= 40):
            raise ValueError("Invalid LiDAR ROI")
        if not (0.02 <= self.voxel_size_m <= 0.20 and
                0.08 <= self.cluster_radius_m <= 1.0):
            raise ValueError("Invalid voxel or clustering radius")
        if not (3 <= self.min_cluster_points < self.max_cluster_points <= 2000):
            raise ValueError("Invalid cluster size")
        if not (self.min_cone_height_m < self.max_cone_height_m <= 1.5):
            raise ValueError("Invalid cone height bounds")


def _load_legacy_cone_fit():
    """Use the vendored legacy classifier with its original node settings."""
    return ConeClassifier(
        radius=0.1, height=0.3, minPoints=1,
        l2LossTh=0.1, linLossPerc=0.2
    )


def _voxel_first(points, size):
    if not len(points):
        return points
    cells = np.floor(points / size).astype(np.int32)
    _, original = np.unique(cells, axis=0, return_index=True)
    return np.ascontiguousarray(points[np.sort(original)], dtype=np.float32)


def _ground_plane(points):
    """Fit a slope-limited ground z=ax+by+c to low per-cell LiDAR returns."""
    if len(points) < 25:
        return np.array([0.0, 0.0, -0.7], dtype=np.float64)
    # Vectorized 1-D cell keys avoid the ~40-60 ms structured-array unique()
    # overhead on a 10k-point cloud. The bounded ROI guarantees no key alias.
    gx = np.floor(points[:, 0] / 0.9).astype(np.int32)
    gy = np.floor(points[:, 1] / 0.9).astype(np.int32)
    keys = gx.astype(np.int64) * 65536 + (gy.astype(np.int64) + 32768)
    # lexsort sorts first by key, then by elevation. The first point of each
    # cell therefore supplies its minimum Z without a slow np.minimum.at.
    ordered = np.lexsort((points[:, 2], keys))
    unique, first = np.unique(keys[ordered], return_index=True)
    minima = points[ordered[first], 2].astype(np.float64)
    counts = np.diff(np.r_[first, len(points)])
    valid = (counts >= 3) & (minima < 0.25) & (minima > -2.0)
    if np.count_nonzero(valid) < 8:
        z = np.median(points[:, 2])
        return np.array([0.0, 0.0, float(z)], dtype=np.float64)
    selected = unique[valid]
    xyz = np.column_stack((
        (selected // 65536).astype(np.float64) * 0.9 + 0.45,
        (selected % 65536 - 32768).astype(np.float64) * 0.9 + 0.45,
        np.ones(np.count_nonzero(valid)),
    ))
    heights = minima[valid]
    med = np.median(heights)
    inliers = np.abs(heights - med) < 0.30
    if np.count_nonzero(inliers) < 8:
        return np.array([0.0, 0.0, float(med)], dtype=np.float64)
    for _ in range(3):
        coefficients = np.linalg.lstsq(xyz[inliers], heights[inliers], rcond=None)[0]
        residuals = np.abs(heights - xyz @ coefficients)
        fitted = residuals < 0.12
        if np.count_nonzero(fitted) < 8:
            break
        inliers = fitted
    slope = np.hypot(coefficients[0], coefficients[1])
    if slope > 0.35:
        return np.array([0.0, 0.0, float(med)], dtype=np.float64)
    return coefficients


def _components(points, radius):
    if not len(points):
        return []
    tree = cKDTree(points)
    pairs = tree.query_pairs(radius, output_type="ndarray")
    n = len(points)
    if not len(pairs):
        # Every component would be a singleton. The minimum accepted cluster
        # size is three, so avoid allocating thousands of unusable arrays.
        return []
    edges = coo_matrix((np.ones(len(pairs), dtype=np.int8),
                        (pairs[:, 0], pairs[:, 1])), shape=(n, n)).tocsr()
    _, labels = connected_components(edges, directed=False)
    order = np.argsort(labels, kind="stable")
    cuts = np.flatnonzero(np.diff(labels[order])) + 1
    return np.split(order, cuts)


class FastConePipeline:
    def __init__(self, params=None):
        self.params = params or Params()
        self.legacy = _load_legacy_cone_fit() if self.params.use_legacy_cone_fit else None

    def process(self, cloud):
        """Return (filtered Nx3, cones, metrics), same sensor-local frame.

        Each cone is (xyz at cone base, *uncalibrated* heuristic confidence).
        This algorithm is bounded but not a validated autonomous safety gate.
        """
        start = perf_counter()
        p = self.params
        if cloud.ndim != 2 or cloud.shape[1] < 3:
            raise ValueError("LiDAR cloud must be N x 3/4")
        if len(cloud) > p.max_input_points:
            # Bound CPU by deterministic decimation rather than silently
            # retaining unbounded ROS queues.
            stride = (len(cloud) + p.max_input_points - 1) // p.max_input_points
            cloud = cloud[::stride]
        xyz = np.asarray(cloud[:, :3], dtype=np.float32)
        keep = (np.isfinite(xyz).all(axis=1) &
                (xyz[:, 0] >= 0.6) & (xyz[:, 0] <= p.max_forward_m) &
                (np.abs(xyz[:, 1]) <= p.max_lateral_m) &
                (xyz[:, 2] >= p.min_z_m) & (xyz[:, 2] <= p.max_z_m))
        roi = xyz[keep]
        if len(roi) < p.min_cluster_points:
            return np.empty((0, 3), dtype=np.float32), [], {
                "input": len(cloud), "after_roi": len(roi),
                "filtered": 0, "clusters": 0, "ms": (perf_counter()-start)*1000}
        plane = _ground_plane(roi)
        above_ground = roi[:, 2] - (roi[:, :2] @ plane[:2] + plane[2])
        foreground = roi[(above_ground > p.ground_margin_m) &
                         (above_ground < 0.85)]
        points = _voxel_first(foreground, p.voxel_size_m)
        # Overload control is applied before constructing the neighbor graph.
        if len(points) > 11000:
            points = _voxel_first(points, max(p.voxel_size_m, .10))
        comps = _components(points, p.cluster_radius_m)
        cones = []
        for idx in comps:
            if not p.min_cluster_points <= len(idx) <= p.max_cluster_points:
                continue
            shape = points[idx]
            low = np.min(shape, axis=0)
            high = np.max(shape, axis=0)
            ext = high - low
            if not (p.min_cone_height_m <= ext[2] <= p.max_cone_height_m):
                continue
            if max(ext[0], ext[1]) > p.max_cone_width_m:
                continue
            # Cones narrow toward the top. Width-versus-height gating
            # rejects nearby pieces of track furniture before legacy fitting.
            half = np.median(shape[:, 2])
            lower = shape[shape[:, 2] <= half, :2]
            upper = shape[shape[:, 2] > half, :2]
            if len(lower) < 2 or len(upper) < 2:
                continue
            center_xy = np.median(shape[:, :2], axis=0)
            near_radius = np.median(np.linalg.norm(lower-center_xy, axis=1))
            far_radius = np.median(np.linalg.norm(upper-center_xy, axis=1))
            if near_radius < far_radius * 0.60:
                continue
            if self.legacy is not None:
                original_pred, _ = self.legacy.isCone(shape.astype(np.float64))
                # The original node only emitted a marker when isCone() passed.
                # Do not turn a rejected cluster into a weaker positive.
                if not original_pred[0]:
                    continue
            base = np.array([center_xy[0],center_xy[1],low[2]],dtype=np.float32)
            score = 0.75 if self.legacy is not None else 0.50
            cones.append((base, score))
        elapsed = (perf_counter() - start) * 1000
        return points, cones, {
            "input": len(cloud), "after_roi": len(roi),
            "filtered": len(points), "clusters": len(comps),
            "cones": len(cones), "ground_z": float(plane[2]),
            "legacy_fit": self.legacy is not None, "ms": elapsed
        }
