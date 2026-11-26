"""Stateful CPU port of oldstack's LUTBoundingBoxFilter and LiDAR callback.

Order is kept: LUT/bounds ROI, remove car, remove ground, voxel average,
Euclidean cluster, strict cone detection and LUT update, final cone classifier.
The historical LUT math is preserved; quantized grid keys, per-cell expiry,
and bounded size prevent cross-session drift and stale state accumulation.
"""

from dataclasses import dataclass
from time import perf_counter

import numpy as np

from .fast_pipeline import _components
from .legacy_box_pipeline import (
    LegacyBoxParams, _remove_ground_ransac, _voxel_centroid,
)
from .legacy_cone_classifier import ConeClassifier


@dataclass(frozen=True)
class LegacyLUTParams:
    # Historical LUTBoundingBoxFilter config/params.yaml values.
    lut_max_distance: float = 20.0
    lut_z_min: float = -2.0
    lut_z_max: float = 2.0
    lut_max_cone_lateral: float = 2.5
    lut_max_track_half_width: float = 2.0
    lut_cone_radius: float = 0.1
    lut_cone_height: float = 0.3
    lut_min_cone_points: int = 5
    lut_l2_loss_threshold: float = 0.05
    lut_lin_loss_percentage: float = 0.1
    lut_resolution: float = 0.1
    lut_ema_alpha: float = 0.2
    lut_max_width_change: float = 0.2
    lut_tolerance_multiplier: float = 5.0
    lut_filter_margin: float = 1.5
    lut_filter_x_margin_before: float = 1.0
    lut_filter_x_margin_after: float = 2.0
    # Additional safety bounds. TTL applies individually to each LUT cell.
    lut_max_entries: int = 512
    lut_ttl_frames: int = 80
    lut_ttl_seconds: float = 6.0

    def __post_init__(self):
        if not (1.0 <= self.lut_max_distance <= 150.0 and
                self.lut_z_min < self.lut_z_max and
                0 < self.lut_max_cone_lateral <= self.lut_max_distance and
                0 < self.lut_max_track_half_width <= self.lut_max_distance):
            raise ValueError("Invalid LUT geographic limits")
        if not (0 < self.lut_cone_radius < 1 and
                0 < self.lut_cone_height < 2 and
                1 <= self.lut_min_cone_points <= 500 and
                0 < self.lut_l2_loss_threshold <= 10 and
                0 < self.lut_lin_loss_percentage <= 10):
            raise ValueError("Invalid LUT classifier settings")
        if not (0.02 <= self.lut_resolution <= 1.0 and
                0 < self.lut_ema_alpha <= 1 and
                0 < self.lut_max_width_change <= 5 and
                0 < self.lut_tolerance_multiplier <= 20):
            raise ValueError("Invalid LUT smoothing and resolution")
        if not (0 <= self.lut_filter_margin <= 20 and
                0 <= self.lut_filter_x_margin_before <= 20 and
                0 <= self.lut_filter_x_margin_after <= 20):
            raise ValueError("Invalid LUT filter margin")
        if not (8 <= self.lut_max_entries <= 4096 and
                1 <= self.lut_ttl_frames <= 100000 and
                0.01 <= self.lut_ttl_seconds <= 3600):
            raise ValueError("Invalid LUT memory and TTL limits")


class LegacyLUTPipeline:
    def __init__(self, params=None, filter_params=None):
        self.params = params or LegacyLUTParams()
        self.filter_params = filter_params or LegacyBoxParams()
        p = self.params
        self.lut_classifier = ConeClassifier(
            radius=p.lut_cone_radius, height=p.lut_cone_height,
            minPoints=p.lut_min_cone_points,
            l2LossTh=p.lut_l2_loss_threshold,
            linLossPerc=p.lut_lin_loss_percentage,
        )
        # The original node intentionally has two independent classifiers.
        self.legacy = ConeClassifier(
            radius=0.1, height=0.3, minPoints=1,
            l2LossTh=0.1, linLossPerc=0.2,
        )
        # Integer bins keep the historical 0.1m grid stable between frames.
        self.lut_centerline = {}
        self.lut_width = {}
        self._seen_frame = {}
        self._seen_stamp = {}
        self.detected_cones = []
        self.frame_number = 0
        self.last_stamp = None
        self.last_frame_id = None
        self.resets = 0

    def clear_lut(self):
        self.lut_centerline.clear()
        self.lut_width.clear()
        self._seen_frame.clear()
        self._seen_stamp.clear()
        self.detected_cones.clear()

    def _forget(self, key):
        self.lut_centerline.pop(key, None)
        self.lut_width.pop(key, None)
        self._seen_frame.pop(key, None)
        self._seen_stamp.pop(key, None)

    def _expire(self):
        p = self.params
        for key in tuple(self.lut_centerline):
            frames_old = self.frame_number - self._seen_frame[key]
            timestamp = self._seen_stamp[key]
            seconds_old = (self.last_stamp - timestamp
                           if self.last_stamp is not None and timestamp is not None
                           else 0.0)
            if frames_old > p.lut_ttl_frames or seconds_old > p.lut_ttl_seconds:
                self._forget(key)
        self._enforce_limit()

    def _enforce_limit(self):
        while len(self.lut_centerline) > self.params.lut_max_entries:
            # Deterministic eviction of the least recently confirmed bin.
            oldest = min(
                self.lut_centerline,
                key=lambda k: (self._seen_frame[k], k),
            )
            self._forget(oldest)

    def _begin_frame(self, stamp_s=None, frame_id=None):
        """Reset on time rewind or coordinate-frame switch, then expire state."""
        valid_stamp = (
            float(stamp_s) if stamp_s is not None and np.isfinite(stamp_s)
            else None
        )
        valid_frame = str(frame_id) if frame_id else None
        rewind = (
            valid_stamp is not None and self.last_stamp is not None and
            valid_stamp < self.last_stamp - 1e-3
        )
        changed_frame = (
            valid_frame is not None and self.last_frame_id is not None and
            valid_frame != self.last_frame_id
        )
        if rewind or changed_frame:
            self.clear_lut()
            self.resets += 1
            self.frame_number = 0
        self.frame_number += 1
        # Empty/zero stamp means no trustworthy sim-time timeline.
        self.last_stamp = valid_stamp if valid_stamp is not None and valid_stamp > 0 else None
        if valid_frame is not None:
            self.last_frame_id = valid_frame
        self._expire()

    def filter_with_lut(self, points):
        """Exact legacy fallback and nearest-key corridor selection."""
        p = self.params
        pts = np.asarray(points, dtype=np.float32)
        if not len(pts):
            return pts
        finite = np.isfinite(pts[:, :3]).all(axis=1)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        dist = np.hypot(x, y)
        basic = (
            finite & (dist <= p.lut_max_distance) &
            (z >= p.lut_z_min) & (z <= p.lut_z_max)
        )
        if not self.lut_centerline:
            return pts[basic]

        keys = np.array(sorted(self.lut_centerline), dtype=np.int32)
        xs = keys.astype(np.float64) * p.lut_resolution
        ys = np.array([self.lut_centerline[k] for k in keys])
        widths = np.array([self.lut_width[k] for k in keys])
        idx = np.clip(np.searchsorted(xs, x, side="left"), 0, len(xs)-1)
        prev = np.maximum(idx - 1, 0)
        # Same nearest-left tie behavior as original LUTBoundingBoxFilter.
        use_prev = (idx > 0) & (np.abs(xs[prev] - x) < np.abs(xs[idx] - x))
        idx = np.where(use_prev, prev, idx)
        x_range = (
            (x >= xs[0] - p.lut_filter_x_margin_before) &
            (x <= xs[-1] + p.lut_filter_x_margin_after)
        )
        corridor = np.abs(y - ys[idx]) <= widths[idx] + p.lut_filter_margin
        return pts[basic & x_range & corridor]

    def detect_lut_cones(self, points, clusters):
        """Run the original strict LUT classifier on post-voxel clusters."""
        cones = []
        for indices in clusters:
            if not len(indices):
                continue
            prediction, top = self.lut_classifier.isCone(
                np.asarray(points[indices], dtype=np.float64)
            )
            if prediction[0] and top is not None and np.isfinite(top).all():
                cones.append(np.asarray(top[0], dtype=np.float64))
        self.detected_cones = cones
        return cones

    def build_lut(self, cones):
        """Pair cones across y=0 and update center/half-width by bounded EMA."""
        p = self.params
        if len(cones) < 2:
            return 0
        cone_array = np.asarray(cones, dtype=np.float64)
        if cone_array.ndim != 2 or cone_array.shape[1] != 3:
            raise ValueError("Cone centers must be N x 3")
        mask = (
            np.isfinite(cone_array).all(axis=1) &
            (np.abs(cone_array[:, 1]) <= p.lut_max_cone_lateral) &
            (np.hypot(cone_array[:, 0], cone_array[:, 1]) <= p.lut_max_distance)
        )
        gated = cone_array[mask]
        if len(gated) < 2:
            return 0
        # Original uses np.arange(min_x,max_x+resolution,resolution) with
        # floating dictionary keys; integer keys allow EMA over repeat frames.
        # Only sample within the gated span, never beyond max_distance ROI.
        start = int(np.floor(np.min(gated[:, 0]) / p.lut_resolution + 0.5))
        end = int(np.floor(np.max(gated[:, 0]) / p.lut_resolution + 0.5))
        if end < start:
            return 0
        steps = np.arange(start, end + 1, dtype=np.int32)
        max_per_frame = p.lut_max_entries * 2
        if len(steps) > max_per_frame:
            # Hard bound on per-frame LUT construction on malformed inputs.
            steps = steps[
                np.linspace(0, len(steps) - 1, max_per_frame, dtype=np.int32)
            ]
        tolerance = p.lut_resolution * p.lut_tolerance_multiplier
        updated = 0
        for key in steps:
            x = float(key) * p.lut_resolution
            nearby = gated[np.abs(gated[:, 0] - x) <= tolerance]
            if len(nearby) < 2:
                continue
            left = nearby[nearby[:, 1] > 0.0]
            right = nearby[nearby[:, 1] < 0.0]
            if not len(left) or not len(right):
                continue
            y_left, y_right = left[:, 1].mean(), right[:, 1].mean()
            half_width = float(abs(y_left - y_right) * 0.5)
            if half_width > p.lut_max_track_half_width:
                continue
            center = float((y_left + y_right) * 0.5)
            k = int(key)
            if k in self.lut_width:
                if abs(half_width - self.lut_width[k]) > p.lut_max_width_change:
                    continue
                center = (
                    p.lut_ema_alpha * center +
                    (1.0 - p.lut_ema_alpha) * self.lut_centerline[k]
                )
                half_width = (
                    p.lut_ema_alpha * half_width +
                    (1.0 - p.lut_ema_alpha) * self.lut_width[k]
                )
            self.lut_centerline[k] = center
            self.lut_width[k] = half_width
            self._seen_frame[k] = self.frame_number
            self._seen_stamp[k] = self.last_stamp
            updated += 1
        # Original removes keys outside median(cone_x) +/- max_distance.
        midpoint = float(np.median(gated[:, 0]))
        for key in tuple(self.lut_centerline):
            if abs(key * p.lut_resolution - midpoint) > p.lut_max_distance:
                self._forget(key)
        self._enforce_limit()
        return updated

    def process(self, cloud, stamp_s=None, frame_id=None):
        """Apply original callback stages, retaining LUT state between frames."""
        started = perf_counter()
        p = self.params
        f = self.filter_params
        if cloud.ndim != 2 or cloud.shape[1] < 3:
            raise ValueError("LiDAR cloud must be N x 3/4")
        self._begin_frame(stamp_s=stamp_s, frame_id=frame_id)
        total = len(cloud)
        if total > f.max_input_points:
            stride = (total + f.max_input_points - 1) // f.max_input_points
            cloud = cloud[::stride]
        xyz = np.asarray(cloud[:, :3], dtype=np.float32)
        roi = self.filter_with_lut(xyz)
        car = (
            (roi[:, 0] >= f.car_x_min) & (roi[:, 0] <= f.car_x_max) &
            (roi[:, 1] >= f.car_y_min) & (roi[:, 1] <= f.car_y_max)
        )
        without_car = roi[~car]
        without_ground = _remove_ground_ransac(without_car, f)
        points = _voxel_centroid(without_ground, f.voxel_size)
        # Old stack stops when no cloud remains. We still return an empty
        # output so ROS can clear its previous cone markers.
        clusters = _components(points, f.cluster_distance_threshold)
        clusters = [
            group for group in clusters
            if f.cluster_min_size <= len(group) <= f.cluster_max_size
        ]
        lut_cones = self.detect_lut_cones(points, clusters)
        updated = self.build_lut(lut_cones) if lut_cones else 0
        markers = []
        for group in clusters:
            shape = np.asarray(points[group], dtype=np.float64)
            prediction, _ = self.legacy.isCone(shape)
            if not prediction[0]:
                continue
            low = np.min(shape, axis=0)
            high = np.max(shape, axis=0)
            # The legacy marker uses AABB center; downstream landmarks in
            # this package continue to use a local cone-base position.
            center = np.array(
                [(low[0] + high[0]) * 0.5,
                 (low[1] + high[1]) * 0.5, low[2]],
                dtype=np.float32,
            )
            markers.append((center, 0.75))
        return points, markers, {
            "input": total,
            "after_roi": len(roi),
            "after_car": len(without_car),
            "after_ground": len(without_ground),
            "filtered": len(points),
            "clusters": len(clusters),
            "cones": len(markers),
            "lut_candidates": len(lut_cones),
            "lut_updated": updated,
            "lut_entries": len(self.lut_centerline),
            "lut_resets": self.resets,
            "lut_fallback": not bool(self.lut_centerline),
            "legacy_mode": "lut",
            "legacy_fit": True,
            "ms": (perf_counter() - started) * 1000,
        }
