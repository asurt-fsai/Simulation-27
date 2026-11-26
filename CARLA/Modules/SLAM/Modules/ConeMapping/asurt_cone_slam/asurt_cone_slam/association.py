"""Port of the old cone_mapping Kalman/Mahalanobis/Hungarian approach.

Pure computations are isolated from ROS, CARLA and any camera SDK for
deterministic testing on either Silverstone or FlatAsphalt.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass
class Landmark:
    uid: int
    x: np.ndarray
    P: np.ndarray
    kind: int
    observations: int
    missed: int
    last_seen: float


class ConeMap:
    def __init__(self, gate=2.0, mahalanobis_gate=5.991, minimum_observations=3,
                 base_variance=.01, range_variance=.02, process_noise=.001,
                 maximum_landmarks=1200, max_tentative_missed=20,
                 max_detection_range=45.0, min_detection_range=.25):
        self.gate = float(gate)
        self.mahalanobis_gate = float(mahalanobis_gate)
        self.minimum_observations = int(minimum_observations)
        self.base_variance = float(base_variance)
        self.range_variance = float(range_variance)
        self.process_noise = float(process_noise)
        self.maximum_landmarks = int(maximum_landmarks)
        self.max_tentative_missed = int(max_tentative_missed)
        self.max_detection_range = float(max_detection_range)
        self.min_detection_range = float(min_detection_range)
        self.landmarks = []
        self.next_id = 0
        self.frames = 0

    def reset(self):
        self.landmarks.clear()
        self.next_id = 0
        self.frames = 0

    def confirmed(self):
        return [lm for lm in self.landmarks if lm.observations >= self.minimum_observations]

    def update(self, detections, stamp):
        """Consume [(map_x,map_y,kind,confidence,range_m), ...].

        Each incoming observation is associated at most once, and each mapped
        landmark can receive at most one observation per frame. World truth
        and cone actor coordinates are never used.
        """
        valid = []
        for values in detections:
            x, y, kind, confidence, distance = values
            if (kind not in (0, 1, 2, 3) or
                not all(math.isfinite(float(v)) for v in (x, y, confidence, distance)) or
                not .0 <= confidence <= 1.0 or
                not self.min_detection_range <= distance <= self.max_detection_range):
                continue
            valid.append((np.array([x, y], dtype=np.float64),
                          int(kind), float(confidence), float(distance)))
        valid = valid[:64]  # Cap work per frame to protect the simulator.
        self.frames += 1
        for lm in self.landmarks:
            lm.P = lm.P + np.eye(2) * self.process_noise
            lm.missed += 1
        n, m = len(valid), len(self.landmarks)
        used_dets = set()
        if n and m:
            cost = np.full((n, m), 1e9, dtype=np.float64)
            for i, (xy, kind, conf, distance) in enumerate(valid):
                variance = (self.base_variance + self.range_variance * distance**2) / max(.25, conf)
                for j, lm in enumerate(self.landmarks):
                    if lm.kind != kind or np.linalg.norm(xy - lm.x) > self.gate:
                        continue
                    S = lm.P + np.eye(2) * variance
                    try:
                        delta = xy - lm.x
                        d2 = float(delta @ np.linalg.solve(S, delta))
                    except np.linalg.LinAlgError:
                        continue
                    if d2 <= self.mahalanobis_gate:
                        cost[i, j] = d2
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] >= 1e9:
                    continue
                lm = self.landmarks[j]
                xy, kind, conf, distance = valid[i]
                R = np.eye(2) * ((self.base_variance + self.range_variance * distance**2)
                                 / max(.25, conf))
                S = lm.P + R
                try:
                    K = np.linalg.solve(S, lm.P).T
                except np.linalg.LinAlgError:
                    continue
                lm.x = lm.x + K @ (xy - lm.x)
                lm.P = (np.eye(2) - K) @ lm.P
                lm.P = (lm.P + lm.P.T) / 2
                lm.observations += 1
                lm.missed = 0
                lm.last_seen = stamp
                used_dets.add(i)
        for i, (xy, kind, conf, distance) in enumerate(valid):
            if i in used_dets or len(self.landmarks) >= self.maximum_landmarks:
                continue
            variance = (self.base_variance + self.range_variance * distance**2) / max(.25, conf)
            self.landmarks.append(Landmark(
                self.next_id, xy, np.eye(2) * max(.1, variance),
                kind, 1, 0, stamp,
            ))
            self.next_id += 1
        # Keep confirmed map features when not visible; prune false positives.
        self.landmarks = [
            lm for lm in self.landmarks
            if lm.observations >= self.minimum_observations or
               lm.missed <= self.max_tentative_missed
        ]
        return len(valid)


def transform_xy(point, translation, rotation):
    """Rotate a base_link point into map using an existing TF quaternion."""
    q = np.array(rotation, dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-9 or not np.isfinite(norm):
        raise ValueError("Invalid TF quaternion")
    x, y, z, w = q / norm
    vec = np.asarray(point, dtype=float)
    uv = np.cross([x, y, z], vec)
    rotated = vec + 2 * (w * uv + np.cross([x, y, z], uv))
    return np.asarray(translation, dtype=float) + rotated
