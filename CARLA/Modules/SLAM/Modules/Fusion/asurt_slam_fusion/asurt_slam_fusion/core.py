"""ROS-free planar measurement fusion and bounded map-relative pose correction.

The sensor confidence fields are heuristics, not calibrated likelihoods.  Noise
floors, gates and the published pose covariance are conservative engineering
estimates, pending measured calibration of camera and LiDAR residuals.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


def wrap(angle):
    return (float(angle) + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def compose(a, b):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2(a.x + c * b.x - s * b.y,
                 a.y + s * b.x + c * b.y, wrap(a.yaw + b.yaw))


def between(a, b):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    dx, dy = b.x - a.x, b.y - a.y
    return Pose2(c * dx + s * dy, -s * dx + c * dy,
                 wrap(b.yaw - a.yaw))


def transform_local(pose, xy):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.array([pose.x + c * xy[0] - s * xy[1],
                     pose.y + s * xy[0] + c * xy[1]], dtype=float)


def rebase_point(point, pose_at_source, pose_at_target):
    """A point in base_link at source stamp, expressed at target stamp."""
    world = transform_local(pose_at_source, point[:2])
    c, s = math.cos(pose_at_target.yaw), math.sin(pose_at_target.yaw)
    delta = world - np.array([pose_at_target.x, pose_at_target.y])
    return (c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1],
            float(point[2]))


@dataclass(frozen=True)
class Cone:
    x: float
    y: float
    z: float
    kind: int
    confidence: float
    identifier: int = -1

    @property
    def xy(self):
        return np.array((self.x, self.y), dtype=float)


def valid_cones(cones, max_count=64, max_range=35.0):
    result = []
    for d in cones:
        if len(result) >= max_count:
            break
        if (d.kind not in (0, 1, 2, 3, 4) or
                not all(math.isfinite(v) for v in (d.x, d.y, d.z, d.confidence)) or
                not 0.0 < d.confidence <= 1.0 or
                not .20 <= math.hypot(d.x, d.y) <= max_range):
            continue
        result.append(d)
    return result


def sensor_variance(d, source):
    rng = math.hypot(d.x, d.y)
    # Long-range stereo depth has substantially higher uncertainty than LiDAR.
    sigma = ((.13 + .035 * rng + .055 * max(0.0, rng - 7.0))
             if source == 'camera' else (.075 + .012 * rng))
    return (sigma / math.sqrt(max(.25, d.confidence))) ** 2


def fuse_cones(camera, lidar, gate_m=1.25, gate_chi2=9.21, max_count=64):
    """One-to-one Hungarian association; return fused cones and source counts.

    Each output cone is supported by *both* sensors. LiDAR type UNKNOWN=4 is
    allowed and gets the camera colour; other known type conflicts are rejected.
    """
    cams = valid_cones(camera, max_count)
    lasers = valid_cones(lidar, max_count)
    costs = np.full((len(cams), len(lasers)), 1e9)
    for i, c in enumerate(cams):
        for j, l in enumerate(lasers):
            if c.kind != l.kind and l.kind != 4 and c.kind != 4:
                continue
            distance2 = float(np.sum((c.xy - l.xy) ** 2))
            noise = sensor_variance(c, 'camera') + sensor_variance(l, 'lidar')
            if distance2 <= gate_m ** 2 and distance2 <= gate_chi2 * noise:
                costs[i, j] = distance2 / noise
    fused = []
    if len(cams) and len(lasers):
        row, col = linear_sum_assignment(costs)
        for i, j in zip(row, col):
            if costs[i, j] >= 1e9:
                continue
            c, l = cams[i], lasers[j]
            wc = 1.0 / sensor_variance(c, 'camera')
            wl = 1.0 / sensor_variance(l, 'lidar')
            xyz = (wc * np.array([c.x, c.y, c.z]) +
                   wl * np.array([l.x, l.y, l.z])) / (wc + wl)
            fused.append(Cone(*map(float, xyz), c.kind if c.kind != 4 else l.kind,
                              min(c.confidence, l.confidence), c.identifier))
    return fused, {'camera': len(cams), 'lidar': len(lasers),
                   'matched': len(fused), 'unmatched_camera': len(cams) - len(fused),
                   'unmatched_lidar': len(lasers) - len(fused)}


@dataclass
class Anchor:
    xy: np.ndarray
    kind: int
    hits: int
    last_seen: float


class LidarPoseCorrector:
    """Integrate independent LiDAR-odom deltas, correct using fused map cones.

    The local landmark reference map is initialized against bridge odometry;
    therefore this is map-relative drift correction, not independently fixed
    global localization. No pose result is returned without at least three
    confirmed camera+LiDAR correspondences and an accepted LiDAR odom delta.
    """

    def __init__(self, max_anchors=500, map_gate=1.4, max_step_m=.85,
                 max_step_yaw=.15, map_ttl_s=120.0):
        self.max_anchors = max_anchors
        self.map_gate = map_gate
        self.max_step_m = max_step_m
        self.max_step_yaw = max_step_yaw
        self.map_ttl_s = map_ttl_s
        self.reset()

    def reset(self):
        self.anchors = []
        self.prediction = None
        self.last_lidar = None
        self.last_lidar_stamp = None
        self.prediction_cov = np.diag([.20 ** 2, .20 ** 2, .07 ** 2])
        self.corrected = 0
        self.last_residual = None
        self.last_matches = 0

    def integrate(self, lidar_pose, stamp, bridge_pose=None):
        if not all(math.isfinite(x) for x in (lidar_pose.x, lidar_pose.y,
                                               lidar_pose.yaw, stamp)):
            return 'invalid_lidar_odom'
        if self.last_lidar_stamp is not None and stamp <= self.last_lidar_stamp:
            return 'lidar_timestamp_rewind' if stamp < self.last_lidar_stamp - .5 else 'duplicate_lidar_odom'
        if self.last_lidar is None:
            if bridge_pose is None:
                return 'waiting_for_bridge_odom'
            self.prediction = bridge_pose
            self.last_lidar = lidar_pose
            self.last_lidar_stamp = stamp
            return 'anchored'
        delta = between(self.last_lidar, lidar_pose)
        travelled = math.hypot(delta.x, delta.y)
        if travelled > 4.0 or abs(delta.yaw) > .8:
            # Never allow a rejected discontinuity to leave a publishable
            # previous prediction in memory; caller also clears ROS queues.
            self.reset()
            return 'lidar_odometry_jump'
        self.prediction = compose(self.prediction, delta)
        self.prediction_cov += np.diag([
            (.025 + .045 * travelled) ** 2,
            (.025 + .045 * travelled) ** 2,
            (.008 + .05 * abs(delta.yaw)) ** 2])
        self.prediction_cov = np.minimum(self.prediction_cov,
                                         np.diag([4.0, 4.0, .5 ** 2]))
        self.last_lidar, self.last_lidar_stamp = lidar_pose, stamp
        return 'integrated'

    def _associations(self, cones, pose, min_hits=2):
        candidates = [(i, a) for i, a in enumerate(self.anchors)
                      if a.hits >= min_hits]
        costs = np.full((len(cones), len(candidates)), 1e9)
        for i, cone in enumerate(cones):
            xy = transform_local(pose, cone.xy)
            for j, (_, a) in enumerate(candidates):
                if cone.kind != a.kind:
                    continue
                d = float(np.linalg.norm(xy - a.xy))
                if d < self.map_gate:
                    costs[i, j] = d
        pairs = []
        if len(cones) and len(candidates):
            row, col = linear_sum_assignment(costs)
            for i, j in zip(row, col):
                if costs[i, j] < 1e9:
                    pairs.append((cones[i].xy, candidates[j][1].xy))
        return pairs

    def correct(self, cones, stamp, max_odom_age=.18):
        """Return corrected (pose, covariance) only on valid sensor constraints."""
        self.last_matches = 0
        if (self.prediction is None or self.last_lidar_stamp is None or
                abs(stamp - self.last_lidar_stamp) > max_odom_age or
                len(cones) < 3):
            return None, 'missing_recent_lidar_odom_or_fused_cones'
        pairs = self._associations(cones, self.prediction)
        self.last_matches = len(pairs)
        if len(pairs) < 3:
            return None, 'insufficient_confirmed_map_matches'
        observed = np.array([p[0] for p in pairs])
        target = np.array([p[1] for p in pairs])
        if np.linalg.svd(observed - observed.mean(axis=0), compute_uv=False)[0] < 1.0:
            return None, 'insufficient_landmark_spread'
        # Procrustes best-fit rigid transform for point-to-point planar data.
        x_mean, y_mean = observed.mean(axis=0), target.mean(axis=0)
        u, _, vt = np.linalg.svd((observed - x_mean).T @ (target - y_mean))
        correction = np.eye(2)
        correction[-1, -1] = np.linalg.det(u @ vt)
        rot = u @ correction @ vt
        # row-vector transform: local @ rot + translation
        translation = y_mean - x_mean @ rot
        fitted_yaw = math.atan2(rot[0, 1], rot[0, 0])
        fitted = Pose2(float(translation[0]), float(translation[1]), wrap(fitted_yaw))
        step = between(self.prediction, fitted)
        if math.hypot(step.x, step.y) > self.max_step_m or abs(step.yaw) > self.max_step_yaw:
            return None, 'pose_innovation_exceeds_gate'
        residual = np.linalg.norm(observed @ rot + translation - target, axis=1)
        rms = float(np.sqrt(np.mean(residual ** 2)))
        if not math.isfinite(rms) or rms > .35:
            return None, 'landmark_residual_exceeds_gate'
        baseline = max(1.0, float(np.linalg.norm(observed - x_mean, axis=1).mean()))
        variance_xy = max(.12 ** 2, (.12 + rms) ** 2 / len(pairs))
        variance_yaw = max(.025 ** 2, variance_xy / (baseline ** 2 * len(pairs)))
        R = np.diag([variance_xy, variance_xy, variance_yaw])
        gain = np.minimum(self.prediction_cov.diagonal() /
                          (self.prediction_cov.diagonal() + R.diagonal()), .65)
        updated = compose(self.prediction, Pose2(float(gain[0] * step.x),
                                                float(gain[1] * step.y),
                                                float(gain[2] * step.yaw)))
        # Conservative variance bounds and Joseph-form-like diagonal update.
        diag = ((1 - gain) ** 2 * self.prediction_cov.diagonal() +
                gain ** 2 * R.diagonal())
        self.prediction_cov = np.diag(np.maximum(diag,
                                                 [.08 ** 2, .08 ** 2, .02 ** 2]))
        self.prediction = updated
        self.corrected += 1
        self.last_residual = rms
        return (updated, self.prediction_cov.copy()), 'corrected'

    def learn(self, cones, bridge_pose, stamp):
        """Create slow-moving local reference anchors from paired observations.

        The bridge supplies a map-coordinate bootstrap. Correlated detections
        never become new independent pose observations in their own frame.
        """
        self.anchors = [a for a in self.anchors if stamp - a.last_seen <= self.map_ttl_s]
        used = set()
        for cone in cones:
            if cone.kind not in (0, 1, 2, 3):
                continue
            xy = transform_local(bridge_pose, cone.xy)
            best = None
            for i, a in enumerate(self.anchors):
                if i in used or a.kind != cone.kind:
                    continue
                distance = float(np.linalg.norm(a.xy - xy))
                if distance < .65 and (best is None or distance < best[0]):
                    best = (distance, i)
            if best is not None:
                a = self.anchors[best[1]]
                # Bounded update to prevent instantaneous map chasing.
                a.xy = .98 * a.xy + .02 * xy
                a.hits = min(a.hits + 1, 1000)
                a.last_seen = stamp
                used.add(best[1])
            elif len(self.anchors) < self.max_anchors:
                self.anchors.append(Anchor(xy, cone.kind, 1, stamp))
                used.add(len(self.anchors) - 1)
