"""ROS-free map-relative camera localization from repeated metric cone detections.

Anchors are initialized in the bridge odometry's map coordinates, so this can
correct drift *relative to that locally established map*. It cannot remove a
global error already present when the anchors were established. Confirmed
anchors are frozen to avoid making the reference chase subsequent drift.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


def wrap(yaw):
    return (float(yaw) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def compose(a, b):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2(a.x + c*b.x - s*b.y, a.y + s*b.x + c*b.y,
                 wrap(a.yaw+b.yaw))


def between(a, b):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    dx, dy = b.x-a.x, b.y-a.y
    return Pose2(c*dx+s*dy, -s*dx+c*dy, wrap(b.yaw-a.yaw))


def transform(pose, local_xy):
    xy = np.asarray(local_xy, dtype=float)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.stack((pose.x+c*xy[..., 0]-s*xy[..., 1],
                     pose.y+s*xy[..., 0]+c*xy[..., 1]), axis=-1)


@dataclass(frozen=True)
class Observation:
    x: float
    y: float
    z: float
    color: int
    confidence: float

    @property
    def range_m(self):
        return math.hypot(self.x, self.y)


@dataclass
class Anchor:
    uid: int
    xy: np.ndarray
    cov: np.ndarray
    color: int
    hits: int
    views: int
    first_pose: Pose2
    last_view_pose: Pose2
    baseline_m: float
    last_seen: float


@dataclass
class Report:
    state: str
    pose: object = None
    covariance: object = None
    matches: int = 0
    inliers: int = 0
    residual_rms_m: object = None
    innovation_m: object = None
    innovation_yaw_rad: object = None
    accepted_observations: int = 0
    reset_reason: str = ""


class CameraMapLocalizer:
    """Bounded one-to-one association, repeated-view map, robust SE(2) fitting."""

    def __init__(self, max_detections=48, max_anchors=400, max_range_m=14.,
                 min_confidence=.45, association_gate_m=1.7, gate_chi2=9.21,
                 min_anchor_hits=4, min_distinct_views=3,
                 min_view_motion_m=.15, min_baseline_m=.45,
                 anchor_ttl_s=240., max_step_m=.9, max_step_yaw_rad=.22,
                 max_residual_rms_m=.55, max_pose_jump_m=10.,
                 max_pose_jump_yaw_rad=1.1):
        self.max_detections = max(3, int(max_detections))
        self.max_anchors = max(3, int(max_anchors))
        self.max_range_m = max(.5, float(max_range_m))
        self.min_confidence = float(min_confidence)
        self.association_gate_m = float(association_gate_m)
        self.gate_chi2 = float(gate_chi2)
        self.min_anchor_hits = max(2, int(min_anchor_hits))
        self.min_distinct_views = max(2, int(min_distinct_views))
        self.min_view_motion_m = float(min_view_motion_m)
        self.min_baseline_m = float(min_baseline_m)
        self.anchor_ttl_s = float(anchor_ttl_s)
        self.max_step_m = float(max_step_m)
        self.max_step_yaw_rad = float(max_step_yaw_rad)
        self.max_residual_rms_m = float(max_residual_rms_m)
        self.max_pose_jump_m = float(max_pose_jump_m)
        self.max_pose_jump_yaw_rad = float(max_pose_jump_yaw_rad)
        self.reset()

    def reset(self):
        self.anchors = []
        self.next_uid = 0
        self.last_odom = None
        self.last_stamp = None
        self.predicted_pose = None
        self.covariance = np.diag([.4**2, .4**2, .13**2])
        self.corrected_frames = 0
        self.last_reset_reason = ""

    def stable(self, anchor):
        return (anchor.hits >= self.min_anchor_hits and
                anchor.views >= self.min_distinct_views and
                anchor.baseline_m >= self.min_baseline_m)

    def confirmed(self):
        return [a for a in self.anchors if self.stable(a)]

    def _valid(self, detections):
        valid = []
        for obs in detections:
            if len(valid) >= self.max_detections:
                break
            if (obs.color not in (0, 1, 2, 3) or
                    not all(math.isfinite(v) for v in
                            (obs.x, obs.y, obs.z, obs.confidence)) or
                    not self.min_confidence <= obs.confidence <= 1.0 or
                    not .3 <= obs.range_m <= self.max_range_m or
                    abs(obs.z) > 2.):
                continue
            valid.append(obs)
        return valid

    @staticmethod
    def _variance(obs):
        """Conservative XY uncertainty; stereo depth errors grow past 7 m."""
        r = obs.range_m
        sigma = .09 + .045*r + .16*max(0., r-7.)
        return (sigma / math.sqrt(max(.25, obs.confidence)))**2

    def _associate(self, dets, pose, anchors):
        if not dets or not anchors:
            return []
        local = np.array([(o.x, o.y) for o in dets])
        pred = transform(pose, local)
        target = np.array([a.xy for a in anchors])
        delta = pred[:, None, :] - target[None, :, :]
        d2 = np.sum(delta**2, axis=2)
        # Include map uncertainty and imperfect pose prediction in the gate.
        var = (np.array([self._variance(d) for d in dets])[:, None] +
               np.array([np.trace(a.cov)*.5 for a in anchors])[None, :] +
               .12**2 + (.04* np.array([d.range_m for d in dets])[:, None])**2)
        colors = (np.array([d.color for d in dets])[:, None] ==
                  np.array([a.color for a in anchors])[None, :])
        allowed = (colors & (d2 <= self.association_gate_m**2) &
                   (d2 <= self.gate_chi2*var))
        cost = np.where(allowed, d2 / np.maximum(var, .0001), 1e8)
        rows, cols = linear_sum_assignment(cost)
        return [(int(i), int(j)) for i, j in zip(rows, cols)
                if cost[i, j] < 1e8]

    @staticmethod
    def _geometry_ok(points):
        if len(points) < 3:
            return False
        singular = np.linalg.svd(points-points.mean(axis=0),
                                 compute_uv=False)
        # A single straight cone row is incapable of robust XY/yaw association.
        return bool(singular[-1] >= .30 and
                    singular[0] >= .85 and
                    singular[0]/max(singular[-1], 1e-6) <= 25.)

    def _fit_pose(self, dets, anchors, pairs, prior, prior_cov):
        if len(pairs) < 3:
            return Report("insufficient_repeated_cones", matches=len(pairs))
        local = np.array([(dets[i].x, dets[i].y) for i, _ in pairs])
        world = np.array([anchors[j].xy for _, j in pairs])
        if not self._geometry_ok(local) or not self._geometry_ok(world):
            return Report("degenerate_landmark_geometry", matches=len(pairs))
        variance = np.array([self._variance(dets[i]) +
                             np.trace(anchors[j].cov)*.5
                             for i, j in pairs])
        # Robust Huber GN with a motion prior; iterative outlier pruning follows.
        keep = np.ones(len(pairs), dtype=bool)
        base = np.array([prior.x, prior.y, prior.yaw], dtype=float)
        solution = base.copy()
        prior_info = np.linalg.inv(prior_cov)
        H = prior_info.copy()
        for pass_id in range(2):
            for _ in range(9):
                yaw = solution[2]
                c, s = math.cos(yaw), math.sin(yaw)
                points = local[keep]
                target = world[keep]
                estimated = transform(Pose2(*solution), points)
                residual = estimated-target
                J = np.zeros((len(points), 2, 3), dtype=float)
                J[:, 0, 0] = 1.
                J[:, 1, 1] = 1.
                J[:, 0, 2] = -s*points[:, 0]-c*points[:, 1]
                J[:, 1, 2] = c*points[:, 0]-s*points[:, 1]
                norm = np.linalg.norm(residual, axis=1)
                sig = np.sqrt(variance[keep])
                robust = np.minimum(1., 1.5*sig/np.maximum(norm, 1e-9))
                weight = robust/variance[keep]
                H = prior_info + np.einsum('n,nij,nik->jk', weight, J, J)
                diff = solution-base
                diff[2] = wrap(diff[2])
                gradient = prior_info @ diff + np.einsum(
                    'n,nij,ni->j', weight, J, residual)
                try:
                    delta = np.linalg.solve(H, gradient)
                except np.linalg.LinAlgError:
                    return Report("singular_pose_system", matches=len(pairs))
                solution -= delta
                solution[2] = wrap(solution[2])
                if np.linalg.norm(delta) < 1e-5:
                    break
            residual_all = np.linalg.norm(
                transform(Pose2(*solution), local)-world, axis=1)
            updated = residual_all <= np.maximum(.5, 2.5*np.sqrt(variance))
            if pass_id == 0 and not np.array_equal(updated, keep):
                keep = updated
                if keep.sum() < 3:
                    return Report("insufficient_robust_inliers",
                                  matches=len(pairs), inliers=int(keep.sum()))
                if not (self._geometry_ok(local[keep]) and
                        self._geometry_ok(world[keep])):
                    return Report("degenerate_inlier_geometry",
                                  matches=len(pairs), inliers=int(keep.sum()))
                solution = base.copy()
                continue
            break
        rms = float(np.sqrt(np.mean(residual_all[keep]**2)))
        fitted = Pose2(float(solution[0]), float(solution[1]),
                       wrap(solution[2]))
        innovation = between(prior, fitted)
        magnitude = math.hypot(innovation.x, innovation.y)
        common = dict(matches=len(pairs), inliers=int(keep.sum()),
                      residual_rms_m=rms, innovation_m=magnitude,
                      innovation_yaw_rad=innovation.yaw)
        if (magnitude > self.max_step_m or
                abs(innovation.yaw) > self.max_step_yaw_rad):
            return Report("pose_innovation_rejected", **common)
        if not math.isfinite(rms) or rms > self.max_residual_rms_m:
            return Report("geometric_residual_rejected", **common)
        try:
            posterior = np.linalg.inv(H)
        except np.linalg.LinAlgError:
            return Report("singular_pose_system", **common)
        # Floor reflects correlated stereo/anchor errors even at high FPS.
        conservative = np.diag([.07**2, .07**2, .025**2])
        posterior += conservative
        posterior *= max(1., rms**2 / max(.01, float(np.mean(variance[keep]))))
        if not np.all(np.isfinite(posterior)):
            return Report("invalid_posterior", **common)
        return Report("corrected", fitted, posterior, **common)

    def _learn(self, dets, pose, stamp, freeze_existing):
        pairs = self._associate(dets, pose, self.anchors)
        used = set()
        for i, j in pairs:
            used.add(i)
            a = self.anchors[j]
            a.hits = min(a.hits+1, 100000)
            a.last_seen = stamp
            travel = math.hypot(pose.x-a.last_view_pose.x,
                                pose.y-a.last_view_pose.y)
            if (travel >= self.min_view_motion_m or
                    abs(wrap(pose.yaw-a.last_view_pose.yaw)) >= .05):
                a.views += 1
                a.last_view_pose = pose
            a.baseline_m = max(a.baseline_m, math.hypot(
                pose.x-a.first_pose.x, pose.y-a.first_pose.y))
            # Once an anchor was already confirmed, freeze its world position.
            if freeze_existing.get(a.uid, False):
                continue
            measurement = transform(pose, (dets[i].x, dets[i].y))
            R = np.eye(2)*self._variance(dets[i])
            S = a.cov+R
            K = np.linalg.solve(S, a.cov).T
            a.xy += K @ (measurement-a.xy)
            a.cov = (np.eye(2)-K)@a.cov@(np.eye(2)-K).T + K@R@K.T
            a.cov += np.eye(2)*.005**2
        for i, det in enumerate(dets):
            if i in used or len(self.anchors) >= self.max_anchors:
                continue
            self.anchors.append(Anchor(
                self.next_uid, transform(pose, (det.x, det.y)),
                np.eye(2)*max(.10**2, self._variance(det)), det.color,
                1, 1, pose, pose, 0., stamp))
            self.next_uid += 1

    def step(self, odom_pose, observations, stamp):
        if (not math.isfinite(float(stamp)) or stamp <= 0 or
                not all(math.isfinite(v) for v in (
                    odom_pose.x, odom_pose.y, odom_pose.yaw))):
            return Report("invalid_time_or_odometry")
        reset_reason = ""
        if self.last_stamp is not None:
            if stamp <= self.last_stamp:
                if stamp < self.last_stamp-.25:
                    self.reset()
                    reset_reason = "time_rewind"
                else:
                    return Report("duplicate_or_out_of_order_stamp")
            else:
                delta = between(self.last_odom, odom_pose)
                if (math.hypot(delta.x, delta.y) > self.max_pose_jump_m or
                        abs(delta.yaw) > self.max_pose_jump_yaw_rad):
                    self.reset()
                    reset_reason = "odom_teleport_or_world_change"
        dets = self._valid(observations)
        self.anchors = [a for a in self.anchors
                        if stamp-a.last_seen <= self.anchor_ttl_s]
        frozen = {a.uid: self.stable(a) for a in self.anchors}
        if self.last_odom is None:
            prediction = odom_pose
            prediction_cov = np.diag([.4**2, .4**2, .13**2])
        else:
            delta = between(self.last_odom, odom_pose)
            prediction = compose(self.predicted_pose, delta)
            motion = math.hypot(delta.x, delta.y)
            # Intentionally drop posterior cross-correlation for the next
            # motion prior; cap each positive variance, preserving SPD.
            # Elementwise clipping a full covariance can destroy PSD.
            prediction_variances = np.diag(self.covariance) + np.array([
                (.025+.05*motion)**2, (.025+.05*motion)**2,
                (.008+.06*abs(delta.yaw))**2])
            prediction_cov = np.diag(np.clip(
                prediction_variances,
                [.08**2, .08**2, .025**2],
                [.75**2, .75**2, .22**2]))
            if (math.hypot(prediction.x-odom_pose.x,
                           prediction.y-odom_pose.y) > 2.5 or
                    abs(wrap(prediction.yaw-odom_pose.yaw)) > .45):
                self.reset()
                reset_reason = "map_relative_pose_diverged_from_odom"
                prediction = odom_pose
                prediction_cov = np.diag([.4**2, .4**2, .13**2])
                frozen = {}
        # Local pose correction must use anchors confirmed BEFORE this frame.
        stable = [a for a in self.anchors if frozen.get(a.uid)]
        pairs = self._associate(dets, prediction, stable)
        report = self._fit_pose(dets, stable, pairs, prediction,
                                prediction_cov)
        self.last_odom = odom_pose
        self.last_stamp = stamp
        self.predicted_pose = report.pose if report.pose else prediction
        self.covariance = (report.covariance if report.covariance is not None
                           else prediction_cov)
        if report.pose is not None:
            self.corrected_frames += 1
        # Do not let a rejected large innovation plant false reference cones.
        if report.state != "pose_innovation_rejected":
            self._learn(dets, self.predicted_pose, stamp, frozen)
        report.accepted_observations = len(dets)
        report.reset_reason = reset_reason
        self.last_reset_reason = reset_reason
        return report
