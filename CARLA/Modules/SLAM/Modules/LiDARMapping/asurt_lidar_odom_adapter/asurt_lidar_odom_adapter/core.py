"""Passive, bounded SE(2) trajectory calibration; NEVER reinterpret axes blindly."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def point(self):
        return np.array((self.x, self.y), dtype=float)


@dataclass(frozen=True)
class Pair:
    t: float
    lidar: Pose
    bridge: Pose
    lidar_var: tuple = (0., 0., 0.)
    bridge_var: tuple = (0., 0., 0.)


@dataclass(frozen=True)
class Alignment:
    angle: float
    rotation: np.ndarray
    translation: np.ndarray
    origin: np.ndarray
    train_rms: float
    heldout_rms: float
    yaw_rms: float
    scale_ratio: float
    lidar_cov_reported: bool


@dataclass(frozen=True)
class Output:
    pose: Pose
    var_xyyaw: tuple
    t: float


def fit_se2(x, y, reflection=False):
    """Least-squares rigid alignment of two planar point arrays; no scale."""
    xc, yc = x.mean(axis=0), y.mean(axis=0)
    u, _, vt = np.linalg.svd((x - xc).T @ (y - yc))
    det = np.linalg.det(vt.T @ u.T)
    sign = 1. if det >= 0 else -1.
    rot = vt.T @ np.diag([1., -sign if reflection else sign]) @ u.T
    offset = yc - rot @ xc
    err = (rot @ x.T).T + offset - y
    rms = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))
    return rot, offset, rms


class TrajectoryCalibration:
    """Calibrate and guard LiDAR-only poses against bridge trajectory.

    Bridge is ONLY used for calibration/consistency, never in the output pose.
    A proper rotation must simultaneously match position and quaternion yaw.
    """
    def __init__(self, min_pairs=40, max_pairs=96, min_distance=5.,
                 min_turn=.48, min_spread=.30, max_rms=.20,
                 max_yaw_rms=.12, max_scale_error=.08,
                 max_disagreement=1., max_yaw_disagreement=.33):
        if not 24 <= min_pairs <= max_pairs <= 192:
            raise ValueError('unbounded_correspondence_count')
        self.min_pairs, self.max_pairs = min_pairs, max_pairs
        self.min_distance, self.min_turn = min_distance, min_turn
        self.min_spread, self.max_rms = min_spread, max_rms
        self.max_yaw_rms, self.max_scale_error = max_yaw_rms, max_scale_error
        self.max_disagreement = max_disagreement
        self.max_yaw_disagreement = max_yaw_disagreement
        self.resets = self.emitted = self.rejected = 0
        self.reset('startup', increment=False)

    def reset(self, reason, increment=True):
        self.samples = deque(maxlen=self.max_pairs)
        self.lock = None
        self.last_pair = None
        self.last_output = None
        self.state = reason
        if increment:
            self.resets += 1

    @staticmethod
    def excitation(pairs):
        raw = np.array([p.lidar.point() for p in pairs])
        bridge = np.array([p.bridge.point() for p in pairs])
        headings = np.unwrap(np.array([p.bridge.yaw for p in pairs]))
        length = float(np.linalg.norm(np.diff(bridge, axis=0), axis=1).sum())
        turn = float(np.ptp(headings))
        spread = float(np.linalg.svd(raw - raw.mean(axis=0),
                                    compute_uv=False)[-1] / math.sqrt(len(raw)))
        return length, turn, spread

    def attempt(self):
        samples = list(self.samples)
        if len(samples) < self.min_pairs:
            return None, 'collecting_correspondences'
        split = int(len(samples) * .65)
        train, heldout = samples[:split], samples[split:]
        distance, turn, spread = self.excitation(train)
        if distance < self.min_distance or turn < self.min_turn:
            return None, 'insufficient_translation_or_turn'
        if spread < self.min_spread:
            return None, 'degenerate_almost_collinear_path'
        held_distance, held_turn, _ = self.excitation(heldout)
        if held_distance < 1. or held_turn < .10:
            return None, 'heldout_excitation_insufficient'
        x = np.array([p.lidar.point() for p in train])
        y = np.array([p.bridge.point() for p in train])
        xc, yc = x - x.mean(axis=0), y - y.mean(axis=0)
        ratio = float(np.linalg.norm(yc) / max(1e-9, np.linalg.norm(xc)))
        if abs(ratio - 1.) > self.max_scale_error:
            return None, 'scale_mismatch'
        r, t, rms = fit_se2(x, y)
        _, _, reflected_rms = fit_se2(x, y, reflection=True)
        if reflected_rms < rms * .6 and rms > .04:
            return None, 'reflection_detected'
        if rms > self.max_rms:
            return None, 'train_position_residual'
        theta = math.atan2(r[1, 0], r[0, 0])
        yaw_errors = np.array([
            wrap(p.lidar.yaw + theta - p.bridge.yaw) for p in samples])
        yaw_rms = float(np.sqrt(np.mean(yaw_errors ** 2)))
        if yaw_rms > self.max_yaw_rms or np.max(np.abs(yaw_errors)) > .25:
            return None, 'quaternion_yaw_inconsistent'
        vx = np.array([p.lidar.point() for p in heldout])
        vy = np.array([p.bridge.point() for p in heldout])
        held_rms = float(np.sqrt(np.mean(np.sum(
            ((r @ vx.T).T + t - vy) ** 2, axis=1))))
        if held_rms > self.max_rms * 1.35:
            return None, 'heldout_position_residual'
        return Alignment(theta, r, t, x[0].copy(), rms, held_rms, yaw_rms,
                         ratio, any(max(p.lidar_var) > 0 for p in train)), 'validated'

    def feed(self, pair):
        values = (pair.t, pair.lidar.x, pair.lidar.y, pair.lidar.yaw,
                  pair.bridge.x, pair.bridge.y, pair.bridge.yaw)
        if not all(math.isfinite(v) for v in values) or pair.t <= 0:
            self.rejected += 1
            self.state = 'invalid_pair'
            return None
        if self.last_pair is not None:
            dt = pair.t - self.last_pair.t
            if dt <= 0:
                if dt < -.5:
                    self.reset('time_rewind')
                else:
                    self.rejected += 1
                    self.state = 'duplicate_pair'
                    return None
            elif dt > 2.:
                self.reset('source_gap')
            else:
                raw_step = np.linalg.norm(pair.lidar.point() -
                                          self.last_pair.lidar.point())
                bridge_step = np.linalg.norm(pair.bridge.point() -
                                             self.last_pair.bridge.point())
                raw_turn = abs(wrap(pair.lidar.yaw - self.last_pair.lidar.yaw))
                if (bridge_step > 30. or max(raw_step, bridge_step) >
                        2. + 18. * dt or raw_turn > .35 + 3. * dt):
                    self.reset('teleport_or_pose_jump')
        self.last_pair = pair
        if self.lock is None:
            self.samples.append(pair)
            self.lock, self.state = self.attempt()
            if self.lock is None:
                return None
        calibrated_map_xy = (self.lock.rotation @ pair.lidar.point()
                             + self.lock.translation)
        delta = np.linalg.norm(calibrated_map_xy - pair.bridge.point())
        yaw_delta = abs(wrap(pair.lidar.yaw + self.lock.angle -
                             pair.bridge.yaw))
        if delta > self.max_disagreement or yaw_delta > self.max_yaw_disagreement:
            self.reset('postlock_bridge_disagreement')
            self.samples.append(pair)
            self.last_pair = pair
            return None
        # Output origin is the initial LiDAR position; no bridge coordinates
        # or bridge heading can enter this normalized output.
        xy = self.lock.rotation @ (pair.lidar.point() - self.lock.origin)
        yaw = wrap(pair.lidar.yaw + self.lock.angle)
        estimate = max(.04 ** 2, self.lock.train_rms ** 2,
                       self.lock.heldout_rms ** 2)
        var_xy = max(.025, estimate + max(pair.lidar_var[:2]))
        var_yaw = max(.012, self.lock.yaw_rms ** 2 + pair.lidar_var[2])
        out = Output(Pose(float(xy[0]), float(xy[1]), yaw),
                     (var_xy, var_xy, var_yaw), pair.t)
        self.last_output = out
        self.emitted += 1
        self.state = 'validated_LeGO_derived'
        return out
