"""ASURT pure-pursuit geometry derived from the FS-System-26 control concepts.

The original fixed- and adaptive-lookahead implementations are retained
unchanged in Legacy; this module removes their old TF and simulated vehicle
publishers. Coordinates are ROS REP-103 (x forward, y left).
"""
from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

Point2 = Tuple[float, float]


@dataclass(frozen=True)
class VehiclePose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Settings:
    wheelbase_m: float = 1.575
    lookahead_base_m: float = 1.8
    lookahead_gain_s: float = 0.55
    lookahead_min_m: float = 1.3
    lookahead_max_m: float = 4.0
    max_steer_rad: float = 0.45
    target_speed_mps: float = 0.8
    min_speed_mps: float = 0.35
    curvature_speed_gain_m: float = 1.6
    max_accel_mps2: float = 0.8

    def validate(self):
        if not all(math.isfinite(float(v)) for v in vars(self).values()):
            raise ValueError("nonfinite_control_setting")
        if (self.wheelbase_m <= 0 or self.lookahead_base_m <= 0
                or self.lookahead_min_m <= 0 or self.lookahead_max_m < self.lookahead_min_m
                or not 0 < self.max_steer_rad <= 1.2
                or not 0 <= self.min_speed_mps <= self.target_speed_mps
                or self.curvature_speed_gain_m < 0 or self.lookahead_gain_s < 0
                or self.max_accel_mps2 <= 0):
            raise ValueError("invalid_control_settings")


def local_to_map(point: Point2, pose: VehiclePose) -> Point2:
    co, si = math.cos(pose.yaw), math.sin(pose.yaw)
    x, y = point
    return pose.x + co*x - si*y, pose.y + si*x + co*y


def map_to_local(point: Point2, pose: VehiclePose) -> Point2:
    co, si = math.cos(pose.yaw), math.sin(pose.yaw)
    dx, dy = point[0]-pose.x, point[1]-pose.y
    return co*dx + si*dy, -si*dx + co*dy


def current_local_path(points: Sequence[Point2], frame: str,
                       now_pose: VehiclePose,
                       source_pose: Optional[VehiclePose] = None) -> Tuple[Point2, ...]:
    """Convert map-fixed or source-time base_link waypoints to current base_link."""
    if frame == "map":
        return tuple(map_to_local(point, now_pose) for point in points)
    if frame == "base_link" and source_pose is not None:
        return tuple(map_to_local(local_to_map(point, source_pose), now_pose)
                     for point in points)
    raise ValueError("missing_source_pose_or_unknown_frame")


def lookahead_distance(speed: float, cfg: Settings) -> float:
    return max(cfg.lookahead_min_m, min(cfg.lookahead_max_m,
               cfg.lookahead_base_m + cfg.lookahead_gain_s * abs(speed)))


def target_on_path(points: Sequence[Point2], lookahead: float) -> Optional[Point2]:
    """Nearest polyline projection then arc-length lookahead toward the end."""
    if len(points) < 2 or not math.isfinite(lookahead) or lookahead <= 0:
        return None
    if any(not (math.isfinite(x) and math.isfinite(y)) for x, y in points):
        return None
    best = (float("inf"), 0, points[0])
    for i in range(len(points)-1):
        ax, ay = points[i]
        bx, by = points[i+1]
        dx, dy = bx-ax, by-ay
        dist_sq = dx*dx + dy*dy
        if dist_sq < 1e-10:
            continue
        t = max(0., min(1., -(ax*dx+ay*dy)/dist_sq))
        px, py = ax+t*dx, ay+t*dy
        candidate = (px*px+py*py, i, (px, py))
        if candidate[0] < best[0]:
            best = candidate
    if not math.isfinite(best[0]):
        return None
    _, index, start = best
    remaining = lookahead
    for j in range(index, len(points)-1):
        end = points[j+1]
        dx, dy = end[0]-start[0], end[1]-start[1]
        distance = math.hypot(dx, dy)
        if distance >= remaining and distance > 1e-8:
            target = (start[0] + remaining*dx/distance,
                      start[1] + remaining*dy/distance)
            return target if target[0] > 0.15 else None
        remaining -= distance
        start = end
    return start if start[0] > 0.15 else None


def steering_and_speed(target: Point2, cfg: Settings) -> Tuple[float, float, float]:
    """Bicycle pure-pursuit curvature -> Ackermann steering and target speed."""
    x, y = target
    if not math.isfinite(x) or not math.isfinite(y) or x <= .15:
        raise ValueError("invalid_forward_target")
    curvature = 2.0*y/(x*x+y*y)
    steer = max(-cfg.max_steer_rad, min(cfg.max_steer_rad,
                math.atan(cfg.wheelbase_m*curvature)))
    speed = cfg.target_speed_mps/(1.0+cfg.curvature_speed_gain_m*abs(curvature))
    speed = min(cfg.target_speed_mps, max(cfg.min_speed_mps, speed))
    return steer, speed, curvature


def ramp_speed(previous: float, requested: float, acceleration: float, dt: float) -> float:
    """Limit speed increases while allowing a zero-speed command immediately."""
    if not all(math.isfinite(x) for x in (previous, requested, acceleration, dt)):
        raise ValueError("invalid_speed_ramp")
    if requested <= 0:
        return 0.
    return max(0., min(requested, previous + max(0., acceleration*dt)))
