"""Signed cross-track Stanley path tracking + look-ahead curvature speed cap.

ROS REP-103: positive steering = left; map coordinates x/y/yaw. The Stanley
controller tracks the front axle, unlike the original vehicle-centered
Pure Pursuit module. The CARLA bridge applies steering-sign translation.
"""
from dataclasses import dataclass
import math


MAX_TRACK_PATH_POINTS = 6000  # Matches TrackGoal's bounded full-course output.


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Settings:
    wheelbase_m: float = 1.575
    max_steer_rad: float = math.radians(16.)
    target_speed_mps: float = 100./3.6
    stanley_gain: float = 1.7
    softening_speed_mps: float = 1.2
    max_lateral_accel_mps2: float = 4.0
    max_accel_mps2: float = 4.0
    max_brake_mps2: float = 7.0
    goal_buffer_m: float = .9
    cross_track_limit_m: float = 2.0
    heading_limit_rad: float = 0.90
    max_path_curvature_m_inv: float = .182

    def validate(self):
        if not all(math.isfinite(float(v)) for v in vars(self).values()):
            raise ValueError("nonfinite_stanley_parameter")
        if not (0.1 < self.wheelbase_m < 4.
                and 0 < self.max_steer_rad <= math.radians(18.)
                and 0 < self.target_speed_mps <= 100./3.6+1e-6
                and .01 <= self.stanley_gain <= 20
                and .01 <= self.softening_speed_mps <= 10
                and 0 < self.max_lateral_accel_mps2 <= 15
                and 0 < self.max_accel_mps2 <= 20
                and 0 < self.max_brake_mps2 <= 20
                and self.goal_buffer_m >= 0
                and .1 <= self.cross_track_limit_m <= 10
                and .1 <= self.heading_limit_rad <= math.pi
                and 0 < self.max_path_curvature_m_inv <= .5):
            raise ValueError("invalid_stanley_parameters")


def wrap(yaw):
    return (yaw+math.pi)%(2*math.pi)-math.pi


def curvature(a, b, c):
    ab, bc, ac = (math.dist(a, b), math.dist(b, c), math.dist(a, c))
    if min(ab, bc, ac) < 1e-7:
        return 0.
    cross = ((b[0]-a[0])*(c[1]-a[1])-
             (b[1]-a[1])*(c[0]-a[0]))
    return 2*cross/(ab*bc*ac)


def project_front_axle(points, pose, wheelbase):
    """Closest path point to the vehicle front axle, with signed heading error."""
    front = (pose.x+wheelbase*math.cos(pose.yaw),
             pose.y+wheelbase*math.sin(pose.yaw))
    best = None
    along = 0.
    for index, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        ds2 = dx*dx+dy*dy
        if ds2 < 1e-10:
            continue
        t = max(0., min(1., ((front[0]-a[0])*dx+(front[1]-a[1])*dy)/ds2))
        projection = (a[0]+t*dx, a[1]+t*dy)
        error = (projection[0]-front[0])**2+(projection[1]-front[1])**2
        progress = along+t*math.sqrt(ds2)
        candidate = (error, -progress, index, projection, progress)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
        along += math.sqrt(ds2)
    if best is None:
        raise ValueError("no_nonzero_path_segments")
    _, _, index, point, progress = best
    tangent = math.atan2(points[index+1][1]-points[index][1],
                         points[index+1][0]-points[index][0])
    path_left_error = -(
        math.cos(tangent)*(front[1]-point[1])-
        math.sin(tangent)*(front[0]-point[0]))
    return index, point, progress, path_left_error, wrap(tangent-pose.yaw)


def track_command(points, pose, speed_mps, cfg=Settings(), *, stop_before_tight_bend=False):
    """Return steering, requested speed and tracking metrics for a map path."""
    cfg.validate()
    if len(points) < 3 or len(points) > MAX_TRACK_PATH_POINTS:
        raise ValueError("path_size")
    if not all(math.isfinite(v) for xy in points for v in xy):
        raise ValueError("nonfinite_path")
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, speed_mps)):
        raise ValueError("nonfinite_pose_or_speed")
    i, nearest, progress, cross_error, heading_error = project_front_axle(
        points, pose, cfg.wheelbase_m)
    if abs(cross_error) > cfg.cross_track_limit_m:
        raise ValueError("cross_track_error_exceeded")
    if abs(heading_error) > cfg.heading_limit_rad:
        raise ValueError("heading_error_exceeded")
    maximum_curvature = 0.
    # Full length along actual path, used for a braking-distance envelope.
    cumulative = [0.]
    for a, b in zip(points, points[1:]):
        cumulative.append(cumulative[-1]+math.dist(a, b))
    final_remaining = max(0., cumulative[-1]-progress-cfg.goal_buffer_m)
    permitted = min(cfg.target_speed_mps,
                    math.sqrt(2*cfg.max_brake_mps2*final_remaining))
    curvature_limit = cfg.max_path_curvature_m_inv
    if stop_before_tight_bend:
        curvature_limit = min(curvature_limit, math.tan(cfg.max_steer_rad)/cfg.wheelbase_m)
    distance_to_tight_bend = None
    local_curvature = 0.
    for j in range(1, len(points)-1):
        k = curvature(points[j-1], points[j], points[j+1])
        maximum_curvature = max(maximum_curvature, abs(k))
        if j == i+1:
            local_curvature = k
        if stop_before_tight_bend and abs(k) > curvature_limit:
            if cumulative[j+1] <= progress:
                continue  # This curvature sample lies entirely behind the axle.
            distance_to_tight_bend = max(0., cumulative[j]-progress)
            # The curvature sample spans both neighbouring segments. Allow
            # room for its entry as well as the normal stopping buffer.
            entry = min(cfg.wheelbase_m, cumulative[j]-cumulative[j-1])
            final_remaining = min(final_remaining, max(
                0., cumulative[j]-progress-entry-cfg.goal_buffer_m))
            permitted = min(permitted, math.sqrt(2*cfg.max_brake_mps2*final_remaining))
            break  # Only the trackable prefix constrains steering and speed.
        ahead = max(0., cumulative[j]-progress)
        if cumulative[j] < progress-.1 or abs(k) < 1e-6:
            continue
        corner_cap = math.sqrt(cfg.max_lateral_accel_mps2/abs(k))
        permitted = min(permitted, math.sqrt(corner_cap**2+
                                             2*cfg.max_brake_mps2*ahead))
    if not stop_before_tight_bend and maximum_curvature > curvature_limit:
        raise ValueError("path_curvature_exceeds_steering")
    ff = math.atan(cfg.wheelbase_m*local_curvature)
    feedback = heading_error+math.atan2(
        cfg.stanley_gain*cross_error,
        max(0., abs(speed_mps))+cfg.softening_speed_mps)
    requested_steer = ff+feedback
    steering = max(-cfg.max_steer_rad, min(cfg.max_steer_rad, requested_steer))
    # Suppress high-speed commands during large but bounded tracking errors.
    permitted *= max(.2, 1.-.7*abs(cross_error)/cfg.cross_track_limit_m)
    permitted *= max(.2, math.cos(heading_error)**2)
    return steering, permitted, {
        "cross_track_error_m": cross_error,
        "heading_error_rad": heading_error,
        "nearest_point": nearest,
        "remaining_path_m": final_remaining,
        "max_route_curvature": maximum_curvature,
        "local_curvature": local_curvature,
        "distance_to_tight_bend_m": distance_to_tight_bend,
        "feedforward_steer_rad": ff,
        "path_index": i,
    }


def speed_ramp(last, requested, accel, dt):
    if not all(math.isfinite(x) for x in (last, requested, accel, dt)):
        raise ValueError("invalid_speed_change")
    return max(0., min(requested, last+max(0., accel*dt)))
