"""Shortest forward-only Dubins paths for free-space simulator goal tests.

Six analytic Dubins word families are sampled into map-frame poses with
bounded curvature. No occupancy map, cones or collision avoidance is used.
"""
from dataclasses import dataclass
import math

TAU = 2.0 * math.pi


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def angle(value):
    return (value + math.pi) % TAU - math.pi


def wrap(value):
    return value % TAU


def local_goal_to_map(goal, source):
    """Latch an RViz base_link goal in map at click time, including its yaw."""
    c, s = math.cos(source.yaw), math.sin(source.yaw)
    return Pose2(source.x+c*goal.x-s*goal.y,
                 source.y+s*goal.x+c*goal.y,
                 angle(source.yaw+goal.yaw))


def _candidates(alpha, beta, d):
    ca, cb = math.cos(alpha), math.cos(beta)
    sa, sb = math.sin(alpha), math.sin(beta)
    cab = math.cos(alpha-beta)
    paths = []

    def add(word, p2, t, q):
        if p2 >= -1e-10:
            paths.append((word, (wrap(t), math.sqrt(max(0., p2)), wrap(q))))

    value = 2+d*d-2*cab+2*d*(sa-sb)
    tmp = math.atan2(cb-ca, d+sa-sb)
    add("LSL", value, -alpha+tmp, beta-tmp)

    value = 2+d*d-2*cab+2*d*(sb-sa)
    tmp = math.atan2(ca-cb, d-sa+sb)
    add("RSR", value, alpha-tmp, -beta+tmp)

    value = -2+d*d+2*cab+2*d*(sa+sb)
    if value >= -1e-10:
        p = math.sqrt(max(0., value))
        tmp = math.atan2(-ca-cb, d+sa+sb) - math.atan2(-2., p)
        paths.append(("LSR", (wrap(-alpha+tmp), p, wrap(-beta+tmp))))

    value = -2+d*d+2*cab-2*d*(sa+sb)
    if value >= -1e-10:
        p = math.sqrt(max(0., value))
        tmp = math.atan2(ca+cb, d-sa-sb) - math.atan2(2., p)
        paths.append(("RSL", (wrap(alpha-tmp), p, wrap(beta-tmp))))

    tmp = (6-d*d+2*cab+2*d*(sa-sb))/8.
    if abs(tmp) <= 1.:
        p = wrap(TAU-math.acos(tmp))
        t = wrap(alpha-math.atan2(ca-cb, d-sa+sb)+p/2.)
        paths.append(("RLR", (t, p, wrap(alpha-beta-t+p))))

    tmp = (6-d*d+2*cab+2*d*(-sa+sb))/8.
    if abs(tmp) <= 1.:
        p = wrap(TAU-math.acos(tmp))
        t = wrap(-alpha-math.atan2(ca-cb, d+sa-sb)+p/2.)
        paths.append(("LRL", (t, p, wrap(beta-alpha-t+p))))
    return paths


def _advance(pose, action, travel, radius):
    x, y, yaw = pose.x, pose.y, pose.yaw
    if action == "S":
        return Pose2(x+travel*math.cos(yaw), y+travel*math.sin(yaw), yaw)
    sign = 1. if action == "L" else -1.
    after = yaw + sign*travel/radius
    return Pose2(x+radius/sign*(math.sin(after)-math.sin(yaw)),
                 y-radius/sign*(math.cos(after)-math.cos(yaw)), after)


def dubins_path(start, finish, radius, resolution=.3, max_length=100.,
                max_points=600):
    """(poses, path_length_m, word) from exact start to goal position + yaw.

    Returns an empty path with a reason for out-of-range routes. All segments
    drive forward and have curvature 0 or +/-1/radius.
    """
    if not all(math.isfinite(v) for v in (start.x, start.y, start.yaw,
                                          finish.x, finish.y, finish.yaw,
                                          radius, resolution, max_length)):
        raise ValueError("nonfinite_dubins_input")
    if radius <= 0 or resolution <= 0 or max_length <= 0 or max_points < 2:
        raise ValueError("invalid_dubins_limits")
    dx, dy = finish.x-start.x, finish.y-start.y
    heading = math.atan2(dy, dx)
    d = math.hypot(dx, dy)/radius
    a, b = wrap(start.yaw-heading), wrap(finish.yaw-heading)
    candidates = sorted(_candidates(a, b, d), key=lambda c: sum(c[1]))
    for word, portions in candidates:
        length = radius*sum(portions)
        if length > max_length:
            continue
        # Bound points before allocating a long list or publishing it to ROS.
        total_points = 1+sum(max(1, math.ceil(radius*v/resolution))
                             for v in portions if v > 1e-10)
        if total_points > max_points:
            continue
        current = start
        points = [start]
        for action, portion in zip(word, portions):
            travel = radius*portion
            if travel < 1e-10:
                continue
            count = max(1, math.ceil(travel/resolution))
            step = travel/count
            for _ in range(count):
                current = _advance(current, action, step, radius)
                points.append(current)
        if (math.hypot(current.x-finish.x, current.y-finish.y) < 1e-4
                and abs(angle(current.yaw-finish.yaw)) < 1e-4):
            points[-1] = Pose2(finish.x, finish.y, finish.yaw)
            return tuple(points), length, word
    return (), float("inf"), "no_bounded_dubins_route"
