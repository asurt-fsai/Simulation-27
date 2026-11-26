"""Bounded, inspectable cone-pair centerline geometry.

The original FS-System-26 Centerline planner matched blue/yellow boundaries,
formed midpoints and ordered the resulting trajectory. This deliberately small
port tests that geometric idea with ROS-independent inputs; it refuses to
invent a path from a single boundary or unconstrained odometry.

Coordinates use REP-103: map x/y and vehicle yaw counter-clockwise.
Landmark.type follows asurt_msgs/Landmark: BLUE=0, YELLOW=1.
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


BLUE = 0
YELLOW = 1
# Bound the deviation from the paired-gate centerline locally at each join.
MAX_CORNER_OFFSET_M = 0.15
Cone = Tuple[float, float, int, float]  # x, y, type, confidence
Point2 = Tuple[float, float]
Pose2 = Tuple[float, float, float]


@dataclass(frozen=True)
class Config:
    max_cones: int = 128
    lookahead_m: float = 22.0
    max_lateral_m: float = 11.0
    min_probability: float = 0.25
    min_track_width_m: float = 2.0
    max_track_width_m: float = 7.0
    max_pair_along_m: float = 2.5
    max_pair_gap_m: float = 10.0
    max_first_pair_m: float = 8.0
    max_first_lateral_m: float = 3.0
    max_bend_rad: float = 1.15
    sample_spacing_m: float = 0.5
    blue_left: bool = True


@dataclass(frozen=True)
class Result:
    points: Tuple[Point2, ...] = ()
    pairs: int = 0
    reason: str = "insufficient_cones"


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def in_map(cones: Sequence[Cone], vehicle: Pose2) -> Tuple[Cone, ...]:
    """Transform base_link cone points to map using the bridge odometry pose."""
    px, py, yaw = vehicle
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((px + c*x - s*y, py + s*x + c*y, kind, p)
                 for x, y, kind, p in cones)


def _sample_centerline(gates: Sequence[Point2], spacing: float) -> Sequence[Point2]:
    """Round each polyline join with a circular arc tangent to both legs."""
    samples = [gates[0]]

    def extend_line(end):
        start = samples[-1]
        length = math.dist(start, end)
        if length < 1e-7:
            return
        steps = math.ceil(length / spacing)
        for i in range(1, steps + 1):
            t = i / steps
            samples.append((start[0]+t*(end[0]-start[0]),
                            start[1]+t*(end[1]-start[1])))

    for previous, corner, following in zip(gates, gates[1:], gates[2:]):
        dx, dy = corner[0]-previous[0], corner[1]-previous[1]
        ex, ey = following[0]-corner[0], following[1]-corner[1]
        incoming = math.atan2(dy, dx)
        outgoing = math.atan2(ey, ex)
        turn = outgoing - incoming  # Both accepted headings are near forward.
        angle = abs(turn)
        if angle < 1e-6:
            extend_line(corner)
            continue
        # Use at most half of each adjacent leg so two arcs cannot overlap.
        # An arc starting trim metres from the corner misses its tip by
        # trim * tan(angle / 4); cap that offset to stay near the gates.
        trim = min(math.hypot(dx, dy)/2, math.hypot(ex, ey)/2,
                   MAX_CORNER_OFFSET_M / math.tan(angle/4))
        radius = trim / math.tan(angle/2)
        entry = (corner[0]-trim*math.cos(incoming),
                 corner[1]-trim*math.sin(incoming))
        exit_point = (corner[0]+trim*math.cos(outgoing),
                      corner[1]+trim*math.sin(outgoing))
        extend_line(entry)
        # Ensure an interior sample even on a very short arc, so downstream
        # curvature checks can still see a tight turn.
        steps = max(2, math.ceil(radius * angle / spacing))
        direction = math.copysign(1., turn)
        for i in range(1, steps):
            tangent = incoming + turn*i/steps
            samples.append((entry[0] + direction*radius*(math.sin(tangent)-math.sin(incoming)),
                            entry[1] - direction*radius*(math.cos(tangent)-math.cos(incoming))))
        samples.append(exit_point)

    extend_line(gates[-1])
    return samples


def centerline(cones: Sequence[Cone], vehicle: Pose2, cfg: Config = Config()) -> Result:
    """Find paired left/right cones ahead of the car and return map-frame path.

    Greedy one-to-one pairing is deterministic and bounded by max_cones. We
    reject crossing/remote/one-sided boundaries, sharp bends and paths that
    begin far from the car. This is a testable planning preview, not a
    certified collision-free trajectory or speed profile.
    """
    px, py, yaw = vehicle
    if not _finite(px, py, yaw):
        return Result(reason="invalid_vehicle_pose")
    if cfg.max_cones < 4 or not (cfg.min_track_width_m < cfg.max_track_width_m):
        return Result(reason="invalid_config")
    if cfg.lookahead_m <= 0 or cfg.sample_spacing_m <= 0:
        return Result(reason="invalid_config")
    c, s = math.cos(yaw), math.sin(yaw)
    boundaries = {BLUE: [], YELLOW: []}
    # A limit on checked records also bounds pathological map messages.
    for x, y, kind, confidence in cones[:cfg.max_cones]:
        if kind not in boundaries or not _finite(x, y, confidence):
            continue
        if confidence < cfg.min_probability:
            continue
        dx, dy = x - px, y - py
        forward, lateral = c*dx + s*dy, -s*dx + c*dy
        if not (-1.0 <= forward <= cfg.lookahead_m):
            continue
        if abs(lateral) > cfg.max_lateral_m:
            continue
        boundaries[kind].append((forward, lateral))
    left_kind = BLUE if cfg.blue_left else YELLOW
    right_kind = YELLOW if cfg.blue_left else BLUE
    left = sorted(boundaries[left_kind])
    right = sorted(boundaries[right_kind])
    if len(left) < 2 or len(right) < 2:
        return Result(reason="need_two_cones_each_side")

    # Pair cones with comparable forward distance and consistent right->left
    # order. Farther gates may lie entirely on one side of the *current* car
    # heading when the course bends, so do not require each pair to straddle
    # its axis; only the first gate must begin near the vehicle's forward ray.
    candidates = []
    for li, (lx, ly) in enumerate(left):
        for ri, (rx, ry) in enumerate(right):
            lateral_width = ly-ry
            width = math.hypot(lx-rx, lateral_width)
            longitudinal = abs(lx-rx)
            # Width must be lateral to the vehicle heading. Otherwise two
            # cones on an impossibly narrow strip could be paired diagonally
            # across consecutive rows and masquerade as a valid track.
            if (cfg.min_track_width_m <= lateral_width <= cfg.max_track_width_m
                    and width <= cfg.max_track_width_m
                    and longitudinal <= min(cfg.max_pair_along_m, .55*lateral_width)):
                candidates.append((longitudinal + .02*width, li, ri))
    candidates.sort()
    used_left, used_right, matches = set(), set(), []
    for _, li, ri in candidates:
        if li in used_left or ri in used_right:
            continue
        lx, ly = left[li]
        rx, ry = right[ri]
        midpoint = ((lx+rx)*.5, (ly+ry)*.5)
        matches.append(midpoint)
        used_left.add(li)
        used_right.add(ri)
    matches.sort()
    if len(matches) < 2:
        return Result(pairs=len(matches), reason="no_consistent_pairs")
    if math.hypot(*matches[0]) > cfg.max_first_pair_m:
        return Result(pairs=len(matches), reason="first_pair_too_far")
    if abs(matches[0][1]) > cfg.max_first_lateral_m:
        return Result(pairs=len(matches), reason="first_pair_outside_corridor")

    # Only keep a forward, connected segment with smooth tangent direction.
    ordered = [matches[0]]
    for candidate in matches[1:]:
        last = ordered[-1]
        dx, dy = candidate[0]-last[0], candidate[1]-last[1]
        distance = math.hypot(dx, dy)
        if distance < 0.25:
            continue
        if distance > cfg.max_pair_gap_m:
            break
        if dx <= 0.10:
            break
        tangent = math.atan2(dy, dx)
        if abs(tangent) > cfg.max_bend_rad:
            break
        if len(ordered) >= 2:
            prev = ordered[-2]
            previous_tangent = math.atan2(last[1]-prev[1], last[0]-prev[0])
            turn = math.atan2(math.sin(tangent-previous_tangent),
                              math.cos(tangent-previous_tangent))
            if abs(turn) > cfg.max_bend_rad:
                break
        ordered.append(candidate)
    if len(ordered) < 2 or ordered[-1][0] - ordered[0][0] < 1.0:
        return Result(pairs=len(ordered), reason="insufficient_forward_path")

    samples = _sample_centerline(ordered, cfg.sample_spacing_m)
    points = tuple((px+c*x-s*y, py+s*x+c*y) for x, y in samples)
    return Result(points=points, pairs=len(ordered), reason="ok")
