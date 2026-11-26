"""Observationally gated figure-eight geometry based on the historic skidpad.

Old source: planning_skidpad/SkidPadPathPlanningClass.py SendPath,
notably fitCircle, meanCircles and _circularPath. The historical code
initializes circles at (15,+/-9.125) from the vehicle and tracks lap events.
This adapter fits TWO centres from observed blue/yellow boundaries before
generating any figure-eight. It never commands a vehicle.
"""
from dataclasses import dataclass
from itertools import combinations
import math

from asurt_classical_planning.geometry import Config, Result, BLUE, YELLOW


@dataclass(frozen=True)
class SkidpadConfig(Config):
    max_cones: int = 160
    radius_m: float = 9.125
    half_track_width_m: float = 1.5
    radial_tolerance_m: float = 0.7
    centre_separation_tolerance_m: float = 2.0
    min_ring_cones_per_color: int = 4
    min_arc_coverage_deg: float = 70.0
    max_entry_distance_m: float = 25.0
    max_entry_lateral_m: float = 2.8
    samples_per_lap: int = 72


def _circle_through(a, b, c):
    """Solve three-point circle analytically, rejecting nearly collinear arcs."""
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by))
    if abs(d) < 1e-3:
        return None
    sa, sb, sc = ax*ax+ay*ay, bx*bx+by*by, cx*cx+cy*cy
    x = (sa*(by-cy)+sb*(cy-ay)+sc*(ay-by))/d
    y = (sa*(cx-bx)+sb*(ax-cx)+sc*(bx-ax))/d
    return x, y, math.hypot(x-ax, y-ay)


def _arc_coverage(center, points):
    """Arc occupied by observations = 360 degrees minus largest empty gap."""
    angles = sorted(math.atan2(y-center[1], x-center[0]) % (2*math.pi)
                    for x, y in points)
    if len(angles) < 2:
        return 0.0
    gaps = [angles[i+1]-angles[i] for i in range(len(angles)-1)]
    gaps.append(angles[0]+2*math.pi-angles[-1])
    return 2*math.pi - max(gaps)


def _centres(cones, cfg):
    """Bounded deterministic candidate ranking and inner/outer ring support."""
    grouped = {
        BLUE: [(x, y) for x, y, kind, prob in cones
               if kind == BLUE and prob >= cfg.min_probability and
               math.isfinite(x) and math.isfinite(y)],
        YELLOW: [(x, y) for x, y, kind, prob in cones
                 if kind == YELLOW and prob >= cfg.min_probability and
                 math.isfinite(x) and math.isfinite(y)],
    }
    if any(len(v) < cfg.min_ring_cones_per_color for v in grouped.values()):
        return []
    proposals = []
    radii = (cfg.radius_m-cfg.half_track_width_m,
             cfg.radius_m+cfg.half_track_width_m)
    for kind in (BLUE, YELLOW):
        points = grouped[kind]
        # Spaced deterministic samples prevent cubic growth on a full map.
        subset = [points[i*len(points)//min(18, len(points))]
                  for i in range(min(18, len(points)))]
        for triple in combinations(subset, 3):
            circle = _circle_through(*triple)
            if circle is None:
                continue
            cx, cy, r = circle
            if not math.isfinite(r) or min(abs(r-target) for target in radii) > cfg.radial_tolerance_m:
                continue
            supports = []
            for assignment in ((radii[0], radii[1]), (radii[1], radii[0])):
                blue = [(x, y) for x, y in grouped[BLUE] if
                        abs(math.hypot(x-cx, y-cy)-assignment[0]) < cfg.radial_tolerance_m]
                yellow = [(x, y) for x, y in grouped[YELLOW] if
                          abs(math.hypot(x-cx, y-cy)-assignment[1]) < cfg.radial_tolerance_m]
                if min(len(blue), len(yellow)) < cfg.min_ring_cones_per_color:
                    continue
                if (min(_arc_coverage((cx, cy), blue),
                        _arc_coverage((cx, cy), yellow))
                        < math.radians(cfg.min_arc_coverage_deg)):
                    continue
                residual = sum(abs(math.hypot(x-cx, y-cy)-assignment[0])
                               for x, y in blue)
                residual += sum(abs(math.hypot(x-cx, y-cy)-assignment[1])
                                for x, y in yellow)
                supports.append((len(blue)+len(yellow), residual))
            if supports:
                count, residual = sorted(supports, key=lambda v: (-v[0], v[1]))[0]
                proposals.append((count, residual, cx, cy))
    ranked = sorted(proposals, key=lambda t: (-t[0], t[1], t[2], t[3]))
    distinct = []
    for candidate in ranked:
        if all(math.hypot(candidate[2]-prev[2], candidate[3]-prev[3]) > 2.
               for prev in distinct):
            distinct.append(candidate)
            if len(distinct) >= 12:
                break
    return distinct


def skidpad_path(cones, vehicle, cfg=SkidpadConfig()):
    """Figure eight: two clockwise right laps then two counterclockwise left.

    Both concentric inner/outer boundaries must independently support circle
    fits, and the vehicle must approach their common tangent in its own
    direction. No path is published for one detected loop, guessed radius,
    arbitrary odom-based template or partial unobservable cone arc.
    """
    if not all(math.isfinite(v) for v in vehicle):
        return Result(reason='invalid_vehicle_pose')
    if (not (4 <= cfg.min_ring_cones_per_color <= 20)
            or not (5 <= cfg.radius_m <= 16)
            or not (0.5 <= cfg.half_track_width_m <= 3.)
            or not (12 <= cfg.samples_per_lap <= 150)):
        return Result(reason='invalid_config')
    selected = cones[:cfg.max_cones]
    circles = _centres(selected, cfg)
    if len(circles) < 2:
        return Result(reason='insufficient_two_circle_observations')
    px, py, yaw = vehicle
    lateral_axis = (-math.sin(yaw), math.cos(yaw))
    forward_axis = (math.cos(yaw), math.sin(yaw))
    best = None
    for a, b in combinations(circles, 2):
        first, second = (a, b) if (
            (a[2]-px)*lateral_axis[0]+(a[3]-py)*lateral_axis[1]
            < (b[2]-px)*lateral_axis[0]+(b[3]-py)*lateral_axis[1]) else (b, a)
        right, left = (first[2], first[3]), (second[2], second[3])
        separation = math.hypot(left[0]-right[0], left[1]-right[1])
        if abs(separation - 2*cfg.radius_m) > cfg.centre_separation_tolerance_m:
            continue
        if abs((left[0]-right[0])*forward_axis[0] +
               (left[1]-right[1])*forward_axis[1]) > cfg.centre_separation_tolerance_m:
            continue
        tangent = ((right[0]+left[0])*.5, (right[1]+left[1])*.5)
        dx, dy = tangent[0]-px, tangent[1]-py
        forward = dx*forward_axis[0]+dy*forward_axis[1]
        lateral = dx*lateral_axis[0]+dy*lateral_axis[1]
        if not (1.0 < forward <= cfg.max_entry_distance_m and
                abs(lateral) <= cfg.max_entry_lateral_m):
            continue
        score = (a[0]+b[0], -abs(separation-2*cfg.radius_m),
                 -abs(lateral))
        if best is None or score > best[0]:
            best = (score, right, left, tangent)
    if best is None:
        return Result(reason='invalid_skidpad_layout_or_entry')
    _, right, left, tangent = best
    # Two laps each. Tangency ensures the right clockwise and left CCW arcs
    # have a common forward heading. This is a geometric test visualization.
    points = [(px, py), tangent]
    for center, clockwise in ((right, True), (left, False)):
        start_theta = math.atan2(tangent[1]-center[1], tangent[0]-center[0])
        for i in range(1, cfg.samples_per_lap*2+1):
            theta = start_theta + ((-1 if clockwise else 1) *
                                   4*math.pi*i/(2*cfg.samples_per_lap))
            points.append((center[0] + cfg.radius_m*math.cos(theta),
                           center[1] + cfg.radius_m*math.sin(theta)))
    return Result(points=tuple(points), pairs=2, reason='ok')
