"""Deterministic legacy LSTM input contract and independent track-safety gates.

Inputs and outputs are REP-103 base_link (x forward, y left). The original
model expects (x right, y forward, blue flag, yellow flag), while its output
has two columns (x right, y forward). This module does not infer cone truth.
"""
from dataclasses import dataclass
import math

BLUE, YELLOW = 0, 1


@dataclass(frozen=True)
class Limits:
    min_probability: float = 0.35
    min_radius_m: float = 0.5
    max_radius_m: float = 15.0
    max_view_angle_rad: float = math.pi / 3
    min_track_width_m: float = 2.0
    max_track_width_m: float = 7.0
    max_pair_along_m: float = 2.0
    max_first_gate_m: float = 8.0
    max_step_m: float = 3.0
    max_lateral_m: float = 7.0
    max_first_point_m: float = 5.0
    min_advance_m: float = 1.5
    corridor_margin_m: float = 0.35
    blue_left: bool = True


def _finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def prepare_cones(observations, cfg=Limits()):
    """Return exactly ten 4-float rows and independent, unpadded observations.

    Each observation is (forward, left, class, probability). Orange classes
    (2/3) and unknown classes are ignored, never remapped to blue or yellow.
    """
    if cfg.max_radius_m <= cfg.min_radius_m or cfg.max_view_angle_rad <= 0:
        return None, (), 'invalid_config'
    valid = []
    for x, y, kind, probability in observations[:2048]:
        if kind not in (BLUE, YELLOW) or not _finite(x, y, probability):
            continue
        if probability < cfg.min_probability:
            continue
        d = math.hypot(x, y)
        if not (cfg.min_radius_m <= d <= cfg.max_radius_m and x > 0.0):
            continue
        if abs(math.atan2(-y, x)) > cfg.max_view_angle_rad:
            continue
        valid.append((d, float(x), float(y), int(kind), float(probability)))
    # Stable sort includes type and position to avoid order-dependent tensors.
    valid.sort(key=lambda c: (c[0], c[3], c[1], c[2]))
    unique = []
    for candidate in valid:
        if any(candidate[3] == other[3] and
               math.hypot(candidate[1]-other[1], candidate[2]-other[2]) < .15
               for other in unique):
            continue
        unique.append(candidate)
        if len(unique) == 10:
            break
    if sum(c[3] == BLUE for c in unique) < 2 or sum(c[3] == YELLOW for c in unique) < 2:
        return None, tuple((c[1], c[2], c[3], c[4]) for c in unique), 'need_two_cones_each_side'
    rows = [(-c[2], c[1], float(c[3] == BLUE), float(c[3] == YELLOW))
            for c in unique]
    rows.extend([(0., 0., 0., 0.)] * (10 - len(rows)))
    return tuple(rows), tuple((c[1], c[2], c[3], c[4]) for c in unique), 'ok'


def paired_gates(cones, cfg=Limits()):
    """Make bounded disjoint front-facing gates for track-corridor checks."""
    left_kind = BLUE if cfg.blue_left else YELLOW
    left = sorted((x, y) for x, y, kind, _ in cones if kind == left_kind)
    right = sorted((x, y) for x, y, kind, _ in cones if kind != left_kind)
    matches = []
    used = set()
    for lx, ly in left:
        candidates = []
        for i, (rx, ry) in enumerate(right):
            width = ly - ry
            along = abs(lx - rx)
            if (i not in used and cfg.min_track_width_m <= width <= cfg.max_track_width_m
                    and along <= cfg.max_pair_along_m
                    and math.hypot(along, width) <= cfg.max_track_width_m):
                candidates.append((along, i, rx, ry))
        if candidates:
            _, index, rx, ry = min(candidates)
            used.add(index)
            matches.append(((lx+rx)*.5, ly, ry))
    matches.sort()
    gates = []
    for gate in matches:
        if gates and gate[0] - gates[-1][0] < .45:
            continue
        gates.append(gate)
    if len(gates) < 2:
        return (), 'no_consistent_gates'
    if gates[0][0] > cfg.max_first_gate_m:
        return (), 'first_gate_too_far'
    if gates[-1][0] - gates[0][0] < cfg.min_advance_m:
        return (), 'insufficient_gate_coverage'
    return tuple(gates), 'ok'


def raw_prediction_points(raw):
    """Convert all 15 finite model waypoints to ROS XY for *diagnostics only*.

    This deliberately does NOT call geometric/track safety acceptance. The
    caller must never publish this to the gated path or to vehicle Control.
    Return an empty tuple when the model contract is violated; never clip,
    invent or partially accept the raw model prediction.
    """
    try:
        if len(raw) != 15 or any(len(p) != 2 for p in raw):
            return ()
        points = tuple((float(p[1]), -float(p[0])) for p in raw)
    except (TypeError, ValueError, IndexError):
        return ()
    if any(not _finite(x, y) for x, y in points):
        return ()
    return points


def preview_age_allowed(now_ns, input_stamp_ns, age_limit_ns, generation_matches):
    """Age gate for *visualization only*, never for safety/control acceptance."""
    return (generation_matches and input_stamp_ns > 0 and age_limit_ns > 0
            and -200_000_000 <= now_ns - input_stamp_ns <= age_limit_ns)


def point_at_x(start, end, x):
    fraction = (x - start[0]) / (end[0] - start[0])
    return x, start[1] + fraction * (end[1] - start[1])


def clip_to_corridor(points, lower, upper, max_step_m):
    """Keep a continuous original polyline segment, without extrapolation."""
    clipped = []
    for start, end in zip(points, points[1:]):
        if start[0] >= upper:
            break
        if end[0] - start[0] <= .02 or math.dist(start, end) > max_step_m:
            return (), 'invalid_waypoint_progress'
        left, right = max(lower, start[0]), min(upper, end[0])
        if left >= right:
            continue
        if not clipped:
            clipped.append(point_at_x(start, end, left))
        clipped.append(point_at_x(start, end, right))
    return tuple(clipped), 'ok'


def corridor_bounds(gates, x):
    if x <= gates[0][0]:
        return gates[0][1:]
    for first, second in zip(gates, gates[1:]):
        if x <= second[0]:
            fraction = (x - first[0]) / (second[0] - first[0])
            return tuple(a + fraction * (b - a) for a, b in zip(first[1:], second[1:]))
    return gates[-1][1:]


def check_prediction(raw, cones, cfg=Limits()):
    """Validate the overlap of a prediction and the observed cone corridor.

    Returns (REP-103 points, reason, gate_count). Interpolate only along
    original waypoint segments and keep the existing bounded lead-in.
    """
    gates, reason = paired_gates(cones, cfg)
    if reason != 'ok':
        return (), reason, 0
    try:
        if len(raw) != 15 or any(len(p) != 2 for p in raw):
            return (), 'wrong_output_shape', len(gates)
        points = tuple((float(p[1]), -float(p[0])) for p in raw)
    except (TypeError, ValueError, IndexError):
        return (), 'invalid_output', len(gates)
    if not all(_finite(x, y) for x, y in points):
        return (), 'nonfinite_output', len(gates)
    if (points[0][0] < 0 or math.hypot(*points[0]) > cfg.max_first_point_m
            or points[-1][0] - points[0][0] < cfg.min_advance_m):
        return (), 'invalid_path_origin_or_progress', len(gates)
    lower, upper = max(0., gates[0][0] - 2.5), gates[-1][0]
    points, reason = clip_to_corridor(points, lower, upper, cfg.max_step_m)
    if reason != 'ok':
        return (), reason, len(gates)
    if not points or points[-1][0] - points[0][0] < cfg.min_advance_m:
        return (), 'path_outside_observed_extent', len(gates)
    if math.hypot(*points[0]) > cfg.max_first_point_m:
        return (), 'path_outside_observed_extent', len(gates)
    if any(x > cfg.max_radius_m or abs(y) > cfg.max_lateral_m for x, y in points):
        return (), 'path_outside_range', len(gates)
    # Corridor edges are piecewise linear. At a gate between waypoints the
    # corridor can narrow or turn even when both waypoints are inside it.
    checks = (*points, *(point_at_x(start, end, gate[0])
                        for start, end in zip(points, points[1:]) for gate in gates
                        if start[0] < gate[0] < end[0]))
    for x, y in checks:
        high, low = corridor_bounds(gates, x)
        if not (low + cfg.corridor_margin_m <= y <= high - cfg.corridor_margin_m):
            return (), 'path_outside_cone_boundaries', len(gates)
    return points, 'ok', len(gates)
