"""Bounded blue/yellow corridor search with lateral cone-avoidance choices.

Inputs are independent, map-frame SLAM landmarks, never CARLA actors/truth.
Blue denotes the left track boundary and yellow the right boundary. Each
opposite-color pair makes a directed gate. Three offset lanes per gate enable
limited avoidance of extra cones (e.g. an orange cone on the centerline).
The result is a *cone-aware* center corridor, not a certified free-space map:
no cone map means no path and unknown obstacles cannot be inferred.
"""
from dataclasses import dataclass
from collections import defaultdict
import heapq
import math
from typing import Tuple


Point = Tuple[float, float]
Cone = Tuple[float, float, int, float]


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Config:
    max_cones: int = 1024
    max_gates: int = 500
    min_probability: float = 0.22
    # /map/global_cones includes only repeatedly observed, CONFIRMED features.
    # Its `probability` is instead inverse POSITION covariance and decays for
    # old static cones: filtering on it erases the already mapped circuit.
    # Use this only with the confirmed-map producer and fresh mapper status.
    confirmed_map: bool = False
    min_width_m: float = 2.2
    max_width_m: float = 7.0
    max_gate_gap_m: float = 9.0
    max_start_m: float = 8.0
    max_goal_m: float = 4.0
    max_goal_distance_m: float = 1200.0
    min_clearance_m: float = 0.72
    lateral_fraction: float = 0.43
    max_heading_change_rad: float = 0.9
    # tan(16 deg) / 1.575 m wheelbase ~= 0.182 1/m.
    max_path_curvature_m_inv: float = 0.18
    resolution_m: float = 0.4
    max_path_points: int = 6000
    blue_left: bool = True


@dataclass(frozen=True)
class Gate:
    center: Point
    left: Point
    right: Point
    heading: float
    width: float


@dataclass(frozen=True)
class Result:
    points: Tuple[Point, ...] = ()
    reason: str = "no_map"
    gates_used: int = 0
    cone_count: int = 0
    minimum_clearance_m: float = 0.
    route_length_m: float = 0.
    lateral_changes: int = 0


def wrap(yaw):
    return (yaw+math.pi)%(2*math.pi)-math.pi


def distance(a: Point, b: Point):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def segment_distance(point: Point, a: Point, b: Point):
    dx, dy = b[0]-a[0], b[1]-a[1]
    sq = dx*dx+dy*dy
    if sq < 1e-10:
        return distance(point, a)
    t = max(0., min(1., ((point[0]-a[0])*dx+
                          (point[1]-a[1])*dy)/sq))
    return math.hypot(point[0]-a[0]-dx*t, point[1]-a[1]-dy*t)


class _ConeIndex:
    """Conservative uniform grid: every possible nearby cone is returned."""
    def __init__(self, cones, cell=4.):
        self.cones = cones
        self.cell = cell
        self.cells = defaultdict(list)
        for cone in cones:
            x, y = cone[:2]
            self.cells[(math.floor(x/cell), math.floor(y/cell))].append(cone)

    def near_segment(self, a, b, pad):
        cell = self.cell
        for ix in range(math.floor((min(a[0], b[0])-pad)/cell),
                        math.floor((max(a[0], b[0])+pad)/cell)+1):
            for iy in range(math.floor((min(a[1], b[1])-pad)/cell),
                            math.floor((max(a[1], b[1])+pad)/cell)+1):
                yield from self.cells.get((ix, iy), ())

    def near_point(self, x, y, radius):
        cell = self.cell
        for ix in range(math.floor((x-radius)/cell),
                        math.floor((x+radius)/cell)+1):
            for iy in range(math.floor((y-radius)/cell),
                            math.floor((y+radius)/cell)+1):
                yield from self.cells.get((ix, iy), ())


def route_clearance(points, cones):
    if not cones or len(points) < 2:
        return 0.
    # Exact nearest segment/cone clearance, using a spatial grid to keep a
    # complete circuit from taking O(path_points * all_map_cones) per tick.
    index = cones if isinstance(cones, _ConeIndex) else _ConeIndex(cones)
    best = min(distance(points[0], c[:2]) for c in index.cones)
    for a, b in zip(points, points[1:]):
        if best < 1e-10:
            return 0.
        if best > 20.:  # Avoid a huge grid scan on an implausible first point.
            nearby = index.cones
        else:
            nearby = index.near_segment(a, b, best)
        for p in nearby:
            best = min(best, segment_distance(p[:2], a, b))
    return best


def _edge_clear(a, b, cones, clearance):
    pad = clearance
    candidates = (cones.near_segment(a, b, pad)
                  if isinstance(cones, _ConeIndex) else cones)
    for x, y, _, _ in candidates:
        if min(a[0], b[0])-pad > x or max(a[0], b[0])+pad < x:
            continue
        if min(a[1], b[1])-pad > y or max(a[1], b[1])+pad < y:
            continue
        if segment_distance((x, y), a, b) < clearance:
            return False
    return True


def _inside_quad(point, gate_a, gate_b):
    # The corridor between consecutive blue/yellow pairs is a quadrilateral.
    polygon = (gate_a.right, gate_b.right, gate_b.left, gate_a.left)
    signs = []
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        cross = ((b[0]-a[0])*(point[1]-a[1]) -
                 (b[1]-a[1])*(point[0]-a[0]))
        if abs(cross) > 1e-7:
            signs.append(1 if cross > 0 else -1)
    return not signs or all(s == signs[0] for s in signs)


def build_gates(cones, cfg, start_xy, goal_xy):
    """Pair boundaries across the track, favouring their local normals."""
    blue = sorted(((x, y, p) for x, y, kind, p in cones if kind == 0),
                  key=lambda t: (t[0], t[1]))
    yellow = sorted(((x, y, p) for x, y, kind, p in cones if kind == 1),
                    key=lambda t: (t[0], t[1]))

    def tangents(points):
        result = []
        nearby_index = _ConeIndex(points, cell=cfg.max_gate_gap_m)
        radius = 2*cfg.max_gate_gap_m
        for i, (x, y, _) in enumerate(points):
            nearby = heapq.nsmallest(4, ((distance((x, y), q[:2]), q)
                                   for q in nearby_index.near_point(x, y, radius)
                                   if q[:2] != (x, y)))
            sxx = sxy = syy = 0.
            for d, (nx, ny, _) in nearby:
                if d > radius or d < .3:
                    continue
                dx, dy = nx-x, ny-y
                w = 1./max(1., d*d)
                sxx += w*dx*dx
                sxy += w*dx*dy
                syy += w*dy*dy
            if sxx+syy < .01:
                result.append(None)
            else:
                a = .5*math.atan2(2*sxy, sxx-syy)
                result.append((math.cos(a), math.sin(a)))
        return result

    bt, yt = tangents(blue), tangents(yellow)
    candidates = []
    for i, b in enumerate(blue):
        for j, y in enumerate(yellow):
            width = distance(b[:2], y[:2])
            if cfg.min_width_m <= width <= cfg.max_width_m:
                across = ((b[0]-y[0])/width, (b[1]-y[1])/width)
                penalty = sum(abs(across[0]*t[0]+across[1]*t[1])
                              for t in (bt[i], yt[j]) if t is not None)
                # A short diagonal can be shorter than the genuine width on
                # tightly spaced cones. Cross-boundary normals resolve it.
                candidates.append((width+cfg.max_width_m*.35*penalty,
                                   width, i, j))
    candidates.sort()
    used_b, used_y, gates = set(), set(), []
    for _, width, i, j in candidates:
        if i in used_b or j in used_y:
            continue
        used_b.add(i)
        used_y.add(j)
        b, y = blue[i], yellow[j]
        left, right = (b[:2], y[:2]) if cfg.blue_left else (y[:2], b[:2])
        transverse = (left[0]-right[0], left[1]-right[1])
        tangent = (transverse[1], -transverse[0])
        gates.append(Gate(((b[0]+y[0])*.5, (b[1]+y[1])*.5),
                          left, right, math.atan2(tangent[1], tangent[0]),
                          width))
    # A behind-the-car goal on a mapped loop requires gates all the way around.
    # Do not trim the far half of the lap by distance to the click chord.
    gates.sort(key=lambda gate: (distance(gate.center, start_xy), gate.center))
    return tuple(gates)


def _lane(gate, lane, cfg):
    if lane == 0:
        return gate.center
    sign = 1. if lane == 1 else -1.
    offset = .5*cfg.lateral_fraction*sign
    return (gate.center[0]+offset*(gate.left[0]-gate.right[0]),
            gate.center[1]+offset*(gate.left[1]-gate.right[1]))


def _interpolate(points, headings, resolution, max_points):
    """C1 cubic Hermite interpolation through gates with tangent headings."""
    if len(points) < 2 or len(points) != len(headings):
        return (), ()
    lengths = [distance(a, b) for a, b in zip(points, points[1:])]
    if any(d < 1e-6 for d in lengths):
        return (), ()
    speeds = [lengths[0]]
    speeds += [min(a, b) for a, b in zip(lengths, lengths[1:])]
    speeds.append(lengths[-1])
    tangents = [(v*math.cos(h), v*math.sin(h))
                for v, h in zip(speeds, headings)]
    result = [points[0]]
    stops = [0]
    for i, (a, b) in enumerate(zip(points, points[1:])):
        steps = max(1, math.ceil((lengths[i]+speeds[i]+speeds[i+1]) /
                                  (2*resolution)))
        if len(result)+steps > max_points:
            return (), ()
        for k in range(1, steps+1):
            t = k/steps
            h00 = 2*t**3-3*t*t+1
            h10 = t**3-2*t*t+t
            h01 = -2*t**3+3*t*t
            h11 = t**3-t*t
            result.append((h00*a[0]+h10*tangents[i][0]+h01*b[0]
                           +h11*tangents[i+1][0],
                           h00*a[1]+h10*tangents[i][1]+h01*b[1]
                           +h11*tangents[i+1][1]))
        stops.append(len(result)-1)
    return tuple(result), tuple(stops)


def _fit_bounded_spline(raw, headings, knot_gates, gates, cone_index, cfg):
    """Try a globally smooth, constrained route if gate Hermite curves fail.

    The fitted line is never accepted by appearance alone: endpoints, heading,
    every connected gate corridor, obstacle clearance and vehicle curvature
    must all remain valid. Limit fitting to local bends to bound CPU work.
    """
    if len(raw) < 5 or len(raw) > 128:
        return ()
    try:
        import numpy as np
        from scipy.interpolate import splprep, splev
    except ImportError:
        return ()
    for skipped in (1, 0):
        # A gate less than 2 m from the current axle can force a tiny spline
        # segment and a fictitious high-curvature corner at the vehicle.
        if skipped and distance(raw[0], raw[1]) >= 2.:
            continue
        base = list(raw[:1]) + list(raw[1+skipped:])
        identifiers = list(knot_gates[:1]) + list(knot_gates[1+skipped:])
        if len(base) < 4 or distance(base[0], base[1]) < 1.8:
            continue
        points = list(base)
        points.insert(1, (raw[0][0]+1.5*math.cos(headings[0]),
                          raw[0][1]+1.5*math.sin(headings[0])))
        points.insert(-1, (raw[-1][0]-1.5*math.cos(headings[-1]),
                           raw[-1][1]-1.5*math.sin(headings[-1])))
        identifiers.insert(1, None)
        identifiers.insert(-1, None)
        if any(distance(a, b) < .15 for a, b in zip(points, points[1:])):
            continue
        weights = np.ones(len(points), dtype=float)
        weights[0] = weights[-1] = 30.
        weights[1] = weights[-2] = 8.
        length = sum(distance(a, b) for a, b in zip(points, points[1:]))
        steps = max(3, int(math.ceil(length/min(.20, cfg.resolution_m)))+1)
        if steps > cfg.max_path_points:
            continue
        for smoothing in (.025, .045, .065, .09):
            try:
                fit, parameters = splprep(
                    [[p[0] for p in points], [p[1] for p in points]],
                    w=weights, s=smoothing*len(points), k=3)
                xs, ys = splev(np.linspace(0, 1, steps), fit)
                curve = list(zip(xs.tolist(), ys.tolist()))
                derivative_0 = splev(0., fit, der=1)
                derivative_1 = splev(1., fit, der=1)
            except (ValueError, TypeError):
                continue
            if (distance(curve[0], raw[0]) > .02 or
                    distance(curve[-1], raw[-1]) > .02 or
                    any(distance(a, b) > 1.25*cfg.resolution_m
                        for a, b in zip(curve, curve[1:]))):
                continue
            if (abs(wrap(math.atan2(derivative_0[1], derivative_0[0])
                         -headings[0])) > .08 or
                    abs(wrap(math.atan2(derivative_1[1], derivative_1[0])
                             -headings[-1])) > .08):
                continue
            # A fitted curve crosses a gate a little before or after its
            # interpolation parameter: validate against the UNION of route
            # quads, not one arbitrarily parameterized quad at a time.
            gate_ids = [i for i in identifiers if i is not None]
            route_quads = [(gates[a], gates[b])
                           for a, b in zip(gate_ids, gate_ids[1:])]
            if not route_quads:
                continue
            entry_radius = distance(raw[0], gates[gate_ids[0]].center)+.25
            exit_radius = distance(raw[-1], gates[gate_ids[-1]].center)+.25
            if any(distance(p, raw[0]) > entry_radius and
                   distance(p, raw[-1]) > exit_radius and
                   not any(_inside_quad(p, a, b) for a, b in route_quads)
                   for p in curve):
                continue
            curve[0], curve[-1] = raw[0], raw[-1]
            if (route_clearance(curve, cone_index) < cfg.min_clearance_m
                    or _max_curvature(curve) > cfg.max_path_curvature_m_inv):
                continue
            return tuple(curve)
    return ()


def _max_curvature(points):
    result = 0.
    for a, b, c in zip(points, points[1:], points[2:]):
        ab, bc, ac = distance(a, b), distance(b, c), distance(a, c)
        if min(ab, bc, ac) < 1e-6:
            continue
        cross = abs((b[0]-a[0])*(c[1]-a[1])-
                    (b[1]-a[1])*(c[0]-a[0]))
        result = max(result, 2.*cross/(ab*bc*ac))
    return result


def plan_track(start: Pose2, goal: Pose2, landmarks, cfg=Config()):
    """Search within mapped gates and around cone centers toward clicked goal."""
    fields = (start.x, start.y, start.yaw, goal.x, goal.y, goal.yaw)
    if not all(math.isfinite(v) for v in fields):
        return Result(reason="invalid_pose")
    if not (.0 < cfg.min_clearance_m < cfg.min_width_m*.5 and
            0. < cfg.lateral_fraction < .9 and
            cfg.max_cones >= 6 and cfg.max_gates >= 3 and
            cfg.max_path_points >= 10 and cfg.resolution_m > 0 and
            cfg.max_path_curvature_m_inv > 0):
        return Result(reason="invalid_track_config")
    if distance((start.x, start.y), (goal.x, goal.y)) > cfg.max_goal_distance_m:
        return Result(reason="goal_beyond_local_map_range")
    start_xy, goal_xy = (start.x, start.y), (goal.x, goal.y)
    # A goal behind the vehicle may require a full lap of the mapped circuit.
    # Limiting to the direct start/goal chord deletes the distant half of it.
    valid = tuple((float(x), float(y), int(kind), float(p))
             for x, y, kind, p in landmarks[:2048]
             if (math.isfinite(x) and math.isfinite(y) and
                 math.isfinite(p) and .0 <= p <= 1.0 and
                 (cfg.confirmed_map or p >= cfg.min_probability)
                 and 0 <= int(kind) <= 3))
    if len(valid) > cfg.max_cones or len(landmarks) > 2048:
        return Result(reason="mapped_track_exceeds_cone_budget",
                      cone_count=len(valid))
    cones = tuple(sorted(valid, key=lambda c: (c[0], c[1], c[2])))
    if len(cones) < 6:
        return Result(reason="insufficient_mapped_cones", cone_count=len(cones))
    gates = build_gates(cones, cfg, start_xy, goal_xy)
    if len(gates) < 3:
        return Result(reason="insufficient_track_gates", cone_count=len(cones))
    if len(gates) > cfg.max_gates:
        return Result(reason="mapped_track_exceeds_gate_budget",
                      cone_count=len(cones))
    cone_index = _ConeIndex(cones)
    goal_forward = ((goal.x-start.x)*math.cos(start.yaw)
                    +(goal.y-start.y)*math.sin(start.yaw))
    # RViz goal just behind the car: it should be approached along the forward
    # mapped track, possibly after one complete circuit, never by reversing.
    needs_lap = (goal_forward < -.5 and
                 distance(start_xy, goal_xy) <= 2.*cfg.max_start_m)
    initial = []
    end = []
    for i, gate in enumerate(gates):
        from_start = distance(start_xy, gate.center)
        to_goal = distance(goal_xy, gate.center)
        if abs(wrap(gate.heading-start.yaw)) <= 0.9 and from_start <= cfg.max_start_m:
            for lane in range(3):
                p = _lane(gate, lane, cfg)
                dx, dy = p[0]-start.x, p[1]-start.y
                forward = math.cos(start.yaw)*dx + math.sin(start.yaw)*dy
                if forward >= -.2 and _edge_clear(start_xy, p, cone_index, cfg.min_clearance_m):
                    initial.append((i, lane))
        if to_goal <= cfg.max_goal_m and abs(wrap(gate.heading-goal.yaw)) <= 1.2:
            for lane in range(3):
                p = _lane(gate, lane, cfg)
                if _edge_clear(p, goal_xy, cone_index, cfg.min_clearance_m):
                    end.append((i, lane))
    if not initial:
        return Result(reason="no_forward_corridor_from_vehicle",
                      cone_count=len(cones))
    if not end:
        return Result(reason="goal_outside_mapped_track_corridor",
                      cone_count=len(cones))
    # Do not enter a long curve at a distant gate merely because skipping the
    # nearest gates slightly shortens the A* distance-to-goal heuristic.
    nearest_start = min(distance(start_xy, gates[i].center) for i, _ in initial)
    initial = [(i, lane) for i, lane in initial if
               distance(start_xy, gates[i].center) <= nearest_start+1.2]
    nearest_goal = min(distance(goal_xy, gates[i].center) for i, _ in end)
    end = [(i, lane) for i, lane in end if
           distance(goal_xy, gates[i].center) <= nearest_goal+1.2]
    terminal = set(end)
    neighbours = {}
    gate_index = _ConeIndex([(g.center[0], g.center[1], j)
                             for j, g in enumerate(gates)],
                            cell=cfg.max_gate_gap_m)
    for i, gate in enumerate(gates):
        available = []
        for _, _, j in gate_index.near_point(
                gate.center[0], gate.center[1], cfg.max_gate_gap_m):
            if i == j:
                continue
            other = gates[j]
            dx = other.center[0]-gate.center[0]
            dy = other.center[1]-gate.center[1]
            along = dx*math.cos(gate.heading)+dy*math.sin(gate.heading)
            d = math.hypot(dx, dy)
            if not (.35 <= along and .4 < d <= cfg.max_gate_gap_m):
                continue
            if abs(wrap(other.heading-gate.heading)) > cfg.max_heading_change_rad:
                continue
            if not _inside_quad(((gate.center[0]+other.center[0])*.5,
                                 (gate.center[1]+other.center[1])*.5), gate, other):
                continue
            available.append((d, j))
        available.sort()
        if available:
            # A long chord through a bend can be shorter than following its
            # successive track gates. Skip a missing gate only if necessary.
            closest = available[0][0]
            neighbours[i] = tuple(j for d, j in available
                                  if d <= max(closest*1.65, closest+1.) )[:3]
        else:
            neighbours[i] = ()
    nodes = {(i, lane): _lane(gate, lane, cfg)
             for i, gate in enumerate(gates) for lane in range(3)}
    best, previous, queue = {}, {}, []
    for gate_index, lane in initial:
        state = (gate_index, lane, 0)
        cost = distance(start_xy, nodes[(gate_index, lane)]) + .18*lane
        best[state] = cost
        previous[state] = None
        heapq.heappush(queue, (cost+distance(nodes[(gate_index, lane)], goal_xy),
                               cost, state))
    found = None
    visited = set()
    while queue:
        _, cost, state = heapq.heappop(queue)
        if state in visited:
            continue
        visited.add(state)
        i, lane, departed = state
        if (i, lane) in terminal and (not needs_lap or departed):
            found = state
            break
        start_point = nodes[(i, lane)]
        for j in neighbours[i]:
            for next_lane in range(3):
                finish = nodes[(j, next_lane)]
                next_departed = (departed or
                                  (needs_lap and distance(finish, start_xy) >
                                   cfg.max_start_m+1.))
                next_state = (j, next_lane, int(next_departed))
                if not _edge_clear(start_point, finish, cone_index,
                                   cfg.min_clearance_m):
                    continue
                middle = ((start_point[0]+finish[0])*.5,
                          (start_point[1]+finish[1])*.5)
                if not _inside_quad(middle, gates[i], gates[j]):
                    continue
                # Smooth central travel preferred until a cone blocks it.
                edge_cost = (distance(start_point, finish)
                             + .12*(next_lane != 0)
                             + .25*(next_lane != lane))
                new = cost+edge_cost
                if new < best.get(next_state, float("inf")):
                    best[next_state] = new
                    previous[next_state] = state
                    heapq.heappush(
                        queue, (new+distance(finish, goal_xy), new, next_state))
        if len(visited) > 6*cfg.max_gates:
            break
    if found is None:
        return Result(reason="no_obstacle_clear_track_connection",
                      cone_count=len(cones))
    route = []
    cursor = found
    while cursor is not None:
        route.append(cursor)
        cursor = previous[cursor]
    route.reverse()
    if len({i for i, _, _ in route}) < 2 and distance(start_xy, goal_xy) > 5.:
        return Result(reason="too_few_connected_track_gates",
                      cone_count=len(cones))
    raw = [start_xy]
    headings = [start.yaw]
    knot_gates = [None]
    for i, lane, _ in route:
        p = nodes[(i, lane)]
        if distance(raw[-1], p) > .03:
            raw.append(p)
            headings.append(gates[i].heading)
            knot_gates.append(i)
    if distance(raw[-1], goal_xy) > .03:
        raw.append(goal_xy)
        headings.append(goal.yaw)
        knot_gates.append(None)
    else:
        raw[-1] = goal_xy
        headings[-1] = goal.yaw
        knot_gates[-1] = None
    # Within the corridor, a lateral avoidance transition needs a small
    # heading component towards the next lane. Forcing every gate tangent to
    # exactly its centerline heading creates artificial steering spikes.
    for k in range(1, len(raw)-1):
        prev, current, following = raw[k-1:k+2]
        before = distance(prev, current)
        after = distance(current, following)
        if before > 1e-5 and after > 1e-5:
            vx = (current[0]-prev[0])/before+(following[0]-current[0])/after
            vy = (current[1]-prev[1])/before+(following[1]-current[1])/after
            if math.hypot(vx, vy) > .1:
                headings[k] = math.atan2(vy, vx)
    # Tangent-continuous path, with start/goal yaw and each gate heading.
    # A curvature or cone-clearance failure still vetoes the entire path.
    smooth, knot_stops = _interpolate(raw, headings, cfg.resolution_m,
                                      cfg.max_path_points)
    if not smooth:
        return Result(reason="route_point_budget_exceeded", cone_count=len(cones))
    # Hermite rounding must also stay between the blue/yellow boundary edges
    # of every connected gate pair, including between sparsely spaced cones.
    corridor_valid = True
    for k, (i, j) in enumerate(zip(knot_gates, knot_gates[1:])):
        if i is not None and j is not None:
            if any(not _inside_quad(p, gates[i], gates[j])
                   for p in smooth[knot_stops[k]:knot_stops[k+1]+1]):
                corridor_valid = False
                break
    minimum_clearance = route_clearance(smooth, cone_index)
    curvature_valid = _max_curvature(smooth) <= cfg.max_path_curvature_m_inv
    if not corridor_valid or minimum_clearance < cfg.min_clearance_m or not curvature_valid:
        fitted = _fit_bounded_spline(raw, headings, knot_gates, gates,
                                      cone_index, cfg)
        if fitted:
            smooth = fitted
            minimum_clearance = route_clearance(smooth, cone_index)
        elif not corridor_valid:
            return Result(reason="interpolation_leaves_mapped_track_corridor",
                          cone_count=len(cones))
        elif minimum_clearance < cfg.min_clearance_m:
            return Result(reason="smoothing_violates_cone_clearance",
                          cone_count=len(cones))
        else:
            return Result(reason="route_turn_exceeds_vehicle_steering_geometry",
                          cone_count=len(cones))
    length = sum(distance(a, b) for a, b in zip(smooth, smooth[1:]))
    return Result(points=smooth, reason="ok", gates_used=len(route),
                  cone_count=len(cones),
                  minimum_clearance_m=minimum_clearance,
                  route_length_m=length,
                  lateral_changes=sum(a[1] != b[1] for a, b in zip(route, route[1:])))
