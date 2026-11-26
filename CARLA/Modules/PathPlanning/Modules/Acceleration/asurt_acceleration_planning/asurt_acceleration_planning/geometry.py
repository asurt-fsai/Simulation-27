"""Track-constrained straight-line acceleration preview.

Historical source: oldstack/Acceleration_Planning_NEW/planning_acceleration/
calculate_path.py. That implementation extends up to 150 metres even with
missing cones. Its straight-gate/heading idea is retained, but unobserved
heading-only paths are deliberately rejected in this experimental adapter.
"""
from dataclasses import dataclass
import math

from asurt_classical_planning.geometry import Config, Result, centerline


@dataclass(frozen=True)
class AccelerationConfig(Config):
    min_forward_gates: int = 2
    max_heading_error_deg: float = 18.0
    max_straight_residual_m: float = 0.8
    max_straight_end_m: float = 30.0
    prediction_extension_m: float = 2.0


def acceleration_path(cones, vehicle, cfg=AccelerationConfig()):
    base = centerline(cones, vehicle, cfg)
    if base.reason != 'ok':
        return base
    if base.pairs < cfg.min_forward_gates:
        return Result(pairs=base.pairs, reason='insufficient_acceleration_gates')
    if len(base.points) < 2:
        return Result(pairs=base.pairs, reason='insufficient_straight_points')
    start, end = base.points[0], base.points[-1]
    dx, dy = end[0]-start[0], end[1]-start[1]
    length = math.hypot(dx, dy)
    if length <= 1.5:
        return Result(pairs=base.pairs, reason='insufficient_straight_length')
    heading = math.atan2(dy, dx)
    heading_error = math.atan2(math.sin(heading-vehicle[2]),
                              math.cos(heading-vehicle[2]))
    if abs(heading_error) > math.radians(cfg.max_heading_error_deg):
        return Result(pairs=base.pairs, reason='non_straight_acceleration_heading')
    ux, uy = dx/length, dy/length
    if max(abs((x-start[0])*uy-(y-start[1])*ux) for x, y in base.points) > cfg.max_straight_residual_m:
        return Result(pairs=base.pairs, reason='acceleration_corridor_curved')
    if not (0 < cfg.max_straight_end_m <= 100 and
            0 <= cfg.prediction_extension_m <= 5):
        return Result(pairs=base.pairs, reason='invalid_config')
    # Existing centreline is known through the last paired cone. Allow only
    # a very short, visible extrapolation beyond observed gates.
    forward = length + cfg.prediction_extension_m
    points = tuple((start[0] + ux*min(t*.5, forward),
                    start[1] + uy*min(t*.5, forward))
                   for t in range(int(min(forward, cfg.max_straight_end_m)*2)+1))
    return Result(points=points, pairs=base.pairs, reason='ok')
