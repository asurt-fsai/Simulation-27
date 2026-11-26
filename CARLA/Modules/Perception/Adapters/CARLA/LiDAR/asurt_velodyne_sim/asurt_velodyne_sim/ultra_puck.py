"""Nominal VLP-32C scan geometry from Ouster's Rev D manual, table 9-2."""

import math
from collections import defaultdict

# (Elevation, clockwise azimuth offset), indexed by hardware laser ID.
VLP32C_BEAMS = (
    (-25, -1.4), (-1, 4.2), (-1.667, -1.4), (-15.639, 1.4),
    (-11.31, -1.4), (0, 1.4), (-0.667, -4.2), (-8.843, 1.4),
    (-7.254, -1.4), (0.333, 4.2), (-0.333, -1.4), (-6.148, 1.4),
    (-5.333, -4.2), (1.333, 1.4), (0.667, -4.2), (-4, 1.4),
    (-4.667, -1.4), (1.667, 4.2), (1, -1.4), (-3.667, 4.2),
    (-3.333, -4.2), (3.333, 1.4), (2.333, -1.4), (-2.667, 1.4),
    (-3, -1.4), (7, 1.4), (4.667, -1.4), (-2.333, 4.2),
    (-2, -4.2), (15, 1.4), (10.333, -1.4), (-1.333, 1.4),
)


def beam_banks():
    """Pack one or two equal-azimuth beams into each stock CARLA actor.

    CARLA interpolates elevation evenly between its limits. With at most
    two beams per actor the specified elevations remain exact.
    """
    by_offset = defaultdict(list)
    for elevation, offset in VLP32C_BEAMS:
        by_offset[offset].append(elevation)
    return tuple(
        (offset, tuple(elevations[first:first + 2]))
        for offset, elevations in sorted(by_offset.items())
        for first in range(0, len(elevations), 2)
    )


def rotate_to_sensor(points, clockwise_yaw):
    """Bring a decoded FLU cloud from the yawed actor into the Velodyne frame."""
    yaw = math.radians(clockwise_yaw)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    x = points[:, 0].copy()
    points[:, 0] = cosine * x + sine * points[:, 1]
    points[:, 1] = cosine * points[:, 1] - sine * x
    return points
