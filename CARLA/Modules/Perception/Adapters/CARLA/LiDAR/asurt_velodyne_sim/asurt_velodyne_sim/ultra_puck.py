"""Nominal VLP-32C scan geometry from Ouster's Rev D manual, table 9-2."""

import math

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
    """Pack the calibrated beams into 14 stock CARLA actors.

    Each group uses one azimuth and evenly spaced elevations, so CARLA's
    interpolation reproduces every beam in the calibration table.
    """
    groups = ((12, 14), (20, 28, 6),
              (0, 4), (8, 16), (24, 2), (10, 18, 22), (26, 30),
              (3, 7), (11, 15), (23, 21), (31, 5, 13), (25, 29),
              (19, 17), (27, 1, 9))
    return tuple((VLP32C_BEAMS[ids[0]][1], tuple(VLP32C_BEAMS[i][0] for i in ids))
                 for ids in groups)


def rotate_to_sensor(points, clockwise_yaw):
    """Bring a decoded FLU cloud from the yawed actor into the Velodyne frame."""
    yaw = math.radians(clockwise_yaw)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    x = points[:, 0].copy()
    points[:, 0] = cosine * x + sine * points[:, 1]
    points[:, 1] = cosine * points[:, 1] - sine * x
    return points
