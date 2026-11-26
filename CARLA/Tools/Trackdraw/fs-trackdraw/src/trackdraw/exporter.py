from __future__ import annotations

import csv
import json
import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


def transform_points_to_export_frame(
    centerline: Sequence[Sequence[float]],
    points: Iterable[Sequence[float]],
    px_per_m: float,
) -> List[Tuple[float, float]]:
    center = np.asarray(centerline, dtype=float)
    if center.ndim != 2 or center.shape[0] < 2:
        raise ValueError("Not enough centerline points to define export frame")

    origin = center[0]
    tangent = center[1] - origin
    theta = math.atan2(float(tangent[1]), float(tangent[0]))
    rotation = np.array(
        [
            [math.cos(-theta), -math.sin(-theta)],
            [math.sin(-theta), math.cos(-theta)],
        ],
        dtype=float,
    )

    transformed: List[Tuple[float, float]] = []
    for point in points:
        local = np.asarray(point, dtype=float) - origin
        local_rot = rotation.dot(local) / float(px_per_m)
        transformed.append((float(local_rot[0]), float(local_rot[1])))
    return transformed


def export_track_csv(
    filename: str,
    centerline: Sequence[Sequence[float]],
    left_cones: Sequence[Sequence[float]],
    right_cones: Sequence[Sequence[float]],
    px_per_m: float,
    orange_cones: Sequence[Sequence[float]] = (),
    orange_big_cones: Sequence[Sequence[float]] = (),
) -> None:
    _write_csv(filename, _track_rows(centerline, left_cones, right_cones, px_per_m,
                                      orange_cones, orange_big_cones))


def export_manual_cones_csv(
    filename: str,
    left_cones: Sequence[Sequence[float]],
    right_cones: Sequence[Sequence[float]],
    px_per_m: float,
    orange_cones: Sequence[Sequence[float]] = (),
    orange_big_cones: Sequence[Sequence[float]] = (),
) -> None:
    """Export individually positioned cones in a canvas-aligned metric frame.

    The first blue, yellow, small orange, or big orange cone is the origin.
    Positive X points right and positive Y points down in the canvas image.
    """
    _write_csv(filename, _manual_rows(left_cones, right_cones, px_per_m,
                                      orange_cones, orange_big_cones))


def export_track_json(
    filename: str, centerline: Sequence[Sequence[float]],
    left_cones: Sequence[Sequence[float]], right_cones: Sequence[Sequence[float]],
    px_per_m: float, orange_cones: Sequence[Sequence[float]] = (),
    orange_big_cones: Sequence[Sequence[float]] = (),
) -> None:
    """Export the same metric tags as CSV in a portable JSON cone list."""
    _write_json(filename, _track_rows(centerline, left_cones, right_cones, px_per_m,
                                       orange_cones, orange_big_cones))


def export_manual_cones_json(
    filename: str, left_cones: Sequence[Sequence[float]],
    right_cones: Sequence[Sequence[float]], px_per_m: float,
    orange_cones: Sequence[Sequence[float]] = (),
    orange_big_cones: Sequence[Sequence[float]] = (),
) -> None:
    _write_json(filename, _manual_rows(left_cones, right_cones, px_per_m,
                                        orange_cones, orange_big_cones))


def _tagged_points(left_cones, right_cones, orange_cones, orange_big_cones):
    return (
        ("yellow", np.asarray(right_cones, dtype=float).reshape(-1, 2)),
        ("blue", np.asarray(left_cones, dtype=float).reshape(-1, 2)),
        ("orange", np.asarray(orange_cones, dtype=float).reshape(-1, 2)),
        ("orange_big", np.asarray(orange_big_cones, dtype=float).reshape(-1, 2)),
    )


def _check_scale(px_per_m: float):
    if not math.isfinite(px_per_m) or px_per_m <= 0:
        raise ValueError("Scale must be finite and positive")


def _track_rows(centerline, left_cones, right_cones, px_per_m, orange_cones, orange_big_cones):
    _check_scale(px_per_m)
    rows = []
    for tag, points in _tagged_points(left_cones, right_cones, orange_cones, orange_big_cones):
        for x, y in transform_points_to_export_frame(centerline, points, px_per_m):
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("Cone positions must be finite")
            rows.append((tag, x, y))
    return rows


def _manual_rows(left_cones, right_cones, px_per_m, orange_cones, orange_big_cones):
    _check_scale(px_per_m)
    groups = _tagged_points(left_cones, right_cones, orange_cones, orange_big_cones)
    if not any(len(points) for _, points in groups):
        raise ValueError("No manually placed cones to export")
    origin = next(points[0] for _, points in
                  (("blue", groups[1][1]), ("yellow", groups[0][1]),
                   ("orange", groups[2][1]), ("orange_big", groups[3][1]))
                  if len(points))
    rows = []
    for tag, points in groups:
        for x, y in (points - origin) / px_per_m:
            if not math.isfinite(float(x)) or not math.isfinite(float(y)):
                raise ValueError("Cone positions must be finite")
            rows.append((tag, float(x), float(y)))
    return rows


def _write_csv(filename: str, rows):
    with open(filename, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["tag", "x", "y"])
        writer.writerows(rows)


def _write_json(filename: str, rows):
    payload = {
        "format": "fs-trackdraw-cones-v1",
        "units": "m",
        "cones": [{"tag": tag, "x": x, "y": y} for tag, x, y in rows],
    }
    with open(filename, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, allow_nan=False)
        handle.write("\n")
