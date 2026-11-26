from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Mapping, Tuple

import numpy as np


PointList = List[Tuple[float, float]]


@dataclass
class ImportedTrackData:
    left_cones_local_m: PointList
    right_cones_local_m: PointList
    control_points_local_m: PointList
    track_width_m: float
    cone_spacing_m: float
    orange_cones_local_m: PointList = field(default_factory=list)
    orange_big_cones_local_m: PointList = field(default_factory=list)


def load_track_csv(filename: str) -> ImportedTrackData:
    with open(filename, "r", encoding="utf-8", newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        if reader.fieldnames is None:
            raise ValueError("CSV file is empty.")

        required = {"tag", "x", "y"}
        present = {field.strip().lower() for field in reader.fieldnames}
        if not required.issubset(present):
            raise ValueError("CSV must contain tag, x, and y columns.")

        reader.fieldnames = [field.strip().lower() for field in reader.fieldnames]
        return _import_cone_records(reader)


def load_track_json(filename: str) -> ImportedTrackData:
    """Load tagged metric cones, preserving both competition orange sizes."""
    with open(filename, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict) or not isinstance(payload.get("cones"), list):
        raise ValueError("Track JSON must contain a cones array.")
    if payload.get("format", "fs-trackdraw-cones-v1") != "fs-trackdraw-cones-v1":
        raise ValueError("Unsupported Trackdraw JSON format.")
    if payload.get("units", "m") != "m":
        raise ValueError("Trackdraw JSON coordinates must be in meters.")
    return _import_cone_records(payload["cones"])


def _import_cone_records(rows: Iterable[Mapping[str, object]]) -> ImportedTrackData:
    by_tag: dict[str, PointList] = {tag: [] for tag in ("blue", "yellow", "orange", "orange_big")}
    for row in rows:
        if not isinstance(row, Mapping):
            raise ValueError("Cone records must contain a tag, x, and y.")
        tag = str(row.get("tag", "")).strip().lower()
        if tag not in by_tag:
            continue
        try:
            point = (float(row["x"]), float(row["y"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{tag} cone has non-numeric coordinates.") from exc
        if not all(math.isfinite(value) for value in point):
            raise ValueError(f"{tag} cone has non-finite coordinates.")
        by_tag[tag].append(point)

    if not any(by_tag.values()):
        raise ValueError("Track file has no blue, yellow, orange, or orange_big cones.")

    # Keep entry/exit/timing markers out of boundary fitting.
    boundaries = by_tag["blue"] + by_tag["yellow"]
    origin_points = boundaries if boundaries else by_tag["orange"] + by_tag["orange_big"]
    origin = np.mean(np.asarray(origin_points, dtype=float), axis=0)

    def centered(tag: str) -> PointList:
        return [(float(x - origin[0]), float(y - origin[1])) for x, y in by_tag[tag]]

    left_local, right_local = centered("blue"), centered("yellow")
    if len(left_local) >= 4 and len(right_local) >= 4:
        controls, width, spacing = _build_centerline_control_points(left_local, right_local)
    else:
        # Partial lanes are editable without inventing a closed centerline.
        controls, width, spacing = [], 0.0, 0.0
    return ImportedTrackData(
        left_cones_local_m=left_local,
        right_cones_local_m=right_local,
        control_points_local_m=controls,
        track_width_m=width,
        cone_spacing_m=spacing,
        orange_cones_local_m=centered("orange"),
        orange_big_cones_local_m=centered("orange_big"),
    )


def _center_points(left_cones: PointList, right_cones: PointList) -> Tuple[PointList, PointList]:
    all_points = np.asarray(left_cones + right_cones, dtype=float)
    centroid = np.mean(all_points, axis=0)

    def _shift(points: PointList) -> PointList:
        arr = np.asarray(points, dtype=float)
        if arr.size == 0:
            return []
        shifted = arr - centroid
        return [(float(x), float(y)) for x, y in shifted]

    return _shift(left_cones), _shift(right_cones)


def _closed_length(points: np.ndarray) -> float:
    if points.ndim != 2 or points.shape[0] < 2:
        return 0.0
    rolled = np.roll(points, -1, axis=0)
    return float(np.sum(np.linalg.norm(rolled - points, axis=1)))


def _resample_closed_path(points: np.ndarray, sample_count: int) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[0] < 2:
        raise ValueError("Not enough points to resample a closed path.")

    segments = np.roll(pts, -1, axis=0) - pts
    segment_lengths = np.linalg.norm(segments, axis=1)
    total_length = float(np.sum(segment_lengths))
    if total_length <= 1e-9:
        raise ValueError("CSV track points collapse to zero length.")

    cumulative = np.zeros(len(pts) + 1, dtype=float)
    cumulative[1:] = np.cumsum(segment_lengths)
    targets = np.linspace(0.0, total_length, sample_count, endpoint=False)

    samples = np.empty((sample_count, 2), dtype=float)
    for idx, target in enumerate(targets):
        seg_index = int(np.searchsorted(cumulative, target, side="right") - 1)
        seg_index = min(max(seg_index, 0), len(pts) - 1)
        seg_length = segment_lengths[seg_index]
        if seg_length <= 1e-9:
            samples[idx] = pts[seg_index]
            continue
        local = (target - cumulative[seg_index]) / seg_length
        next_index = (seg_index + 1) % len(pts)
        samples[idx] = pts[seg_index] + local * (pts[next_index] - pts[seg_index])
    return samples


def _align_boundary_pair(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    best = None
    for candidate in (right, right[::-1].copy()):
        for shift in range(candidate.shape[0]):
            shifted = np.roll(candidate, shift, axis=0)
            score = float(np.mean(np.linalg.norm(left - shifted, axis=1)))
            if best is None or score < best[0]:
                best = (score, shifted.copy())
    return best[1]


def _mean_point_spacing(points: np.ndarray) -> float:
    if points.ndim != 2 or points.shape[0] < 2:
        return 0.0
    rolled = np.roll(points, -1, axis=0)
    segment_lengths = np.linalg.norm(rolled - points, axis=1)
    return float(np.mean(segment_lengths))


def _build_centerline_control_points(left_cones: PointList, right_cones: PointList) -> Tuple[PointList, float, float]:
    dense_count = 256
    left_dense = _resample_closed_path(np.asarray(left_cones, dtype=float), dense_count)
    right_dense = _resample_closed_path(np.asarray(right_cones, dtype=float), dense_count)
    right_dense = _align_boundary_pair(left_dense, right_dense)

    midpoint = 0.5 * (left_dense + right_dense)
    width_m = float(np.mean(np.linalg.norm(left_dense - right_dense, axis=1)))
    length_m = _closed_length(midpoint)
    control_count = int(np.clip(round(length_m / 25.0), 8, 20))
    control_points = _resample_closed_path(midpoint, control_count)

    spacing_candidates = [
        _mean_point_spacing(np.asarray(left_cones, dtype=float)),
        _mean_point_spacing(np.asarray(right_cones, dtype=float)),
    ]
    spacing_candidates = [value for value in spacing_candidates if value > 1e-9]
    cone_spacing_m = float(np.mean(spacing_candidates)) if spacing_candidates else 3.5

    return (
        [(float(x), float(y)) for x, y in control_points],
        max(width_m, 1.0),
        max(cone_spacing_m, 0.5),
    )
