"""Replaceable baseline HSV cone detector and stereo-disparity association.

Image-derived detections, NOT CSV/actor oracle. Not a trained ZED SDK detector.
"""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class Candidate:
    color: str
    x: int
    y: int
    w: int
    h: int
    area: float
    confidence: float = None  # 0–100; None retains the HSV baseline heuristic

    @property
    def u(self):
        return self.x + self.w / 2.0

    @property
    def v(self):
        return self.y + self.h / 2.0


@dataclass(frozen=True)
class Cone:
    color: str
    confidence: float
    x: float
    y: float
    z: float
    box: tuple


_HSV = {
    "blue": ((90, 60, 45), (135, 255, 255)),
    # Bright sunlight washes the yellow cones nearly white: on actual
    # Silverstone camera frames their HSV saturation falls below 70.
    "yellow": ((17, 28, 130), (44, 255, 255)),
    "orange": ((5, 85, 65), (16, 255, 255)),
}
_BGR = {"blue": (255, 85, 0), "yellow": (0, 210, 255),
        "orange": (0, 130, 255), "large_orange": (0, 95, 235)}

# The ASURT Landmark message reserves kind 2 for small orange and 3 for
# large orange. YOLO uses the spelling 'large_orange'; the old ZED converter
# recognizes the same spelling in its object label.
CONE_LABEL_IDS = {"blue": 0, "yellow": 1, "orange": 2, "large_orange": 3}


def nominal_cone_height(color):
    """Physical FS-AI cone height (m), for image-based stereo range gating."""
    return 0.505 if color == "large_orange" else 0.325


def stereo_color_compatible(left, right):
    """Right-eye HSV detects orange hue but cannot infer orange cone size."""
    return left == right or (left == "large_orange" and right == "orange")


def candidates(bgr):
    if bgr is None or bgr.size == 0:
        return []
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # Coloured stripes on one cone are separated by pale/white bands. Closing
    # them vertically avoids counting one physical blue cone 2–3 times.
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 11))
    minimum = max(8, bgr.shape[0] * bgr.shape[1] * 0.000025)
    result = []
    for color, (low, high) in _HSV.items():
        mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < minimum:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if (w < 3 or h < 4 or w / max(1, h) > 1.65
                    or y < bgr.shape[0] * 0.28
                    or y > bgr.shape[0] * 0.87):
                continue
            result.append(Candidate(color, x, y, w, h, float(area)))
    # When glare creates an adjacent second contour, keep only the dominant
    # overlapping region of the same colour. Different cones remain distinct.
    kept = []
    for c in sorted(result, key=lambda c: c.area, reverse=True):
        if any(c.color == k.color and _iou(c, k) >= 0.28 for k in kept):
            continue
        kept.append(c)
    return sorted(kept, key=lambda c: (c.color, c.x))


def _iou(a, b):
    x0 = max(a.x, b.x)
    y0 = max(a.y, b.y)
    w = max(0, min(a.x + a.w, b.x + b.w) - x0)
    h = max(0, min(a.y + a.h, b.y + b.h) - y0)
    intersection = w * h
    return intersection / max(1, a.w * a.h + b.w * b.h - intersection)


def detect_stereo(left_bgr, right_bgr, cal):
    """Cross-match same-colored components along an epipolar line."""
    return associate_candidates(candidates(left_bgr), candidates(right_bgr), cal)


def associate_candidates(left_candidates, right_candidates, cal):
    """Match independent image detections; no simulator/actor truth is used."""
    left = sorted(left_candidates, key=lambda c: c.h, reverse=True)
    right = list(right_candidates)
    used = set()
    detections = []
    for l in left:
        ranked = []
        for i, r in enumerate(right):
            disparity = l.u - r.u
            # The cone's approximate real height constrains disparity
            # without consulting the track CSV or the CARLA actor registry.
            expected = cal.baseline_m * l.h / nominal_cone_height(l.color)
            if (i in used or not stereo_color_compatible(l.color, r.color) or disparity <= 0.75
                    or disparity > max(2.8, expected * 2.25)
                    or disparity < max(0.75, expected * 0.38)
                    or abs(l.v - r.v) > max(3.5, l.h * 0.27)
                    or not 0.55 < l.h / max(1, r.h) < 1.8):
                continue
            score = (2.5 * abs(disparity - expected) +
                     1.5 * abs(l.v - r.v) + abs(l.h-r.h))
            ranked.append((score, i, disparity))
        if not ranked:
            continue
        _, i, disparity = min(ranked)
        # At 640 px horizontal resolution the disparity of a distant cone
        # can be just 1–2 pixels. Fuse triangulation with apparent-height
        # ranging to prevent wildly scattered or collapsed positions.
        stereo_depth = cal.fx * cal.baseline_m / disparity
        size_depth = cal.fx * nominal_cone_height(l.color) / max(1, l.h)
        w_stereo = 0.60 if l.h >= 15 else 0.25
        depth = w_stereo * stereo_depth + (1-w_stereo) * size_depth
        point = cal.point_from_depth(l.u, l.v, depth)
        if point is None:
            continue
        used.add(i)
        fill = l.area / max(1, l.w * l.h)
        confidence = (l.confidence if l.confidence is not None else
                      min(95.0, max(25.0, 55.0 + 35.0 * fill)))
        detections.append(Cone(l.color, confidence, *point, (l.x, l.y, l.w, l.h)))
    return detections


def draw_overlay(bgr, cones):
    overlay = bgr.copy()
    for cone in cones:
        x, y, w, h = cone.box
        color = _BGR.get(cone.color, (255, 255, 255))
        cv2.rectangle(overlay, (x, y), (x + w, y + h), color, 2)
        cv2.putText(overlay, f"{cone.color} {cone.x:.1f}m", (x, max(13, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    return overlay
