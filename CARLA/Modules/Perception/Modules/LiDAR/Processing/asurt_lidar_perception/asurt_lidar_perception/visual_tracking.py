"""Small bounded tracker for LiDAR *RViz markers*, never landmark output.

Cone candidates originate in a LiDAR-local frame. Short visual continuity
removes RViz scan-to-scan blinking; both miss count and capture-age caps prevent
old sensor-frame positions persisting as the vehicle moves.
"""

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class VisualCone:
    id: int
    center: tuple
    estimate: float


@dataclass
class _VisualTrack:
    visual: VisualCone
    hits: int
    misses: int
    last_seen_s: float


class LidarVisualTracker:
    def __init__(self, *, min_hits=2, max_misses=2, max_hold_s=0.45,
                 match_radius_m=0.85, max_candidates=128):
        self.min_hits = min_hits
        self.max_misses = max_misses
        self.max_hold_s = max_hold_s
        self.match_radius_m = match_radius_m
        self.max_candidates = max_candidates
        self.tracks = {}
        self.next_id = 1
        self.frame_id = None

    def reset(self):
        self.tracks.clear()
        self.frame_id = None

    def update(self, cones, frame_id, *, now=None):
        """Use only this scan's candidates, retain short missed visual tracks.

        The real /lidar/cones LandmarkArray must be published from `cones`
        directly and must never contain a historical VisualCone.
        """
        now = time.monotonic() if now is None else float(now)
        if self.frame_id is not None and frame_id != self.frame_id:
            self.tracks.clear()
        self.frame_id = frame_id

        valid = []
        for center, estimate in cones[:self.max_candidates]:
            xyz = tuple(float(value) for value in center[:3])
            estimate = float(estimate)
            if len(xyz) != 3 or not all(math.isfinite(v) for v in (*xyz, estimate)):
                continue
            valid.append((xyz, estimate))

        choices = []
        for index, (xyz, _) in enumerate(valid):
            for ident, t in self.tracks.items():
                age = now - t.last_seen_s
                if not 0 <= age <= self.max_hold_s:
                    continue
                previous = t.visual.center
                distance = math.hypot(xyz[0]-previous[0], xyz[1]-previous[1])
                if distance <= self.match_radius_m and abs(xyz[2]-previous[2]) <= 0.40:
                    choices.append((distance, index, ident))
        matched = {}
        used = set()
        for _, index, ident in sorted(choices):
            if index not in matched and ident not in used:
                matched[index] = ident
                used.add(ident)

        for t in self.tracks.values():
            t.misses += 1

        visible = []
        for index, (xyz, estimate) in enumerate(valid):
            if index in matched:
                t = self.tracks[matched[index]]
                old = t.visual
                # Mild smoothing, but every correction comes from this scan.
                position = tuple(0.75*new + 0.25*previous
                                 for new, previous in zip(xyz, old.center))
                confidence = 0.75*estimate + 0.25*old.estimate
                t.visual = VisualCone(old.id, position, confidence)
                t.hits += 1
                t.misses = 0
                t.last_seen_s = now
            else:
                ident = self.next_id
                self.next_id += 1
                t = _VisualTrack(VisualCone(ident, xyz, estimate), 1, 0, now)
                self.tracks[ident] = t
            if t.hits >= self.min_hits:
                visible.append(t.visual)

        for ident, t in list(self.tracks.items()):
            age = now - t.last_seen_s
            if t.misses > self.max_misses or not 0 <= age <= self.max_hold_s:
                del self.tracks[ident]
            elif t.misses and t.hits >= self.min_hits:
                visible.append(t.visual)

        return sorted(visible, key=lambda item: item.id)
