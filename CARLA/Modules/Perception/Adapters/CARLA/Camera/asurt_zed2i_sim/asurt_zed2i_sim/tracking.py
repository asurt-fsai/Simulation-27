"""Short-lived, image-derived cone tracks for stable RViz/legacy output.

Never consumes CARLA actors, CSV positions or ground-truth cone coordinates.
Odometry (when available) is used only to compensate for ego motion.
No old observation is published as if it were a fresh detection.
"""
from dataclasses import dataclass, replace
import math
import time


@dataclass
class Track:
    id: int
    cone: object
    wx: float
    wy: float
    hits: int = 1
    misses: int = 0
    last_seen_s: float = 0.0


@dataclass(frozen=True)
class TrackedDetection:
    id: int
    color: str
    confidence: float
    x: float
    y: float
    z: float
    box: tuple


def to_world(x, y, pose):
    if pose is None:
        return x, y
    px, py, yaw = pose
    co, si = math.cos(yaw), math.sin(yaw)
    return px + co*x - si*y, py + si*x + co*y


def to_base(x, y, pose):
    if pose is None:
        return x, y
    px, py, yaw = pose
    co, si = math.cos(yaw), math.sin(yaw)
    dx, dy = x-px, y-py
    return co*dx+si*dy, -si*dx+co*dy


class ConeTracker:
    """Confirmation + one-to-one data association, without persistent ghosts.

    A newly seen cone needs 2 consecutive observations. A temporarily missed
    cone is kept for association but is NOT emitted until it is seen again.
    Published coordinates are a mild filter in a global frame when /odom is
    available; for non-odom operation, filtering is conservative.
    """
    def __init__(self, min_hits=2, max_misses=2, alpha=0.65):
        self.min_hits = int(min_hits)
        self.max_misses = int(max_misses)
        self.alpha = float(alpha)
        self.tracks = {}
        self.next_id = 1
        self.last_pose_mode = None

    def update(self, observations, pose=None, *, observed_at=None):
        now = time.monotonic() if observed_at is None else float(observed_at)
        # Prevent mixing previous base-frame tracks with newly georeferenced
        # tracks if /odom appears or drops out during a run.
        mode = pose is not None
        if self.last_pose_mode is not None and mode != self.last_pose_mode:
            self.tracks.clear()
        self.last_pose_mode = mode
        assignments = []
        for index, c in enumerate(observations):
            if not all(math.isfinite(float(v)) for v in (c.x, c.y, c.z)):
                continue
            if c.x < 0.35 or c.x > 30.0 or abs(c.y) > 30.0:
                continue
            wx, wy = to_world(c.x, c.y, pose)
            for ident, t in self.tracks.items():
                if t.cone.color != c.color:
                    continue
                separation = math.hypot(wx-t.wx, wy-t.wy)
                # A moving car needs ego compensation; when unavailable,
                # accept somewhat more displacement but never huge jumps.
                gate = max(1.5, min(3.5, 0.20*max(1.0, c.x)))
                if separation > gate:
                    continue
                # Bounding box bearing is a secondary cue, not an oracle.
                bearing = abs(c.box[0] + c.box[2]/2 -
                              (t.cone.box[0] + t.cone.box[2]/2))
                if bearing > 115 and pose is None:
                    continue
                score = separation/gate + 0.001*bearing
                assignments.append((score, index, ident, wx, wy))
        selected = {}
        used_indices, used_tracks = set(), set()
        for _, i, ident, wx, wy in sorted(assignments):
            if i in used_indices or ident in used_tracks:
                continue
            selected[i] = (ident, wx, wy)
            used_indices.add(i)
            used_tracks.add(ident)

        for t in self.tracks.values():
            t.misses += 1
        emitted = []
        for i, c in enumerate(observations):
            if not all(math.isfinite(float(v)) for v in (c.x, c.y, c.z)):
                continue
            if c.x < 0.35 or c.x > 30.0 or abs(c.y) > 30.0:
                continue
            wx, wy = to_world(c.x, c.y, pose)
            if i in selected:
                ident, wx, wy = selected[i]
                t = self.tracks[ident]
                alpha = self.alpha if pose is not None else 0.85
                t.wx = alpha*wx + (1-alpha)*t.wx
                t.wy = alpha*wy + (1-alpha)*t.wy
                t.cone = c
                t.hits += 1
                t.misses = 0
                t.last_seen_s = now
            else:
                ident = self.next_id
                self.next_id += 1
                t = Track(ident, c, wx, wy, last_seen_s=now)
                self.tracks[ident] = t
            if t.hits >= self.min_hits:
                bx, by = to_base(t.wx, t.wy, pose)
                emitted.append(TrackedDetection(
                    t.id, c.color, c.confidence, bx, by, c.z, c.box))
        for ident, t in list(self.tracks.items()):
            if t.misses > self.max_misses:
                del self.tracks[ident]
        # Suppress two nearby same-colour fragments that refer to one cone.
        unique = []
        for c in sorted(emitted, key=lambda x: -x.confidence):
            if any(c.color == other.color and
                   math.hypot(c.x-other.x, c.y-other.y) < 0.45
                   for other in unique):
                continue
            unique.append(c)
        return sorted(unique, key=lambda c: c.id)

    def held_visualization(self, displayed_ids, pose=None, *, now=None,
                           max_hold_s=0.85):
        """Return briefly missed *previously displayed* cones for RViz only.

        These are historical observations: do not include them in ObjectsStamped,
        the camera detection overlay, or any planning/SLAM data topic. Reproject
        the last measured world position with the latest odometry when possible.
        The time cap also bounds how long a track may appear after inference slows.
        """
        if self.last_pose_mode is None or (pose is not None) != self.last_pose_mode:
            return []
        now = time.monotonic() if now is None else float(now)
        # Without odometry, an old base_link coordinate drifts as the car moves.
        max_age = min(max_hold_s, 0.30) if pose is None else max_hold_s
        held = []
        for ident in displayed_ids:
            t = self.tracks.get(ident)
            if t is None or t.hits < self.min_hits:
                continue
            if not 0 < t.misses <= self.max_misses:
                continue
            age = now - t.last_seen_s
            if not 0 <= age <= max_age:
                continue
            bx, by = to_base(t.wx, t.wy, pose)
            c = t.cone
            held.append(TrackedDetection(t.id, c.color, c.confidence,
                                          bx, by, c.z, c.box))
        return sorted(held, key=lambda c: c.id)
