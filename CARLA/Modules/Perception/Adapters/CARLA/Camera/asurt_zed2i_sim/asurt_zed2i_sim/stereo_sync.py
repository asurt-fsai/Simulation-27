"""Bounded approximate-time synchronization for independently scheduled CARLA eyes.

Separate CARLA RGB actors do not share a hardware trigger, so their images
frequently have different simulation frame IDs. Use *simulation timestamps*
with a strict maximum skew. Never invent/duplicate a missing eye's frame.
"""


class StereoSynchronizer:
    def __init__(self, max_skew_s=0.05, max_pending_per_eye=10):
        if not 0.0 <= max_skew_s <= 0.1:
            raise ValueError("Stereo timestamp skew must be between 0 and 0.1 s")
        if max_pending_per_eye < 2:
            raise ValueError("Need room for at least two frames per eye")
        self.max_skew_s = float(max_skew_s)
        self.max_pending = int(max_pending_per_eye)
        self.frames = {"left": {}, "right": {}}
        self.last_used_frame = {"left": -1, "right": -1}
        self.dropped_unpaired = 0
        self.dropped_late = 0

    def add(self, eye, frame, timestamp, image, arrival_wall=0.0):
        if eye not in self.frames:
            raise ValueError(f"Unknown stereo eye: {eye}")
        if frame <= self.last_used_frame[eye]:
            self.dropped_late += 1
            return
        queue = self.frames[eye]
        queue[frame] = (image, float(timestamp), float(arrival_wall))
        # Bounded memory, including when an entire camera stream disappears.
        while len(queue) > self.max_pending:
            queue.pop(min(queue))
            self.dropped_unpaired += 1

    def pop_latest(self):
        """Return (left, right, skew), with each eye=(frame, image, stamp, arrival).

        Prefer the newest usable pair to reduce motion/visualization latency.
        A frame may appear in at most one pair; unmatched older frames are
        discarded, not published as synthetic observations.
        """
        best = None
        for lf, (li, lt, la) in self.frames["left"].items():
            for rf, (ri, rt, ra) in self.frames["right"].items():
                skew = abs(lt - rt)
                if skew > self.max_skew_s:
                    continue
                rank = (max(lt, rt), -skew)
                if best is None or rank > best[0]:
                    best = (rank, (lf, li, lt, la), (rf, ri, rt, ra), skew)
        if best is None:
            return None
        _, left, right, skew = best
        for eye, frame in (("left", left[0]), ("right", right[0])):
            queue = self.frames[eye]
            self.dropped_unpaired += sum(f < frame for f in queue)
            self.frames[eye] = {f: value for f, value in queue.items() if f > frame}
            self.last_used_frame[eye] = frame
        return left, right, skew
