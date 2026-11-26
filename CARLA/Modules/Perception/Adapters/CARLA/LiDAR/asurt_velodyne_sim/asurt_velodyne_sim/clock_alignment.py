"""Bounded, one-time alignment of CARLA elapsed time with bridge epoch time.

The bridge stamps real sensor frames as epoch0 + (sim_time - sim0), while
/clock extrapolates beyond its latest server snapshot. Taking one /clock
message minus one CARLA snapshot therefore adds frame/extrapolation jitter.
"""

import math
from statistics import median


class ClockOffsetCalibrator:
    """Freeze the median of 21 spaced clock/snapshot pairs (roughly 2 s)."""

    def __init__(self, target_samples=21, min_samples=15,
                 sample_interval_s=0.095, max_window_s=3.0):
        if not 1 <= min_samples <= target_samples <= 25:
            raise ValueError("invalid clock sample count")
        if sample_interval_s <= 0 or max_window_s <= 0:
            raise ValueError("invalid clock sampling interval")
        self.target_samples = target_samples
        self.min_samples = min_samples
        self.sample_interval_s = sample_interval_s
        self.max_window_s = max_window_s
        self.samples = []
        self.first_sample_at = None
        self.last_sample_at = None
        self.last_frame = None
        self.offset = None
        self.window_seconds = 0.0

    def due(self, now):
        return self.offset is None and (
            self.last_sample_at is None
            or now - self.last_sample_at >= self.sample_interval_s
        )

    def defer(self, now):
        """Throttle failed snapshot reads as well as successful sample pairs."""
        if self.offset is None:
            self.last_sample_at = now

    def observe(self, clock_epoch, sim_elapsed, now, frame):
        """Record one valid distinct world frame; return True on finalization."""
        if not self.due(now):
            return False
        self.defer(now)
        if not all(map(math.isfinite, (clock_epoch, sim_elapsed, now))):
            return False
        if self.last_frame is not None and frame <= self.last_frame:
            return False
        self.last_frame = frame
        if self.first_sample_at is None:
            self.first_sample_at = now
        self.samples.append(clock_epoch - sim_elapsed)
        self.window_seconds = now - self.first_sample_at
        if len(self.samples) >= self.target_samples or (
            len(self.samples) >= self.min_samples
            and self.window_seconds >= self.max_window_s
        ):
            self.offset = median(self.samples)
            return True
        return False
