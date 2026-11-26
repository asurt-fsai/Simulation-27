"""Stereo pinhole calibration. Factory per-unit calibration supersedes defaults."""
from dataclasses import dataclass
from math import radians, tan


@dataclass(frozen=True)
class Calibration:
    width: int = 640
    height: int = 360
    hfov_deg: float = 110.0
    baseline_m: float = 0.120
    mount_x: float = 0.4561834931
    mount_y: float = 0.0208417531
    mount_z: float = 0.7304209471

    def __post_init__(self):
        if not (self.width > 20 and self.height > 20 and 10 < self.hfov_deg < 170):
            raise ValueError("Invalid image dimensions or horizontal field of view")
        if not 0.02 < self.baseline_m < 0.5:
            raise ValueError("Invalid stereo baseline")

    @property
    def fx(self):
        return self.width / (2 * tan(radians(self.hfov_deg) / 2))

    @property
    def fy(self):
        return self.fx

    @property
    def cx(self):
        return self.width / 2.0

    @property
    def cy(self):
        return self.height / 2.0

    def point_in_base_link(self, u, v, disparity):
        """Return x-forward, y-left, z-up (REP-103), in metres.

        Uses rectified stereo disparity; never looks up CARLA cone actor positions.
        """
        if disparity <= 0.75:
            return None
        depth = self.fx * self.baseline_m / disparity
        return self.point_from_depth(u, v, depth)

    def point_from_depth(self, u, v, depth):
        """Project a rectified left-eye pixel at estimated optical depth."""
        if not 0.35 <= depth <= 35.0:
            return None
        return (
            float(depth + self.mount_x),
            float(self.mount_y + self.baseline_m / 2 - (u - self.cx) * depth / self.fx),
            float(self.mount_z - (v - self.cy) * depth / self.fy),
        )
