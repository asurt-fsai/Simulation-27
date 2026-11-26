"""Approximate extra optical loss and backscatter for a Velodyne scan."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WeatherReturns:
    two_way_extinction: float = 0.0
    clutter_rate: float = 0.0

    @classmethod
    def from_world(cls, weather):
        rain, fog, dust = (
            max(0.0, min(1.0, float(getattr(weather, field, 0.0)) / 100.0))
            for field in ("precipitation", "fog_density", "dust_storm")
        )
        if max(fog, dust) < 0.05 and rain == 0.0:
            return cls()
        return cls(two_way_extinction=0.012 * rain + 0.085 * fog**2 + 0.030 * dust**2,
                   clutter_rate=0.004 * rain**2 + 0.003 * fog**2 + 0.006 * dust**2)

    def apply(self, points, timestamp):
        if self.two_way_extinction == 0.0 or not len(points):
            return points

        xyz = points[:, :3]
        distances = np.sqrt(np.einsum("ij,ij->i", xyz, xyz))
        transmission = np.exp(-self.two_way_extinction * distances)
        intensities = points[:, 3] * transmission
        # Weak and distant returns are most likely to fall below threshold.
        weak = 1.0 - np.clip(intensities / 0.12, 0.0, 1.0)
        loss = (1.0 - transmission) * (0.15 + 0.85 * weak)
        rng = np.random.default_rng(int(round(timestamp * 1_000_000)))
        keep = rng.random(len(points), dtype=np.float32) >= loss

        # A particle can win the single-return range bin. Reuse real ray
        # directions to avoid inventing arbitrary points between beams.
        scatter = (distances > 4.0) & (
            rng.random(len(points), dtype=np.float32) < self.clutter_rate)
        keep |= scatter
        result = points[keep]
        result[:, 3] = intensities[keep]
        replaced = scatter[keep]
        count = np.count_nonzero(replaced)
        scatter_distance = rng.uniform(2.0, 20.0, count)
        scale = np.minimum(scatter_distance / distances[scatter], 0.9)
        result[replaced, :3] *= scale[:, None]
        result[replaced, 3] = rng.uniform(0.01, 0.06, count)
        return result
