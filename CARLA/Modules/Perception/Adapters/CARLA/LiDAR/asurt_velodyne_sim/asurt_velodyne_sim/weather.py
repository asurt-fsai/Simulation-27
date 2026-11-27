"""Approximate optical loss, range noise and backscatter for a Velodyne scan."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WeatherReturns:
    two_way_extinction: float = 0.0
    clutter_rate: float = 0.0
    rain: float = 0.0

    @classmethod
    def from_world(cls, weather):
        rain, fog, dust = (
            max(0.0, min(1.0, float(getattr(weather, field, 0.0)) / 100.0))
            for field in ("precipitation", "fog_density", "dust_storm")
        )
        if max(fog, dust) < 0.05 and rain == 0.0:
            return cls()
        # The CARLA slider is relative intensity, so these coefficients are
        # tuned to its range. See Goodin et al. (10.3390/electronics8010089)
        # for range-dependent attenuation and Choe et al. (10.3390/s24010014)
        # for intensity and point-count measurements on a VLP-32.
        return cls(two_way_extinction=0.012 * rain**0.85 + 0.085 * fog**2 + 0.030 * dust**2,
                   clutter_rate=0.003 * fog**2 + 0.006 * dust**2,
                   rain=rain)

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
        detected = rng.random(len(points), dtype=np.float32) >= loss

        # With one return per beam, weak particle echoes usually lose to a
        # solid target. See Montalban et al. (10.3390/atmos12060738), §6.3.
        chance = self.clutter_rate + (0.0009 * self.rain**1.6) * weak * np.minimum(distances / 20.0, 1.0)
        scatter = (distances > 4.0) & (rng.random(len(points), dtype=np.float32) < chance)
        keep = detected | scatter
        result = points[keep]
        result[:, 3] = intensities[keep]

        if self.rain:
            # Centimetre-scale timing error grows at low signal; move each
            # return along its original beam. CARLA supplies the dry noise.
            distance = distances[keep]
            noise = rng.standard_normal(len(result), dtype=np.float32)
            sigma_m = self.rain * (0.008 + 0.032 * weak[keep])
            offset = np.clip(sigma_m * noise, -0.12, 0.12)
            result[:, :3] *= (1.0 + offset / np.maximum(distance, 0.25))[:, None]
            noise = rng.standard_normal(len(result), dtype=np.float32)
            result[:, 3] *= 1.0 + 0.07 * self.rain * np.clip(noise, -3.0, 3.0)

        replaced = scatter[keep]
        count = np.count_nonzero(replaced)
        if count:
            distance = distances[scatter]
            limit = np.minimum(0.9 * distance, 20.0)
            # Near-field backscatter falls off quickly with distance.
            u = rng.random(count, dtype=np.float32)
            scatter_distance = 2.0 - 6.0 * np.log1p(u * np.expm1((2.0 - limit) / 6.0))
            result[replaced, :3] = xyz[scatter] * (scatter_distance / distance)[:, None]
            result[replaced, 3] = rng.uniform(0.005, 0.035, count)
        return result
