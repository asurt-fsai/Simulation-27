"""ZED glare, lens water and sensor noise over Unreal's scene rendering."""
from functools import lru_cache
from math import exp, radians, sin, tan

import cv2
import numpy as np


MIN_LENS_WATER = 0.03


def _get(weather, key, default=0.0):
    return float(getattr(weather, key, default))


def _fraction(weather, key):
    return max(0.0, min(1.0, _get(weather, key) / 100.0))


def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


@lru_cache(maxsize=16)
def lens_pattern(height, width, seed):
    """Precompute small refraction maps in drop-local coordinates."""
    rng = np.random.default_rng(seed)
    drops = []
    for _ in range(34):
        radius = int(rng.integers(3, max(5, min(width, height) // 38)))
        length = int(radius * rng.uniform(1.15, 1.85))
        margin = 3 * radius + 3
        if min(width, height) <= 2 * margin:
            break
        cx, cy = rng.integers(margin, (width - margin, height - margin))
        phase = float(rng.uniform(0.0, 2.0 * np.pi))
        y, x = np.mgrid[:2 * length + 1, :2 * radius + 1].astype(np.float32)
        v = (y - length) / length
        u = (x - radius) / (radius * (1.0 + 0.23 * v))
        distance = u * u + (v + 0.13 * u * u) ** 2
        body = np.clip((1.0 - distance) * 2.6, 0.0, 1.0)
        map_x = x + 0.30 * body * (x - radius)
        map_y = y + 0.30 * body * (y - length)
        maps = cv2.convertMaps(map_x, map_y, cv2.CV_16SC2)
        rim = np.clip(1.0 - np.abs(distance - 0.80) * 6.0, 0.0, 1.0)
        edge = rim * (37.0 * np.clip(-0.55 * u - 0.78 * v, 0.0, 1.0)
                      - 27.0 * np.clip(0.50 * u + 0.75 * v, 0.0, 1.0))
        drops.append((cx - radius, cy - length, radius, phase, maps,
                      0.82 * body[..., None], edge[..., None]))
    return drops


def shade_lens_droplets(image, water, wind, scale, seed, frame):
    height, width = image.shape[:2]
    opacity = min(1.0, water / 0.50)
    count = min(34, int(water * scale * 42))
    for left, top, radius, phase, maps, body, edge in lens_pattern(height, width, seed)[:count]:
        drift = phase + frame * 0.028
        left += round(radius * wind * 0.6 * sin(drift))
        top += round(radius * 0.6 * sin(drift * 0.63))
        rows, cols = body.shape[:2]
        patch = image[top:top + rows, left:left + cols]
        refracted = cv2.remap(patch, *maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        patch += opacity * ((refracted - patch) * body + edge)


class WeatherEffects:
    def __init__(self, enabled=True, glare_scale=0.8, droplets_scale=0.75,
                 noise_std=1.5, seed=431):
        self.enabled = enabled
        self.glare_scale = max(0.0, glare_scale)
        self.droplets_scale = max(0.0, droplets_scale)
        self.noise_std = max(0.0, noise_std)
        self.seed = seed
        # Camera pose/weather are normally queried at a slower cadence than
        # RGB frames. Reuse the sun halo until those inputs actually change.
        self._glare_key = None
        self._glare_mask = None
        self._noise_buffer = None
        self._lenses = {}

    def lens_is_wet(self, eye):
        return eye in self._lenses and self._lenses[eye][0] >= MIN_LENS_WATER

    def lens_water(self, eye, rain, cloud, wind, sun_altitude, timestamp, frame):
        if timestamp is None:
            return rain, frame
        previous, last_time, pattern = self._lenses.get(eye, (0.0, timestamp, frame))
        if timestamp < last_time:
            return previous, pattern
        seconds = timestamp - last_time
        sun = max(0.0, sin(radians(sun_altitude))) * (1.0 - cloud)
        drying_rate = 0.055 + 0.055 * sun + 0.020 * wind
        water = max(rain, previous * exp(-drying_rate * seconds))
        if rain >= MIN_LENS_WATER:
            pattern = frame
        self._lenses[eye] = water, timestamp, pattern
        return water, pattern

    def process(self, bgr, weather, camera_yaw_deg, camera_pitch_deg, frame, eye,
                horizontal_fov_deg=110.0, *, timestamp=None):
        """Return an independent image; never mutate raw CARLA frame data."""
        if not self.enabled:
            return bgr.copy()
        cloud = _fraction(weather, "cloudiness")
        rain = _fraction(weather, "precipitation")
        wind = _fraction(weather, "wind_intensity")
        sun_alt = _get(weather, "sun_altitude_angle", -30.0)
        sun_yaw = _get(weather, "sun_azimuth_angle")
        water, pattern = self.lens_water(eye, rain, cloud, wind, sun_alt, timestamp, frame) if self.droplets_scale else (0.0, frame)
        droplets = water >= MIN_LENS_WATER
        glare = sun_alt > -5 and self.glare_scale > 0
        if not (droplets or glare or self.noise_std):
            return bgr.copy()
        h, w = bgr.shape[:2]
        out = bgr.astype(np.float32)

        # Extra atmospheric flare is intentionally mild: CARLA already renders
        # sunlight/weather in the scene. Glare is strongest near optical axis.
        if glare:
            horiz = radians(_wrap_deg(sun_yaw - camera_yaw_deg))
            vert = radians(sun_alt - camera_pitch_deg)
            fx = w / (2 * tan(radians(horizontal_fov_deg) / 2))
            sx = w / 2 + fx * tan(np.clip(horiz, -1.35, 1.35))
            sy = h / 2 - fx * tan(np.clip(vert, -1.35, 1.35))
            if -w < sx < 2*w and -h < sy < 2*h:
                strength = (1 - 0.78 * cloud) * self.glare_scale
                radius = max(w * 0.16, 25)
                key = (h, w, float(sx), float(sy), float(radius), float(strength))
                if key != self._glare_key:
                    # Broadcasting avoids per-frame meshgrid allocations;
                    # the cache updates whenever sun direction, cloudiness,
                    # camera heading, FOV, or image resolution changes.
                    xx = np.arange(w, dtype=np.float32)[None, :]
                    yy = np.arange(h, dtype=np.float32)[:, None]
                    halo = np.exp(-((xx-sx)**2 + (yy-sy)**2) / (2*radius**2))
                    self._glare_mask = (halo * (95 * strength)).astype(np.float32)
                    self._glare_key = key
                out += self._glare_mask[:, :, None]
                # Bloom and veiling glare lower contrast in the sunward view.
                out *= (1 - 0.08*strength)
                out += (12*strength)

        if droplets:
            shade_lens_droplets(out, water, wind, self.droplets_scale, self.seed + 173 * eye, pattern)

        if self.noise_std > 0:
            darkness = max(0, 0.4 - float(out.mean()) / 255.0)
            std = self.noise_std * (1 + 5*darkness + rain*0.3)
            # OpenCV's native float32 Gaussian RNG is substantially faster
            # than NumPy's float64 normal + conversion (measured 4.4 vs
            # 15.3 ms at 640x360). Each eye/frame still gets an independent,
            # reproducible per-channel noise realization.
            cv2.setRNGSeed(int((self.seed + int(frame)*13 +
                                eye*100003) % 2147483647))
            if self._noise_buffer is None or self._noise_buffer.shape != out.shape:
                self._noise_buffer = np.empty_like(out)
            cv2.randn(self._noise_buffer, (0, 0, 0), (std, std, std))
            out += self._noise_buffer
        return cv2.add(out, 0, dtype=cv2.CV_8U)
