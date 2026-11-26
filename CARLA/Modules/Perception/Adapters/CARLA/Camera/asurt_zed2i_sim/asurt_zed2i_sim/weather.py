"""Approximate sensor-side weather response layered on CARLA's rendered weather.

Engine rendering supplies scene lighting/rain/fog. These image-domain effects
approximate additional physical-lens artifacts (wet glass, veiling flare,
electronic noise). They are not a validated physical ZED 2i sensor model.
"""
from math import cos, radians, sin, tan

import cv2
import numpy as np


def _get(weather, key, default=0.0):
    if weather is None:
        return default
    return float(getattr(weather, key, default))


def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


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

    def process(self, bgr, weather, camera_yaw_deg, camera_pitch_deg, frame, eye,
                horizontal_fov_deg=110.0):
        """Return an independent image; never mutate raw CARLA frame data."""
        if not self.enabled:
            return bgr.copy()
        h, w = bgr.shape[:2]
        out = bgr.astype(np.float32)
        cloud = np.clip(_get(weather, "cloudiness") / 100.0, 0, 1)
        rain = np.clip(_get(weather, "precipitation") / 100.0, 0, 1)
        fog = np.clip(_get(weather, "fog_density") / 100.0, 0, 1)
        wind = np.clip(_get(weather, "wind_intensity") / 100.0, 0, 1)
        sun_alt = _get(weather, "sun_altitude_angle", -30.0)
        sun_yaw = _get(weather, "sun_azimuth_angle", 0.0)

        # Extra atmospheric flare is intentionally mild: CARLA already renders
        # sunlight/weather in the scene. Glare is strongest near optical axis.
        if sun_alt > -5 and self.glare_scale:
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

        if fog > 0.06:
            # Fog already exists in rendered image; this adds a sensor-side
            # reduction in local contrast at high simulated haze densities.
            amount = min(0.16, fog * 0.20)
            out *= (1 - amount)
            out += np.array([196, 201, 207], dtype=np.float32)*amount

        # Droplets move slowly relative to incoming sensor frames. Left/right
        # have separate glass and therefore separate droplet patterns.
        if rain > 0.03 and self.droplets_scale:
            rng = np.random.default_rng(self.seed + 173 * eye + frame // 60)
            num = int(min(45, rain * self.droplets_scale * 36))
            if num:
                mask = np.zeros((h, w), np.uint8)
                for _ in range(num):
                    x = int(rng.integers(0, w))
                    y = int(rng.integers(0, h))
                    radius = int(rng.integers(2, max(4, min(w,h)//35)))
                    cv2.ellipse(mask, (x, y),
                                (radius, max(2, int(radius * (1.1 + wind)))),
                                0, 0, 360, int(rng.integers(100, 220)), -1)
                glass_blur = cv2.GaussianBlur(out, (0, 0), sigmaX=max(2.0, min(w, h)/120.0))
                alpha = (mask.astype(np.float32)/255.0 * 0.9)[:, :, None]
                out *= (1-alpha)
                out += glass_blur*alpha
                # Water scatters light and creates bright halo rims.
                rim = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT,
                                        np.ones((3,3), np.uint8)).astype(np.float32)
                out += rim[:, :, None]*0.18

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
        return np.clip(out, 0, 255).astype(np.uint8)
