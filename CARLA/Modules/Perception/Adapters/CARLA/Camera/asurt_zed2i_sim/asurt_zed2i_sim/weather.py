"""Depth-based visibility and approximate ZED lens effects over CARLA RGB."""
from math import cos, exp, radians, sin, tan

import cv2
import numpy as np


MIN_HAZE = 0.05
MIN_LENS_WATER = 0.03


def _get(weather, key, default=0.0):
    if weather is None:
        return default
    return float(getattr(weather, key, default))


def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def needs_scene_depth(weather):
    return (max(_get(weather, "fog_density"), _get(weather, "dust_storm")) >= 100 * MIN_HAZE
            or road_wetness(weather) >= 0.10)


def road_wetness(weather):
    actual_water = max(_get(weather, "wetness"), _get(weather, "precipitation_deposits"))
    return float(np.clip(actual_water / 100.0, 0.0, 1.0))


def decode_scene_depth(image):
    bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
    depth = bgra[:, :, 0].astype(np.float32) * 65536.0
    depth += bgra[:, :, 1].astype(np.float32) * 256.0
    depth += bgra[:, :, 2]
    return depth * (1000.0 / 16777215.0)


def camera_ground_plane(pose, ground_z):
    forward = pose.get_forward_vector()
    right = pose.get_right_vector()
    up = pose.get_up_vector()
    return (pose.location.z - ground_z, forward.z, right.z, up.z,
            pose.location.x, pose.location.y, forward.x, forward.y, right.x, right.y)


def attenuate(image, distance, extinction, airlight):
    transmission = np.exp(-extinction * distance)[..., None]
    image *= transmission
    image += np.asarray(airlight, dtype=np.float32) * (1.0 - transmission)


def shade_lens_droplets(image, water, wind, scale, seed, frame):
    height, width = image.shape[:2]
    rng = np.random.default_rng(seed + frame // 90)
    age = (frame % 90) / 90.0
    opacity = min(1.0, water / 0.50)
    for _ in range(min(34, int(water * scale * 42))):
        radius = int(rng.integers(3, max(5, min(width, height) // 29)))
        drift = float(rng.uniform(-0.24, 0.24)) * wind
        cx = int(rng.integers(0, width) + age * radius * drift * 4.0)
        cy = int(rng.integers(0, height) + age * radius * rng.uniform(0.15, 1.15))
        length = radius * rng.uniform(1.1, 2.1 + wind)
        x0, x1 = max(0, cx - 2 * radius), min(width, cx + 2 * radius + 1)
        y0, y1 = max(0, cy - int(length) - 2), min(height, cy + int(length) + 3)
        if x0 >= x1 or y0 >= y1:
            continue
        y, x = np.mgrid[y0:y1, x0:x1].astype(np.float32)
        v = (y - cy) / length
        u = ((x - cx) / radius - drift * v) / (1.0 + 0.23 * v)
        distance = u * u + (v + 0.13 * u * u) ** 2
        body = np.clip((1.0 - distance) * 2.6, 0.0, 1.0)
        # The curved bead refracts its background. The highlight occupies one
        # side of the drop and the opposite edge absorbs a little contrast.
        bend = 0.32 * body
        refracted = cv2.remap(image, x + bend * (x - cx), y + bend * (y - cy),
                              cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        rim = np.clip(1.0 - np.abs(distance - 0.80) * 6.0, 0.0, 1.0)
        highlight = 37.0 * rim * np.clip(-0.55 * u - 0.78 * v, 0.0, 1.0)
        shadow = 27.0 * rim * np.clip(0.50 * u + 0.75 * v, 0.0, 1.0)
        patch = image[y0:y1, x0:x1]
        patch += (refracted - patch) * (0.82 * opacity * body[..., None])
        patch += opacity * (highlight - shadow)[..., None]


def shade_wet_road(image, depth, camera_plane, horizontal_fov_deg, wetness, deposits):
    """Darken ground shading and reflect sky light at grazing angles."""
    rows, cols = depth.shape
    focal_length = cols / (2.0 * tan(radians(horizontal_fov_deg) / 2.0))
    right = (np.arange(cols, dtype=np.float32)[None, :] + 0.5 - cols / 2) / focal_length
    up = (rows / 2 - np.arange(rows, dtype=np.float32)[:, None] - 0.5) / focal_length
    camera_height, forward_z, right_z, up_z, x, y, fx, fy, rx, ry = camera_plane
    vertical = forward_z + right * right_z + up * up_z
    elevation = camera_height + depth * vertical
    tolerance = 0.10 + np.minimum(depth, 35.0) * 0.010
    ground = np.clip(1.0 - np.abs(elevation) / tolerance, 0.0, 1.0)
    ground *= (depth > 1.0) & (depth < 85.0)
    incidence = np.clip(-vertical / np.sqrt(1.0 + right**2 + up**2), 0.0, 1.0)
    fresnel = (1.0 - incidence)**5
    world_x = x + depth * (fx + right * rx)
    world_y = y + depth * (fy + right * ry)
    patches = np.sin(0.46 * world_x + 0.24 * world_y) + 0.5 * np.sin(1.13 * world_y - 0.37 * world_x)
    puddles = np.clip((patches + deposits - 0.5) * 0.8, 0.0, 1.0) * deposits
    h, w = image.shape[:2]
    if (cols, rows) != (w, h):
        ground = cv2.resize(ground, (w, h), interpolation=cv2.INTER_LINEAR)
        fresnel = cv2.resize(fresnel, (w, h), interpolation=cv2.INTER_LINEAR)
        puddles = cv2.resize(puddles, (w, h), interpolation=cv2.INTER_LINEAR)
    # Reject saturated cone paint and grass, and bright bodywork, at RGB pixel
    # resolution. Neither the road mask nor its edge is blurred across objects.
    blue, green, red = cv2.split(image)
    chroma = np.maximum(np.maximum(blue, green), red) - np.minimum(np.minimum(blue, green), red)
    ground *= np.clip((64.0 - chroma) / 30.0, 0.0, 1.0) * np.clip((240.0 - green) / 38.0, 0.0, 1.0)
    diffuse = (0.37 + 0.10 * puddles) * wetness * ground
    reflected = (0.08 + 0.48 * puddles) * wetness * ground * fresnel
    sky = cv2.resize(image[:max(1, h // 5)][::-1], (w, h), interpolation=cv2.INTER_LINEAR)
    sky = cv2.GaussianBlur(sky, (0, 0), 4.0)
    image *= (1.0 - diffuse[..., None]) * (1.0 - reflected[..., None])
    image += sky * reflected[..., None]


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
                horizontal_fov_deg=110.0, depth_m=None, camera_plane=None, *, timestamp=None):
        """Return an independent image; never mutate raw CARLA frame data."""
        if not self.enabled:
            return bgr.copy()
        h, w = bgr.shape[:2]
        out = bgr.astype(np.float32)
        cloud = np.clip(_get(weather, "cloudiness") / 100.0, 0, 1)
        rain = np.clip(_get(weather, "precipitation") / 100.0, 0, 1)
        fog = np.clip(_get(weather, "fog_density") / 100.0, 0, 1)
        dust = np.clip(_get(weather, "dust_storm") / 100.0, 0, 1)
        wind = np.clip(_get(weather, "wind_intensity") / 100.0, 0, 1)
        sun_alt = _get(weather, "sun_altitude_angle", -30.0)
        sun_yaw = _get(weather, "sun_azimuth_angle", 0.0)

        wetness = road_wetness(weather)
        if wetness >= 0.10 and depth_m is not None and camera_plane is not None:
            deposits = np.clip(_get(weather, "precipitation_deposits") / 100.0, 0.0, 1.0)
            shade_wet_road(out, depth_m, camera_plane, horizontal_fov_deg, wetness, deposits)

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

        if max(fog, dust) >= MIN_HAZE:
            # Bounded fallback while depth warms up or is momentarily late.
            distance = np.float32(28.0) if depth_m is None else cv2.resize(
                depth_m, (w, h), interpolation=cv2.INTER_LINEAR)
            if fog >= MIN_HAZE:
                start = max(0.0, _get(weather, "fog_distance"))
                attenuate(out, np.maximum(0.0, distance - start), 0.095 * fog**2,
                          (196, 201, 207))
            if dust >= MIN_HAZE:
                attenuate(out, distance, 0.064 * dust**1.5, (145, 177, 205))

        # After the rain stops, the last droplets fade without adding new ones.
        if self.droplets_scale:
            water, pattern = self.lens_water(eye, rain, cloud, wind, sun_alt, timestamp, frame)
            if water >= MIN_LENS_WATER:
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
        return np.clip(out, 0, 255).astype(np.uint8)
