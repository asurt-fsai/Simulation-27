"""ZED glare, lens water and sensor noise over Unreal's scene rendering."""
from functools import lru_cache
from math import exp, radians, sin, tan

import cv2
import numpy as np


MIN_LENS_WATER = 0.03
MIN_HAZE = 0.05
LEGACY_FRAME_SECONDS = 0.033


def _get(weather, key, default=0.0):
    return float(getattr(weather, key, default))


def _fraction(weather, key):
    return max(0.0, min(1.0, _get(weather, key) / 100.0))


def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def needs_scene_depth(weather):
    """Older editable camera nodes use this to decide whether to spawn depth cameras."""
    return False


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


class AtmosphericDepth:
    """A small depth stream for fog and dust, paired to an existing RGB view."""

    def __init__(self):
        self.camera = None
        self.sample = None
        self.rgb_id = None

    def update(self, carla, world, rgb, weather):
        if max(_fraction(weather, "fog_density"), _fraction(weather, "dust_storm")) < MIN_HAZE:
            self.stop()
            return
        if self.rgb_id == rgb.id and self.camera is not None:
            return
        self.stop()
        settings = rgb.attributes
        width = min(192, int(settings["image_size_x"]))
        height = max(1, round(width * int(settings["image_size_y"]) / int(settings["image_size_x"])))
        blueprint = world.get_blueprint_library().find("sensor.camera.depth")
        for field, value in (("image_size_x", width), ("image_size_y", height), ("fov", settings["fov"]),
                             ("sensor_tick", 0.2), ("role_name", "asurt_atmospheric_depth")):
            blueprint.set_attribute(field, str(value))
        camera = world.spawn_actor(blueprint, carla.Transform(), attach_to=rgb)
        self.camera, self.rgb_id = camera, rgb.id
        try:
            camera.listen(lambda frame: self.receive(camera, frame))
        except RuntimeError:
            self.stop()
            raise

    def receive(self, camera, frame):
        if camera is self.camera:
            self.sample = camera, frame.timestamp, decode_scene_depth(frame)

    def for_frame(self, timestamp):
        sample = self.sample
        if sample is not None and sample[0] is self.camera and abs(timestamp - sample[1]) <= 0.5:
            return sample[2]
        return None

    def stop(self):
        camera, self.camera = self.camera, None
        self.sample = self.rgb_id = None
        if camera is not None:
            try:
                camera.stop()
            finally:
                camera.destroy()


def dust_veil(image, dust):
    # A subtle veil while the depth stream is warming up.
    opacity = 0.42 * dust**1.5
    transmission = 1.0 - opacity
    matrix = np.array(((transmission, 0, 0, 116 * opacity),
                       (0, transmission, 0, 166 * opacity),
                       (0, 0, transmission, 203 * opacity)), dtype=np.float32)
    return cv2.transform(image, matrix)


@lru_cache(maxsize=8)
def airlight_image(height, width, color):
    return np.full((height, width, 3), color, dtype=np.uint8)


def attenuate_visibility(image, depth, fog, start, dust):
    fog = 0.095 * fog**2 if fog >= MIN_HAZE else 0.0
    dust = 0.064 * dust**1.5 if dust >= MIN_HAZE else 0.0
    thickness = fog * np.maximum(depth - start, 0.0) + dust * depth
    transmission = np.exp(-thickness)
    h, w = image.shape[:2]
    transmission = cv2.resize(transmission, (w, h), interpolation=cv2.INTER_LINEAR)
    color = tuple(round((fog * cool + dust * warm) / (fog + dust))
                  for cool, warm in zip((196, 201, 207), (145, 177, 205)))
    return cv2.blendLinear(image, airlight_image(h, w, color), transmission, 1.0 - transmission)


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
            # The bridge ticks at a fixed step. Older editable ZED nodes pass
            # the CARLA world frame ID without the capture timestamp.
            timestamp = frame * LEGACY_FRAME_SECONDS
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
        cloud = _fraction(weather, "cloudiness")
        rain = _fraction(weather, "precipitation")
        fog = _fraction(weather, "fog_density")
        dust = _fraction(weather, "dust_storm")
        wind = _fraction(weather, "wind_intensity")
        sun_alt = _get(weather, "sun_altitude_angle", -30.0)
        sun_yaw = _get(weather, "sun_azimuth_angle")
        water, pattern = self.lens_water(eye, rain, cloud, wind, sun_alt, timestamp, frame) if self.droplets_scale else (0.0, frame)
        droplets = water >= MIN_LENS_WATER
        glare = sun_alt > -5 and self.glare_scale > 0
        atmosphere = max(fog, dust) >= MIN_HAZE and depth_m is not None
        if atmosphere:
            bgr = attenuate_visibility(bgr, depth_m, fog, max(0.0, _get(weather, "fog_distance")), dust)
        elif dust >= MIN_HAZE:
            bgr = dust_veil(bgr, dust)
        if not (droplets or glare or self.noise_std):
            return bgr if atmosphere or dust >= MIN_HAZE else bgr.copy()
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
