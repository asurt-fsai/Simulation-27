import pathlib
import sys
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / "Adapters/CARLA/Camera/asurt_zed2i_sim"))
from asurt_zed2i_sim.optics import Calibration
from asurt_zed2i_sim.detection import candidates, detect_stereo
from asurt_zed2i_sim.weather import WeatherEffects


class StereoTests(unittest.TestCase):
    def test_stereo_baseline_and_forward_depth(self):
        c = Calibration(width=640, height=360, hfov_deg=110, baseline_m=0.12)
        depth = 5.0
        p = c.point_in_base_link(320, 180, c.fx * c.baseline_m / depth)
        self.assertAlmostEqual(p[0], depth + c.mount_x, places=5)
        self.assertAlmostEqual(p[1], c.mount_y + c.baseline_m / 2, places=5)
        self.assertIsNone(c.point_in_base_link(320, 180, 0))

    def test_cone_from_stereo_pixels_not_oracle(self):
        c = Calibration()
        a = np.zeros((360, 640, 3), np.uint8)
        b = np.zeros_like(a)
        cv2.rectangle(a, (318, 125), (341, 165), (255, 0, 0), -1)
        cv2.rectangle(b, (310, 125), (333, 165), (255, 0, 0), -1)
        objs = detect_stereo(a, b, c)
        self.assertEqual(len(objs), 1)
        self.assertEqual(objs[0].color, "blue")
        self.assertGreater(objs[0].x, 3.0)

    def test_weather_effects_do_not_change_input(self):
        src = np.full((80, 150, 3), 80, np.uint8)
        copy = src.copy()
        effects = WeatherEffects(noise_std=0, glare_scale=0)
        out = effects.process(src, None, 0, 0, 101, 0)
        self.assertTrue(np.array_equal(src, copy))
        self.assertTrue(np.array_equal(out, src))

    def test_relative_cones_change_when_image_moves(self):
        cal = Calibration()

        def pair(shift):
            a = np.zeros((360, 640, 3), np.uint8)
            b = np.zeros_like(a)
            cv2.rectangle(a, (318+shift, 125), (341+shift, 165), (255, 0, 0), -1)
            cv2.rectangle(b, (310+shift, 125), (333+shift, 165), (255, 0, 0), -1)
            return detect_stereo(a, b, cal)

        before, after = pair(0), pair(40)
        self.assertEqual(len(before), 1)
        self.assertEqual(len(after), 1)
        self.assertLess(after[0].y, before[0].y)

    def test_bright_yellow_and_striped_blue_are_distinct_cones(self):
        # Simulate the low-saturation yellow and pale bands that occurred in
        # an actual sunlit CARLA camera image; no track CSV/oracle involved.
        hsv_l = np.zeros((360, 640, 3), dtype=np.uint8)
        hsv_r = np.zeros_like(hsv_l)
        for canvas, shift in [(hsv_l, 0), (hsv_r, -10)]:
            cv2.rectangle(canvas, (166+shift, 213), (188+shift, 240), (111, 210, 245), -1)
            # The pale stripes split the original baseline into three contours.
            cv2.rectangle(canvas, (166+shift, 221), (188+shift, 224), (0, 0, 244), -1)
            cv2.rectangle(canvas, (166+shift, 229), (188+shift, 232), (0, 0, 244), -1)
        for canvas, shift in [(hsv_l, 0), (hsv_r, -9)]:
            cv2.rectangle(canvas, (446+shift, 209), (465+shift, 233), (29, 38, 250), -1)
        left = cv2.cvtColor(hsv_l, cv2.COLOR_HSV2BGR)
        right = cv2.cvtColor(hsv_r, cv2.COLOR_HSV2BGR)
        raw = candidates(left)
        self.assertEqual([c.color for c in raw].count('blue'), 1)
        self.assertEqual([c.color for c in raw].count('yellow'), 1)
        cones = detect_stereo(left, right, Calibration())
        self.assertEqual(sorted(c.color for c in cones), ['blue', 'yellow'])
        self.assertGreater(abs(cones[0].y-cones[1].y), 2.0)

    def test_rain_and_sun_affect_frames_without_changing_world(self):
        clear = SimpleNamespace(precipitation=0, cloudiness=0,
                                fog_density=0, wind_intensity=0,
                                sun_altitude_angle=-30, sun_azimuth_angle=0)
        rainy = SimpleNamespace(precipitation=95, cloudiness=85,
                                fog_density=60, wind_intensity=60,
                                sun_altitude_angle=30, sun_azimuth_angle=0)
        frame = np.full((180, 320, 3), 100, np.uint8)
        effects = WeatherEffects(noise_std=0, droplets_scale=1.0, glare_scale=1.0)
        a = effects.process(frame, clear, 0, 0, 12, 0)
        b = effects.process(frame, rainy, 0, 0, 12, 0)
        self.assertFalse(np.array_equal(a, b))
        self.assertTrue(np.array_equal(frame, np.full_like(frame, 100)))

    def test_glare_cache_reuses_halo_and_tracks_sun_and_camera_pose(self):
        sky = SimpleNamespace(precipitation=0, cloudiness=25,
                              fog_density=0, wind_intensity=0,
                              sun_altitude_angle=30, sun_azimuth_angle=0)
        effects = WeatherEffects(noise_std=0, glare_scale=1.0)
        frame = np.full((180, 320, 3), 90, np.uint8)
        facing = effects.process(frame, sky, 0, 0, 123, 0)
        cached = effects._glare_mask
        again = effects.process(frame, sky, 0, 0, 124, 0)
        self.assertIs(effects._glare_mask, cached)
        np.testing.assert_array_equal(facing, again)
        rotated = effects.process(frame, sky, 45, 0, 125, 0)
        self.assertIsNot(effects._glare_mask, cached)
        self.assertFalse(np.array_equal(rotated, facing))
        sky.cloudiness = 90
        clouded = effects.process(frame, sky, 45, 0, 126, 0)
        self.assertFalse(np.array_equal(clouded, rotated))

    def test_noise_is_reproducible_independent_and_handles_resolution(self):
        effects = WeatherEffects(noise_std=2.0, glare_scale=0)
        frame = np.full((80, 100, 3), 80, np.uint8)
        left = effects.process(frame, None, 0, 0, 22, 0)
        right = effects.process(frame, None, 0, 0, 22, 1)
        left_again = effects.process(frame, None, 0, 0, 22, 0)
        np.testing.assert_array_equal(left, left_again)
        self.assertFalse(np.array_equal(left, right))
        larger = np.full((100, 120, 3), 80, np.uint8)
        self.assertEqual(effects.process(larger, None, 0, 0, 23, 0).shape,
                         larger.shape)


if __name__ == "__main__":
    unittest.main()
