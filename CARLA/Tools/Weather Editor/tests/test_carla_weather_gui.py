"""Offline tests for CARLA Weather Studio."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

source = Path(__file__).resolve().parents[1] / "carla_weather_gui.py"
spec = importlib.util.spec_from_file_location("carla_weather_gui", source)
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)


class FakeWeather:
    sun_azimuth_angle = 220.0
    sun_altitude_angle = 45.0
    cloudiness = 10.0
    precipitation = 0.0
    precipitation_deposits = 0.0
    wetness = 0.0
    wind_intensity = 5.0
    fog_density = 2.0
    fog_distance = 0.75
    fog_falloff = 0.25
    dust_storm = 0.0
    mie_scattering_scale = 0.003996
    scattering_intensity = 1.0


class FakeWorld:
    def __init__(self):
        self.weather = FakeWeather()
        self.calls = 0

    def get_weather(self):
        return SimpleNamespace(**{key: getattr(self.weather, key)
             for key in dir(self.weather) if not key.startswith("_")})

    def set_weather(self, weather):
        self.calls += 1
        self.weather = weather


class WeatherLogicTests(unittest.TestCase):
    def test_fields_unique(self):
        self.assertEqual(len(gui.FIELDS), len(gui.BY_NAME))

    def test_clamping_and_finite_validation(self):
        self.assertEqual(gui.clean_value("cloudiness", -20), 0)
        self.assertEqual(gui.clean_value("cloudiness", 300), 100)
        self.assertEqual(gui.clean_value("sun_altitude_angle", 10.7), 11)
        self.assertEqual(gui.clean_value("fog_falloff", 1.12), 1.10)
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                gui.clean_value("cloudiness", bad)

    def test_unknown_field_cannot_mutate_world(self):
        w = FakeWorld()
        with self.assertRaises(ValueError):
            gui.set_weather_fields(w, {"car_controls": 100})
        self.assertEqual(w.calls, 0)

    def test_preserve_non_displayed_weather(self):
        w = FakeWorld()
        a = gui.set_weather_fields(w, {"sun_altitude_angle": 10, "cloudiness": 0})
        self.assertEqual(w.calls, 1)
        self.assertEqual(a["sun_altitude_angle"], 10)
        self.assertEqual(a["cloudiness"], 0)
        self.assertEqual(w.weather.sun_azimuth_angle, 220)
        self.assertAlmostEqual(w.weather.mie_scattering_scale, .003996)

    def test_read_only_empty_update(self):
        w = FakeWorld()
        gui.set_weather_fields(w, {})
        self.assertEqual(w.calls, 0)

    def test_camera_facing_preset(self):
        d = gui.scene_preset("sun_low", 38.54)
        self.assertEqual(gui.clean_value("sun_azimuth_angle", d["sun_azimuth_angle"]), 39)
        self.assertEqual(d["sun_altitude_angle"], 10)
        self.assertEqual(d["precipitation"], 0)

    def test_sun_behind_wraps(self):
        self.assertEqual(gui.scene_preset("sun_behind", 230)["sun_azimuth_angle"], 50)
        with self.assertRaises(ValueError):
            gui.scene_preset("sun_low", None)

    def test_adverse_conditions_presets(self):
        self.assertGreater(gui.scene_preset("heavy_rain", None)["precipitation"], 50)
        self.assertGreater(gui.scene_preset("dense_fog", None)["fog_density"], 50)
        with self.assertRaises(ValueError):
            gui.scene_preset("unknown", 45)


if __name__ == "__main__":
    unittest.main()
