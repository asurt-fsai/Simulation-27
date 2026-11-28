"""Offline tests for CARLA Weather Studio."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "Simulator/Launchers"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]
                       / "Modules/Perception/Adapters/CARLA/Camera/asurt_zed2i_sim"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]
                       / "Modules/Perception/Adapters/CARLA/LiDAR/asurt_velodyne_sim"))
import weather_control as gui
from asurt_velodyne_sim.weather import WeatherReturns
from asurt_zed2i_sim.weather import WeatherEffects, needs_scene_depth
from road_weather import RoadWeather, rain_cloud_cover, road_conditions
from weather_grip import WeatherGrip, dry_grip_multiplier, grip_multiplier


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
        self.elapsed = 0.0

    def get_weather(self):
        return SimpleNamespace(**{key: getattr(self.weather, key)
             for key in dir(self.weather) if not key.startswith("_")})

    def set_weather(self, weather):
        self.calls += 1
        self.weather = weather

    def get_snapshot(self):
        return SimpleNamespace(timestamp=SimpleNamespace(elapsed_seconds=self.elapsed))


class WeatherLogicTests(unittest.TestCase):
    def test_rain_never_rebuilds_physx_while_moving_or_spinning(self):
        world = FakeWorld()
        world.get_actors = Mock()
        world.get_actors.return_value.filter.return_value = []
        zero = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        hero = Mock()
        hero.get_velocity.return_value = SimpleNamespace(x=3.0, y=0.0, z=0.0)
        hero.get_angular_velocity.return_value = zero
        rotation = SimpleNamespace(pitch=0.0, roll=0.0)
        hero.get_transform.return_value = SimpleNamespace(rotation=rotation)
        hero.get_control.return_value = SimpleNamespace(throttle=0.0)
        wheels = [SimpleNamespace(tire_friction=0.7) for _ in range(4)]
        hero.get_physics_control.return_value = SimpleNamespace(wheels=wheels)
        grip = WeatherGrip(world, hero)
        rain = SimpleNamespace(wetness=100, precipitation_deposits=100)
        dry = SimpleNamespace(wetness=0, precipitation_deposits=0)
        grip.update(rain, 30)
        hero.get_velocity.return_value = zero
        rotation.roll = 50.0
        grip.update(rain, 30)
        hero.apply_physics_control.assert_not_called()
        rotation.roll = 0.0
        grip.update(rain, 30)
        hero.apply_physics_control.assert_not_called()
        world.elapsed = 1.0
        self.assertEqual(grip.update(rain, 30), 0.65)
        self.assertEqual(hero.apply_physics_control.call_count, 1)
        self.assertAlmostEqual(wheels[0].tire_friction, 0.455)
        hero.set_target_velocity.assert_not_called()
        hero.set_target_angular_velocity.assert_not_called()
        grip.update(dry, 30)
        world.elapsed = 2.0
        self.assertEqual(grip.update(dry, 30), 1.0)
        self.assertEqual([wheel.tire_friction for wheel in wheels], [0.7] * 4)
        wheels[0].tire_friction = 0.0
        grip.update(rain, 30)
        world.elapsed = 3.0
        self.assertIsNone(grip.update(rain, 30))
        self.assertEqual(hero.apply_physics_control.call_count, 2)

    def test_standing_water_reduces_grip_as_water_accumulates(self):
        weather = SimpleNamespace(wetness=100, precipitation_deposits=0)
        film_grip = grip_multiplier(weather, 30)
        weather.precipitation_deposits = 50
        partial_grip = grip_multiplier(weather, 30)
        weather.precipitation_deposits = 100
        self.assertEqual(film_grip, 0.72)
        self.assertGreater(film_grip, partial_grip)
        self.assertGreater(partial_grip, 0.65)
        self.assertEqual(grip_multiplier(weather, 30), 0.65)

    def test_fields_unique(self):
        self.assertEqual(len(gui.CONTROLS), len(gui.BY_NAME))

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
        self.assertLess(gui.scene_preset("light_rain", None)["precipitation"], 30)
        self.assertGreater(gui.scene_preset("dense_fog", None)["fog_density"], 50)
        with self.assertRaises(ValueError):
            gui.scene_preset("unknown", 45)

    def test_rain_presets_wet_the_road_at_their_selected_intensity(self):
        conditions = {}
        for preset, intensity_override in (("light_rain", {}), ("heavy_rain", {}),
                                           ("heavy_rain", {"precipitation": 15, "wind_intensity": 20})):
            world = FakeWorld()
            with tempfile.TemporaryDirectory() as directory:
                road = RoadWeather(directory)
                road.update(world)
                gui.set_weather_fields(world, {**gui.scene_preset(preset, None), **intensity_override}, directory)
                self.assertEqual(world.weather.wetness, 0)
                self.assertEqual(world.weather.precipitation_deposits, 0)
                for world.elapsed in range(1, 61):
                    road.update(world)
                conditions[(preset, bool(intensity_override))] = (
                    world.weather.wetness, world.weather.precipitation_deposits,
                )
        light, light_water = conditions[("light_rain", False)]
        heavy, heavy_water = conditions[("heavy_rain", False)]
        self.assertGreater(light, 10)
        self.assertGreater(heavy, light * 2.5)
        self.assertEqual(light_water, 0)
        self.assertGreater(heavy_water, 10)
        self.assertEqual(conditions[("heavy_rain", True)], (light, light_water))

    def test_wetness_and_fog_are_rendered_before_camera_processing(self):
        effects = WeatherEffects(glare_scale=0, noise_std=0)
        weather = SimpleNamespace(wetness=90, precipitation_deposits=40, fog_density=90)
        image = np.random.default_rng(0).integers(0, 256, (180, 320, 3), dtype=np.uint8)
        np.testing.assert_array_equal(effects.process(image, weather, 0, 0, 1, 0), image)

    def test_dust_veil_strengthens_with_intensity(self):
        effects = WeatherEffects(glare_scale=0, noise_std=0)
        image = np.full((180, 320, 3), 140, dtype=np.uint8)
        mild, heavy = (effects.process(image, SimpleNamespace(dust_storm=dust), 0, 0, 1, 0)
                       for dust in (30, 85))
        self.assertGreater(heavy[0, 0, 2], mild[0, 0, 2])
        self.assertLess(heavy[0, 0, 0], mild[0, 0, 0])
        self.assertEqual(image[0, 0, 2], 140)

    def test_fog_and_dust_reduce_distant_cone_contrast_during_rain(self):
        image = np.zeros((18, 32, 3), dtype=np.uint8)
        image[:, :16] = (240, 30, 30)
        image[:, 16:] = (240, 30, 30)
        depth = np.full((18, 32), 3, dtype=np.float32)
        depth[:, 16:] = 45
        effects = WeatherEffects(glare_scale=0, noise_std=0, droplets_scale=0)
        for field in ("fog_density", "dust_storm"):
            def contrast(intensity):
                conditions = SimpleNamespace(precipitation=85, fog_distance=5, **{field: intensity})
                output = effects.process(image, conditions, 0, 0, 1, 0, depth_m=depth)
                return tuple(int(output[10, x, 0]) - int(output[10, x, 2]) for x in (4, 27))
            near_mild, far_mild = contrast(30)
            near_heavy, far_heavy = contrast(85)
            self.assertGreater(near_heavy, far_heavy)
            self.assertGreater(far_mild, far_heavy + 70)
            self.assertGreater(near_mild, 150)

    def test_older_editable_camera_still_dries_without_using_depth(self):
        effects = WeatherEffects(glare_scale=0, noise_std=0)
        rain = SimpleNamespace(precipitation=85, wetness=90)
        clear = SimpleNamespace(precipitation=0, sun_altitude_angle=70)
        image = np.full((180, 320, 3), 120, np.uint8)
        self.assertFalse(needs_scene_depth(rain))
        effects.process(image, rain, 0, 0, 10, 0, 110, None, None)
        effects.process(image, clear, 0, 0, 100, 0, 110)
        self.assertTrue(effects.lens_is_wet(0))
        dried = effects.process(image, clear, 0, 0, 3000, 0, 110)
        np.testing.assert_array_equal(dried, image)

    def test_velodyne_rain_loses_weak_targets_and_adds_small_radial_noise(self):
        # Two ranges and bearings let us distinguish scatter from true targets.
        near = np.tile(np.array([8, 6, 0, 0.3], dtype=np.float32), (25_000, 1))
        far = np.tile(np.array([30, 0, 40, 0.1], dtype=np.float32), (25_000, 1))
        cloud = np.concatenate((near, far))
        weather = lambda precipitation: WeatherReturns.from_world(SimpleNamespace(precipitation=precipitation))
        self.assertIs(weather(0).apply(cloud, 17), cloud)
        light, heavy = (weather(rain).apply(cloud, 17) for rain in (15, 90))
        light_near, heavy_near = (scan[scan[:, 1] > 5] for scan in (light, heavy))
        self.assertGreater(len(heavy_near), len(near) * 0.9)
        self.assertLess(np.count_nonzero(heavy[:, 2] > 30), np.count_nonzero(light[:, 2] > 30))
        self.assertLess(np.median(heavy_near[:, 3]), np.median(light_near[:, 3]))
        self.assertGreater(heavy_near[:, 0].std(), light_near[:, 0].std() * 3)
        self.assertLess(np.abs(heavy_near[:, 0] - 8).max(), 0.12)
        np.testing.assert_allclose(heavy_near[:, 0] * 0.75, heavy_near[:, 1], atol=1e-6)
        scatter = (heavy[:, 1] == 0) & (heavy[:, 2] < 16)
        self.assertGreater(np.count_nonzero(scatter), 0)
        self.assertLess(np.count_nonzero(scatter), len(cloud) * 0.001)

    def test_camera_droplets_persist_after_rain_and_dry_faster_in_the_sun(self):
        clear = SimpleNamespace(precipitation=0, cloudiness=0, sun_altitude_angle=70)
        cloudy = SimpleNamespace(precipitation=0, cloudiness=95, sun_altitude_angle=70)
        rain = SimpleNamespace(precipitation=85, cloudiness=90, sun_altitude_angle=70)
        image = np.full((180, 320, 3), 120, np.uint8)

        def after_sixteen_seconds(weather):
            effects = WeatherEffects(glare_scale=0, noise_std=0)
            effects.process(image, rain, 0, 0, 30, 0, timestamp=1.0)
            after_rain = effects.process(image, weather, 0, 0, 510, 0, timestamp=17.0)
            self.assertTrue(effects.lens_is_wet(0))
            self.assertFalse(effects.lens_is_wet(1))
            dried = effects.process(image, weather, 0, 0, 3000, 0, timestamp=100.0)
            np.testing.assert_array_equal(dried, image)
            self.assertFalse(effects.lens_is_wet(0))
            return np.count_nonzero(after_rain != image)

        sunny_drops = after_sixteen_seconds(clear)
        self.assertGreater(sunny_drops, 0)
        self.assertGreater(after_sixteen_seconds(cloudy), sunny_drops)

    def test_timed_rain_gathers_clouds_and_wet_road_survives_clearing(self):
        world = FakeWorld()
        with tempfile.TemporaryDirectory() as directory:
            weather = RoadWeather(directory)
            weather.update(world)
            gui.set_weather_fields(world, {"rain_duration_s": 10, "precipitation": 90}, directory)
            for world.elapsed in range(1, 10):
                weather.update(world)
            storm = world.weather.cloudiness
            self.assertGreater(storm, 40)
            self.assertLessEqual(storm, 60)
            world.elapsed = 10
            weather.update(world)
            self.assertEqual(world.weather.precipitation, 0)
            self.assertGreater(world.weather.wetness, 15)
            self.assertLess(world.weather.cloudiness, storm)
            gui.set_weather_fields(world, gui.scene_preset("clear_noon", None), directory)
            world.elapsed = 11
            weather.update(world)
            self.assertEqual(world.weather.cloudiness, 0)
            self.assertGreater(world.weather.wetness, 15)

    def test_storm_cloud_limit_preserves_explicit_overcast(self):
        self.assertEqual(rain_cloud_cover(92, 92, 85, 30), 92)

    def test_warmer_sunnier_road_dries_faster_and_restores_thermal_grip(self):
        weather = FakeWorld().get_weather()
        weather.wetness, weather.precipitation_deposits = 12, 5
        cold = road_conditions(weather, 5, None, 60)
        hot = road_conditions(weather, 35, None, 60)
        weather.sun_altitude_angle = 0
        night = road_conditions(weather, 35, None, 60)
        self.assertLess(hot[1], cold[1])
        self.assertLess(hot[1], night[1])
        self.assertLess(grip_multiplier(weather, 12), dry_grip_multiplier(12))
        weather.wetness, weather.precipitation_deposits = hot[1:3]
        partly_dry = grip_multiplier(weather, 12)
        _, weather.wetness, weather.precipitation_deposits, _ = road_conditions(weather, 35, None, 180)
        self.assertEqual(weather.wetness, 0)
        self.assertEqual(weather.precipitation_deposits, 0)
        self.assertGreater(grip_multiplier(weather, 12), partly_dry)
        self.assertEqual(grip_multiplier(weather, 12), dry_grip_multiplier(12))
        self.assertGreater(grip_multiplier(weather, 32), grip_multiplier(weather, 12))
        self.assertGreater(grip_multiplier(weather, 32), grip_multiplier(weather, 58))

    def test_clear_rain_drains_the_road_and_cancels_timed_rain(self):
        world = FakeWorld()
        with tempfile.TemporaryDirectory() as directory:
            gui.set_weather_fields(world, {"rain_duration_s": 60, "wetness": 90,
                                           "precipitation_deposits": 70, "air_temperature_c": 19}, directory)
            applied = gui.set_weather_fields(world, gui.scene_preset("clear_rain", None), directory)
            self.assertEqual([applied[k] for k in ("precipitation", "wetness", "precipitation_deposits")], [0, 0, 0])
            self.assertEqual(applied["cloudiness"], 10)
            thermal = gui.thermal_status(directory, 0)
            self.assertIsNone(thermal["rain_remaining_s"])
            self.assertEqual(thermal["air_temperature_c"], 19)

    def test_start_rain_without_timer_cancels_an_existing_countdown(self):
        world = FakeWorld()
        with tempfile.TemporaryDirectory() as directory:
            gui.set_weather_fields(world, {"rain_duration_s": 60, "wetness": 80}, directory)
            applied = gui.set_weather_fields(world, {"rain_duration_s": 0, "precipitation": 45}, directory)
            self.assertEqual(applied["precipitation"], 45)
            self.assertEqual(applied["wetness"], 80)
            self.assertIsNone(gui.thermal_status(directory, 0)["rain_remaining_s"])
            applied = gui.set_weather_fields(world, {"rain_duration_s": 0}, directory)
            self.assertEqual(applied["precipitation"], 0)


if __name__ == "__main__":
    unittest.main()
