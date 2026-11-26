"""Session-owned rain clock, road water and asphalt temperature model."""

from contextlib import contextmanager
import fcntl
import json
import math
from pathlib import Path


DEFAULT_AIR_C = 22.0
SETTINGS = "road-weather-settings.json"
STATUS = "road-weather-status.json"


def read_weather_file(directory, filename):
    try:
        return json.loads((Path(directory) / filename).read_text())
    except FileNotFoundError:
        return {}


def write_weather_file(directory, filename, values):
    target = Path(directory) / filename
    staged = target.with_suffix(".next")
    staged.write_text(json.dumps(values) + "\n")
    staged.replace(target)


@contextmanager
def weather_lock(directory):
    with (Path(directory) / "road-weather.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def percent(value):
    return max(0.0, min(100.0, value))


def approach(current, target, seconds, time_constant):
    if current is None:
        return target
    return target + (current - target) * math.exp(-seconds / time_constant)


def rain_cloud_cover(current, background, rainfall, seconds):
    rain = percent(rainfall) / 100.0
    target = background + (100.0 - background) * min(1.0, 0.45 + 0.55 * rain) if rain else background
    cover = approach(current, target, seconds, 5.0 if rain else 22.0)
    return target if abs(target - cover) < 0.05 else cover


def road_conditions(weather, air_c, asphalt_c, seconds):
    solar = max(0.0, math.sin(math.radians(weather.sun_altitude_angle))) * (1.0 - 0.86 * percent(weather.cloudiness) / 100)
    rain = percent(weather.precipitation) / 100.0
    target_c = air_c + 22.0 * solar - 5.0 * rain
    asphalt_c = approach(asphalt_c, target_c, seconds, 75.0)
    wet, water = weather.wetness, weather.precipitation_deposits
    drying = max(0.45, 1.1 + 4.3 * solar + 0.095 * (asphalt_c - 15) + 0.014 * weather.wind_intensity)
    if rain:
        wetting = 0.032 * rain
        # Evaporation competes with the incoming water film. The dry-road
        # rate is in percentage points per minute, converted here to 1/s.
        film_rate = wetting + drying / 6000.0
        wet = approach(wet, 100.0 * wetting / film_rate, seconds, 1.0 / film_rate)
        standing_target = max(0.0, rain - 0.28) * wet
        if water < standing_target:
            water = approach(water, standing_target, seconds, 45.0)
        else:
            water = max(standing_target, water - drying * seconds / 42.0)
    else:
        wet = max(0.0, wet - drying * seconds / 60.0)
        water = max(0.0, water - drying * seconds / 42.0)
    return asphalt_c, percent(wet), percent(water), drying


class RoadWeather:
    def __init__(self, directory):
        self.directory = directory
        self.elapsed = None
        self.asphalt_c = None
        self.tire_c = None
        self.last_cloudiness = None
        self.background_cloudiness = None
        self.previous_status = None

    def update(self, world):
        elapsed = world.get_snapshot().timestamp.elapsed_seconds
        seconds = 0.0 if self.elapsed is None else max(0.0, min(5.0, elapsed - self.elapsed))
        self.elapsed = elapsed
        with weather_lock(self.directory):
            weather = world.get_weather()
            settings = read_weather_file(self.directory, SETTINGS)
            air_c = settings.get("air_temperature_c", DEFAULT_AIR_C)
            end = settings.get("rain_ends_at")
            precipitation = weather.precipitation
            cloudiness = weather.cloudiness
            if self.last_cloudiness is None or abs(cloudiness - self.last_cloudiness) > 1e-4:
                self.background_cloudiness = percent(cloudiness)
            if end is not None and (elapsed >= end or not precipitation):
                settings.pop("rain_ends_at")
                write_weather_file(self.directory, SETTINGS, settings)
                weather.precipitation = 0.0
                end = None
            weather.cloudiness = rain_cloud_cover(cloudiness, self.background_cloudiness, weather.precipitation, seconds)
            self.last_cloudiness = weather.cloudiness
            self.asphalt_c, wet, water, drying = road_conditions(weather, air_c, self.asphalt_c, seconds)
            # Keep sub-percent changes in the CARLA state. Repeated rounding here
            # would quantize the drying rate or strand a small amount of water.
            if (abs(weather.wetness - wet) >= 1e-5 or abs(weather.precipitation_deposits - water) >= 1e-5
                    or (wet == 0.0 and weather.wetness != 0.0)
                    or (water == 0.0 and weather.precipitation_deposits != 0.0)
                    or abs(cloudiness - weather.cloudiness) >= 1e-5
                    or weather.precipitation != precipitation):
                weather.wetness, weather.precipitation_deposits = wet, water
                world.set_weather(weather)

        # Approximate rubber temperature from pavement contact and ambient air.
        # Rubber responds more slowly than the weather controls.
        self.tire_c = approach(self.tire_c, 0.30 * air_c + 0.70 * self.asphalt_c, seconds, 60.0)
        status = {"air_temperature_c": round(air_c, 1), "asphalt_temperature_c": round(self.asphalt_c, 1),
                  "tire_temperature_c": round(self.tire_c, 1),
                  "rain_remaining_s": max(0, math.ceil(end - elapsed)) if end is not None else None,
                  "drying_percent_per_minute": round(drying, 2)}
        if status != self.previous_status:
            write_weather_file(self.directory, STATUS, status)
            self.previous_status = status
        return weather
