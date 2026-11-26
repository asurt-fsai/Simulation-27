"""Shared live CARLA weather controls for the editor, System Manager, and MCP."""

from __future__ import annotations

import math
from contextlib import nullcontext

# name, display label, min, max, resolution, units
FIELDS = (
    ("sun_azimuth_angle", "Sun azimuth", 0, 360, 1, "°"),
    ("sun_altitude_angle", "Sun altitude", -90, 90, 1, "°"),
    ("cloudiness", "Cloud cover", 0, 100, 1, "%"),
    ("precipitation", "Rainfall", 0, 100, 1, "%"),
    ("precipitation_deposits", "Standing water", 0, 100, 1, "%"),
    ("wetness", "Road wetness", 0, 100, 1, "%"),
    ("wind_intensity", "Wind", 0, 100, 1, "%"),
    ("fog_density", "Fog density", 0, 100, 1, "%"),
    ("fog_distance", "Fog start distance", 0, 200, 0.5, "m"),
    ("fog_falloff", "Fog height falloff", 0, 10, 0.05, ""),
    ("dust_storm", "Dust storm", 0, 100, 1, "%"),
)
TEMPERATURE = ("air_temperature_c", "Air temperature", -20, 65, 1, "°C")
RAIN_DURATION = ("rain_duration_s", "Rain duration", 0, 3600, 1, "s")
CONTROLS = (*FIELDS, TEMPERATURE, RAIN_DURATION)
BY_NAME = {item[0]: item for item in CONTROLS}
LIGHT_FIELDS = ("sun_azimuth_angle", "sun_altitude_angle", "cloudiness")
ATMOS_FIELDS = tuple(x[0] for x in FIELDS if x[0] not in LIGHT_FIELDS)

def clean_value(name: str, raw: float) -> float:
    """Validate and quantize one allowed CARLA WeatherParameters control."""
    if name not in BY_NAME:
        raise ValueError("Unknown weather parameter: " + name)
    _, _, lower, upper, resolution, _ = BY_NAME[name]
    value = float(raw)
    if not math.isfinite(value):
        raise ValueError("Weather values must be finite")
    value = min(upper, max(lower, value))
    steps = round((value - lower) / resolution)
    return round(lower + steps * resolution, 5)


def weather_values(weather) -> dict[str, float]:
    return {name: float(getattr(weather, name))
            for name, *_ in FIELDS if hasattr(weather, name)}


def set_weather_fields(world, updates: dict[str, float], directory=None) -> dict[str, float]:
    """Preserve unedited CARLA weather settings; only set explicitly touched fields."""
    checked = {key: clean_value(key, value) for key, value in updates.items()}
    temperature = checked.pop("air_temperature_c", None)
    duration = checked.pop("rain_duration_s", None)
    if directory:
        from road_weather import SETTINGS, read_weather_file, weather_lock, write_weather_file
    elif temperature is not None or duration is not None:
        raise ValueError("Temperature and timed rain need an active managed ASURT session.")
    with weather_lock(directory) if directory else nullcontext():
        current = world.get_weather()
        settings = read_weather_file(directory, SETTINGS) if directory else {}
        previous = settings.copy()
        if temperature is not None:
            settings["air_temperature_c"] = temperature
        if duration is not None and duration > 0.0:
            settings["rain_ends_at"] = world.get_snapshot().timestamp.elapsed_seconds + duration
            checked.setdefault("precipitation", current.precipitation or 75.0)
            if checked["precipitation"] <= 0.0:
                raise ValueError("Timed rain needs a positive rainfall intensity.")
        elif duration == 0.0:
            settings.pop("rain_ends_at", None)
            checked.setdefault("precipitation", 0.0)
        if checked.get("precipitation") == 0.0:
            settings.pop("rain_ends_at", None)
        for key, value in checked.items():
            setattr(current, key, value)
        if checked:
            world.set_weather(current)
        if settings != previous:
            write_weather_file(directory, SETTINGS, settings)
    return weather_values(world.get_weather())


def thermal_status(directory, elapsed=None):
    if not directory:
        return {}
    from road_weather import DEFAULT_AIR_C, SETTINGS, STATUS, read_weather_file

    status = read_weather_file(directory, STATUS)
    settings = read_weather_file(directory, SETTINGS)
    status["air_temperature_c"] = settings.get("air_temperature_c", DEFAULT_AIR_C)
    end = settings.get("rain_ends_at")
    if end is None:
        status["rain_remaining_s"] = None
    elif elapsed is not None:
        status["rain_remaining_s"] = max(0, math.ceil(end - elapsed))
    return status


CAMERA_PRESETS = ("sun_low", "sun_high", "sun_behind")
PRESETS = (*CAMERA_PRESETS, "clear_noon", "overcast", "light_rain", "heavy_rain", "clear_rain",
           "dense_fog", "dust", "wet_road")


def scene_preset(name: str, heading: float | None) -> dict[str, float]:
    """Return changed fields only. Heading comes from the live ZED left camera."""
    stop_rain = dict(precipitation=0)
    sunny = dict(stop_rain, cloudiness=0, fog_density=0, fog_distance=0, dust_storm=0)
    if name in CAMERA_PRESETS:
        if heading is None:
            raise ValueError("Camera heading unavailable. Check that the ZED stereo camera is streaming.")
        sunny["sun_azimuth_angle"] = (heading + (180 if name == "sun_behind" else 0)) % 360
        sunny["sun_altitude_angle"] = 10 if name != "sun_high" else 35
        return sunny
    presets = {
        "clear_noon": dict(sunny, sun_altitude_angle=70),
        "overcast": dict(stop_rain, cloudiness=90, fog_density=6, fog_distance=0, dust_storm=0),
        "light_rain": dict(precipitation=15, wind_intensity=20, fog_density=0, fog_distance=0, dust_storm=0),
        "heavy_rain": dict(precipitation=85, wind_intensity=65, fog_density=0, fog_distance=0, dust_storm=0),
        "clear_rain": dict(stop_rain, wetness=0, precipitation_deposits=0),
        "dense_fog": dict(stop_rain, cloudiness=70,
                          fog_density=85, fog_distance=0, fog_falloff=2.0,
                          dust_storm=0),
        "dust": dict(stop_rain, cloudiness=20, fog_density=12, fog_distance=0,
                     dust_storm=85, wind_intensity=70),
        "wet_road": dict(cloudiness=60, precipitation=0, precipitation_deposits=40,
                         wetness=85, fog_density=0, fog_distance=0, dust_storm=0),
    }
    if name not in presets:
        raise ValueError("Unknown preset: " + name)
    return presets[name]


def camera_heading(world):
    cameras = [actor for actor in world.get_actors().filter("sensor.camera.rgb")
               if actor.attributes.get("role_name") == "asurt_zed2i_sim_left"]
    return float(cameras[0].get_transform().rotation.yaw) if len(cameras) == 1 else None


def read_status(world, map_name=None, directory=None):
    from weather_grip import dry_grip_multiplier, grip_multiplier

    weather = world.get_weather()
    actors = world.get_actors()
    heroes = [a for a in actors.filter("vehicle.vehicle.asurt_fsai") if a.attributes.get("role_name") == "hero"]
    configured, effective = None, None
    if len(heroes) == 1:
        hero = heroes[0]
        configured = [round(wheel.tire_friction, 4) for wheel in hero.get_physics_control().wheels]
        telemetry = hero.get_telemetry_data()
        if telemetry.speed > 0.25 and len(telemetry.wheels) == 4:
            effective = [round(wheel.tire_friction, 4) for wheel in telemetry.wheels]
    thermal = thermal_status(directory, world.get_snapshot().timestamp.elapsed_seconds if directory else None)
    tire_c = thermal.get("tire_temperature_c", 30.0)
    multiplier = grip_multiplier(weather, tire_c)
    return {"map": map_name or world.get_map().name, "weather": weather_values(weather), "road": thermal,
            "road_grip": {"weather_multiplier": multiplier,
                          "dry_temperature_multiplier": dry_grip_multiplier(tire_c),
                          "manual_or_map_override": bool(actors.filter("static.trigger.friction")),
                          "wheel_order": ["FL", "FR", "RL", "RR"],
                          "configured": configured, "effective_while_rolling": effective}}
