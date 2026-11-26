"""Apply a road-weather multiplier to the existing CARLA wheel physics."""

import math


def dry_grip_multiplier(tire_c):
    # Generic rubber response around an operating band. CARLA's map materials
    # still determine the underlying grip at each contact patch.
    cold = max(0.0, 25.0 - tire_c)
    hot = max(0.0, tire_c - 45.0)
    return round(max(0.74, 1.0 - 0.005 * cold - 0.004 * hot), 2)


def grip_multiplier(weather, tire_c=30.0):
    wetness = max(0.0, min(1.0, weather.wetness / 100.0))
    deposits = max(0.0, min(1.0, weather.precipitation_deposits / 100.0))
    return round((1.0 - 0.40 * max(wetness, deposits) - 0.10 * deposits)
                 * dry_grip_multiplier(tire_c), 2)


class WeatherGrip:
    def __init__(self, world, hero):
        self.world = world
        self.hero = hero
        self.multiplier = 1.0
        self.dry_friction = None
        self.last_friction = None

    def update(self, weather, tire_c):
        multiplier = grip_multiplier(weather, tire_c)
        dry = weather.wetness == 0.0 and weather.precipitation_deposits == 0.0
        # Applying wheel physics rebuilds the PhysX wheel state. Limit small
        # intermediate updates, but always restore the exact temperature-
        # adjusted dry baseline when the last film of water disappears.
        if multiplier == self.multiplier or (not dry and abs(multiplier - self.multiplier) < 0.019):
            return None
        # The existing manual ROS service and map-specific friction zones
        # also write the wheel values. Leave them in control while present.
        if self.world.get_actors().filter("static.trigger.friction"):
            return None

        physics = self.hero.get_physics_control()
        wheels = physics.wheels
        current = tuple(wheel.tire_friction for wheel in wheels)
        if self.last_friction is None or any(
                not math.isclose(a, b, rel_tol=1e-5, abs_tol=1e-6)
                for a, b in zip(current, self.last_friction)):
            self.dry_friction = current
        for wheel, dry in zip(wheels, self.dry_friction):
            wheel.tire_friction = dry * multiplier
        physics.wheels = wheels

        # CARLA recreates the vehicle physics on apply. Preserve its motion
        # just as the bridge does when updating live drag settings.
        linear = self.hero.get_velocity()
        angular = self.hero.get_angular_velocity()
        self.hero.apply_physics_control(physics)
        self.last_friction = tuple(wheel.tire_friction for wheel in wheels)
        self.multiplier = multiplier
        self.hero.set_target_velocity(linear)
        self.hero.set_target_angular_velocity(angular)
        return multiplier
