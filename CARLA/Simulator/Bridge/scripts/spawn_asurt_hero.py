#!/usr/bin/env python3
"""Spawn/configure the ASURT hero with the simulator-matching Python API."""

import argparse
import time

import carla
import yaml


def load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    ccfg = cfg["carla"]
    wcfg = cfg["world"]
    vcfg = cfg["vehicle"]
    pcfg = vcfg.get("physics", {})
    tcfg = vcfg.get("transmission", {})

    client = carla.Client(ccfg.get("host", "127.0.0.1"), int(ccfg.get("port", 2000)))
    client.set_timeout(20.0)
    world = client.get_world()
    blueprint_id = vcfg["blueprint"]
    role_name = vcfg.get("role_name") or "hero"

    heroes = [
        actor
        for actor in world.get_actors().filter(blueprint_id)
        if actor.attributes.get("role_name") == role_name
    ]
    if len(heroes) > 1:
        raise RuntimeError(
            f"expected at most one {blueprint_id} role={role_name}, found {len(heroes)}"
        )

    if heroes:
        hero = heroes[0]
        print(f"Using existing hero id={hero.id}")
    else:
        blueprint = world.get_blueprint_library().find(blueprint_id)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", role_name)
        color = vcfg.get("color")
        if color and blueprint.has_attribute("color"):
            blueprint.set_attribute("color", str(color))

        coords = wcfg.get("spawn_point_coords")
        if coords:
            transform = carla.Transform(
                carla.Location(
                    x=float(coords.get("x", 0.0)),
                    y=float(coords.get("y", 0.0)),
                    z=float(coords.get("z", 0.5)),
                ),
                carla.Rotation(
                    roll=float(coords.get("roll", 0.0)),
                    pitch=float(coords.get("pitch", 0.0)),
                    yaw=float(coords.get("yaw", 0.0)),
                ),
            )
        else:
            spawn_points = world.get_map().get_spawn_points()
            if not spawn_points:
                raise RuntimeError("current map has no recommended spawn points")
            index = int(wcfg.get("spawn_point_index", 0))
            transform = spawn_points[index if 0 <= index < len(spawn_points) else 0]

        hero = world.try_spawn_actor(blueprint, transform)
        if hero is None:
            raise RuntimeError(f"failed to spawn {blueprint_id} at {transform}")
        print(f"Spawned hero id={hero.id} transform={transform}")

    if pcfg.get("enabled", False):
        physics = hero.get_physics_control()
        physics.mass = float(pcfg.get("mass", physics.mass))
        physics.drag_coefficient = float(
            pcfg.get("drag_coefficient", physics.drag_coefficient)
        )
        if pcfg.get("override_center_of_mass", False):
            center = pcfg.get("center_of_mass", {})
            physics.center_of_mass = carla.Location(
                x=float(center.get("x", 0.0)),
                y=float(center.get("y", 0.0)),
                z=float(center.get("z", 0.0)),
            )
        physics.max_rpm = float(pcfg.get("max_rpm", physics.max_rpm))
        physics.moi = float(pcfg.get("moi", physics.moi))
        physics.damping_rate_full_throttle = float(
            pcfg.get("damping_rate_full_throttle", physics.damping_rate_full_throttle)
        )
        physics.damping_rate_zero_throttle_clutch_engaged = float(
            pcfg.get(
                "damping_rate_zero_throttle_clutch_engaged",
                physics.damping_rate_zero_throttle_clutch_engaged,
            )
        )
        physics.damping_rate_zero_throttle_clutch_disengaged = float(
            pcfg.get(
                "damping_rate_zero_throttle_clutch_disengaged",
                physics.damping_rate_zero_throttle_clutch_disengaged,
            )
        )

        torque_curve = pcfg.get("torque_curve") or []
        if torque_curve:
            physics.torque_curve = [
                carla.Vector2D(float(rpm), float(torque))
                for rpm, torque in torque_curve
            ]
        steering_curve = pcfg.get("steering_curve") or []
        if steering_curve:
            physics.steering_curve = [
                carla.Vector2D(float(speed), float(fraction))
                for speed, fraction in steering_curve
            ]
        physics.use_sweep_wheel_collision = bool(
            pcfg.get("use_sweep_wheel_collision", physics.use_sweep_wheel_collision)
        )

        physics.use_gear_autobox = tcfg.get("type", "automatic") == "automatic"
        physics.gear_switch_time = float(
            tcfg.get("gear_switch_time", physics.gear_switch_time)
        )
        physics.clutch_strength = float(
            tcfg.get("clutch_strength", physics.clutch_strength)
        )
        physics.final_ratio = float(tcfg.get("final_ratio", physics.final_ratio))
        forward_gears = tcfg.get("forward_gears") or []
        if forward_gears:
            physics.forward_gears = [
                carla.GearPhysicsControl(
                    ratio=float(gear["ratio"]),
                    down_ratio=float(gear["down_ratio"]),
                    up_ratio=float(gear["up_ratio"]),
                )
                for gear in forward_gears
            ]

        wheel_cfg = pcfg.get("wheels") or []
        wheels = physics.wheels
        for index, wheel in enumerate(wheel_cfg[: len(wheels)]):
            wheels[index].tire_friction = float(
                wheel.get("tire_friction", wheels[index].tire_friction)
            )
            wheels[index].damping_rate = float(
                wheel.get("damping_rate", wheels[index].damping_rate)
            )
            wheels[index].max_steer_angle = float(
                wheel.get("max_steer_angle", wheels[index].max_steer_angle)
            )
            wheels[index].radius = float(wheel.get("radius", wheels[index].radius))
            wheels[index].max_brake_torque = float(
                wheel.get("max_brake_torque", wheels[index].max_brake_torque)
            )
            wheels[index].max_handbrake_torque = float(
                wheel.get(
                    "max_handbrake_torque", wheels[index].max_handbrake_torque
                )
            )

        drive_mode = str(vcfg.get("drive_mode", "AWD")).upper()
        if len(wheels) >= 4 and drive_mode == "FWD":
            wheels[2].tire_friction = 0.1
            wheels[3].tire_friction = 0.1
        elif len(wheels) >= 4 and drive_mode == "RWD":
            wheels[0].tire_friction = 0.1
            wheels[1].tire_friction = 0.1
        physics.wheels = wheels
        hero.apply_physics_control(physics)
        print(
            f"Applied physics mass={physics.mass} max_rpm={physics.max_rpm} "
            f"drive={drive_mode}"
        )

    time.sleep(1.0)
    persisted = world.get_actors().find(hero.id)
    if persisted is None or persisted.attributes.get("role_name") != role_name:
        raise RuntimeError("hero did not persist after spawn/physics setup")
    print(
        f"HERO_READY id={hero.id} role={role_name} type={hero.type_id} "
        f"transform={hero.get_transform()}"
    )


if __name__ == "__main__":
    main()
