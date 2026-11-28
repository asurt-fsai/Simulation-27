"""Cook the ADS-DV chassis against a measured UE asset audit and IMechE HiL model."""

import json
import math
import os
from pathlib import Path

import unreal


VEHICLE = "/Game/Carla/Blueprints/Vehicles/fs-ai/fasi_asu"
FRONT_WHEEL = "/Game/Carla/Blueprints/Vehicles/fs-ai/front_fsai_wheel"
REAR_WHEEL = "/Game/Carla/Blueprints/Vehicles/fs-ai/rear_fsai_wheel"
WHEELBASE_CM = 153.0
TRACK_CM = 120.0
FRONTAL_AREA_CM2 = 4900.0
ROLLING_RADIUS_CM = 100000.0 / (616 * math.tau)
# The packaged world runs fixed 33 ms frames. Six wheel steps bound each
# vehicle integration step to about 5.5 ms at both low and high speed.
VEHICLE_SUBSTEPS = 6


def model_value(values, key):
    return [float(value) for value in values[key].split()]


def model_fields(lines):
    return dict(tuple(part.strip() for part in line.split(" = ", 1)) for line in lines if " = " in line)


def reference_mass_properties(values):
    components = ["Body"] + [f"{kind}.{corner}" for kind in ("Wheel", "WheelCarrier")
                             for corner in ("fl", "fr", "rl", "rr")]
    masses = [model_value(values, f"{item}.mass")[0] for item in components]
    positions = [model_value(values, f"{item}.pos") for item in components]
    mass = sum(masses)
    cg = [sum(m * p[axis] for m, p in zip(masses, positions)) / mass for axis in range(3)]
    yaw = sum(model_value(values, f"{item}.I")[2] + m * ((p[0] - cg[0]) ** 2 + (p[1] - cg[1]) ** 2)
              for item, m, p in zip(components, masses, positions))
    rear = positions[components.index("Wheel.rl")][0]
    return mass, cg, yaw, cg[0] - rear


def table(lines, heading):
    start = next(i for i, line in enumerate(lines) if line.strip() == heading)
    for line in lines[start + 1:]:
        if line.strip() in ("", '""'):
            continue
        if not line.startswith((" ", "\t")):
            break
        yield [float(value) for value in line.split()]


def suspension_setup(vehicle_lines, values, kinematics_path, axle, quarter_mass):
    lines = kinematics_path.read_text().splitlines()
    fields = model_fields(lines)
    name = f"{axle}.Kin.0.L"
    compression = model_value(fields, f"{name}.Arg0")
    steering = model_value(fields, f"{name}.Arg1") if f"{name}.Arg1" in fields else [0]
    center = min(range(len(compression)), key=lambda i: abs(compression[i]))
    straight = min(range(len(steering)), key=lambda i: abs(steering[i]))
    # SKC columns 4 and 8 are vertical hub travel and spring length in metres.
    samples = {int(row[0]): row for row in table(lines, f"{name}.Data:") if int(row[1]) == straight}
    lower, upper = samples[center - 1], samples[center + 1]
    motion_ratio = abs((upper[8] - lower[8]) / (upper[4] - lower[4]))
    rate = model_value(values, f"{axle}.Spring")[0] * motion_ratio**2
    damping_slopes = []
    for direction in ("Push", "Pull"):
        speed, force = next(row[:2] for row in table(vehicle_lines, f"{axle}.Damp_{direction}:") if row[0] > 0)
        damping_slopes.append(force / speed)
    damping = sum(damping_slopes) / len(damping_slopes) * motion_ratio**2
    return {
        # Unreal stores angular natural frequency. Its PhysX setup uses k = m * omega**2.
        "suspension_natural_frequency": math.sqrt(rate / quarter_mass),
        "suspension_damping_ratio": damping / (2 * math.sqrt(rate * quarter_mass)),
        "suspension_max_raise": 100 * model_value(values, f"{axle}.Buf_Push.tz0")[0],
        "suspension_max_drop": -100 * model_value(values, f"{axle}.Buf_Pull.tz0")[0],
        # The asset uses the wheel center as the origin for the tire force application point.
        "suspension_force_offset": -ROLLING_RADIUS_CM,
    }


audit = json.loads(Path(os.environ["ASURT_FSAI_ASSET_AUDIT"]).read_text())
model = Path(os.environ["ASURT_FSAI_HIL_VEHICLE"])
model_lines = model.read_text().splitlines()
values = model_fields(model_lines)
mass, cg, yaw, cg_from_rear = reference_mass_properties(values)
asset = unreal.EditorAssetLibrary.load_asset(VEHICLE)
vehicle = unreal.get_default_object(unreal.EditorAssetLibrary.load_blueprint_class(VEHICLE))
movement = vehicle.get_editor_property("vehicle_movement")
diff = movement.get_editor_property("differential_setup")
diff.set_editor_property("differential_type", unreal.VehicleDifferential4W.OPEN_REAR_DRIVE)
movement.set_editor_property("differential_setup", diff)
movement.set_editor_property("mass", mass)
movement.set_editor_property("low_forward_speed_sub_step_count", VEHICLE_SUBSTEPS)
movement.set_editor_property("high_forward_speed_sub_step_count", VEHICLE_SUBSTEPS)
width = movement.get_editor_property("chassis_width")
movement.set_editor_property("chassis_height", FRONTAL_AREA_CM2 / width)
movement.set_editor_property("drag_coefficient", 0.467)
inertia = movement.get_editor_property("inertia_tensor_scale")
inertia.z = yaw * 10000.0 * audit["component_mass"] / (audit["inertia_kg_cm2"][2] * mass)
measurement_path = os.environ.get("ASURT_FSAI_YAW_MEASUREMENT")
if measurement_path:
    measurement = json.loads(Path(measurement_path).read_text())
    inertia.z = measurement["Scale"] * yaw / measurement["MeasuredKgM2"]
movement.set_editor_property("inertia_tensor_scale", inertia)

wheels = movement.get_editor_property("wheel_setups")
positions = [audit["wheels"][str(wheel.get_editor_property("bone_name"))] for wheel in wheels]
mid_x = sum(p[0] for p in positions) / len(positions)
mid_y = sum(p[1] for p in positions) / len(positions)
for i, (wheel, p) in enumerate(zip(wheels, positions)):
    x = mid_x + (WHEELBASE_CM / 2 if i < 2 else -WHEELBASE_CM / 2)
    y = mid_y + (-TRACK_CM / 2 if i % 2 == 0 else TRACK_CM / 2)
    other = positions[i ^ 1]
    z = (p[2] + other[2]) / 2
    wheel.set_editor_property("additional_offset", unreal.Vector(x - p[0], y - p[1], z - p[2]))
movement.set_editor_property("wheel_setups", wheels)

if not unreal.EditorAssetLibrary.save_loaded_asset(asset, only_if_is_dirty=False):
    raise RuntimeError("Could not save the ADS-DV chassis")

suspension = {}
for axle, wheel_path in (("SuspF", FRONT_WHEEL), ("SuspR", REAR_WHEEL)):
    relative = values[f"{axle}.Kin.0.FName"]
    kinematics = model.parent.parent / "Chassis" / relative
    setup = suspension_setup(model_lines, values, kinematics, axle, mass / len(wheels))
    wheel_class = unreal.EditorAssetLibrary.load_blueprint_class(wheel_path)
    wheel = unreal.get_default_object(wheel_class)
    for name, value in setup.items():
        wheel.set_editor_property(name, value)
    if axle == "SuspF":
        # CARLA's Ackermann conversion uses the wheel class steering limit.
        wheel.set_editor_property("steer_angle", 30.0)
    wheel_asset = unreal.EditorAssetLibrary.load_asset(wheel_path)
    if not unreal.EditorAssetLibrary.save_loaded_asset(wheel_asset, only_if_is_dirty=False):
        raise RuntimeError(f"Could not save the ADS-DV wheel {wheel_path}")
    suspension[axle] = setup
settings = {
    "reference_kg": mass,
    "reference_yaw_kg_m2": yaw,
    "reference_cg_height_m": cg[2],
    "reference_cg_from_rear_m": cg_from_rear,
    "reference_cg_from_front_m": WHEELBASE_CM / 100.0 - cg_from_rear,
    "yaw_asset_scale": inertia.z,
    "suspension": suspension,
    "runtime_com_nudge_m": [
        (mid_x - WHEELBASE_CM / 2 + cg_from_rear * 100 - audit["com"][0]) / 100,
        (mid_y - audit["com"][1]) / 100,
        (sum(p[2] for p in positions) / len(positions) - ROLLING_RADIUS_CM + cg[2] * 100 - audit["com"][2]) / 100,
    ],
}
Path(os.environ["ASURT_FSAI_AUTHOR_RECEIPT"]).write_text(json.dumps(settings, indent=2) + "\n")
unreal.log("ASURT_FSAI_CHASSIS_SAVED")
