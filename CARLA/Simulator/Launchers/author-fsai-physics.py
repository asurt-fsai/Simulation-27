"""Cook the ADS-DV chassis against a measured UE asset audit and IMechE HiL model."""

import json
import math
import os
from pathlib import Path

import unreal


VEHICLE = "/Game/Carla/Blueprints/Vehicles/fs-ai/fasi_asu"
FRONT_WHEEL = "/Game/Carla/Blueprints/Vehicles/fs-ai/front_fsai_wheel"
WHEELBASE_CM = 153.0
TRACK_CM = 120.0
FRONTAL_AREA_CM2 = 4900.0
ROLLING_RADIUS_CM = 100000.0 / (616 * math.tau)


def model_value(values, key):
    return [float(value) for value in values[key].split()]


def reference_mass_properties(path):
    values = dict(line.strip().split(" = ", 1) for line in path.read_text().splitlines() if " = " in line)
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


audit = json.loads(Path(os.environ["ASURT_FSAI_ASSET_AUDIT"]).read_text())
model = Path(os.environ["ASURT_FSAI_HIL_VEHICLE"])
mass, cg, yaw, cg_from_rear = reference_mass_properties(model)
asset = unreal.EditorAssetLibrary.load_asset(VEHICLE)
vehicle = unreal.get_default_object(unreal.EditorAssetLibrary.load_blueprint_class(VEHICLE))
movement = vehicle.get_editor_property("vehicle_movement")
diff = movement.get_editor_property("differential_setup")
diff.set_editor_property("differential_type", unreal.VehicleDifferential4W.OPEN_REAR_DRIVE)
movement.set_editor_property("differential_setup", diff)
movement.set_editor_property("mass", mass)
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
    wheel.set_editor_property("additional_offset", unreal.Vector(x - p[0], y - p[1], 0))
movement.set_editor_property("wheel_setups", wheels)

if not unreal.EditorAssetLibrary.save_loaded_asset(asset, only_if_is_dirty=False):
    raise RuntimeError("Could not save the ADS-DV chassis")

# CARLA's Ackermann controller gets its scale from the wheel class defaults.
# Keep its radian-to-normalized conversion aligned with the PhysX wheel limits.
front_asset = unreal.EditorAssetLibrary.load_asset(FRONT_WHEEL)
front_class = unreal.EditorAssetLibrary.load_blueprint_class(FRONT_WHEEL)
front_wheel = unreal.get_default_object(front_class)
front_wheel.set_editor_property("steer_angle", 30.0)
if not unreal.EditorAssetLibrary.save_loaded_asset(front_asset, only_if_is_dirty=False):
    raise RuntimeError("Could not save the ADS-DV front steering limits")
settings = {
    "reference_kg": mass,
    "reference_yaw_kg_m2": yaw,
    "reference_cg_height_m": cg[2],
    "reference_cg_from_rear_m": cg_from_rear,
    "reference_cg_from_front_m": WHEELBASE_CM / 100.0 - cg_from_rear,
    "yaw_asset_scale": inertia.z,
    "runtime_com_nudge_m": [
        (mid_x - WHEELBASE_CM / 2 + cg_from_rear * 100 - audit["com"][0]) / 100,
        (mid_y - audit["com"][1]) / 100,
        (sum(p[2] for p in positions) / len(positions) - ROLLING_RADIUS_CM + cg[2] * 100 - audit["com"][2]) / 100,
    ],
}
Path(os.environ["ASURT_FSAI_AUTHOR_RECEIPT"]).write_text(json.dumps(settings, indent=2) + "\n")
unreal.log("ASURT_FSAI_CHASSIS_SAVED")
