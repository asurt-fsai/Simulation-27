#!/usr/bin/env python3
"""Preview or load a Formula Student cone CSV/JSON into the running CARLA world.

Does not change maps, weather, simulation settings, actors other than its own
tagged props, or advance the world clock. Use preview before physical spawning.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any
from uuid import uuid4


# Preserve the existing actor ownership register in the bridge.  A changed
# default path would lose track of already spawned cones across GUI upgrades.
ASURT_ROOT = Path(os.environ.get("ASURT_ROOT", Path(__file__).resolve().parents[2]))
BRIDGE = ASURT_ROOT / "Simulation/CARLA/Bridge/Carla_ASU_Bridge"
if not BRIDGE.is_dir():
    BRIDGE = ASURT_ROOT / "Carla_ASU_Bridge"  # legacy migration fallback
STATE_FILE = BRIDGE / "scripts" / ".cone_track_actors.json"
COLORS = ("blue", "yellow", "orange", "orange_big", "unknown")
GENERIC_PROPS = {
    "blue": "static.prop.trafficcone01",
    "yellow": "static.prop.trafficcone02",
    "orange": "static.prop.constructioncone",
    "orange_big": "static.prop.constructioncone",
    "unknown": "static.prop.constructioncone",
}
# These IDs become available only AFTER the ASURTCones package is imported,
# cooked and registered in a matching CARLA source build.
CUSTOM_CONE_PROPS = {
    "blue": "static.prop.asurtbluecone",
    "yellow": "static.prop.asurtyellowcone",
    "orange": "static.prop.asurtorangecone",
    "orange_big": "static.prop.asurtbigorangecone",
}
# WEMAS cone sizes and masses from the FSG 2026 Event Handbook, table 2.
# The cooked meshes already have these dimensions and simple collision hulls.
CONE_PHYSICS = {
    "static.prop.asurtbluecone": ("ASURTBlueCone", "0.45"),
    "static.prop.asurtyellowcone": ("ASURTYellowCone", "0.45"),
    "static.prop.asurtorangecone": ("ASURTOrangeCone", "0.45"),
    "static.prop.asurtbigorangecone": ("ASURTBigOrangeCone", "1.05"),
}


@dataclass(frozen=True)
class Cone:
    color: str
    x: float
    y: float
    z: float = 0.0


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or value is None:
        raise ValueError(f"Invalid {field}: {value!r}")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {field}: {value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"Non-finite {field}: {value!r}")
    return number


def _cone(color: Any, coords: Any, index: int) -> Cone:
    tag = str(color).strip().lower()
    if tag not in COLORS:
        raise ValueError(f"Cone {index}: unsupported color {color!r}; supported: {', '.join(COLORS)}")
    if isinstance(coords, dict):
        if "position" in coords and "x" not in coords:
            coords = coords["position"]
        else:
            coords = [coords.get("x"), coords.get("y"), coords.get("z", 0.0)]
    if not isinstance(coords, (tuple, list)) or len(coords) not in (2, 3):
        raise ValueError(f"Cone {index}: position must have x/y and optional z")
    x, y = (_number(value, f"cone {index} {axis}") for axis, value in zip("xy", coords[:2]))
    z = _number(coords[2], f"cone {index} z") if len(coords) == 3 else 0.0
    return Cone(tag, x, y, z)


def load_cones(filename: str | Path, max_cones: int = 1000) -> list[Cone]:
    """CSV: tag,x,y[,z]. JSON: array, {cones:[...]}, {blue:[...], yellow:[...]}.

    JSON records accept tag/color plus x,y,z or position: [x,y,z].
    Parallel JSON arrays {"x":[...], "y":[...], "color":[...]} are also
    supported, including orange_big and unknown labels in FSD datasets.
    Units must be metres in a local coordinate frame.
    """
    path = Path(filename).expanduser()
    if not path.is_file():
        raise ValueError(f"Track file does not exist: {path}")
    if path.suffix.lower() == ".csv":
        with path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                raise ValueError("CSV is empty")
            reader.fieldnames = [name.strip().lower() for name in reader.fieldnames]
            if not {"tag", "x", "y"}.issubset(reader.fieldnames):
                raise ValueError("CSV must have tag,x,y columns (optional z)")
            cones = [
                _cone(row["tag"], [row["x"], row["y"], row.get("z") or 0.0], idx)
                for idx, row in enumerate(reader, 1)
            ]
    elif path.suffix.lower() == ".json":
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
        if isinstance(data, dict) and data.get("units", "m") not in ("m", "meters", "metres"):
            raise ValueError("JSON coordinates must be in metres")
        if isinstance(data, dict) and all(key in data for key in ("x", "y", "color")):
            arrays = [data[key] for key in ("x", "y", "color")]
            if any(not isinstance(array, list) for array in arrays):
                raise ValueError("Parallel JSON x, y and color fields must all be lists")
            if len({len(array) for array in arrays}) != 1:
                raise ValueError("Parallel JSON x, y and color arrays have different lengths")
            cones = [_cone(color, [x, y], idx) for idx, (x, y, color)
                     in enumerate(zip(*arrays), 1)]
        else:
            cones = None
        if cones is None and isinstance(data, dict) and "cones" in data:
            data = data["cones"]
        if cones is not None:
            pass
        elif isinstance(data, list):
            cones = []
            for idx, record in enumerate(data, 1):
                if not isinstance(record, dict):
                    raise ValueError(f"Cone {idx}: JSON array items must be objects")
                cones.append(_cone(record.get("tag", record.get("color")), record, idx))
        elif isinstance(data, dict):
            cones = []
            for color, keys in (("blue", ("blue", "blue_cones", "left_cones")),
                                ("yellow", ("yellow", "yellow_cones", "right_cones")),
                                ("orange_big", ("orange_big", "orange_big_cones")),
                                ("orange", ("orange", "orange_cones")),
                                ("unknown", ("unknown", "unknown_cones"))):
                for key in keys:
                    if key in data:
                        if not isinstance(data[key], list):
                            raise ValueError(f"{key} must be a list of coordinates")
                        for record in data[key]:
                            cones.append(_cone(color, record, len(cones) + 1))
                        break
        else:
            raise ValueError("JSON must be a cone array or an object containing cones/blue/yellow")
    else:
        raise ValueError("Track file must end in .csv or .json")
    if not cones:
        raise ValueError("No cones found. Expected CSV tag,x,y or JSON cones / x,y,color arrays")
    if len(cones) > max_cones:
        raise ValueError(f"{len(cones)} cones exceeds --max-cones {max_cones}")
    span_x = max(c.x for c in cones) - min(c.x for c in cones)
    span_y = max(c.y for c in cones) - min(c.y for c in cones)
    if max(span_x, span_y) > 1000:
        raise ValueError("Track exceeds 1000 m span; check CSV units (expected metres)")
    return cones


def print_summary(cones: list[Cone]) -> None:
    blue = sum(c.color == "blue" for c in cones)
    yellow = sum(c.color == "yellow" for c in cones)
    other = {color: sum(c.color == color for c in cones) for color in COLORS[2:]}
    extra = "".join(f", {count} {color}" for color, count in other.items() if count)
    print(f"Loaded {len(cones)} cone positions: {blue} blue, {yellow} yellow{extra} (metres)")
    if other["unknown"]:
        print("WARNING: unknown labels are retained as unclassified cones; review before physical spawning.")
    print(f"Local X: {min(c.x for c in cones):.2f} .. {max(c.x for c in cones):.2f} m")
    print(f"Local Y: {min(c.y for c in cones):.2f} .. {max(c.y for c in cones):.2f} m")


def transform_xy(x: float, y: float, origin_x: float, origin_y: float,
                 yaw_degrees: float, invert_y: bool = False) -> tuple[float, float]:
    if invert_y:
        y = -y
    theta = math.radians(yaw_degrees)
    return (origin_x + math.cos(theta) * x - math.sin(theta) * y,
            origin_y + math.sin(theta) * x + math.cos(theta) * y)


def track_start(cones: list[Cone], invert_y: bool = False) -> tuple[float, float, float, float]:
    """First paired blue/yellow gate: center x/y, heading (degrees), width.

    Cone rows must be ordered around the track in driving direction. In CARLA
    coordinates blue is to the vehicle's left and yellow to its right.
    """
    blue = [c for c in cones if c.color == "blue"]
    yellow = [c for c in cones if c.color == "yellow"]
    if len(blue) < 2 or len(yellow) < 2:
        raise ValueError("At least two ordered blue and two ordered yellow cones are needed to locate the start")

    def xy(c):
        return c.x, -c.y if invert_y else c.y

    b0, b1 = xy(blue[0]), xy(blue[1])
    y0, y1 = xy(yellow[0]), xy(yellow[1])
    side = (y0[0] - b0[0], y0[1] - b0[1])
    width = math.hypot(*side)
    if not (2.7 <= width <= 15.0):
        raise ValueError(f"First blue/yellow pair is {width:.2f} m apart; expected a single starting gate (about 3-10 m)")

    def unit(dx, dy):
        magnitude = math.hypot(dx, dy)
        if magnitude < 0.2:
            raise ValueError("The first two cones of each color must have distinct, ordered positions")
        return (dx / magnitude, dy / magnitude)

    bt = unit(b1[0] - b0[0], b1[1] - b0[1])
    yt = unit(y1[0] - y0[0], y1[1] - y0[1])
    if bt[0] * yt[0] + bt[1] * yt[1] < 0.3:
        raise ValueError("Blue and yellow cone rows disagree on driving direction; reorder the first cones")
    tx, ty = unit(bt[0] + yt[0], bt[1] + yt[1])
    # In CARLA's left-handed XY system, the right boundary lies to the
    # positive cross-product side of the vehicle's forward direction.
    cross = tx * side[1] - ty * side[0]
    if cross < 0.35 * width:
        raise ValueError("Blue cones are on the right of the driving direction. Toggle 'Mirror Y' or reverse the cone order")
    return (0.5 * (b0[0] + y0[0]), 0.5 * (b0[1] + y0[1]),
            math.degrees(math.atan2(ty, tx)), width)


def suggested_invert_y(cones: list[Cone]) -> bool:
    """Choose image-to-CARLA handedness from an ordered start gate."""
    try:
        track_start(cones, False)
        return False
    except ValueError:
        track_start(cones, True)
        return True


def car_start_transform(cones, args, carla, world):
    local_x, local_y, local_yaw, width = track_start(cones, args.invert_y)
    if args.anchor == "hero":
        hero = find_hero(world)
        if hero is None:
            raise RuntimeError("No hero is available as a moving anchor. Choose --anchor coords for a stable course origin.")
        anchor = hero.get_transform()
        ox, oy, oz = anchor.location.x, anchor.location.y, anchor.location.z
        yaw = anchor.rotation.yaw + args.yaw
    else:
        if args.origin_x is None or args.origin_y is None:
            raise ValueError("Car placement requires --origin-x and --origin-y with --anchor coords")
        ox, oy, oz, yaw = args.origin_x, args.origin_y, args.origin_z, args.yaw
    ox, oy = transform_xy(args.forward_offset, args.side_offset, ox, oy, yaw)
    theta = math.radians(local_yaw)
    local_x += args.start_forward * math.cos(theta)
    local_y += args.start_forward * math.sin(theta)
    x, y = transform_xy(local_x, local_y, ox, oy, yaw)
    if args.ground == "project":
        z = ground_z(carla, world, x, y, oz, args) + args.car_z_offset
    else:
        z = oz + args.car_z_offset
    rotation = carla.Rotation(yaw=yaw + local_yaw)
    return carla.Transform(carla.Location(x=x, y=y, z=z), rotation), width


def ground_z(carla, world, x: float, y: float, anchor_z: float, args) -> float:
    """Project from just above the course elevation so roofs aren't mistaken for asphalt."""
    probe = carla.Location(x=x, y=y, z=anchor_z + args.ray_height)
    result = world.ground_projection(probe, max(80.0, args.ray_height * 2.0))
    if result is None:
        raise RuntimeError(
            f"No ground near course Z={anchor_z:.2f} at x={x:.2f} y={y:.2f}; "
            "negative coordinates are valid. The terrain ray found no surface "
            "here; move/rotate the course onto collidable terrain."
        )
    z = result.location.z
    if abs(z - anchor_z) > args.max_height_delta:
        raise RuntimeError(
            f"Suspicious ground elevation at ({x:.2f},{y:.2f}): "
            f"{z:.2f} m vs origin {anchor_z:.2f} m. "
            "Track may cross a building or elevated surface; move the origin."
        )
    return z


def start_overlaps_cone(hero, target, cone_actor, margin: float = 0.15,
                        *, body_bbox=None, cone_bbox=None) -> bool:
    """Conservative 2-D collision check using the actual car/cone bounding boxes.

    The old fixed 1.5 m centre-distance threshold falsely rejected a car
    centred in a valid 3 m start gate: each cone is exactly 1.5 m away and
    CARLA float rounding makes that fractionally smaller. It also missed
    obstacles ahead of the long vehicle nose. Check the full oriented
    vehicle footprint expanded by the cone's enclosing radius instead.
    """
    body = body_bbox if body_bbox is not None else hero.bounding_box
    cone = cone_bbox if cone_bbox is not None else cone_actor.bounding_box
    yaw = math.radians(target.rotation.yaw)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    car_x = target.location.x + cos_yaw * body.location.x - sin_yaw * body.location.y
    car_y = target.location.y + sin_yaw * body.location.x + cos_yaw * body.location.y

    cone_tf = cone_actor.get_transform()
    box_yaw = yaw + math.radians(body.rotation.yaw)
    if abs(cone_tf.rotation.pitch) > 5 or abs(cone_tf.rotation.roll) > 5:
        # A knocked-over cone's tip can extend well beyond its upright base.
        longitudinal, lateral = [], []
        for vertex in cone.get_world_vertices(cone_tf):
            dx, dy = vertex.x - car_x, vertex.y - car_y
            longitudinal.append(math.cos(box_yaw) * dx + math.sin(box_yaw) * dy)
            lateral.append(-math.sin(box_yaw) * dx + math.cos(box_yaw) * dy)
        return (min(longitudinal) < body.extent.x + margin
                and max(longitudinal) > -body.extent.x - margin
                and min(lateral) < body.extent.y + margin
                and max(lateral) > -body.extent.y - margin)

    cone_yaw = math.radians(cone_tf.rotation.yaw)
    cone_x = (cone_tf.location.x + math.cos(cone_yaw) * cone.location.x
              - math.sin(cone_yaw) * cone.location.y)
    cone_y = (cone_tf.location.y + math.sin(cone_yaw) * cone.location.x
              + math.cos(cone_yaw) * cone.location.y)
    # The vehicle's bounding box can also have an orientation relative to
    # the vehicle root. Cone radius encloses either cone mesh orientation.
    dx, dy = cone_x - car_x, cone_y - car_y
    longitudinal = math.cos(box_yaw) * dx + math.sin(box_yaw) * dy
    lateral = -math.sin(box_yaw) * dx + math.cos(box_yaw) * dy
    outside_x = max(0.0, abs(longitudinal) - body.extent.x)
    outside_y = max(0.0, abs(lateral) - body.extent.y)
    cone_radius = math.hypot(cone.extent.x, cone.extent.y)
    return math.hypot(outside_x, outside_y) < cone_radius + margin


def validate_car_start(world, hero, tf, args) -> None:
    """Fail before moving the hero when a managed cone intersects its footprint."""
    if not args.state.exists():
        return
    state = _read_state(args.state)
    managed = owned_actors(world, state, args)
    # Actor.bounding_box is a synchronous CARLA RPC, unlike the cached actor
    # transform. Querying vehicle and cone boxes for EACH of 470 managed
    # props took 94s on the i7/GTX1050 host. All cones created from the same
    # mesh at the same scale have the same local bounds. Fetch these once per
    # mesh so checking a full course does not make a synchronous RPC per cone.
    body_bbox = hero.bounding_box if managed else None
    cone_bboxes = {}
    for actor in managed:
        mesh = (actor.type_id, actor.attributes.get("mesh_path"), actor.attributes.get("scale"))
        if mesh not in cone_bboxes:
            cone_bboxes[mesh] = actor.bounding_box
        if start_overlaps_cone(hero, tf, actor, body_bbox=body_bbox,
                               cone_bbox=cone_bboxes[mesh]):
            raise RuntimeError(
                f"Car start footprint overlaps cone actor {actor.id} ({actor.type_id}). "
                "Adjust 'Car start offset (m)' to move only the car along the lane, "
                "or correct the track geometry. The course origin moves the cones too."
            )
    print(f"Start footprint clearance verified against {len(managed)} managed cones.")


def place_hero(cones, args, carla, world, *, preview: bool = False):
    tf, width = car_start_transform(cones, args, carla, world)
    print(f"Track start: x={tf.location.x:.3f} y={tf.location.y:.3f} "
          f"z={tf.location.z:.3f} yaw={tf.rotation.yaw:.2f} lane={width:.2f} m")
    hero = find_hero(world)
    if preview:
        if hero is not None:
            validate_car_start(world, hero, tf, args)
        else:
            print("No hero exists yet; physical car footprint clearance cannot be checked.")
        origin = carla.Location(x=tf.location.x, y=tf.location.y, z=tf.location.z + 0.7)
        theta = math.radians(tf.rotation.yaw)
        end = carla.Location(x=origin.x + 3 * math.cos(theta),
                             y=origin.y + 3 * math.sin(theta), z=origin.z)
        world.debug.draw_arrow(origin, end, thickness=0.12, arrow_size=0.25,
                               color=carla.Color(0, 255, 110),
                               life_time=args.marker_seconds, persistent_lines=False)
        world.debug.draw_string(origin, "FSAI START", color=carla.Color(0, 255, 110),
                                life_time=args.marker_seconds, persistent_lines=False)
        print("Car start previewed as a green arrow; hero has not been moved.")
        return

    if hero is not None and hero.type_id != "vehicle.vehicle.asurt_fsai":
        raise RuntimeError(f"Existing role_name=hero is {hero.type_id}; refusing to alter it")
    if hero is None:
        if args.port != 2000 or args.host not in ("127.0.0.1", "localhost"):
            raise RuntimeError("Automatic FSAI spawning uses the local native server on 2000. Start the matching hero on other servers first.")
        config = args.state.with_name("carla_interface_config.yaml")
        if not config.is_file():
            config = BRIDGE / "config" / "carla_interface_config.yaml"
        print("Creating missing FSAI hero with the existing ASURT physics configuration...")
        result = subprocess.run([sys.executable, str(BRIDGE / "scripts" / "spawn_asurt_hero.py"),
                                 "--config", str(config)], capture_output=True, text=True, timeout=40)
        if result.returncode:
            raise RuntimeError(f"FSAI spawn helper failed: {result.stderr[-900:]} {result.stdout[-900:]}")
        hero = find_hero(world)
        if hero is None:
            raise RuntimeError("FSAI spawn helper returned but no hero actor was found")

    if math.hypot(hero.get_velocity().x, hero.get_velocity().y) > 0.5:
        raise RuntimeError("Stop the FSAI car before relocating it")

    # Check the actual car footprint (not an arbitrary radius around its
    # root) against managed cone geometry before any car-control mutation.
    validate_car_start(world, hero, tf, args)

    old = hero.get_transform()
    hero.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    zero = carla.Vector3D(0, 0, 0)
    hero.set_target_velocity(zero)
    hero.set_target_angular_velocity(zero)
    hero.set_transform(tf)
    hero.set_target_velocity(zero)
    hero.set_target_angular_velocity(zero)
    print(f"Positioned existing FSAI hero id={hero.id} from ({old.location.x:.2f},"
          f"{old.location.y:.2f}) to track start; ROS sensors and physics configuration retained.")


def connect(args: argparse.Namespace):
    import carla  # Import only when a live CARLA operation is requested.

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    print(f"Connected to CARLA {args.host}:{args.port}; map={world.get_map().name}; "
          f"synchronous={world.get_settings().synchronous_mode}")
    return carla, client, world


def find_hero(world):
    # A new client can expose an empty frame-zero cache on an already running server.
    if world.get_snapshot().frame == 0:
        world.wait_for_tick(5.0)
    heroes = [a for a in world.get_actors().filter("vehicle.*")
              if a.attributes.get("role_name") == "hero"]
    if len(heroes) > 1:
        raise RuntimeError(f"Found {len(heroes)} hero actors; cannot select an anchor")
    return heroes[0] if heroes else None


def placed_cones(cones: list[Cone], args: argparse.Namespace, carla, world, blueprints=None):
    hero = find_hero(world)
    if args.anchor == "hero":
        if hero is None:
            raise RuntimeError("No vehicle with role_name=hero found. Start the FSAI bridge or use --anchor coords")
        tf = hero.get_transform()
        ox, oy, oz = tf.location.x, tf.location.y, tf.location.z
        yaw = tf.rotation.yaw + args.yaw
    else:
        if args.origin_x is None or args.origin_y is None:
            raise ValueError("--anchor coords requires --origin-x and --origin-y")
        ox, oy, oz = args.origin_x, args.origin_y, args.origin_z
        yaw = args.yaw
    ox, oy = transform_xy(args.forward_offset, args.side_offset, ox, oy, yaw)
    print(f"Anchor=({ox:.2f}, {oy:.2f}, {oz:.2f}); yaw={yaw:.2f} deg")
    placed = []
    invalid = []
    for index, cone in enumerate(cones, 1):
        x, y = transform_xy(cone.x, cone.y, ox, oy, yaw, args.invert_y)
        z_offset = args.z_offset
        if z_offset is None:
            # The cooked ASURT meshes have their origin at the center of the base.
            is_asurt = blueprints is not None and blueprints[cone.color].id == CUSTOM_CONE_PROPS.get(cone.color)
            z_offset = 0.002 if is_asurt else 0.12
        if args.ground == "project":
            try:
                z = ground_z(carla, world, x, y, oz, args) + z_offset + cone.z
            except RuntimeError as exc:
                # Keep every location in the preview so users can see the
                # failed section. Never use the estimate for physical spawn.
                estimated = carla.Location(x=x, y=y, z=oz + z_offset + cone.z)
                invalid.append((index, cone, estimated, str(exc)))
                continue
        else:
            z = oz + z_offset + cone.z
        location = carla.Location(x=x, y=y, z=z)
        if hero is not None and args.mode == "spawn":
            h = hero.get_location()
            if math.hypot(h.x - x, h.y - y) < args.hero_clearance:
                raise RuntimeError(f"Cone {index} within {args.hero_clearance:.2f} m of hero. "
                                   "Use --forward-offset to relocate the track")
        placed.append((cone, location))
    return placed, invalid


def search_ground_placement(cones: list[Cone], args: argparse.Namespace, carla, world):
    """Find a nearby, *fully projected* candidate; never move/spawn actors.

    The search is bounded, sampled first for speed, then completely checked.
    Ground projection proves collision at the candidate XY, not road legality.
    """
    if args.anchor != "coords" or args.origin_x is None or args.origin_y is None:
        raise ValueError("Ground placement search requires --anchor coords with origin X/Y")
    if args.ground != "project":
        raise ValueError("Ground placement search requires --ground project")
    if not cones:
        raise ValueError("A nonempty track is required")

    step = args.search_step
    radius = args.search_radius
    # Enumerate the nearest world translations first; retain the original
    # forward/side offsets so the GUI only needs to update X/Y/Yaw.
    shifts = [(dx, dy) for dx in range(-radius, radius + 1, step)
              for dy in range(-radius, radius + 1, step)
              if math.hypot(dx, dy) <= radius + 1e-6]
    shifts.append((0, 0))
    shifts = sorted(set(shifts), key=lambda shift: (math.hypot(*shift), abs(shift[0]) + abs(shift[1]), shift))
    rotations = (0, 90, -90, 180)
    # Do not exhaust a small search budget entirely at the centre: cover both
    # nearby positions and the requested outer radius.
    translation_budget = max(1, args.search_candidates // len(rotations))
    if len(shifts) > translation_budget:
        near_count = min(5, max(1, translation_budget // 3))
        chosen = shifts[:near_count]
        remaining = shifts[near_count:]
        far_count = translation_budget - len(chosen)
        if far_count:
            chosen += [remaining[round(k * (len(remaining) - 1) / max(1, far_count - 1))]
                       for k in range(far_count)]
        shifts = chosen
    positions = [(dx, dy, rotation) for dx, dy in shifts for rotation in rotations]
    positions.sort(key=lambda item: (math.hypot(item[0], item[1]),
                                     abs(item[2]), item[0], item[1]))
    positions = positions[:args.search_candidates]
    # Representative distributed positions, not only the first few cones.
    count = min(12, len(cones))
    sample = sorted({round(k * (len(cones) - 1) / max(1, count - 1))
                     for k in range(count)})
    hero = find_hero(world)
    hero_position = hero.get_location() if hero is not None else None

    def probe(cone, origin_x, origin_y, yaw):
        anchor_x, anchor_y = transform_xy(args.forward_offset, args.side_offset,
                                          origin_x, origin_y, yaw)
        x, y = transform_xy(cone.x, cone.y, anchor_x, anchor_y, yaw, args.invert_y)
        if hero_position is not None and math.hypot(hero_position.x - x, hero_position.y - y) < args.hero_clearance:
            return None
        try:
            z = ground_z(carla, world, x, y, args.origin_z, args)
        except RuntimeError:
            return None
        return z

    screened = []
    for dx, dy, rotation in positions:
        x = args.origin_x + dx
        y = args.origin_y + dy
        yaw = args.yaw + rotation
        passes = sum(probe(cones[i], x, y, yaw) is not None for i in sample)
        screened.append((passes, dx, dy, rotation))
    screened.sort(key=lambda item: (-item[0], math.hypot(item[1], item[2]), abs(item[3])))

    verified = 0
    best = None
    # A few full verifications; no result can be marked 'found' from sampling.
    for sample_passes, dx, dy, rotation in screened[:args.search_finalists]:
        if best is not None and best["invalid"] == 0:
            break
        x = args.origin_x + dx
        y = args.origin_y + dy
        yaw = args.yaw + rotation
        failed = [i + 1 for i, cone in enumerate(cones)
                  if probe(cone, x, y, yaw) is None]
        verified += 1
        result = {
            "found": not failed, "invalid": len(failed),
            "total": len(cones), "origin_x": x, "origin_y": y,
            "yaw": yaw, "forward_offset": args.forward_offset,
            "sample_passed": sample_passes, "sample_total": len(sample),
            "invalid_indices": failed,
        }
        if best is None or (result["invalid"], math.hypot(dx, dy), abs(rotation)) < (
                best["invalid"], math.hypot(best["origin_x"] - args.origin_x,
                                             best["origin_y"] - args.origin_y),
                abs(best["yaw"] - args.yaw)):
            best = result
    if best is None:
        raise RuntimeError("No placement candidates were checked")
    best["candidates_sampled"] = len(screened)
    best["candidates_verified"] = verified
    return best


def draw_preview(world, carla, placed, seconds: float, prefix: str = "", invalid=()) -> None:
    """Show short-lived, thin guide ticks only in explicit preview mode.

    CARLA's draw_point and draw_string primitives can appear as large,
    camera-facing yellow/white squares rather than useful markers. Use
    short line segments so the real cones remain visually distinguishable.
    """
    palette = {
        "blue": carla.Color(30, 90, 255),
        "yellow": carla.Color(255, 220, 0),
        "orange": carla.Color(255, 135, 0),
        "orange_big": carla.Color(255, 100, 0),
        "unknown": carla.Color(165, 165, 165),
    }
    for cone, pos in placed:
        color = palette[cone.color]
        start = carla.Location(x=pos.x, y=pos.y, z=pos.z + 0.03)
        end = carla.Location(x=pos.x, y=pos.y, z=pos.z + 0.28)
        world.debug.draw_line(start, end, thickness=0.025,
                              color=color, life_time=seconds, persistent_lines=False)
    for index, cone, estimated, reason in invalid:
        z = estimated.z + 1.5
        red = carla.Color(255, 30, 30)
        for xsign in (-1, 1):
            start = carla.Location(x=estimated.x - 0.16, y=estimated.y - xsign * 0.16, z=z)
            end = carla.Location(x=estimated.x + 0.16, y=estimated.y + xsign * 0.16, z=z)
            world.debug.draw_line(start, end, thickness=0.035, color=red,
                                  life_time=seconds, persistent_lines=False)
    print(f"Thin preview guide lines displayed for {seconds:g} seconds (no collision).")
    if invalid:
        print(f"WARNING: GROUND CHECK FAILED for {len(invalid)}/{len(placed) + len(invalid)} cones.")
        for index, cone, estimated, reason in invalid[:8]:
            print(f"  #{index} {cone.color}: local ({cone.x:.2f}, {cone.y:.2f}) "
                  f"world ({estimated.x:.2f}, {estimated.y:.2f}) — {reason}")
        print("INVALID_INDICES: " + ",".join(str(index) for index, *_ in invalid))
        print("Red markers are estimated at origin elevation; they are not verified ground locations. "
              "Adjust X/Y/Yaw/Forward shift and preview again before physical spawning.")
        print("Negative local/world X, Y and Z coordinates are supported. A red marker means "
              "there was no acceptable surface at that world location, not a sign error.")


def _write_state(path: Path, state: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def _read_state(path: Path) -> dict:
    if not path.is_file():
        raise RuntimeError(f"No managed cone track found at {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _live_actor_snapshot(world, args: argparse.Namespace):
    """Never interpret an uninitialized CARLA client cache as an empty map.

    Fresh CARLA clients can return frame 0 and *no* actors even when dozens of
    tagged physical cones are alive. Only wait for another process's tick;
    never advance CARLA synchronously from the importer.
    """
    frame = world.get_snapshot().frame
    if frame == 0:
        try:
            # CARLA's Boost.Python binding accepts this timeout positionally.
            world.wait_for_tick(min(float(getattr(args, "timeout", 10.0)), 20.0))
        except Exception as exc:
            raise RuntimeError(
                "CARLA actor snapshot was uninitialized (frame 0) and no "
                "external tick arrived. Ownership is unknown; no actors "
                "or saved state were changed."
            ) from exc
        frame = world.get_snapshot().frame
        if frame == 0:
            raise RuntimeError(
                "CARLA still reports frame 0. Ownership cannot be verified; "
                "no actors or saved state were changed."
            )
    return list(world.get_actors())


def _tagged_cones(actors):
    return [a for a in actors if a.type_id.startswith("static.prop.") and
            a.attributes.get("role_name", "").startswith("asurt_cone_")]


def inspect_managed_ownership(world, state: dict, args: argparse.Namespace):
    """Verify both saved IDs and their tags across a live actor snapshot."""
    if (state.get("host"), state.get("port")) != (args.host, args.port):
        raise RuntimeError("State belongs to a different server/port; use the original connection")
    if state.get("map") != world.get_map().name:
        raise RuntimeError("World map has changed since spawning this track; refusing actor deletion")
    live_actors = _live_actor_snapshot(world, args)
    by_id = {actor.id: actor for actor in live_actors}
    found, missing, reused = [], [], []
    for saved in state.get("actors", []):
        actor_id = int(saved["id"])
        role = saved["role_name"]
        if not role.startswith("asurt_cone_") or not saved["type_id"].startswith("static.prop."):
            raise RuntimeError("Invalid cone ownership record; refusing to trust saved actor IDs")
        actor = by_id.get(actor_id)
        if actor is None:
            missing.append(actor_id)
            continue
        if actor.type_id != saved["type_id"] or actor.attributes.get("role_name") != role:
            reused.append((actor_id, actor.type_id))
        else:
            found.append(actor)
    confirmed_owned_ids = {actor.id for actor in found}
    # A different active importer token must also block automatic replacement:
    # otherwise a lost register could lead to overlapping physical tracks.
    untracked = [a for a in live_actors if a.id not in confirmed_owned_ids and
                 a.attributes.get("role_name", "").startswith("asurt_cone_")]
    return found, missing, reused, untracked


def recover_exact_archived_state(world, state_path: Path, args: argparse.Namespace) -> bool:
    """Restore only a unique, complete archive matching EVERY live tagged cone.

    A previous frame-zero bug could archive active ownership while all actors
    were still present. Never adopt arbitrary or partial actors, never delete
    by role prefix, and keep the archived audit file for diagnosis.
    """
    if state_path.exists():
        return False
    live = _live_actor_snapshot(world, args)
    tagged = _tagged_cones(live)
    if not tagged:
        return False
    matching = []
    for archived in sorted(state_path.parent.glob(state_path.name + ".stale-*.json")):
        try:
            saved = _read_state(archived)
            recorded = saved.get("actors", [])
            if not recorded or len(recorded) != len(tagged):
                continue
            owned, missing, reused, untracked = inspect_managed_ownership(world, saved, args)
            if (not missing and not reused and not untracked and
                    len(owned) == len(tagged) and
                    {a.id for a in owned} == {a.id for a in tagged}):
                matching.append((archived, saved))
        except (ValueError, RuntimeError, KeyError, TypeError, OSError):
            continue
    if len(matching) != 1:
        raise RuntimeError(
            f"Found {len(tagged)} live tagged physical cones without an active "
            f"ownership file. Matching archived records: {len(matching)}. "
            "Automatic recovery requires exactly one complete ID/type/role, "
            "map and server match; no actors were changed."
        )
    archived, saved = matching[0]
    restored = dict(saved)
    restored["recovered_from"] = str(archived)
    _write_state(state_path, restored)
    print(f"OWNERSHIP_RECOVERED: exactly {len(tagged)} live tagged cones matched "
          f"{archived}. Active record restored to {state_path}; the audit "
          "archive was preserved.")
    return True


def owned_actors(world, state: dict, args: argparse.Namespace):
    owned, missing, reused, untracked = inspect_managed_ownership(world, state, args)
    if untracked or (owned and reused):
        raise RuntimeError(
            f"Ambiguous cone ownership: {len(owned)} tagged cones remain, "
            f"{len(reused)} actor IDs were reused and {len(untracked)} saved tags "
            "occur at unexpected IDs. No actors were changed."
        )
    if not owned and reused:
        print(f"WARNING: {len(reused)} saved cone IDs now belong to other actors; "
              "none will be touched.")
    return owned


def archive_stale_state(world, state_path: Path, args: argparse.Namespace) -> bool:
    """Archive only when no recorded cones or matching tags exist in CARLA."""
    if not state_path.exists():
        return False
    state = _read_state(state_path)
    owned, missing, reused, untracked = inspect_managed_ownership(world, state, args)
    if owned or untracked:
        if reused or untracked:
            raise RuntimeError(
                f"Managed track is mixed/ambiguous: {len(owned)} owned, "
                f"{len(reused)} recycled IDs, {len(untracked)} unexpected cone tags. "
                "No actors removed; inspect manually before replacing the track."
            )
        return False
    archive = state_path.with_name(
        f"{state_path.name}.stale-{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}.json"
    )
    state_path.rename(archive)
    print(f"STALENESS_RECOVERED: archived obsolete cone state to {archive} "
          f"({len(missing)} missing, {len(reused)} reused by unrelated actors). "
          "No CARLA actors changed.")
    return True


def clear_managed(world, state_path: Path, args: argparse.Namespace) -> None:
    if not state_path.exists():
        if not recover_exact_archived_state(world, state_path, args):
            print("No live managed cones found. No CARLA actors changed.")
            return
    if archive_stale_state(world, state_path, args):
        return
    state = _read_state(state_path)
    actors = owned_actors(world, state, args)  # Check ALL ownership before deleting anything.
    for actor in actors:
        if not actor.destroy():
            raise RuntimeError(f"CARLA did not confirm destruction of actor {actor.id}; state preserved")
    state_path.unlink()
    print(f"Removed {len(actors)} owned physical cones. Existing vehicles and unrelated actors unchanged.")


def choose_blueprints(cones: list[Cone], args: argparse.Namespace, world):
    explicit = {
        "blue": args.blue_blueprint, "yellow": args.yellow_blueprint,
        "orange": args.orange_blueprint,
        "orange_big": args.orange_big_blueprint, "unknown": None,
    }
    library = world.get_blueprint_library()
    required = {c.color for c in cones}
    blueprints = {}
    for color in sorted(required):
        preferred = explicit[color] or CUSTOM_CONE_PROPS.get(color)
        name = preferred
        if not preferred:
            if not args.allow_neutral_props:
                raise RuntimeError(f"Unclassified cone type {color!r} has no custom model. "
                                   "Classify or remove it before physical spawn; "
                                   "use --allow-neutral-props only for a deliberate neutral fallback.")
            name = GENERIC_PROPS[color]
        else:
            try:
                library.find(preferred)
            except (IndexError, RuntimeError) as exc:
                if explicit[color] or not args.allow_neutral_props:
                    raise RuntimeError(
                        f"Custom {color} cone blueprint {preferred!r} is not on this CARLA server. "
                        "Cook/import the ASURTCones package into this compatible CARLA build "
                        "and restart the server. No props spawned."
                    ) from exc
                name = GENERIC_PROPS[color]
                print(f"WARNING: Custom {color} model {preferred} unavailable. "
                      f"Explicit --allow-neutral-props selects {name} instead.")
        if not name.startswith("static.prop."):
            raise RuntimeError(f"Expected a static.prop blueprint, got {name!r}")
        try:
            blueprint = library.find(name)
        except (IndexError, RuntimeError) as exc:
            raise RuntimeError(f"Blueprint {name!r} is not available in this CARLA package") from exc
        if not blueprint.has_attribute("role_name") or not blueprint.get_attribute("role_name").is_modifiable:
            raise RuntimeError(f"Blueprint {name!r} cannot be tagged for safe cleanup")
        blueprints[color] = blueprint
        print(f"Selected cone model {color}: {name}")
    return blueprints


def spawn_cones(blueprints, placed, args, carla, world, state_path: Path) -> None:
    weighted = None
    if any(model.id in CONE_PHYSICS for model in blueprints.values()):
        weighted = world.get_blueprint_library().find("static.prop.mesh")
        for attribute in ("mesh_path", "mass", "scale", "role_name"):
            if not weighted.has_attribute(attribute) or not weighted.get_attribute(attribute).is_modifiable:
                raise RuntimeError(f"CARLA cannot set {attribute} on the dynamic cone blueprint")
        weighted.set_attribute("scale", "1.0")
    if state_path.exists():
        if not archive_stale_state(world, state_path, args):
            raise RuntimeError(f"A live managed track state exists: {state_path}. Run --mode clear first.")
    else:
        tagged = _tagged_cones(_live_actor_snapshot(world, args))
        if tagged:
            raise RuntimeError(
                f"{len(tagged)} live importer-tagged cones have no active state "
                "record. Use --mode clear to recover the exact archived "
                "ownership before spawning a new track."
            )
    token = uuid4().hex[:12]
    state = {"version": 1, "host": args.host, "port": args.port, "map": world.get_map().name,
             "source": str(Path(args.track).expanduser().resolve()),
             "created_at": datetime.now(timezone.utc).isoformat(), "actors": [], "map_ready": False}
    _write_state(state_path, state)
    try:
        for cone, pos in placed:
            blueprint = blueprints[cone.color]
            physical = CONE_PHYSICS.get(blueprint.id)
            if physical:
                mesh_name, mass = physical
                blueprint = weighted
                blueprint.set_attribute(
                    "mesh_path", f"/Game/ASURTCones/Static/Static/{mesh_name}/{mesh_name}.{mesh_name}"
                )
                blueprint.set_attribute("mass", mass)
            role = f"asurt_cone_{token}_{cone.color}"
            blueprint.set_attribute("role_name", role)
            transform = carla.Transform(pos, carla.Rotation())
            actor = world.try_spawn_actor(blueprint, transform)
            if actor is None:
                raise RuntimeError(f"CARLA refused to spawn {cone.color} cone at {pos}")
            state["actors"].append({"id": actor.id, "type_id": actor.type_id,
                                    "role_name": role, "color": cone.color,
                                    "position": [pos.x, pos.y, pos.z]})
            _write_state(state_path, state)
            if not physical:
                # Custom and neutral models use their own cooked mass.
                actor.set_simulate_physics(True)
        state["map_ready"] = True
        _write_state(state_path, state)
    except Exception:
        print(f"Spawn interrupted; {len(state['actors'])} tagged actors recorded in {state_path}.", file=sys.stderr)
        raise
    print(f"Spawned {len(state['actors'])} managed physical props. State: {state_path}")
    # Physical cones are visible themselves. Do not also draw 100 debug
    # billboards on top: they look like large yellow/white floating boxes.
    if args.allow_neutral_props:
        print("WARNING: Neutral-fallback mode enabled. Verify actual physical colors; "
              "generic cones do not match Formula Student blue/yellow markings.")
    print("Use --mode status to check actors; --mode clear to remove ONLY this tool's tagged cones.")


def command_line(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("track", nargs="?", help="Path to FS Trackdraw CSV or cone JSON")
    p.add_argument("--mode", choices=("inspect", "preview", "spawn", "find-placement", "status", "clear", "blueprints",
                                      "car-preview", "place-car"), default="inspect")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=10.0)
    p.add_argument("--anchor", choices=("hero", "coords"), default="hero")
    p.add_argument("--origin-x", type=float)
    p.add_argument("--origin-y", type=float)
    p.add_argument("--origin-z", type=float, default=0.0)
    p.add_argument("--yaw", type=float, default=0.0, help="Additional degrees after hero yaw, or absolute degrees for coords anchor")
    p.add_argument("--invert-y", action="store_true", help="Flip local Y axis if layout orientation needs reversing")
    p.add_argument("--forward-offset", type=float, default=0.0, help="Shift layout from anchor in its forward direction (m)")
    p.add_argument("--side-offset", type=float, default=0.0, help="Shift layout right of its forward direction (m)")
    p.add_argument("--ground", choices=("project", "fixed"), default="project")
    p.add_argument("--ray-height", type=float, default=3.0, help="Ground probe starts this many metres above course origin; keep below nearby roofs")
    p.add_argument("--max-height-delta", type=float, default=8.0, help="Reject projected surfaces more than this many metres from course origin elevation")
    p.add_argument("--z-offset", type=float,
                   help="Height above ground (m). Defaults to 0.002 for ASURT props and 0.12 otherwise.")
    p.add_argument("--hero-clearance", type=float, default=1.5, help="Minimum distance of spawned cone from existing hero (m)")
    p.add_argument("--start-forward", type=float, default=0.0, help="Meters forward from first paired cone gate for car start")
    p.add_argument("--car-z-offset", type=float, default=0.12, help="Initial FSAI vehicle origin above projected ground (m)")
    p.add_argument("--marker-seconds", type=float, default=120.0)
    p.add_argument("--max-cones", type=int, default=1000)
    p.add_argument("--blue-blueprint", help="Verified CARLA static.prop blueprint ID for blue cones")
    p.add_argument("--yellow-blueprint", help="Verified CARLA static.prop blueprint ID for yellow cones")
    p.add_argument("--orange-blueprint", help="Verified CARLA static.prop blueprint ID for orange cones")
    p.add_argument("--orange-big-blueprint", help="Verified CARLA static.prop blueprint ID for large orange cones")
    p.add_argument("--allow-neutral-props", action="store_true",
                   help="Deliberately permit generic cone fallback when custom ASURT blueprints are absent")
    p.add_argument("--search-radius", type=int, default=80, help="Maximum ground search distance from origin in world XY metres")
    p.add_argument("--search-step", type=int, default=20, help="Ground search translation grid spacing in metres")
    p.add_argument("--search-candidates", type=int, default=60, help="Maximum placements to sample before full checking")
    p.add_argument("--search-finalists", type=int, default=5, help="Maximum best placements to verify for all cones")
    p.add_argument("--state", type=Path, default=STATE_FILE, help="Managed actor IDs for status/clear")
    args = p.parse_args(argv)
    if args.mode not in ("status", "clear", "blueprints") and not args.track:
        p.error("a CSV/JSON track path is required for inspect, preview and spawn")
    if args.marker_seconds <= 0 or args.ray_height <= 0 or args.timeout <= 0 or args.max_cones < 1:
        p.error("marker-seconds, ray-height, timeout and max-cones must be positive")
    if not (20 <= args.search_radius <= 250 and 5 <= args.search_step <= 100
            and 1 <= args.search_candidates <= 300 and 1 <= args.search_finalists <= 12):
        p.error("search-radius 20..250, search-step 5..100, search-candidates 1..300 and search-finalists 1..12 required")
    return args


def main(argv=None) -> int:
    args = command_line(argv)
    try:
        cones = load_cones(args.track, args.max_cones) if args.track else []
        if cones:
            print_summary(cones)
        if args.mode == "inspect":
            print("Offline format check passed. Preview in CARLA before spawning physical props.")
            return 0
        carla, client, world = connect(args)
        if args.mode == "blueprints":
            print("Available cone blueprints (check material colors in Unreal):")
            for bp in world.get_blueprint_library():
                if "cone" in bp.id.lower():
                    print(" ", bp.id, "attributes:", [attr.id for attr in bp])
            for color, name in CUSTOM_CONE_PROPS.items():
                try:
                    world.get_blueprint_library().find(name)
                    print(f"ASURT custom {color}: READY ({name})")
                except (IndexError, RuntimeError):
                    print(f"ASURT custom {color}: NOT INSTALLED ({name})")
            return 0
        if args.mode in ("status", "clear"):
            if args.mode == "clear":
                clear_managed(world, args.state, args)
            else:
                if not args.state.exists():
                    tagged = _tagged_cones(_live_actor_snapshot(world, args))
                    if tagged:
                        print(f"WARNING: {len(tagged)} live importer-tagged cones "
                              "have no active ownership record. Use Clear to "
                              "recover an exactly matching archive, or inspect "
                              "their IDs and roles manually.")
                    else:
                        print("No live managed cone track. Preview and spawn when ready.")
                else:
                    state = _read_state(args.state)
                    actors = owned_actors(world, state, args)
                    print(f"Live managed cones: {len(actors)}/{len(state['actors'])} (source={state['source']})")
            return 0
        if args.mode in ("car-preview", "place-car"):
            place_hero(cones, args, carla, world, preview=args.mode == "car-preview")
            return 0
        if args.mode == "find-placement":
            result = search_ground_placement(cones, args, carla, world)
            print(f"Ground search sampled {result['candidates_sampled']} placements and "
                  f"fully checked {result['candidates_verified']} candidates.")
            if result["found"]:
                print("FOUND: All cones have nearby ground collision and hero clearance at the suggested placement. "
                      "Inspect the course visually in CARLA before spawning.")
            else:
                print(f"NOT FOUND: Best fully checked candidate still has "
                      f"{result['invalid']}/{result['total']} unsupported cones. "
                      "Original placement unchanged; try another region or search radius.")
            print("PLACEMENT_RESULT: " + json.dumps(result, separators=(",", ":")))
            print("No actors spawned, no ticker or vehicle changed.")
            return 0
        blueprints = choose_blueprints(cones, args, world) if args.mode == "spawn" else None
        placed, invalid = placed_cones(cones, args, carla, world, blueprints)
        if args.mode == "preview":
            if args.ground == "fixed":
                print("WARNING: Fixed-Z preview skips ground verification. Markers may float or appear underground.")
            draw_preview(world, carla, placed, args.marker_seconds, invalid=invalid)
            print("No actors spawned, no ticker or vehicle changed.")
        else:
            if invalid:
                print(f"BLOCKED: {len(invalid)}/{len(cones)} cone positions have no verified ground. "
                      "Preview the red markers and adjust track placement. No actors were spawned.")
                print("INVALID_INDICES: " + ",".join(str(index) for index, *_ in invalid))
                for index, cone, estimated, reason in invalid[:8]:
                    print(f"  #{index} {cone.color} at ({estimated.x:.2f}, {estimated.y:.2f}): {reason}")
                return 1
            if args.ground == "fixed":
                print("WARNING: Fixed elevation was selected; cones may be floating or underground. "
                      "Ground collision has not been verified.")
            spawn_cones(blueprints, placed, args, carla, world, args.state)
        return 0
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
