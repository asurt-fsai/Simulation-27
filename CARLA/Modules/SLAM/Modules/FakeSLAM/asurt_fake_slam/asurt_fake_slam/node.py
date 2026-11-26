"""Publish a planning fixture from the active CARLA course.

No simulator actors, vehicle commands or clock ticks are owned here. FakeMAP
publishes imported placements. FakeSLAM accumulates first observations.
"""

import json
import math
import os
from pathlib import Path
from random import Random, getrandbits
import signal
from threading import Event

import carla
from asurt_msgs.msg import Landmark, LandmarkArray
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from slam_settings import read_settings


COLORS = {
    "blue": (Landmark.BLUE_CONE, (0., .3, 1.), .325, .228),
    "yellow": (Landmark.YELLOW_CONE, (1., 1., .05), .325, .228),
    "orange": (Landmark.ORANGE_CONE, (1., .45, 0.), .325, .228),
    "orange_big": (Landmark.LARGE_CONE, (1., .15, 0.), .505, .285),
    "unknown": (Landmark.CONE_TYPE_UNKNOWN, (.55, .55, .55), .325, .228),
}


def map_name(name):
    return name.removeprefix("/Game/").lstrip("/")


def verified_records(records, live):
    """Only accept a record whose actor ID, type and role still match."""
    for record in records:
        actor_id = record["id"]
        actor = live.get(actor_id)
        role = record["role_name"]
        if (actor is not None and role.startswith("asurt_cone_")
                and actor.type_id.startswith("static.prop.")
                and actor.type_id == record["type_id"]
                and actor.attributes.get("role_name") == role):
            yield record, actor


def placed_landmarks(records, live, legacy_poses):
    landmarks = {}
    retained_poses = {}
    for record, actor in verified_records(records, live):
        actor_id, role = record["id"], record["role_name"]
        if (position := record.get("position")) is None:
            # Courses placed by earlier releases have no saved transform.
            key = (actor_id, role)
            position = legacy_poses.get(key)
            if position is None:
                location = actor.get_location()
                position = (location.x, location.y, location.z)
            retained_poses[key] = position
        x, y, z = (float(component) for component in position)
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        landmarks[actor_id] = map_point(record, x, y, z)
    legacy_poses.clear()
    legacy_poses.update(retained_poses)
    return landmarks


def map_point(record, x, y, z):
    color = record.get("color", record["role_name"].split("_", 3)[-1])
    # The standard ASURT bridge reflects CARLA's world Y into ROS map Y.
    return float(x), -float(y), float(z), COLORS.get(color, COLORS["unknown"])


def visible_landmarks(landmarks, transform):
    """Use the forward half of a 4.5 m horizontal circle around the hero."""
    if transform is None:
        return {}
    x, y = transform.location.x, -transform.location.y
    yaw = math.radians(-transform.rotation.yaw)
    front_x, front_y = math.cos(yaw), math.sin(yaw)
    visible = {}
    for actor_id, point in landmarks.items():
        dx, dy = point[0] - x, point[1] - y
        if dx * dx + dy * dy <= 4.5 ** 2 and dx * front_x + dy * front_y > 1e-9:
            visible[actor_id] = point
    return visible


def update_observations(owned, snapshot, hero, mapped):
    """Keep the first observation for each surviving cone in this course."""
    retained = {}
    candidates = {}
    for actor_id, record in owned.items():
        actor = snapshot.find(actor_id)
        if actor is None:
            continue
        if actor_id in mapped and mapped[actor_id][0] == record["role_name"]:
            retained[actor_id] = mapped[actor_id]
        elif hero is not None:
            location = actor.get_transform().location
            if all(math.isfinite(value) for value in (location.x, location.y, location.z)):
                candidates[actor_id] = map_point(record, location.x, location.y, location.z)
    for actor_id, point in visible_landmarks(candidates, hero).items():
        retained[actor_id] = (owned[actor_id]["role_name"], point)
    return retained


def noisy_landmarks(landmarks, sigma, seed):
    if not sigma:
        return landmarks
    observed = {}
    for actor_id, (x, y, z, style) in landmarks.items():
        rng = Random(f"{seed}:{actor_id}")
        observed[actor_id] = (x + sigma * rng.gauss(0, 1), y + sigma * rng.gauss(0, 1), z, style)
    return observed


class CourseMap(Node):
    def __init__(self, state, legacy_state, host, port, *, observations=False):
        self.mode = "fake_slam" if observations else "fake_map"
        # Keep the old ROS node name for existing FakeMAP tools.
        super().__init__("asurt_fake_slam_observer" if observations else "asurt_fake_slam")
        self.observations = observations
        self.paths = (state, legacy_state)
        self.host, self.port = host, port
        self.client = carla.Client(host, port)
        self.client.set_timeout(1.0)
        self.world_info = None
        self.last_frame = None
        self.frames = 0
        self.resets = 0
        self.legacy_poses = {}
        self.last_error = None
        self.settings_error = None
        self.seed = getrandbits(64)
        self.owned = {}
        self.mapped = {}
        self.hero_id = None
        self.metadata_refresh = 0.
        self.map_pub = self.create_publisher(LandmarkArray, "/map/global_cones", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/map/global_cones_markers", 10)
        self.status_pub = self.create_publisher(String, "/slam/cone_map_status", 10)
        self.mode_status_pub = (self.create_publisher(String, "/slam/fake_slam_status", 10)
                                if observations else None)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02 if observations else .4, self.refresh, clock=self.steady)

    def owned_records(self, world_name):
        for path in self.paths:
            try:
                document = json.loads(path.read_text(encoding="utf-8"))
            except FileNotFoundError:
                continue
            if (document.get("host"), document.get("port")) != (self.host, self.port):
                continue
            if world_name != map_name(document["map"]) or not document.get("map_ready", True):
                continue
            yield from document["actors"]

    def refresh(self):
        try:
            world = self.client.get_world()
            snapshot = world.get_snapshot()
            frame = snapshot.frame
            if frame == 0:
                self.publish({}, "waiting for CARLA actor data")
                return
            if self.world_info is None or self.world_info[0] != world.id or (
                    self.last_frame is not None and frame < self.last_frame):
                if self.world_info is not None:
                    self.resets += 1
                self.world_info = (world.id, map_name(world.get_map().name))
                self.last_frame = None
                self.legacy_poses.clear()
                self.mapped.clear()
                self.seed = getrandbits(64)
                self.metadata_refresh = 0.
            if self.observations:
                now = self.steady.now().nanoseconds / 1e9
                heartbeat = now >= self.metadata_refresh
                if heartbeat:
                    actors = world.get_actors()
                    records = self.owned_records(self.world_info[1])
                    self.owned = {record["id"]: record for record, _ in
                                  verified_records(records, {actor.id: actor for actor in actors})}
                    self.hero_id = next((actor.id for actor in actors.filter("vehicle.*")
                                         if actor.attributes.get("role_name") == "hero"), None)
                    self.metadata_refresh = now + .4
                elif frame == self.last_frame:
                    return
                hero = snapshot.find(self.hero_id) if self.hero_id is not None else None
                pose = hero.get_transform() if hero is not None else None
                mapped = update_observations(self.owned, snapshot, pose, self.mapped)
                should_publish = heartbeat or mapped != self.mapped or self.last_error is not None
                self.mapped = mapped
                landmarks = ({actor_id: observation[1] for actor_id, observation in mapped.items()}
                             if should_publish else None)
            else:
                records = list(self.owned_records(self.world_info[1]))
                live = {actor.id: actor for actor in world.get_actors()} if records else {}
                landmarks = placed_landmarks(records, live, self.legacy_poses)
            if frame != self.last_frame:
                self.frames += 1
            self.last_frame = frame
        except (OSError, RuntimeError, ValueError, TypeError, KeyError) as error:
            if str(error) != self.last_error:
                self.get_logger().warning(f"Waiting for a verified course: {error}")
            self.last_error = str(error)
            self.publish({}, "course unavailable")
            return
        self.last_error = None
        if landmarks is None:
            return
        try:
            sigma = read_settings()[self.mode]
            self.settings_error = None
        except (OSError, ValueError) as error:
            if str(error) != self.settings_error:
                self.get_logger().warning(f"Ignoring invalid cone noise settings: {error}")
            self.settings_error = str(error)
            sigma = 0.0
        if self.observations:
            state = "mapping cones" if landmarks else "looking for cones ahead"
            if hero is None:
                state = "waiting for hero"
        else:
            state = "imported course" if landmarks else "waiting for imported cones"
        self.publish(noisy_landmarks(landmarks, sigma, self.seed), state)

    def publish(self, landmarks, state):
        cones = LandmarkArray()
        cones.header.frame_id = "map"
        cones.header.stamp = self.get_clock().now().to_msg()
        clear = Marker(header=cones.header, action=Marker.DELETEALL)
        markers = MarkerArray(markers=[clear])
        for actor_id, (x, y, z, (kind, rgb, height, width)) in sorted(landmarks.items()):
            point = Landmark(identifier=actor_id, type=kind, probability=1.0)
            point.position.x, point.position.y, point.position.z = x, y, z
            cones.landmarks.append(point)
            marker = Marker(header=cones.header, ns="slam_cones", id=actor_id,
                            type=Marker.CYLINDER, action=Marker.ADD)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z + height / 2
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = width
            marker.scale.z = height
            marker.color.r, marker.color.g, marker.color.b = rgb
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=1.5).to_msg()
            markers.markers.append(marker)
        self.map_pub.publish(cones)
        self.marker_pub.publish(markers)
        status = String(data=json.dumps({
            "source": self.mode, "state": state, "map_frame": "map",
            "detections_frames": self.frames, "confirmed": len(landmarks),
            "total": len(landmarks), "reset_count": self.resets,
        }))
        self.status_pub.publish(status)
        if self.mode_status_pub is not None:
            self.mode_status_pub.publish(status)


def main(args=None, *, observations=False):
    state = Path(os.environ["ASURT_CONE_STATE"])
    odom = yaml.safe_load(state.with_name("carla_interface_config.yaml").read_text())["odometry"]
    if odom.get("mode") != "standard" or odom.get("frame_id") != "map":
        raise SystemExit("Fake SLAM requires standard ASURT odometry in the map frame.")
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = CourseMap(state, Path(os.environ["ASURT_LEGACY_CONE_STATE"]),
                     os.environ["ASURT_CARLA_HOST"], int(os.environ["ASURT_CARLA_PORT"]),
                     observations=observations)
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.25)
    finally:
        if rclpy.ok():
            node.publish({}, "disabled")
            for publisher in (node.map_pub, node.marker_pub):
                publisher.wait_for_all_acked(Duration(seconds=.3))
        node.destroy_node()
        rclpy.try_shutdown()


def observer_main():
    main(observations=True)
