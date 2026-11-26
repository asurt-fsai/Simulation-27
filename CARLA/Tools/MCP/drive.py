"""Reference driving, ROS goal dispatch and ground truth for an owned CARLA run."""

from contextlib import contextmanager
import json
import math
import os
import time

import carla
from shapely.geometry import LineString, Point


def world_xy(x, y, origin):
    ox, oy, _, yaw = origin
    angle = math.radians(yaw)
    return ox + math.cos(angle) * x + math.sin(angle) * y, oy + math.sin(angle) * x - math.cos(angle) * y


class TrackProgress:
    def __init__(self, points, origin):
        world = [world_xy(*point, origin) for point in points]
        self.line = LineString([*world, world[0]])
        self.previous = None
        self.distance = 0.0
        self.errors = []

    def update(self, location):
        point = Point(location.x, location.y)
        along = self.line.project(point)
        if self.previous is not None:
            length = self.line.length
            delta = (along - self.previous + length / 2) % length - length / 2
            if abs(delta) <= 2.0:
                self.distance += delta
        self.previous = along
        self.errors.append(self.line.distance(point))
        return along

    def aim(self, along, lookahead):
        return self.line.interpolate((along + lookahead) % self.line.length)

    def metrics(self, elapsed):
        return {"simulation_seconds": round(elapsed, 3), "progress_m": round(max(0., self.distance), 3),
                "lap_completed": self.distance >= self.line.length - 0.75,
                "centerline_max_error_m": round(max(self.errors, default=0.), 3),
                "centerline_rms_error_m": round(math.sqrt(sum(e * e for e in self.errors) / max(1, len(self.errors))), 3)}


def reference_steering(transform, target):
    location = transform.location
    bearing = math.atan2(target.y - location.y, target.x - location.x)
    error = bearing - math.radians(transform.rotation.yaw)
    distance = max(0.5, math.hypot(target.x - location.x, target.y - location.y))
    angle = -math.atan2(2.0 * 1.55 * math.sin(error), distance)
    return max(-.5, min(.5, angle))


@contextmanager
def driver_output(driver, goal, origin, domain, stop):
    if driver == "observe":
        yield None
        return
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration

    os.environ["ROS_DOMAIN_ID"] = str(domain)
    rclpy.init(args=[])
    node = rclpy.create_node("asurt_mcp_driver")
    reference = driver == "reference"
    topic = "/drive" if reference else "/goal_pose"
    publisher = node.create_publisher(AckermannDriveStamped if reference else PoseStamped, topic, 10)

    def send_drive(speed, steer, stamp):
        if node.count_publishers(topic) > 1:
            raise RuntimeError("A second ROS publisher owns /drive in this scenario.")
        message = AckermannDriveStamped()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(stamp * 1e9), 10**9)
        message.header.frame_id = "base_link"
        message.drive.speed, message.drive.steering_angle = float(speed), steer
        publisher.publish(message)

    try:
        deadline = time.monotonic() + 12.
        while not stop.wait(.1) and publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"The simulator has no subscriber for {topic}.")
        if goal is not None and not stop.is_set():
            x, y = world_xy(goal[0], goal[1], origin)
            angle = math.radians(goal[2] - origin[3])
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = x, -y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(angle / 2), math.cos(angle / 2)
            publisher.publish(pose)
        yield send_drive if reference else None
    finally:
        if reference and publisher.get_subscription_count() and node.count_publishers(topic) == 1:
            send_drive(0., 0., 0.)
        publisher.wait_for_all_acked(Duration(seconds=.4))
        node.destroy_node()
        rclpy.try_shutdown()


def coordinates(transform):
    location, rotation = transform.location, transform.rotation
    return [location.x, location.y, location.z, rotation.roll, rotation.pitch, rotation.yaw]


def sample_truth(stream, snapshot, car_transform, records):
    cones = []
    for record in records:
        observed = snapshot.find(record["id"])
        if observed is not None:
            cones.append({"actor_id": record["id"], "color": record["color"],
                          "transform": coordinates(observed.get_transform())})
    stream.write(json.dumps({"frame": snapshot.frame, "simulation_time_s": snapshot.timestamp.elapsed_seconds,
                             "hero_transform": coordinates(car_transform), "cones": cones}) + "\n")


def drive_scenario(directory, spec, session, origin, ownership, env, stop):
    client = carla.Client(session["host"], session["port"])
    client.set_timeout(4.)
    world = client.get_world()
    world.wait_for_tick(8.)
    hero = next((actor for actor in world.get_actors().filter("vehicle.vehicle.asurt_fsai")
                 if actor.attributes.get("role_name") == "hero"), None)
    if hero is None:
        raise RuntimeError("The owned simulator did not expose its FSAI hero.")
    track = json.loads((directory / "track/centerline.json").read_text())
    progress = TrackProgress(track, origin)
    elapsed = 0.
    deadline = time.monotonic() + max(30., 3. * spec["duration_s"])
    records = ownership["actors"]
    goal = spec["goal"] if spec["driver"] == "stanley" else None
    if goal is not None:
        gx, gy = world_xy(*goal[:2], origin)
    stop_reason = "duration"
    start_sim = None
    next_sample = 0.
    with (directory / "ground-truth.jsonl").open("w", encoding="utf-8") as truth, \
            driver_output(spec["driver"], goal, origin, env["ROS_DOMAIN_ID"], stop) as command:
        while not stop.is_set() and elapsed < spec["duration_s"]:
            if time.monotonic() > deadline:
                raise RuntimeError("The simulator did not advance in the allotted wall time.")
            frame = world.wait_for_tick(8.)
            observed = frame.find(hero.id)
            if observed is None:
                raise RuntimeError("The FSAI hero disappeared during the run.")
            sim_time = frame.timestamp.elapsed_seconds
            if start_sim is None:
                start_sim = sim_time
            elapsed = sim_time - start_sim
            if elapsed < 0:
                raise RuntimeError("Simulation time rewound during the run.")
            transform = observed.get_transform()
            if spec["bag"] == "training":
                sample_truth(truth, frame, transform, records)
            if elapsed + 1e-6 < next_sample:
                continue
            next_sample = elapsed + .095
            along = progress.update(transform.location)
            if spec["bag"] != "training":
                sample_truth(truth, frame, transform, records)
            if command is not None:
                target = progress.aim(along, 2.7)
                command(spec["speed_mps"], reference_steering(transform, target), sim_time)
                if progress.distance >= progress.line.length - .75:
                    stop_reason = "lap_completed"
                    break
            if goal is not None and math.hypot(transform.location.x - gx, transform.location.y - gy) < 1.0 \
                    and observed.get_velocity().length() < .5:
                stop_reason = "goal_reached"
                break
    result = progress.metrics(elapsed)
    result["stop_reason"] = "cancelled" if stop.is_set() else stop_reason
    result["driver"] = spec["driver"]
    return result
