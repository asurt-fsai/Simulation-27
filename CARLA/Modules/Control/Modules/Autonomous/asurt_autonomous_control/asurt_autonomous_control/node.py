"""Two CARLA controllers using a live, unambiguous planner path as input."""

from collections import deque
from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path as FilePath
import signal
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from asurt_pure_pursuit_control import core as pursuit
from asurt_stanley_control import core as stanley
from module_control import autonomous_planner, autonomous_planners


DRIVE = "/drive"
ODOM = "/odom"
MODE = "/sim/feedback/autonomous_mode"
PATH_TYPE = "nav_msgs/msg/Path"
NANOSECOND = 1_000_000_000


@dataclass(frozen=True)
class Command:
    state: str
    waiting_for: tuple = ()
    speed: float = 0.
    steering: float = 0.


def stamp_ns(stamp):
    return stamp.sec * NANOSECOND + stamp.nanosec


def fresh(now, stamp, max_age):
    return stamp > 0 and -200_000_000 <= now - stamp <= max_age


def pure_pursuit_command(points, pose, speed, settings):
    local = pursuit.current_local_path(points, "map", pose)
    _, nearest, along, cross_track, heading = stanley.project_front_axle(local, stanley.Pose2(0., 0., 0.), 0.)
    # A rolling planner may start at the next observed gate several metres ahead.
    if abs(cross_track) > 2. or math.hypot(*nearest) > 8. or abs(heading) > .9:
        raise ValueError("vehicle is outside the path tracking tolerance")
    target = pursuit.target_on_path(local, pursuit.lookahead_distance(speed, settings))
    if target is None:
        return 0., 0.
    steering, speed, _ = pursuit.steering_and_speed(target, settings)
    remaining = sum(math.dist(a, b) for a, b in zip(local, local[1:])) - along - .3
    return steering, min(speed, math.sqrt(max(0., 2. * remaining)))


class AutonomousControl(Node):
    def __init__(self, algorithm):
        self.algorithm = algorithm
        self.module_id = f"control_{algorithm}_auto"
        super().__init__(f"asurt_{algorithm}_auto")
        settings = (pursuit.Settings(max_steer_rad=math.radians(16.))
                    if algorithm == "pure_pursuit" else stanley.Settings())
        for key, default in vars(settings).items():
            self.declare_parameter(key, default)
        self.settings = replace(settings, **{key: self.get_parameter(key).value for key in vars(settings)})
        self.settings.validate()
        self.declare_parameter("planner_topic", "")
        self.add_on_set_parameters_callback(self.set_tuning)
        self.path_max_age = 1_250_000_000 if algorithm == "pure_pursuit" else 750_000_000
        self.period = .1 if algorithm == "pure_pursuit" else .05
        self.mode = False
        self.mode_stamp = 0
        self.was_driving = False
        self.source = None
        self.subscription = None
        self.next_discovery = 0.
        self.source_wait = ("one autonomous PathPlanning publisher",)
        self.last_clock = 0
        self.last_clock_change = 0.
        self.last_tick = 0.
        self.status_key = None
        log = os.environ.get("ASURT_SESSION_LOG")
        self.status_file = FilePath(log) / f"{self.module_id}-readiness.json" if log else None
        self.clear_tracking()
        self.drive = self.create_publisher(AckermannDriveStamped, DRIVE, 1)
        self.status = self.create_publisher(String, f"/control/{algorithm}_auto/status", 10)
        self.preview = self.create_publisher(AckermannDriveStamped, f"/control/{algorithm}_auto/command_preview", 10)
        self.create_subscription(Odometry, ODOM, self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, MODE, self.receive_mode, 10)
        self.create_timer(self.period, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def set_tuning(self, values):
        changes = {param.name: param.value for param in values if param.name in vars(self.settings)}
        try:
            settings = replace(self.settings, **changes)
            settings.validate()
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.settings = settings
        return SetParametersResult(successful=True)

    def clear_tracking(self):
        self.path = None
        self.odom_history = deque(maxlen=256)
        self.pose = None
        self.odom_stamp = 0
        self.actual_speed = 0.
        self.last_speed = 0.

    def discover(self):
        try:
            requested = self.get_parameter("planner_topic").value or autonomous_planner()
            configured = autonomous_planners()
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.select_source(None)
            self.source_wait = (f"a valid PathPlanning registry ({error})",)
            return
        if requested and requested not in configured:
            self.select_source(None)
            self.source_wait = (f"an autonomous planner registered for {requested}",)
            return
        topics = (requested,) if requested else configured
        sources = [source for topic in topics for source in self.publishers(topic)]
        self.select_source(sources[0] if len(sources) == 1 else None)
        if len(sources) > 1:
            active = ", ".join(sorted({topic for topic, _ in sources}))
            self.source_wait = (f"a single planner publisher (currently: {active})",)
        else:
            self.source_wait = (f"{configured[requested]} to publish on {requested}" if requested
                                else "one running autonomous PathPlanning module",)

    def select_source(self, source):
        if source == self.source:
            return
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        self.source = source
        self.path = None
        self.last_speed = 0.
        if source is not None:
            self.subscription = self.create_subscription(
                Path, source[0], lambda msg: self.receive_path(msg, source), qos_profile_sensor_data)

    def publishers(self, topic):
        return [(topic, bytes(info.endpoint_gid)) for info in self.get_publishers_info_by_topic(topic)
                if info.topic_type == PATH_TYPE]

    def receive_path(self, message, source):
        if source != self.source:
            return
        # Humble's Python executor does not pass MessageInfo to callbacks.
        # Accept data only while the selected publisher remains unique.
        if self.publishers(source[0]) != [source]:
            self.path = None
            return
        frame = message.header.frame_id.lstrip("/")
        stamp = stamp_ns(message.header.stamp)
        count = len(message.poses)
        if (frame not in ("map", "base_link") or stamp <= 0 or
                not 2 <= count <= stanley.MAX_TRACK_PATH_POINTS):
            self.path = None
            return
        if self.path is not None and stamp < self.path[1]:
            return
        points = tuple((p.pose.position.x, p.pose.position.y) for p in message.poses)
        self.path = (frame, stamp, points) if all(math.isfinite(coord) for p in points for coord in p) else None

    def receive_odom(self, message):
        if (message.header.frame_id.lstrip("/") != "map" or
                message.child_frame_id.lstrip("/") != "base_link"):
            return
        position, orientation = message.pose.pose.position, message.pose.pose.orientation
        velocity = message.twist.twist.linear
        q = orientation
        values = (position.x, position.y, q.x, q.y, q.z, q.w, velocity.x, velocity.y)
        stamp = stamp_ns(message.header.stamp)
        if stamp <= 0 or not all(math.isfinite(v) for v in values) or not .98 <= sum(v*v for v in values[2:6]) <= 1.02:
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = pursuit.VehiclePose(position.x, position.y, yaw)
        if self.pose and (stamp < self.odom_stamp - 100_000_000 or
                          math.hypot(pose.x-self.pose.x, pose.y-self.pose.y) > 8.):
            self.clear_tracking()
        if stamp > self.odom_stamp:
            self.pose, self.odom_stamp = pose, stamp
            self.actual_speed = math.hypot(velocity.x, velocity.y)
            self.odom_history.append((stamp, pose))

    def receive_mode(self, msg):
        self.mode = msg.data
        self.mode_stamp = self.get_clock().now().nanoseconds

    def command(self, now, wall, dt, conflict):
        if conflict:
            return Command("drive_conflict", ("exclusive /drive ownership",))
        if now <= 0 or wall - self.last_clock_change > .6:
            return Command("waiting_for_clock", ("advancing /clock",))
        if not fresh(now, self.odom_stamp, 400_000_000):
            return Command("waiting_for_odometry", ("fresh /odom in map → base_link",))
        if self.source is None:
            return Command("waiting_for_planner", self.source_wait)
        if self.path is None or not fresh(now, self.path[1], self.path_max_age):
            return Command("waiting_for_path", (f"a fresh, valid path on {self.source[0]}",))
        if not fresh(now, self.mode_stamp, 750_000_000):
            return Command("waiting_for_ros_mode", (f"fresh {MODE}",))
        if not self.mode:
            return Command("manual_mode", ("ROS driving mode (press B in the driving window)",))
        frame, stamp, points = self.path
        if frame == "base_link":
            source_time, source_pose = min(self.odom_history, key=lambda sample: abs(sample[0]-stamp))
            if abs(source_time-stamp) > 180_000_000:
                return Command("waiting_for_path_odometry", ("/odom at the planner's path timestamp",))
            points = tuple(pursuit.local_to_map(point, source_pose) for point in points)
        try:
            if self.algorithm == "pure_pursuit":
                steer, requested = pure_pursuit_command(points, self.pose, self.actual_speed, self.settings)
            else:
                if len(points) == 2:
                    first, last = points
                    midpoint = tuple((a + b) / 2 for a, b in zip(first, last))
                    points = (first, midpoint, last)
                steer, requested, metrics = stanley.track_command(
                    points, self.pose, self.actual_speed, self.settings, stop_before_tight_bend=True)
                if requested == 0. and metrics["distance_to_tight_bend_m"] is not None:
                    return Command("waiting_for_trackable_bend", (
                        f"a path within Stanley's steering limit on {self.source[0]}",))
            speed = pursuit.ramp_speed(self.last_speed, requested, self.settings.max_accel_mps2, dt)
            return Command("tracking" if requested > 0 else "end_of_path", speed=speed, steering=steer)
        except ValueError as error:
            return Command("path_rejected", (f"a trackable path on {self.source[0]} ({error})",))

    def message(self, command):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(command.speed)
        msg.drive.steering_angle = float(command.steering)
        return msg

    def tick(self):
        now = self.get_clock().now().nanoseconds
        wall = time.monotonic()
        if self.last_clock and now < self.last_clock:
            self.clear_tracking()
            self.mode_stamp = 0
        if now != self.last_clock:
            self.last_clock, self.last_clock_change = now, wall
        if wall >= self.next_discovery:
            self.next_discovery = wall + .25
            self.discover()
        conflict = self.count_publishers(DRIVE) != 1
        dt = min(.25, wall-self.last_tick) if self.last_tick else self.period
        command = self.command(now, wall, dt, conflict)
        self.last_tick, self.last_speed = wall, command.speed
        message = self.message(command)
        self.preview.publish(message)
        active = self.mode and fresh(now, self.mode_stamp, 750_000_000)
        sent = not conflict and (active or self.was_driving)
        if sent:
            self.drive.publish(message)
        self.was_driving = not conflict and active
        route = self.source[0] if self.source else None
        report = {"pid": os.getpid(), "state": command.state, "waiting_for": command.waiting_for,
                  "path_topic": route, "command_sent": sent, "speed_mps": command.speed,
                  "steering_rad": command.steering}
        self.status.publish(String(data=json.dumps(report)))
        key = (command.state, command.waiting_for, route)
        if self.status_file and key != self.status_key:
            temp = self.status_file.with_suffix(".tmp")
            try:
                temp.write_text(json.dumps(report), encoding="utf-8")
                temp.replace(self.status_file)
                self.status_key = key
            except OSError as error:
                self.get_logger().warning(f"Could not write module readiness: {error}", throttle_duration_sec=5.)


def run(algorithm):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AutonomousControl(algorithm)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stopped = False

    def shutdown(*_args):
        nonlocal stopped
        stopped = True

    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, shutdown)
    try:
        while not stopped and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
    finally:
        if rclpy.ok() and node.mode and node.count_publishers(DRIVE) == 1:
            for _ in range(3):
                node.drive.publish(node.message(Command("stopping")))
                time.sleep(.025)
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        rclpy.try_shutdown()


def pure_pursuit_main():
    run("pure_pursuit")


def stanley_main():
    run("stanley")
