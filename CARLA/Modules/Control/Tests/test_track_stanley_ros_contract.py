#!/usr/bin/env python3
"""Synthetic ROS contract: map cones + clicked goal -> Stanley Ackermann.

Runs ONLY on isolated ROS_DOMAIN_ID=217; never starts CARLA, bridge or actor.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_prefix
from asurt_msgs.msg import Landmark, LandmarkArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rcl_interfaces.msg import Parameter as RosParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String


def ros_time(value):
    sec = int(value)
    return Time(sec=sec, nanosec=int(round((value-sec)*1e9)))


class Probe(Node):
    def __init__(self):
        super().__init__("asurt_stanley_isolated_contract")
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.map_pub = self.create_publisher(LandmarkArray, "/map/global_cones", 10)
        self.health_pub = self.create_publisher(String, "/slam/cone_map_status", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cancel_pub = self.create_publisher(
            Bool, "/planning/track_goal/cancel", 10)
        self.mode_pub = self.create_publisher(
            Bool, "/sim/feedback/autonomous_mode", 10)
        self.paths = []
        self.commands = []
        self.planner_states = []
        self.control_states = []
        self.create_subscription(RosPath, "/planning/track_goal/path",
                                 self.paths.append, 10)
        self.create_subscription(AckermannDriveStamped, "/drive",
                                 self.commands.append, 10)
        self.create_subscription(
            String, "/planning/track_goal/status",
            lambda m: self.planner_states.append(json.loads(m.data)), 10)
        self.create_subscription(
            String, "/control/stanley/status",
            lambda m: self.control_states.append(json.loads(m.data)), 10)
        self.frames = 0

    def cycle(self, t, *, goal=False, block=False, cancel=False, mode=True,
              publish_map=True, loop=False):
        self.clock_pub.publish(Clock(clock=ros_time(t)))
        o = Odometry()
        o.header.frame_id = "map"
        o.child_frame_id = "base_link"
        o.header.stamp = ros_time(t)
        o.pose.pose.position.x = 1.
        o.pose.pose.orientation.w = 1.
        self.odom_pub.publish(o)
        self.mode_pub.publish(Bool(data=mode))
        if publish_map:
            self.frames += 1
            m = LandmarkArray()
            m.header.frame_id = "map"
            m.header.stamp = ros_time(t)
            if loop:
                cones = []
                radius, count = 30., 75
                for i in range(count):
                    angle = math.pi/2-2*math.pi*i/count
                    x = 1+radius*math.cos(angle)
                    y = -radius+radius*math.sin(angle)
                    nx, ny = math.cos(angle), math.sin(angle)
                    cones.extend(((x+2*nx, y+2*ny, 0, 1.),
                                  (x-2*nx, y-2*ny, 1, 1.)))
            else:
                cones = [(float(x), float(y), kind, 1.)
                         for x in range(0, 53, 4)
                         for kind, y in ((0, 2.), (1, -2.))]
                cones.append((14., 0., 2, 1.))
                if block:
                    cones.extend([(14., -1., 2, 1.), (14., 1., 2, 1.)])
            for i, (x, y, kind, conf) in enumerate(cones):
                cone = Landmark()
                cone.identifier = i
                cone.position.x, cone.position.y = x, y
                # The real mapper publishes only CONFIRMED landmarks, but
                # their covariance-based score can age to approximately .1.
                cone.type, cone.probability = kind, .11
                m.landmarks.append(cone)
            self.map_pub.publish(m)
            self.health_pub.publish(String(data=json.dumps({
                "detections_frames": self.frames,
                "confirmed": len(m.landmarks),
                "state": "mapping",
            })))
        if goal:
            g = PoseStamped()
            g.header.frame_id = "map"
            g.header.stamp = ros_time(t)
            if loop:
                # Goal immediately behind a vehicle heading +x. The valid
                # approach is forward around the complete clockwise loop.
                angle = math.pi/2+2*math.pi/75
                g.pose.position.x = 1+30*math.cos(angle)
                g.pose.position.y = -30+30*math.sin(angle)
                yaw = math.atan2(-math.cos(angle), math.sin(angle))
                g.pose.orientation.z = math.sin(yaw/2)
                g.pose.orientation.w = math.cos(yaw/2)
            else:
                g.pose.position.x = 44.
                g.pose.orientation.w = 1.
            self.goal_pub.publish(g)
        if cancel:
            self.cancel_pub.publish(Bool(data=True))
        rclpy.spin_once(self, timeout_sec=.035)


def run():
    if os.environ.get("ROS_DOMAIN_ID") != "217":
        raise RuntimeError("Only run on isolated ROS_DOMAIN_ID=217")
    rclpy.init()
    probe = Probe()
    children = []
    for package, binary in (
        ("asurt_track_goal_planning", "track_goal_planner_node"),
        ("asurt_stanley_control", "stanley_control_node"),
    ):
        executable = Path(get_package_prefix(package)) / "lib" / package / binary
        arguments = [str(executable), "--ros-args", "-p", "use_sim_time:=true"]
        if package == "asurt_track_goal_planning":
            arguments.extend(["-p", "confirmed_map:=true"])
        children.append(subprocess.Popen(
            arguments,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
    conflict_pub = None
    try:
        t = 100.
        deadline = time.monotonic()+12
        while time.monotonic() < deadline:
            probe.cycle(t)
            t += .06
            if (probe.goal_pub.get_subscription_count() > 0
                    and probe.map_pub.get_subscription_count() > 0
                    and probe.odom_pub.get_subscription_count() >= 2
                    and probe.planner_states and probe.control_states):
                break
            assert all(c.poll() is None for c in children), "node startup failed"
        else:
            raise AssertionError("Both ROS nodes failed to discover expected topics")

        deadline = time.monotonic()+9
        while time.monotonic() < deadline and not any(
                len(p.poses) > 20 for p in probe.paths):
            probe.cycle(t, goal=True)
            t += .06
        paths = [p for p in probe.paths if len(p.poses) > 20]
        assert paths, ("No SLAM corridor path: "+
                       str(probe.planner_states[-4:]))
        assert any(s.get("confirmed_map_mode") and
                   s.get("low_covariance_confidence_cones", 0) >= 20
                   for s in probe.planner_states), (
            "Confirmed map confidence interpretation was not applied")
        path = paths[-1]
        assert path.header.frame_id == "map"
        assert any(abs(p.pose.position.y) > .7 and
                   12 < p.pose.position.x < 17 for p in path.poses), (
            "Failed to steer around centerline orange cone")

        # Accumulate a few simulated seconds of acceleration on the same route.
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not any(
                c.drive.speed > 2.5 for c in probe.commands):
            probe.cycle(t)
            t += .06
        assert any(2.5 < c.drive.speed <= 100./3.6+.001
                   for c in probe.commands), (
            "High-speed Stanley never exceeded reference 2 km/h speed: "+
            str(probe.control_states[-4:]))
        # Operator speed comparisons (30/60/100 km/h) use a bounded,
        # validated ROS parameter update, not a bridge/physics rewrite.
        client = probe.create_client(
            SetParameters, "/asurt_stanley_control/set_parameters")
        assert client.wait_for_service(timeout_sec=3.), "No Stanley parameter service"

        def set_speed(kmh):
            req = SetParameters.Request()
            req.parameters = [RosParameter(
                name="target_speed_mps",
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=kmh/3.6))]
            request = client.call_async(req)
            rclpy.spin_until_future_complete(probe, request, timeout_sec=4.)
            assert request.result() and request.result().results[0].successful

        set_speed(60.)
        deadline = time.monotonic()+4.
        while time.monotonic() < deadline and not any(
                abs(s["configured_max_speed_kmh"]-60.) < .01
                for s in probe.control_states[-10:]):
            probe.cycle(t)
            t += .06
        assert any(abs(s["configured_max_speed_kmh"]-60.) < .01
                   for s in probe.control_states[-10:])
        set_speed(100.)

        # A complete SLAM circuit must pass through the planner and Stanley
        # without the historical 600-waypoint truncation in either ROS node.
        before_paths, before_commands = len(probe.paths), len(probe.commands)
        deadline = time.monotonic()+9
        while time.monotonic() < deadline and not (
                any(len(p.poses) > 600 for p in probe.paths[before_paths:])
                and any(s["state"] == "tracking" and s["command_sent"]
                        for s in probe.control_states[-10:])
                and any(c.drive.speed > .1 for c in probe.commands[before_commands:])):
            probe.cycle(t, loop=True, goal=True)
            t += .06
        assert any(len(p.poses) > 600 for p in probe.paths[before_paths:]), (
            "Behind-car full-loop goal did not produce a long path: "+
            str(probe.planner_states[-6:]))
        assert any(s["state"] == "tracking" and s["command_sent"]
                   for s in probe.control_states[-10:]), (
            "Stanley rejected the long circuit: "+str(probe.control_states[-10:]))
        assert any(c.drive.speed > .1 for c in probe.commands[before_commands:])

        # Restore the original short route for the existing conflict, blocked
        # corridor and cancellation checks below.
        before_paths = len(probe.paths)
        deadline = time.monotonic()+9
        while time.monotonic() < deadline and not any(
                20 < len(p.poses) < 600 for p in probe.paths[before_paths:]):
            probe.cycle(t, goal=True)
            t += .06
        assert any(20 < len(p.poses) < 600
                   for p in probe.paths[before_paths:])

        # The installed pure-pursuit node would be a second /drive publisher.
        # Simulate that situation without starting a second controller.
        conflict_pub = probe.create_publisher(AckermannDriveStamped, "/drive", 10)
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not any(
                s["state"] == "another_drive_publisher_active"
                for s in probe.control_states[-10:]):
            probe.cycle(t)
            t += .06
        assert any(s["state"] == "another_drive_publisher_active"
                   for s in probe.control_states[-10:]), (
            "Stanley failed to detect second /drive publisher")
        probe.destroy_publisher(conflict_pub)
        conflict_pub = None

        # An obstacle extending across the full corridor must invalidate the
        # route instead of leaving an old forward speed command active.
        before_paths, before_commands = len(probe.paths), len(probe.commands)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not (
                any(not p.poses for p in probe.paths[before_paths:])
                and any(c.drive.speed == 0. for c in probe.commands[before_commands:])):
            probe.cycle(t, block=True)
            t += .06
        assert any(not p.poses for p in probe.paths[before_paths:]), (
            "Blocked corridor did not clear output route")
        assert any(c.drive.speed == 0. for c in probe.commands[before_commands:]), (
            "Stanley failed to request zero when corridor blocked")

        # Restore map and reselect a goal; cancel it through the dedicated topic.
        deadline = time.monotonic()+8
        before_paths = len(probe.paths)
        while time.monotonic() < deadline and not any(
                len(p.poses) > 20 for p in probe.paths[before_paths:]):
            probe.cycle(t, goal=True)
            t += .06
        assert any(len(p.poses) > 20 for p in probe.paths[before_paths:])
        # The mapper may continue to exist while its input stream stalls.
        # A stale SLAM snapshot must not remain a commandable route.
        before_paths, before_commands = len(probe.paths), len(probe.commands)
        deadline = time.monotonic()+7
        while time.monotonic() < deadline and not (
                any(s["state"] == "missing_or_stale_slam_map"
                    for s in probe.planner_states[-15:])
                and any(not p.poses for p in probe.paths[before_paths:])
                and any(c.drive.speed == 0 for c in probe.commands[before_commands:])):
            probe.cycle(t, publish_map=False)
            t += .06
        assert any(s["state"] == "missing_or_stale_slam_map"
                   for s in probe.planner_states), (
            "Stale mapper still accepted: "+
            str(probe.planner_states[-12:]))
        assert any(not p.poses for p in probe.paths[before_paths:])
        assert any(c.drive.speed == 0 for c in probe.commands[before_commands:])

        # Reacquire the map before testing explicit goal cancellation.
        before_paths = len(probe.paths)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not any(
                len(p.poses) > 20 for p in probe.paths[before_paths:]):
            probe.cycle(t, goal=True)
            t += .06
        assert any(len(p.poses) > 20 for p in probe.paths[before_paths:])
        before_commands = len(probe.commands)
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not (
                any(s["state"] == "cancelled" for s in probe.planner_states[-10:])
                and any(c.drive.speed == 0. for c in probe.commands[before_commands:])):
            probe.cycle(t, cancel=True)
            t += .06
        assert any(s["state"] == "cancelled" for s in probe.planner_states[-10:])
        assert any(c.drive.speed == 0. for c in probe.commands[before_commands:])
        print("PASS: isolated cone-aware manual goal -> Stanley /drive (>2.5 m/s), "
              "full-loop goal behind (>600 path points), "
              "live 60/100 km/h parameter updates, detour, blocked corridor, "
              "stale SLAM stream, authority conflict, cancellation")
    finally:
        if conflict_pub is not None:
            probe.destroy_publisher(conflict_pub)
        for process in reversed(children):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in reversed(children):
            try:
                _, errors = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                _, errors = process.communicate(timeout=4)
            if process.returncode not in (0, -2):
                print("Node exit", process.returncode,
                      errors.decode(errors="replace")[-1600:])
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run()
