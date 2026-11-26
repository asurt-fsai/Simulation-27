#!/usr/bin/env python3
"""Isolated end-to-end RViz Goal Pose -> Dubins -> Pure Pursuit -> /drive.

This publishes ONLY in the explicit synthetic ROS_DOMAIN_ID=216. No real
CARLA, autonomous hero, bridge or hardware is launched by this test.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker


def stamped(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t-sec)*1e9)))


def orientation(yaw):
    from geometry_msgs.msg import Quaternion
    return Quaternion(z=math.sin(yaw/2.), w=math.cos(yaw/2.))


class Probe(Node):
    def __init__(self):
        super().__init__("asurt_goal_pose_integration_contract")
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.mode_pub = self.create_publisher(
            Bool, "/sim/feedback/autonomous_mode", 10)
        self.cancel_pub = self.create_publisher(
            Bool, "/planning/goal_pose/cancel", 10)
        self.paths = []
        self.commands = []
        self.goal_status = []
        self.control_status = []
        self.driven = []
        self.marker_count = 0
        self.create_subscription(RosPath, "/planning/goal_pose/path",
                                 self.paths.append, 10)
        self.create_subscription(AckermannDriveStamped, "/drive",
                                 self.commands.append, 10)
        self.create_subscription(String, "/planning/goal_pose/status",
                                 lambda m: self.goal_status.append(json.loads(m.data)), 10)
        self.create_subscription(String, "/control/pure_pursuit/status",
                                 lambda m: self.control_status.append(json.loads(m.data)), 10)
        self.create_subscription(RosPath, "/control/pure_pursuit/driven_path",
                                 self.driven.append, 10)
        self.create_subscription(Marker, "/control/pure_pursuit/diagnostic_marker",
                                 lambda _: setattr(self, "marker_count",
                                                   self.marker_count+1), 10)

    def publish(self, t, pose=(0., 0., 0.), goal=None, mode=True, cancel=False):
        self.clock_pub.publish(Clock(clock=stamped(t)))
        odom = Odometry()
        odom.header.stamp = stamped(t)
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(pose[0])
        odom.pose.pose.position.y = float(pose[1])
        odom.pose.pose.orientation = orientation(pose[2])
        self.odom_pub.publish(odom)
        self.mode_pub.publish(Bool(data=mode))
        if goal is not None:
            frame, x, y, yaw = goal
            g = PoseStamped()
            g.header.frame_id = frame
            g.header.stamp = stamped(t)
            g.pose.position.x, g.pose.position.y = float(x), float(y)
            g.pose.orientation = orientation(yaw)
            self.goal_pub.publish(g)
        if cancel:
            self.cancel_pub.publish(Bool(data=True))
        rclpy.spin_once(self, timeout_sec=.035)


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "216":
        raise RuntimeError("Test requires isolated ROS_DOMAIN_ID=216")
    rclpy.init()
    probe = Probe()
    children = []
    for package, executable in (
        ("asurt_goal_pose_planning", "goal_pose_planner_node"),
        ("asurt_pure_pursuit_control", "pure_pursuit_control_node"),
    ):
        path = (Path(get_package_prefix(package)) / "lib" / package / executable)
        children.append(subprocess.Popen(
            [str(path), "--ros-args", "-p", "use_sim_time:=true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
    try:
        t = 50.
        deadline = time.monotonic()+12
        while time.monotonic() < deadline:
            probe.publish(t)
            t += .05
            if (probe.goal_pub.get_subscription_count() >= 1
                    and probe.odom_pub.get_subscription_count() >= 2
                    and probe.mode_pub.get_subscription_count() >= 1
                    and probe.goal_status and probe.control_status):
                break
            if any(p.poll() is not None for p in children):
                raise AssertionError("ROS node exited during discovery")
        else:
            raise AssertionError("ROS nodes not discovered")

        # Map goal: the planner generates a forward path and the existing
        # controller follows it without any Perception or LSTM messages.
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not (
                any(len(p.poses) > 3 and p.header.frame_id == "map"
                    for p in probe.paths)
                and any(c.drive.speed > 0. for c in probe.commands)):
            probe.publish(t, goal=("map", 6., 0., 0.))
            t += .05
        assert any(len(p.poses) > 3 and
                   abs(p.poses[-1].pose.position.x-6.) < .001
                   for p in probe.paths), "Dubins path not generated from map goal"
        assert any(0. < c.drive.speed <= .8 and
                   abs(c.drive.steering_angle) < .2
                   for c in probe.commands), "Pure Pursuit did not drive map goal"
        assert probe.marker_count, "RViz Control diagnostic marker missing"

        # Cancel must clear the route and command exactly zero speed.
        before_paths, before_cmds = len(probe.paths), len(probe.commands)
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not (
                any(not p.poses for p in probe.paths[before_paths:])
                and any(c.drive.speed == 0. for c in probe.commands[before_cmds:])):
            probe.publish(t, cancel=True)
            t += .05
        assert any(not p.poses for p in probe.paths[before_paths:]), "Cancel did not clear route"
        assert any(c.drive.speed == 0. for c in probe.commands[before_cmds:]), (
            "Controller failed to stop after goal cancellation")

        # Base_link-frame RViz goal is anchored in map when clicked.
        deadline = time.monotonic()+7
        while time.monotonic() < deadline and not any(
                s["state"] == "tracking" and s["goal_xyyaw"]
                and abs(s["goal_xyyaw"][0]-4.) < .01
                and abs(s["goal_xyyaw"][1]-1.) < .01
                for s in probe.goal_status[-40:]):
            probe.publish(t, goal=("base_link", 4., 1., .1))
            t += .05
        assert any(s["state"] == "tracking" and s["goal_xyyaw"]
                   and abs(s["goal_xyyaw"][0]-4.) < .01
                   and abs(s["goal_xyyaw"][1]-1.) < .01
                   for s in probe.goal_status), "Local RViz goal not anchored to map"

        # Simulated odometry reaches that exact map goal (without teleport).
        before_paths, before_cmds = len(probe.paths), len(probe.commands)
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not (
                any(s["state"] == "arrived" for s in probe.goal_status[-30:])
                and any(not p.poses for p in probe.paths[before_paths:])
                and any(c.drive.speed == 0. for c in probe.commands[before_cmds:])):
            probe.publish(t, pose=(4., 1., .1))
            t += .05
        assert any(s["state"] == "arrived" for s in probe.goal_status), (
            "Goal arrival was not reported")
        assert any(not p.poses for p in probe.paths[before_paths:]), (
            "Path should clear on arrival")
        assert any(c.drive.speed == 0. for c in probe.commands[before_cmds:]), (
            "Controller did not stop at destination")
        assert any(len(p.poses) >= 2 for p in probe.driven), (
            "Actual map-frame vehicle trail was not published")
        print("PASS: isolated 2D Goal Pose -> Dubins Path -> Ackermann /drive, "
              "RViz markers, cancellation, base_link transform, arrival stop, actual trail")
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for child in reversed(children):
            try:
                _, errors = child.communicate(timeout=4)
            except subprocess.TimeoutExpired:
                child.kill()
                _, errors = child.communicate(timeout=3)
            if child.returncode not in (0, -2):
                print("Node exit:", child.returncode,
                      errors.decode(errors="replace")[-1300:])
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
