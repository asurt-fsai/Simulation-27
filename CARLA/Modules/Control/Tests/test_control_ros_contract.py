#!/usr/bin/env python3
"""Isolated ROS-domain contract; never run on the live CARLA ROS_DOMAIN_ID.

Publishes synthetic odometry, clock, ROS-mode feedback and an LSTM-shaped path
only inside a test domain. The actual /drive topic therefore has no live car.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as ROSPath
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String


def stamped(seconds):
    from builtin_interfaces.msg import Time
    s = int(seconds)
    return Time(sec=s, nanosec=int(round((seconds-s)*1e9)))


class Probe(Node):
    def __init__(self):
        super().__init__("asurt_control_ros_contract_probe")
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.path_pub = self.create_publisher(ROSPath, "/planning/deep_learning/raw_path", 10)
        self.mode_pub = self.create_publisher(Bool, "/sim/feedback/autonomous_mode", 10)
        self.commands = []
        self.statuses = []
        self.create_subscription(AckermannDriveStamped, "/drive",
                                 self.commands.append, 10)
        self.create_subscription(String, "/control/pure_pursuit/status",
                                 self.statuses.append, 10)

    def publish(self, t, mode=True, path=True, y=1.0, path_time=None):
        self.clock_pub.publish(Clock(clock=stamped(t)))
        o = Odometry()
        o.header.frame_id = "map"
        o.header.stamp = stamped(t)
        o.child_frame_id = "base_link"
        o.pose.pose.orientation.w = 1.
        self.odom_pub.publish(o)
        self.mode_pub.publish(Bool(data=mode))
        p = ROSPath()
        p.header.frame_id = "base_link"
        p.header.stamp = stamped(t if path_time is None else path_time)
        if path:
            for i in range(15):
                q = PoseStamped()
                q.header = p.header
                q.pose.position.x = 1.+.7*i
                q.pose.position.y = y
                q.pose.orientation.w = 1.
                p.poses.append(q)
        self.path_pub.publish(p)


def run():
    if os.environ.get("ROS_DOMAIN_ID") not in ("215", "216"):
        raise RuntimeError("Only run this synthetic test with ROS_DOMAIN_ID=215 or 216")
    rclpy.init()
    probe = Probe()
    # Signal the actual ROS executable rather than the ros2-run CLI wrapper.
    # The wrapper is a separate process and may not forward SIGINT reliably.
    executable = (Path(get_package_prefix("asurt_pure_pursuit_control")) /
                  "lib/asurt_pure_pursuit_control/pure_pursuit_control_node")
    child = subprocess.Popen(
        [str(executable), "--ros-args", "-p", "use_sim_time:=true",
         "-p", "path_topic:=/planning/deep_learning/raw_path"],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        # DDS discovery; standalone node is never started in the live domain.
        deadline = time.monotonic()+12
        while time.monotonic() < deadline and (
            probe.odom_pub.get_subscription_count() == 0 or
            probe.path_pub.get_subscription_count() == 0 or
            probe.mode_pub.get_subscription_count() == 0):
            if child.poll() is not None:
                raise AssertionError("Controller exited before DDS discovery")
            rclpy.spin_once(probe, timeout_sec=.08)
        assert probe.odom_pub.get_subscription_count(), "No /odom subscriber"
        assert probe.path_pub.get_subscription_count(), "No path subscriber"
        t = 20.
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not any(
                0 < c.drive.speed <= .8 and c.drive.steering_angle > 0
                for c in probe.commands):
            probe.publish(t)
            rclpy.spin_once(probe, timeout_sec=.03)
            t += .05
        assert any(0 < c.drive.speed <= .8 and c.drive.steering_angle > 0
                   for c in probe.commands), "No valid positive-left /drive command"
        # Empty path must produce an immediate zero-speed command, not
        # keep the previous steering and speed.
        before = len(probe.commands)
        for _ in range(12):
            probe.publish(t, path=False)
            rclpy.spin_once(probe, timeout_sec=.03)
            t += .05
        assert any(c.drive.speed == 0 and c.drive.steering_angle == 0
                   for c in probe.commands[before:]), "No zero command after path clear"
        # A full 15-point RViz path stamped two seconds before current odom
        # is a valid visualization but NOT a current control reference.
        before = len(probe.commands)
        for _ in range(12):
            probe.publish(t, path_time=t-2.0)
            rclpy.spin_once(probe, timeout_sec=.04)
            t += .05
        assert not any(c.drive.speed > 0 for c in probe.commands[before:]), (
            "Late source-stamped LSTM path incorrectly caused movement")
        assert any('"state": "stale_path"' in s.data for s in probe.statuses), (
            "Controller did not diagnose stale model input")
        # Manual mode must not publish any positive drive commands.
        before = len(probe.commands)
        for _ in range(10):
            probe.publish(t, mode=False)
            rclpy.spin_once(probe, timeout_sec=.03)
            t += .05
        assert not any(c.drive.speed > 0 for c in probe.commands[before:]), (
            "Manual mode received a positive controller command")
        assert probe.statuses, "No status publisher"
        # Restore ROS mode and a valid moving command; then send SIGINT while
        # the node is active. Its final /drive command must explicitly be zero.
        before = len(probe.commands)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not any(
                c.drive.speed > 0 for c in probe.commands[before:]):
            probe.publish(t, mode=True)
            rclpy.spin_once(probe, timeout_sec=.04)
            t += .05
        assert any(c.drive.speed > 0 for c in probe.commands[before:]), (
            "Could not restore active control before shutdown; child=%s; statuses=%s"
            % (child.poll(), [s.data for s in probe.statuses[-4:]]))
        before = len(probe.commands)
        child.send_signal(signal.SIGINT)
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.08)
            if any(c.drive.speed == 0. for c in probe.commands[before:]):
                break
        assert any(c.drive.speed == 0. for c in probe.commands[before:]), (
            "SIGINT shutdown failed to publish a zero-speed command")
        print("PASS: isolated Ackermann steer, speed, clear, stale-LSTM gate, "
              "manual mode, SIGINT stop")
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        try:
            child.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            child.kill()
            child.communicate(timeout=3)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run()
