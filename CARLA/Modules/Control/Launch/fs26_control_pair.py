#!/usr/bin/python3
"""Connect unmodified July Control to ASURT planner topics and TF."""
from math import atan2, cos, sin
import os
from pathlib import Path
import subprocess
import sys

import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

root = Path(os.environ["ASURT_ROOT"])
stock_tools = root / "libexec/asurt-stock-tools/RViz"
sys.path.insert(0, str(stock_tools if stock_tools.is_dir() else root / "tools/RViz"))
from module_control import autonomous_planner, autonomous_planners


class PlannerInput(Node):
    def __init__(self):
        super().__init__("asurt_fs26_control_input")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.output = self.create_publisher(RosPath, "/asurt/fs26/control/path", 10)
        self.odometry = self.create_publisher(Odometry, "/asurt/fs26/control/odom", 10)
        self.create_subscription(Odometry, "/odom", self.odometry.publish, qos_profile_sensor_data)
        self.topic = None
        self.input = None
        self.create_timer(0.5, self.select_path)

    def select_path(self):
        try:
            selected = autonomous_planner()
            available = autonomous_planners(include_explicit=bool(selected))
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            selected, available = "", {}
        if selected:
            topic = selected if selected in available else None
        else:
            live = [name for name in available if self.get_publishers_info_by_topic(name)]
            topic = live[0] if len(live) == 1 else None
        if topic == self.topic:
            return
        if self.input is not None:
            self.destroy_subscription(self.input)
        self.topic = topic
        self.input = self.create_subscription(RosPath, topic, self.forward, 10) if topic else None

    def forward(self, path):
        frame = path.header.frame_id.lstrip("/")
        if path.poses and frame != "base_link":
            try:
                transform = self.buffer.lookup_transform("base_link", frame, Time()).transform
            except TransformException as error:
                self.get_logger().warn(str(error), throttle_duration_sec=5.0)
                return
            q = transform.rotation
            heading = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            c, s = cos(heading), sin(heading)
            for pose in path.poses:
                p = pose.pose.position
                p.x, p.y = (c * p.x - s * p.y + transform.translation.x,
                            s * p.x + c * p.y + transform.translation.y)
        path.header.frame_id = "base_link"
        for pose in path.poses:
            pose.header.frame_id = "base_link"
        self.output.publish(path)


def main():
    executable, config, command_topic, *args = sys.argv[1:]
    rclpy.init(args=[])
    proxy = PlannerInput()
    relay = ["-r", "/path:=/asurt/fs26/control/path", "-r", f"{command_topic}:=/drive"]
    if command_topic == "/ackermann_cmd":
        relay += ["-p", "control.frame_id:=base_link"]
    else:
        relay += ["-r", "/zed/zed_node/odom:=/asurt/fs26/control/odom"]
    with subprocess.Popen([executable, "--ros-args", "--params-file", config, *relay, *args]) as child:
        try:
            while rclpy.ok() and child.poll() is None:
                rclpy.spin_once(proxy, timeout_sec=0.25)
        except KeyboardInterrupt:
            pass
        finally:
            if child.poll() is None:
                child.terminate()
            proxy.destroy_node()
            rclpy.try_shutdown()
    return child.returncode


if __name__ == "__main__":
    raise SystemExit(main())
