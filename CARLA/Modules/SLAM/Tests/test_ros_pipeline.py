#!/usr/bin/env python3
"""Isolated ROS_DOMAIN_ID test: no CARLA, sensors, /drive, or live ROS traffic.

Run after sourcing Humble, the ASURT message underlay and the SLAM overlay:
  ROS_DOMAIN_ID=79 python3 Simulation/Systems/SLAM/Tests/test_ros_pipeline.py
"""
import math
import time

from asurt_msgs.msg import Landmark, LandmarkArray
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import rclpy

from asurt_cone_slam.cone_mapper_node import ConeMapperNode
from asurt_cone_slam.trajectory_node import TrajectoryNode


def run():
    rclpy.init()
    mapper, trajectory = ConeMapperNode(), TrajectoryNode()
    probe = Node("slam_integration_test_probe")
    publisher_odom = probe.create_publisher(Odometry, "/odom", 10)
    publisher_cones = probe.create_publisher(LandmarkArray, "/perception_landmarks", 10)
    maps, paths, statuses = [], [], []
    probe.create_subscription(LandmarkArray, "/map/global_cones", maps.append, 10)
    probe.create_subscription(Path, "/slam/path", paths.append, 10)
    probe.create_subscription(String, "/slam/cone_map_status", statuses.append, 10)
    ex = SingleThreadedExecutor()
    for node in (mapper, trajectory, probe):
        ex.add_node(node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=.03)

    def emit(base_x, base_y, label=0):
        stamp = probe.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = stamp
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = base_x, base_y
        odom.pose.pose.orientation.w = 1.
        publisher_odom.publish(odom)
        spin(.09)
        landmarks = LandmarkArray()
        landmarks.header.frame_id = "base_link"
        landmarks.header.stamp = stamp
        cone = Landmark()
        cone.position.x, cone.position.y = 3., 0.
        cone.type = label
        cone.probability = .95
        landmarks.landmarks.append(cone)
        publisher_cones.publish(landmarks)
        spin(.10)

    try:
        spin(.35)
        # Silverstone coordinate sample (ROS does not depend on map name).
        for _ in range(4):
            emit(11.2, 26.9, label=0)
        spin(.8)
        observed = [m for msg in maps for m in msg.landmarks]
        assert any(m.type == 0 and math.isclose(m.position.x, 14.2, abs_tol=.5)
                   for m in observed), f"Expected silverstone blue cone, got {observed}"
        assert paths and paths[-1].header.frame_id == "map"
        assert statuses and '"confirmed"' in statuses[-1].data

        # FlatAsphalt's distant spawn triggers automatic map/trajectory reset.
        for _ in range(4):
            emit(-50.8, -4.15, label=1)
        spin(.8)
        latest = maps[-1].landmarks
        assert latest and all(m.type == 1 for m in latest), (
            f"Cross-session landmarks incorrectly retained: {latest}")
        assert any(math.isclose(m.position.x, -47.8, abs_tol=.5)
                   for m in latest), "FlatAsphalt coordinate transform incorrect"
        assert mapper.reset_count >= 1
        assert len(paths[-1].poses) <= 2, "Trajectory should reset on teleport"
        print("ROS SLAM integration PASS: both coordinate profiles, cone mapping, "
              "Hungarian/Kalman confirmations, trajectory, status, teleport reset")
    finally:
        ex.shutdown()
        for node in (probe, trajectory, mapper):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    run()
