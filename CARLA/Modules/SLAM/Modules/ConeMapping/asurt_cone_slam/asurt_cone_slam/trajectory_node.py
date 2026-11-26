"""Read-only odometry path visualization and distance from existing CARLA bridge."""
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

from .cone_mapper_node import stamp_seconds


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__("asurt_slam_trajectory")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_path_poses", 2400)
        self.declare_parameter("min_path_step_m", .15)
        self.declare_parameter("reset_on_pose_jump_m", 30.)
        self.frame = self.get_parameter("map_frame").value
        self.max_step = float(self.get_parameter("min_path_step_m").value)
        self.jump = float(self.get_parameter("reset_on_pose_jump_m").value)
        self.path = deque(maxlen=int(self.get_parameter("max_path_poses").value))
        self.latest = None
        self.last_xyz = None
        self.last_path_xyz = None
        self.last_stamp = None
        self.distance = 0.
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self.on_odometry, qos_profile_sensor_data)
        self.path_pub = self.create_publisher(Path, "/slam/path", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/slam/pose", 10)
        self.distance_pub = self.create_publisher(Float32, "/slam/distance", 10)
        self.create_timer(.5, self.publish)

    def on_odometry(self, msg):
        if msg.header.frame_id.lstrip("/") != self.frame:
            return
        xyz = (msg.pose.pose.position.x, msg.pose.pose.position.y,
               msg.pose.pose.position.z)
        stamp = stamp_seconds(msg.header.stamp)
        if not all(math.isfinite(a) for a in xyz):
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp - .5:
                self.path.clear()
                self.distance = 0.
                self.last_xyz = None
                self.last_path_xyz = None
            else:
                return
        if self.last_xyz is not None:
            delta = math.dist(xyz, self.last_xyz)
            if delta >= self.jump:
                self.path.clear()
                self.distance = 0.
                self.last_path_xyz = None
            else:
                self.distance += delta
        self.last_stamp = stamp
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.latest = pose
        if (not self.path or self.last_path_xyz is None or
            math.dist(xyz, self.last_path_xyz) >= self.max_step):
            self.path.append(pose)
            self.last_path_xyz = xyz
        self.last_xyz = xyz

    def publish(self):
        if self.latest is None:
            return
        path = Path()
        path.header = self.latest.header
        path.poses = list(self.path)
        self.path_pub.publish(path)
        self.pose_pub.publish(self.latest)
        self.distance_pub.publish(Float32(data=float(self.distance)))


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
