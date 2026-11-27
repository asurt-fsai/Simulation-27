#!/usr/bin/python3
"""Select the section of the classical cone route ending at an RViz goal."""
from copy import deepcopy
from math import hypot

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


def route_to_goal(route, goal):
    selected = Path(header=route.header)
    target = goal.pose.position
    nearest = None
    for index, (start, end) in enumerate(zip(route.poses, route.poses[1:])):
        a, b = start.pose.position, end.pose.position
        dx, dy = b.x - a.x, b.y - a.y
        length_squared = dx * dx + dy * dy
        if not length_squared:
            continue
        fraction = max(0.0, min(1.0, ((target.x - a.x) * dx + (target.y - a.y) * dy) / length_squared))
        x, y = a.x + fraction * dx, a.y + fraction * dy
        error = hypot(target.x - x, target.y - y)
        if nearest is None or error < nearest[0]:
            nearest = (error, index, x, y, fraction)
    if nearest and nearest[0] <= 1.5:
        _, index, x, y, fraction = nearest
        selected.poses = list(route.poses[:index + 1])
        if fraction > 0:
            endpoint = deepcopy(route.poses[index + 1])
            endpoint.pose.position.x, endpoint.pose.position.y = x, y
            selected.poses.append(endpoint)
        if len(selected.poses) < 2:
            selected.poses = []
    return selected


class ClassicalGoal(Node):
    def __init__(self):
        super().__init__("asurt_classical_goal")
        self.goal = None
        self.route = None
        self.output = self.create_publisher(Path, "/planning/goal_pose/path", 10)
        self.marker = self.create_publisher(Marker, "/planning/goal_pose/goal_marker", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.receive_goal, 10)
        self.create_subscription(Path, "/planning/classical/path", self.receive_route, 10)
        self.create_subscription(Bool, "/planning/goal_pose/cancel", self.cancel, 10)

    def receive_goal(self, goal):
        if goal.header.frame_id.lstrip("/") == "map":
            self.goal = goal
            marker = Marker(header=goal.header, pose=goal.pose, type=Marker.ARROW, action=Marker.ADD)
            marker.scale.x, marker.scale.y, marker.scale.z = 1.2, 0.2, 0.2
            marker.color.r, marker.color.g, marker.color.a = 1.0, 0.6, 1.0
            self.marker.publish(marker)
            self.publish()

    def receive_route(self, route):
        if route.header.frame_id.lstrip("/") == "map":
            self.route = route
            self.publish()

    def cancel(self, message):
        if message.data:
            self.goal = None
            marker = Marker(action=Marker.DELETE)
            marker.header.frame_id = "map"
            self.marker.publish(marker)
            self.publish()

    def publish(self):
        if self.goal is not None and self.route is not None:
            self.output.publish(route_to_goal(self.route, self.goal))
        else:
            path = Path()
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            self.output.publish(path)


if __name__ == "__main__":
    rclpy.init()
    node = ClassicalGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
