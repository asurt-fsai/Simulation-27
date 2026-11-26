"""RViz 2D Goal Pose -> map-frame Dubins path for existing ASURT Pure Pursuit.

Accepts map or base_link goal PoseStamped (the latter is common with the
existing Perception RViz view). A clicked local goal is anchored to map using
timestamp-matched odometry, once, rather than moving with the vehicle.
"""
from collections import deque
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from .geometry import Pose2, angle, dubins_path, local_goal_to_map


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


def fresh(now, stamp, max_age_ns):
    return stamp > 0 and -200_000_000 <= now-stamp <= max_age_ns


def planar_yaw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm = sum(float(v)*float(v) for v in values)
    if not all(math.isfinite(v) for v in values) or not .98 <= norm <= 1.02:
        raise ValueError("invalid_pose_quaternion")
    return math.atan2(2*(quaternion.w*quaternion.z+quaternion.x*quaternion.y),
                      1-2*(quaternion.y**2+quaternion.z**2))


class GoalPosePlanner(Node):
    def __init__(self):
        super().__init__("asurt_goal_pose_planner")
        params = (
            ("goal_topic", "/goal_pose"),
            ("odometry_topic", "/odom"),
            ("path_topic", "/planning/goal_pose/path"),
            ("planning_period_s", .15),
            ("max_odom_age_s", .5),
            ("max_goal_click_age_s", 1.5),
            ("base_link_goal_tolerance_s", .25),
            ("teleport_distance_m", 8.),
            ("wheelbase_m", 1.575),
            ("max_steer_rad", .45),
            ("path_resolution_m", .3),
            ("max_path_length_m", 100.),
            ("max_goal_distance_m", 50.),
            ("goal_position_tolerance_m", .65),
            ("goal_heading_tolerance_rad", .45),
        )
        for key, default in params:
            self.declare_parameter(key, default)
        p = lambda k: self.get_parameter(k).value
        self.period = float(p("planning_period_s"))
        self.odom_max_ns = int(float(p("max_odom_age_s"))*1e9)
        self.click_max_ns = int(float(p("max_goal_click_age_s"))*1e9)
        self.source_tolerance_ns = int(float(p("base_link_goal_tolerance_s"))*1e9)
        self.teleport_distance = float(p("teleport_distance_m"))
        self.radius = float(p("wheelbase_m"))/math.tan(float(p("max_steer_rad")))
        self.resolution = float(p("path_resolution_m"))
        self.max_length = float(p("max_path_length_m"))
        self.max_goal_range = float(p("max_goal_distance_m"))
        self.goal_tolerance = float(p("goal_position_tolerance_m"))
        self.heading_tolerance = float(p("goal_heading_tolerance_rad"))
        if (not 0.02 <= self.period <= 1. or self.odom_max_ns <= 0
                or self.click_max_ns <= 0 or self.source_tolerance_ns <= 0
                or not 0 < self.radius < 100 or not 0 < self.resolution <= 1
                or not 1 <= self.max_length <= 500
                or not 1 <= self.max_goal_range <= self.max_length
                or not .1 <= self.goal_tolerance <= 2.
                or not .1 <= self.heading_tolerance <= 1.):
            raise ValueError("invalid_goal_pose_parameters")

        self.goal_topic = str(p("goal_topic"))
        self.odom_topic = str(p("odometry_topic"))
        self.path_topic = str(p("path_topic"))
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.status_pub = self.create_publisher(
            String, "/planning/goal_pose/status", 10)
        self.goal_marker_pub = self.create_publisher(
            Marker, "/planning/goal_pose/goal_marker", 10)
        self.goal = None
        self.route = ()
        self.route_length = None
        self.route_word = None
        self.state = "waiting_for_goal"
        self.click_time_source = None
        self.odom = None
        self.odom_stamp = 0
        self.history = deque(maxlen=256)
        self.last_clock = 0

        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, "/planning/goal_pose/cancel",
                                 self._on_cancel, 10)
        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            "RViz Goal Pose -> Dubins: %s -> %s (map frame, free space)" %
            (self.goal_topic, self.path_topic))

    def _clear(self, reason):
        self.goal = None
        self.route = ()
        self.route_length = None
        self.route_word = None
        self.state = reason
        self.click_time_source = None

    def _on_cancel(self, message):
        if message.data:
            self._clear("cancelled")

    def _on_odom(self, message):
        if (message.header.frame_id.lstrip("/") != "map"
                or message.child_frame_id.lstrip("/") != "base_link"):
            return
        stamp = stamp_ns(message.header.stamp)
        pos = message.pose.pose.position
        if not all(math.isfinite(x) for x in (pos.x, pos.y)):
            return
        try:
            yaw = planar_yaw(message.pose.pose.orientation)
        except ValueError:
            return
        pose = Pose2(float(pos.x), float(pos.y), yaw)
        if self.odom is not None and (
            stamp < self.odom_stamp-100_000_000
            or math.hypot(pose.x-self.odom.x, pose.y-self.odom.y) >
                self.teleport_distance
        ):
            self.history.clear()
            self.odom = None
            self.odom_stamp = 0
            self._clear("odometry_rewind_or_teleport")
        if stamp <= self.odom_stamp:
            return
        self.odom, self.odom_stamp = pose, stamp
        self.history.append((stamp, pose))

    def _source_pose(self, click_stamp):
        if not self.history:
            return None
        closest, pose = min(self.history, key=lambda item: abs(item[0]-click_stamp))
        return pose if abs(closest-click_stamp) <= self.source_tolerance_ns else None

    def _on_goal(self, message):
        # A new click supersedes the previous goal, even when malformed.
        self._clear("invalid_goal")
        now = self.get_clock().now().nanoseconds
        if not fresh(now, self.odom_stamp, self.odom_max_ns):
            self.state = "waiting_for_fresh_odometry"
            return
        clicked = stamp_ns(message.header.stamp)
        if fresh(now, clicked, self.click_max_ns):
            self.click_time_source = "rviz_sim_stamp"
        else:
            # The SetGoal tool is an interactive, volatile ROS message.
            # RViz may use wall time while CARLA and this node use /clock.
            # Anchor the click to its actual receipt, not a mismatched clock.
            clicked = now
            self.click_time_source = "planner_receipt_clock"
            self.get_logger().warn(
                "RViz goal timestamp uses another clock; anchoring at receipt",
                throttle_duration_sec=5.0)
        pos = message.pose.position
        if not all(math.isfinite(v) for v in (pos.x, pos.y)):
            self.state = "invalid_goal_position"
            return
        try:
            yaw = planar_yaw(message.pose.orientation)
        except ValueError:
            self.state = "invalid_goal_orientation"
            return
        local = Pose2(float(pos.x), float(pos.y), yaw)
        frame = message.header.frame_id.lstrip("/")
        if frame == "map":
            goal = local
        elif frame == "base_link":
            source = self._source_pose(clicked)
            if source is None:
                self.state = "missing_click_time_odometry"
                return
            goal = local_goal_to_map(local, source)
        else:
            self.state = "unsupported_goal_frame"
            return

        distance = math.hypot(goal.x-self.odom.x, goal.y-self.odom.y)
        if distance > self.max_goal_range:
            self.state = "goal_too_far"
            return
        self.goal = goal
        if (distance <= self.goal_tolerance and
                abs(angle(goal.yaw-self.odom.yaw)) <= self.heading_tolerance):
            self.state = "arrived"
            return
        points, length, word = dubins_path(
            self.odom, goal, self.radius, self.resolution, self.max_length)
        if not points:
            self._clear(word)
            return
        self.route, self.route_length, self.route_word = points, length, word
        self.state = "tracking"

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock and now < self.last_clock-100_000_000:
            self.history.clear()
            self.odom = None
            self.odom_stamp = 0
            self._clear("simulation_clock_rewind")
        self.last_clock = now
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        current = self.state
        distance = heading_error = None
        odom_ok = fresh(now, self.odom_stamp, self.odom_max_ns)
        if self.goal is not None and odom_ok:
            distance = math.hypot(self.goal.x-self.odom.x,
                                  self.goal.y-self.odom.y)
            heading_error = abs(angle(self.goal.yaw-self.odom.yaw))
            if (distance <= self.goal_tolerance
                    and heading_error <= self.heading_tolerance):
                self.route = ()
                self.state = "arrived"
        if self.route:
            current = "tracking" if odom_ok else "stale_odometry"
            if odom_ok:
                for pose in self.route:
                    point = PoseStamped()
                    point.header = path.header
                    point.pose.position.x, point.pose.position.y = pose.x, pose.y
                    point.pose.orientation.z = math.sin(pose.yaw/2.)
                    point.pose.orientation.w = math.cos(pose.yaw/2.)
                    path.poses.append(point)
        else:
            current = self.state
        self.path_pub.publish(path)  # empty on arrival/cancel/invalid/stale
        marker = Marker()
        marker.header = path.header
        marker.ns = "rviz_goal_pose_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD if self.goal is not None else Marker.DELETE
        marker.pose.orientation.w = 1.
        marker.scale.x, marker.scale.y, marker.scale.z = 1.0, .15, .15
        marker.color.r, marker.color.g, marker.color.b = 1., .65, .1
        marker.color.a = 1.
        if self.goal is not None:
            marker.pose.position.x = self.goal.x
            marker.pose.position.y = self.goal.y
            marker.pose.orientation.z = math.sin(self.goal.yaw/2.)
            marker.pose.orientation.w = math.cos(self.goal.yaw/2.)
        self.goal_marker_pub.publish(marker)
        self.status_pub.publish(String(data=json.dumps({
            "state": current,
            "click_time_source": self.click_time_source,
            "goal_frame": "map" if self.goal else None,
            "goal_xyyaw": [round(self.goal.x, 3), round(self.goal.y, 3),
                           round(self.goal.yaw, 3)] if self.goal else None,
            "distance_m": round(distance, 3) if distance is not None else None,
            "heading_error_rad": (round(heading_error, 3)
                                  if heading_error is not None else None),
            "path_length_m": (round(self.route_length, 3)
                              if self.route_length is not None else None),
            "dubins_word": self.route_word,
            "path_points": len(path.poses),
            "no_obstacle_check": True,
        }, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
