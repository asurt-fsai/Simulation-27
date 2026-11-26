"""Independent CARLA Pure Pursuit controller for existing ROS 2 planning paths.

Only this node may publish its own Ackermann commands. The established bridge
continues to own the hero, world clock, odometry and ROS/manual arbitration.
"""
from collections import deque
import json
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from .core import (Settings, VehiclePose, current_local_path,
                   lookahead_distance, ramp_speed, steering_and_speed,
                   target_on_path)


def ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


def recent(now, past, max_age):
    return past > 0 and -200_000_000 <= now-past <= max_age


class PurePursuitControl(Node):
    def __init__(self):
        super().__init__("asurt_pure_pursuit_control")
        for name, default in (
            ("path_topic", "/planning/goal_pose/path"),
            ("odometry_topic", "/odom"),
            ("autonomous_mode_topic", "/sim/feedback/autonomous_mode"),
            ("drive_topic", "/drive"),
            ("output_enabled", True),
            ("update_period_s", 0.1),
            ("max_path_age_s", 1.25),
            ("max_odom_age_s", .4),
            ("max_mode_age_s", 1.0),
            ("source_odom_tolerance_s", .18),
            ("teleport_distance_m", 8.0),
        ):
            self.declare_parameter(name, default)
        for key, val in vars(Settings()).items():
            self.declare_parameter(key, val)
        p = lambda key: self.get_parameter(key).value
        self.cfg = Settings(**{k: float(p(k)) for k in vars(Settings())})
        self.cfg.validate()
        self.path_topic = str(p("path_topic"))
        self.odom_topic = str(p("odometry_topic"))
        self.mode_topic = str(p("autonomous_mode_topic"))
        self.drive_topic = str(p("drive_topic"))
        self.output_enabled = bool(p("output_enabled"))
        self.period = float(p("update_period_s"))
        self.path_max_ns = int(float(p("max_path_age_s"))*1e9)
        self.odom_max_ns = int(float(p("max_odom_age_s"))*1e9)
        self.mode_max_ns = int(float(p("max_mode_age_s"))*1e9)
        self.source_tolerance_ns = int(float(p("source_odom_tolerance_s"))*1e9)
        self.teleport_distance = float(p("teleport_distance_m"))
        if (not .02 <= self.period <= 1. or self.path_max_ns <= 0
                or self.odom_max_ns <= 0 or self.mode_max_ns <= 0
                or not 0 < self.source_tolerance_ns <= self.path_max_ns
                or self.teleport_distance <= 0 or not self.path_topic.startswith("/")
                or not self.odom_topic.startswith("/") or not self.mode_topic.startswith("/")
                or not self.drive_topic.startswith("/")):
            raise ValueError("invalid_pure_pursuit_ros_configuration")

        self.path = None
        self.path_frame = ""
        self.path_stamp = 0
        self.odom_history = deque(maxlen=256)
        self.odom_pose = None
        self.odom_stamp = 0
        self.speed = 0.
        self.mode = None
        self.mode_stamp = 0
        self.last_clock = 0
        self.last_speed_command = 0.
        self.last_tick_ns = 0
        self.drive_active_before = False
        self.driven_history = deque(maxlen=600)
        self.status_pub = self.create_publisher(
            String, "/control/pure_pursuit/status", 10)
        self.preview_pub = self.create_publisher(
            AckermannDriveStamped, "/control/pure_pursuit/command_preview", 10)
        self.target_pub = self.create_publisher(
            Marker, "/control/pure_pursuit/target", 10)
        self.driven_pub = self.create_publisher(
            Path, "/control/pure_pursuit/driven_path", 10)
        self.text_pub = self.create_publisher(
            Marker, "/control/pure_pursuit/diagnostic_marker", 10)
        self.drive_pub = (self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic, 1)
                          if self.output_enabled else None)
        self.create_subscription(Path, self.path_topic, self._on_path, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, self.mode_topic, self._on_mode, 10)
        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            "Pure Pursuit: %s -> %s; output_enabled=%s" %
            (self.path_topic, self.drive_topic, self.output_enabled))

    def _reset(self):
        self.path = None
        self.path_stamp = 0
        self.odom_history.clear()
        self.odom_pose = None
        self.odom_stamp = 0
        self.last_speed_command = 0.
        self.last_tick_ns = 0
        self.driven_history.clear()

    def _on_path(self, message):
        frame = message.header.frame_id.lstrip("/")
        stamp = ns(message.header.stamp)
        points = tuple((float(p.pose.position.x), float(p.pose.position.y))
                       for p in message.poses)
        if (frame not in ("base_link", "map") or stamp <= 0
                or not 2 <= len(points) <= 600
                or any(not (math.isfinite(x) and math.isfinite(y))
                       for x, y in points)):
            self.path = None
            self.path_stamp = 0
            return
        self.path = points
        self.path_frame = frame
        self.path_stamp = stamp

    def _on_odom(self, message):
        if (message.header.frame_id.lstrip("/") != "map"
                or message.child_frame_id.lstrip("/") != "base_link"):
            return
        stamp = ns(message.header.stamp)
        pos = message.pose.pose.position
        q = message.pose.pose.orientation
        values = (pos.x, pos.y, q.x, q.y, q.z, q.w,
                  message.twist.twist.linear.x, message.twist.twist.linear.y)
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if stamp <= 0 or not all(math.isfinite(v) for v in values) or not .98 <= norm <= 1.02:
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),
                         1-2*(q.y*q.y+q.z*q.z))
        pose = VehiclePose(float(pos.x), float(pos.y), yaw)
        if self.odom_pose is not None:
            if (stamp < self.odom_stamp-100_000_000 or
                    math.hypot(pose.x-self.odom_pose.x,
                               pose.y-self.odom_pose.y) > self.teleport_distance):
                self._reset()
        if stamp <= self.odom_stamp:
            return
        self.odom_pose, self.odom_stamp = pose, stamp
        self.speed = math.hypot(message.twist.twist.linear.x,
                                message.twist.twist.linear.y)
        self.odom_history.append((stamp, pose))
        if self.mode is True and (
                not self.driven_history or
                math.hypot(pose.x-self.driven_history[-1][1].x,
                           pose.y-self.driven_history[-1][1].y) >= .1 or
                stamp-self.driven_history[-1][0] >= 500_000_000):
            self.driven_history.append((stamp, pose))

    def _on_mode(self, message):
        if bool(message.data) and self.mode is not True:
            self.driven_history.clear()
        self.mode = bool(message.data)
        self.mode_stamp = self.get_clock().now().nanoseconds

    def _source_pose(self, stamp):
        if not self.odom_history:
            return None
        t, pose = min(self.odom_history, key=lambda item: abs(item[0]-stamp))
        return pose if abs(t-stamp) <= self.source_tolerance_ns else None

    def _command(self, now, speed=0., steer=0.):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        # Zero requests CARLA's existing native Ackermann speed controller,
        # rather than replaying legacy PID throttle output.
        msg.drive.acceleration = 0.
        msg.drive.jerk = 0.
        msg.drive.steering_angle_velocity = 0.
        return msg

    def _target_marker(self, target, now):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "control_pure_pursuit_target"
        marker.id = 0
        marker.action = Marker.ADD if target else Marker.DELETE
        marker.type = Marker.POINTS  # small point, not a cone/Perception sphere
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = .18
        marker.color.g = 1.
        marker.color.a = 1.
        if target:
            marker.points = [Point(x=float(target[0]), y=float(target[1]), z=.15)]
        self.target_pub.publish(marker)

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock and now < self.last_clock - 100_000_000:
            self._reset()
            self.mode_stamp = 0
        self.last_clock = now
        reason = "waiting_for_clock"
        target = None
        steer = speed = curvature = 0.
        mode_fresh = recent(now, self.mode_stamp, self.mode_max_ns)
        ros_mode = mode_fresh and self.mode is True
        if now > 0:
            if not recent(now, self.odom_stamp, self.odom_max_ns):
                reason = "waiting_for_odom"
            elif self.path is None:
                reason = "waiting_for_path"
            elif not recent(now, self.path_stamp, self.path_max_ns):
                reason = "stale_path"
            else:
                source_pose = (self._source_pose(self.path_stamp)
                               if self.path_frame == "base_link" else None)
                if self.path_frame == "base_link" and source_pose is None:
                    reason = "missing_source_time_odometry"
                else:
                    local = current_local_path(self.path, self.path_frame,
                                               self.odom_pose, source_pose)
                    target = target_on_path(
                        local, lookahead_distance(self.speed, self.cfg))
                    if target is None:
                        reason = "no_forward_target"
                    else:
                        steer, requested, curvature = steering_and_speed(target, self.cfg)
                        dt = (max(.0, min(.5, (now-self.last_tick_ns)*1e-9))
                              if self.last_tick_ns else self.period)
                        speed = ramp_speed(self.last_speed_command, requested,
                                           self.cfg.max_accel_mps2, dt)
                        reason = "tracking"
        # Preview-only mode should accumulate the same speed ramp as live
        # output, so its diagnostic command remains representative.
        if not ros_mode:
            self.last_speed_command = 0.
        else:
            self.last_speed_command = speed
        self.last_tick_ns = now
        cmd = self._command(now, speed, steer)
        self.preview_pub.publish(cmd)
        self._target_marker(target, now)
        driven = Path()
        driven.header.frame_id = "map"
        driven.header.stamp = self.get_clock().now().to_msg()
        for sample_stamp, pose in self.driven_history:
            point = PoseStamped()
            point.header.frame_id = "map"
            point.header.stamp.sec = sample_stamp//1_000_000_000
            point.header.stamp.nanosec = sample_stamp%1_000_000_000
            point.pose.position.x, point.pose.position.y = pose.x, pose.y
            point.pose.orientation.z = math.sin(pose.yaw/2)
            point.pose.orientation.w = math.cos(pose.yaw/2)
            driven.poses.append(point)
        self.driven_pub.publish(driven)
        diagnostic = Marker()
        diagnostic.header.frame_id = "base_link"
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        diagnostic.ns = "pure_pursuit_test_diagnostics"
        diagnostic.id = 0
        diagnostic.type = Marker.TEXT_VIEW_FACING
        diagnostic.action = Marker.ADD
        diagnostic.pose.position.x = 0.
        diagnostic.pose.position.y = 2.
        diagnostic.pose.position.z = 1.6
        diagnostic.pose.orientation.w = 1.
        diagnostic.scale.z = .36
        diagnostic.color.r = .25
        diagnostic.color.g = 1.
        diagnostic.color.b = .75
        diagnostic.color.a = 1.
        diagnostic.text = (
            "Control: %s | %s\nActual %.2f m/s | Requested %.2f m/s | Steering %.1f deg" %
            ("ROS" if ros_mode else "MANUAL", reason,
             self.speed, speed, math.degrees(steer))
        )
        self.text_pub.publish(diagnostic)
        # The Pygame/manual bridge is the authority selector. In manual mode,
        # do not issue competing commands. In ROS mode, send zero if no path.
        sent = self.drive_pub is not None and (ros_mode or self.drive_active_before)
        if sent:
            self.drive_pub.publish(cmd if ros_mode else self._command(now))
        self.drive_active_before = bool(ros_mode)
        self.status_pub.publish(String(data=json.dumps({
            "state": reason,
            "path_topic": self.path_topic,
            "path_frame": self.path_frame,
            "path_age_s": round(max(0, now-self.path_stamp)*1e-9, 3) if self.path_stamp else None,
            "odom_age_s": round(max(0, now-self.odom_stamp)*1e-9, 3) if self.odom_stamp else None,
            "ros_mode": bool(ros_mode),
            "output_enabled": self.output_enabled,
            "command_sent": bool(sent),
            "steer_rad": round(steer, 5),
            "speed_mps": round(speed, 4),
            "curvature": round(curvature, 5),
            "target_base_link": [round(c, 4) for c in target] if target else None,
        }, sort_keys=True)))

    def destroy_node(self):
        # The current bridge holds its last received /drive command. Reset it
        # when the node is stopped via System Manager, while ROS is still up.
        if self.drive_pub is not None and self.mode is True and rclpy.ok():
            for _ in range(3):
                self.drive_pub.publish(self._command(self.get_clock().now().nanoseconds))
                time.sleep(.025)
        super().destroy_node()


def main(args=None):
    # Let SIGINT run our cleanup before the ROS publisher is shut down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PurePursuitControl()
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
