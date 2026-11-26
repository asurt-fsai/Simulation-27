"""Independent high-speed Stanley controller for SLAM track-goal paths.

This does NOT reuse the Pure Pursuit /drive publisher. It only issues
Ackermann commands while the bridge signals ROS mode and while no other
/drive publisher exists. Missing/slipping planner input requests zero speed.
"""
from collections import deque
from dataclasses import replace
import json
import math
import signal
import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from .core import MAX_TRACK_PATH_POINTS, Pose2, Settings, speed_ramp, track_command


def stamp_ns(stamp):
    return int(stamp.sec)*1000000000+int(stamp.nanosec)


def fresh(now, stamp, max_age_ns):
    return stamp > 0 and -200000000 <= now-stamp <= max_age_ns


class StanleyController(Node):
    def __init__(self):
        super().__init__("asurt_stanley_control")
        for key, default in (
            ("path_topic", "/planning/track_goal/path"),
            ("planner_status_topic", "/planning/track_goal/status"),
            ("odometry_topic", "/odom"),
            ("mode_topic", "/sim/feedback/autonomous_mode"),
            ("drive_topic", "/drive"),
            ("output_enabled", True),
            ("update_period_s", .05),
            ("max_odom_age_s", .4),
            ("max_path_age_s", .75),
            ("max_status_age_s", .75),
            ("max_mode_age_s", .75),
            ("teleport_distance_m", 8.),
        ):
            self.declare_parameter(key, default)
        for name, value in vars(Settings()).items():
            self.declare_parameter(name, value)
        p = lambda key: self.get_parameter(key).value
        self.cfg = Settings(**{name: float(p(name))
                               for name in vars(Settings())})
        self.cfg.validate()
        self.tunable = frozenset(vars(self.cfg))
        self.add_on_set_parameters_callback(self.on_parameters)
        self.path_topic = str(p("path_topic"))
        self.drive_topic = str(p("drive_topic"))
        self.output_enabled = bool(p("output_enabled"))
        self.period = float(p("update_period_s"))
        self.max_odom_ns = int(float(p("max_odom_age_s"))*1e9)
        self.max_path_ns = int(float(p("max_path_age_s"))*1e9)
        self.max_status_ns = int(float(p("max_status_age_s"))*1e9)
        self.max_mode_ns = int(float(p("max_mode_age_s"))*1e9)
        self.teleport_m = float(p("teleport_distance_m"))
        if (not .02 <= self.period <= .5 or
                min(self.max_odom_ns, self.max_path_ns,
                    self.max_status_ns, self.max_mode_ns) <= 0 or
                self.teleport_m <= 0 or not self.drive_topic.startswith("/")):
            raise ValueError("invalid_stanley_ros_config")
        self.odom = None
        self.odom_stamp = 0
        self.actual_speed = 0.
        self.path = ()
        self.path_stamp = 0
        self.path_frame = ""
        self.planner_ready = False
        self.planner_stamp = 0
        self.planner_state = "waiting_for_planner"
        self.mode = None
        self.mode_stamp = 0
        self.was_driving = False
        self.last_clock = 0
        self.last_tick = 0
        self.last_command_speed = 0.
        self.trail = deque(maxlen=600)
        self.status_pub = self.create_publisher(
            String, "/control/stanley/status", 10)
        self.preview_pub = self.create_publisher(
            AckermannDriveStamped, "/control/stanley/command_preview", 10)
        self.target_pub = self.create_publisher(
            Marker, "/control/stanley/target", 10)
        self.text_pub = self.create_publisher(
            Marker, "/control/stanley/diagnostic_marker", 10)
        self.trail_pub = self.create_publisher(
            Path, "/control/stanley/driven_path", 10)
        self.drive_pub = (self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 1)
            if self.output_enabled else None)
        self.create_subscription(
            Path, self.path_topic, self.on_path, 10)
        self.create_subscription(
            String, p("planner_status_topic"), self.on_planner, 10)
        self.create_subscription(
            Odometry, p("odometry_topic"), self.on_odom,
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, p("mode_topic"), self.on_mode, 10)
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            "Stanley track control -> /drive: up to %.0f km/h, independent "
            "of existing Pure Pursuit" % (self.cfg.target_speed_mps*3.6))

    def on_parameters(self, parameters):
        updates = {}
        for parameter in parameters:
            if parameter.name not in self.tunable:
                continue
            if isinstance(parameter.value, bool):
                return SetParametersResult(successful=False, reason="no_boolean_tuning")
            try:
                updates[parameter.name] = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(successful=False, reason="invalid_tuning_type")
        if updates:
            try:
                candidate = replace(self.cfg, **updates)
                candidate.validate()
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
            self.cfg = candidate
            self.get_logger().info(
                "Stanley test limit now %.1f km/h" %
                (self.cfg.target_speed_mps*3.6))
        return SetParametersResult(successful=True)

    def reset(self):
        self.path, self.path_stamp = (), 0
        self.planner_ready = False
        self.planner_stamp = 0
        self.odom, self.odom_stamp = None, 0
        self.last_command_speed = 0.
        self.last_tick = 0
        self.trail.clear()

    def on_path(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        points = tuple((float(p.pose.position.x), float(p.pose.position.y))
                       for p in msg.poses)
        if (msg.header.frame_id.lstrip("/") != "map" or stamp <= 0
                or not 3 <= len(points) <= MAX_TRACK_PATH_POINTS
                or any(not all(math.isfinite(v) for v in point)
                       for point in points)):
            self.path, self.path_stamp = (), 0
            return
        self.path, self.path_stamp = points, stamp
        self.path_frame = "map"

    def on_planner(self, msg):
        try:
            state = json.loads(msg.data)
            self.planner_ready = (state["state"] == "tracking"
                                  and state["valid_route"] is True)
            self.planner_state = str(state["state"])
        except (ValueError, KeyError, TypeError):
            self.planner_ready = False
            self.planner_state = "invalid_planner_status"
        self.planner_stamp = self.get_clock().now().nanoseconds

    def on_odom(self, msg):
        if (msg.header.frame_id.lstrip("/") != "map"
                or msg.child_frame_id.lstrip("/") != "base_link"):
            return
        stamp = stamp_ns(msg.header.stamp)
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        values = (position.x, position.y, q.x, q.y, q.z, q.w,
                  msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if stamp <= 0 or not all(math.isfinite(v) for v in values) or not .98 <= norm <= 1.02:
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),
                         1-2*(q.y*q.y+q.z*q.z))
        pose = Pose2(float(position.x), float(position.y), yaw)
        if self.odom is not None and (
                stamp < self.odom_stamp-100000000 or
                math.hypot(pose.x-self.odom.x, pose.y-self.odom.y) >
                    self.teleport_m):
            self.reset()
        if stamp <= self.odom_stamp:
            return
        self.odom, self.odom_stamp = pose, stamp
        self.actual_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if self.mode is True and (
                not self.trail or stamp-self.trail[-1][0] > 150000000 or
                math.hypot(pose.x-self.trail[-1][1].x,
                           pose.y-self.trail[-1][1].y) > .4):
            self.trail.append((stamp, pose))

    def on_mode(self, msg):
        if msg.data and self.mode is not True:
            self.trail.clear()
        self.mode = bool(msg.data)
        self.mode_stamp = self.get_clock().now().nanoseconds

    def command(self, speed=0., steer=0.):
        msg = AckermannDriveStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        return msg

    def publish_visuals(self, metrics, speed, steer, reason, ros_mode):
        now_msg = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = now_msg
        marker.ns, marker.id = "stanley_front_axle_projection", 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD if metrics else Marker.DELETE
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = .17
        marker.color.r, marker.color.g, marker.color.a = .2, 1., 1.
        if metrics:
            marker.pose.position.x, marker.pose.position.y = metrics["nearest_point"]
            marker.pose.position.z = .15
        self.target_pub.publish(marker)
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = now_msg
        text.ns, text.id = "stanley_track_control_label", 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.y, text.pose.position.z = 2.5, 1.7
        text.pose.orientation.w = 1.
        text.scale.z = .3
        text.color.r, text.color.g, text.color.b, text.color.a = .8, 1., .5, 1.
        text.text = (
            "Stanley: %s | %s\nActual %.1f km/h | Cmd %.1f km/h | steer %.1f deg\nCTE %.2f m" %
            ("ROS" if ros_mode else "MANUAL", reason,
             self.actual_speed*3.6, speed*3.6, math.degrees(steer),
             metrics["cross_track_error_m"] if metrics else 0.))
        self.text_pub.publish(text)
        trail = Path()
        trail.header.frame_id = "map"
        trail.header.stamp = now_msg
        for sample_time, pose in self.trail:
            point = PoseStamped()
            point.header.frame_id = "map"
            point.header.stamp.sec = sample_time//1000000000
            point.header.stamp.nanosec = sample_time%1000000000
            point.pose.position.x, point.pose.position.y = pose.x, pose.y
            point.pose.orientation.z = math.sin(pose.yaw/2.)
            point.pose.orientation.w = math.cos(pose.yaw/2.)
            trail.poses.append(point)
        self.trail_pub.publish(trail)

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock and now < self.last_clock-100000000:
            self.reset()
            self.mode_stamp = 0
        self.last_clock = now
        ros_mode = self.mode is True and fresh(now, self.mode_stamp, self.max_mode_ns)
        reason = "waiting_for_sim_clock"
        steer, speed = 0., 0.
        metrics = None
        conflict = self.drive_pub is not None and self.count_publishers(self.drive_topic) > 1
        if now > 0:
            if not fresh(now, self.odom_stamp, self.max_odom_ns):
                reason = "waiting_for_fresh_odometry"
            elif not fresh(now, self.planner_stamp, self.max_status_ns):
                reason = "waiting_for_track_planner_status"
            elif not self.planner_ready:
                reason = "planner_"+self.planner_state
            elif not self.path:
                reason = "waiting_for_valid_track_path"
            elif not fresh(now, self.path_stamp, self.max_path_ns):
                reason = "stale_track_path"
            else:
                try:
                    steer, wanted, metrics = track_command(
                        self.path, self.odom, self.actual_speed, self.cfg)
                    dt = (max(0., min(.3, (now-self.last_tick)/1e9))
                          if self.last_tick else self.period)
                    speed = speed_ramp(self.last_command_speed, wanted,
                                       self.cfg.max_accel_mps2, dt)
                    reason = "tracking"
                except ValueError as exc:
                    reason = str(exc)
        self.last_tick = now
        self.last_command_speed = speed if ros_mode and not conflict else 0.
        msg = self.command(speed, steer)
        self.preview_pub.publish(msg)
        sent = bool(self.drive_pub is not None and
                    not conflict and (ros_mode or self.was_driving))
        if sent:
            self.drive_pub.publish(msg if ros_mode else self.command())
        self.was_driving = bool(ros_mode and not conflict)
        if conflict:
            reason = "another_drive_publisher_active"
        self.publish_visuals(metrics, speed, steer, reason, ros_mode)
        self.status_pub.publish(String(data=json.dumps({
            "state": reason, "ros_mode": bool(ros_mode),
            "has_command_authority": not conflict,
            "output_enabled": bool(self.output_enabled),
            "command_sent": sent,
            "target_speed_kmh": round(speed*3.6, 2),
            "actual_speed_kmh": round(self.actual_speed*3.6, 2),
            "configured_max_speed_kmh": round(self.cfg.target_speed_mps*3.6, 2),
            "steering_deg": round(math.degrees(steer), 2),
            "path_age_s": (round((now-self.path_stamp)/1e9, 3)
                           if self.path_stamp else None),
            "cross_track_error_m": (round(metrics["cross_track_error_m"], 3)
                                    if metrics else None),
        }, sort_keys=True)))

    def destroy_node(self):
        if (self.drive_pub is not None and self.mode is True and
                rclpy.ok() and self.count_publishers(self.drive_topic) == 1):
            for _ in range(3):
                self.drive_pub.publish(self.command())
                time.sleep(.03)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = StanleyController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    shutting_down = False
    previous_signal = signal.getsignal(signal.SIGINT)

    def request_shutdown(_signum, _frame):
        nonlocal shutting_down
        shutting_down = True

    signal.signal(signal.SIGINT, request_shutdown)
    try:
        while rclpy.ok() and not shutting_down:
            executor.spin_once(timeout_sec=.12)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, previous_signal)
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
