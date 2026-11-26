"""Manual RViz goal -> cone-aware local corridor from the SLAM map.

The cone map is not a complete occupancy grid. An ambiguous, incomplete or
stale mapped corridor produces an empty path (not a guessed straight line).
"""
from collections import deque
import json
import math
import signal

from asurt_msgs.msg import LandmarkArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from .geometry import Config, Pose2, Result, distance, plan_track, wrap


def nanos(stamp):
    return int(stamp.sec)*1000000000+int(stamp.nanosec)


def recent(now, past, limit):
    return past > 0 and -200000000 <= now-past <= limit


def yaw_of(q):
    norm = sum(float(v)*float(v) for v in (q.x, q.y, q.z, q.w))
    if not math.isfinite(norm) or not .98 <= norm <= 1.02:
        raise ValueError("invalid_quaternion")
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class TrackGoalPlanner(Node):
    def __init__(self):
        super().__init__("asurt_track_goal_planner")
        params = (
            ("goal_topic", "/goal_pose"),
            ("map_topic", "/map/global_cones"),
            ("map_status_topic", "/slam/cone_map_status"),
            ("odometry_topic", "/odom"),
            ("planning_period_s", .3),
            ("max_odom_age_s", .55),
            ("max_map_age_s", 1.5),
            ("max_map_status_age_s", 2.0),
            ("max_detection_pause_s", 3.0),
            ("click_odom_tolerance_s", .3),
            ("teleport_distance_m", 8.),
            ("arrival_radius_m", .85),
            ("arrival_heading_rad", .55),
        )
        for name, default in params:
            self.declare_parameter(name, default)
        for name, default in vars(Config()).items():
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value
        self.cfg = Config(**{name: p(name) for name in vars(Config())})
        self.period = float(p("planning_period_s"))
        self.odom_max_ns = int(float(p("max_odom_age_s"))*1e9)
        self.map_max_ns = int(float(p("max_map_age_s"))*1e9)
        self.status_max_ns = int(float(p("max_map_status_age_s"))*1e9)
        self.progress_max_ns = int(float(p("max_detection_pause_s"))*1e9)
        self.click_tolerance_ns = int(float(p("click_odom_tolerance_s"))*1e9)
        self.teleport_distance = float(p("teleport_distance_m"))
        self.arrival_radius = float(p("arrival_radius_m"))
        self.arrival_heading = float(p("arrival_heading_rad"))
        if (not .05 <= self.period <= 1.
                or min(self.odom_max_ns, self.map_max_ns, self.status_max_ns,
                       self.progress_max_ns, self.click_tolerance_ns) <= 0
                or self.teleport_distance <= 0 or self.arrival_radius <= 0
                or not 0 < self.arrival_heading < 1.57):
            raise ValueError("invalid_track_planner_parameters")
        self.pose = None
        self.odom_stamp = 0
        self.odom_history = deque(maxlen=128)
        self.cones = ()
        self.map_stamp = 0
        self.map_frames = -1
        self.health_stamp = 0
        self.progress_stamp = 0
        self.goal = None
        self.result = Result()
        self.state = "waiting_for_goal"
        self.click_source = None
        self.last_clock = 0
        self.last_plan = 0
        self.path_pub = self.create_publisher(
            Path, "/planning/track_goal/path", 10)
        self.status_pub = self.create_publisher(
            String, "/planning/track_goal/status", 10)
        self.marker_pub = self.create_publisher(
            Marker, "/planning/track_goal/goal_marker", 10)
        self.create_subscription(PoseStamped, p("goal_topic"), self.on_goal, 10)
        self.create_subscription(LandmarkArray, p("map_topic"), self.on_map, 10)
        self.create_subscription(String, p("map_status_topic"), self.on_health, 10)
        self.create_subscription(Odometry, p("odometry_topic"), self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, "/planning/track_goal/cancel",
                                 self.on_cancel, 10)
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            "Track goal reads /goal_pose + validated /map/global_cones; "
            "publishes /planning/track_goal/path, no vehicle control")

    def clear(self, reason):
        self.goal = None
        self.state = reason
        self.result = Result(reason=reason)
        self.click_source = None

    def on_cancel(self, msg):
        if msg.data:
            self.clear("cancelled")

    def on_odom(self, msg):
        if (msg.header.frame_id.lstrip("/") != "map"
                or msg.child_frame_id.lstrip("/") != "base_link"):
            return
        stamp = nanos(msg.header.stamp)
        p = msg.pose.pose.position
        if stamp <= 0 or not all(math.isfinite(v) for v in (p.x, p.y)):
            return
        try:
            yaw = yaw_of(msg.pose.pose.orientation)
        except ValueError:
            return
        pose = Pose2(float(p.x), float(p.y), yaw)
        if self.pose is not None and (
                stamp < self.odom_stamp-100000000 or
                distance((pose.x, pose.y),
                         (self.pose.x, self.pose.y)) > self.teleport_distance):
            self.odom_history.clear()
            self.odom_stamp = 0
            self.cones = ()
            self.map_stamp = 0
            self.map_frames = -1
            self.progress_stamp = 0
            self.clear("odometry_teleport_or_rewind")
        if stamp <= self.odom_stamp:
            return
        self.pose, self.odom_stamp = pose, stamp
        self.odom_history.append((stamp, pose))

    def on_map(self, msg):
        stamp = nanos(msg.header.stamp)
        if msg.header.frame_id.lstrip("/") != "map" or stamp <= 0:
            self.cones, self.map_stamp = (), 0
            return
        if self.map_stamp and stamp < self.map_stamp-100000000:
            self.clear("map_clock_rewind")
            self.progress_stamp = 0
            self.map_frames = -1
        self.map_stamp = stamp
        self.cones = tuple(
            (float(l.position.x), float(l.position.y),
             int(l.type), float(l.probability))
            for l in msg.landmarks[:2048]
            if all(math.isfinite(v) for v in
                   (l.position.x, l.position.y, l.probability)))

    def on_health(self, msg):
        try:
            data = json.loads(msg.data)
            frames = int(data["detections_frames"])
            confirmed = int(data["confirmed"])
        except (ValueError, TypeError, KeyError):
            return
        now = self.get_clock().now().nanoseconds
        if self.map_frames >= 0 and frames < self.map_frames:
            self.clear("mapper_reset")
            self.cones, self.map_stamp = (), 0
        if frames > self.map_frames and confirmed >= 0:
            self.progress_stamp = now
        self.map_frames = frames
        self.health_stamp = now

    def on_goal(self, msg):
        self.clear("invalid_goal")
        now = self.get_clock().now().nanoseconds
        if self.pose is None or not recent(now, self.odom_stamp, self.odom_max_ns):
            self.state = "waiting_for_fresh_odometry"
            return
        p = msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y)):
            self.state = "invalid_goal_position"
            return
        try:
            yaw = yaw_of(msg.pose.orientation)
        except ValueError:
            self.state = "invalid_goal_orientation"
            return
        clicked = nanos(msg.header.stamp)
        if not recent(now, clicked, int(1.5e9)):
            clicked = now
            self.click_source = "planner_receipt"
        else:
            self.click_source = "rviz_sim_clock"
        frame = msg.header.frame_id.lstrip("/")
        if frame == "map":
            self.goal = Pose2(float(p.x), float(p.y), yaw)
        elif frame == "base_link":
            if not self.odom_history:
                self.state = "no_odometry_for_local_click"
                return
            time_ns, origin = min(self.odom_history,
                                  key=lambda t: abs(t[0]-clicked))
            if abs(clicked-time_ns) > self.click_tolerance_ns:
                self.state = "no_odometry_for_local_click"
                return
            c, s = math.cos(origin.yaw), math.sin(origin.yaw)
            self.goal = Pose2(origin.x+c*p.x-s*p.y,
                              origin.y+s*p.x+c*p.y,
                              wrap(origin.yaw+yaw))
        else:
            self.state = "unsupported_goal_frame"
            return
        self.last_plan = 0
        self.state = "goal_selected_waiting_for_slam_corridor"

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock and now < self.last_clock-100000000:
            self.odom_history.clear()
            self.pose = None
            self.odom_stamp = 0
            self.cones, self.map_stamp = (), 0
            self.progress_stamp = 0
            self.map_frames = -1
            self.clear("simulation_clock_rewind")
        self.last_clock = now
        state = self.state
        if self.goal is not None:
            if not recent(now, self.odom_stamp, self.odom_max_ns):
                state = "stale_odometry"
            elif not recent(now, self.map_stamp, self.map_max_ns):
                state = "missing_or_stale_slam_map"
            elif not recent(now, self.health_stamp, self.status_max_ns):
                state = "missing_slam_mapper_status"
            elif not recent(now, self.progress_stamp, self.progress_max_ns):
                state = "slam_mapper_detection_progress_stopped"
            elif (distance((self.pose.x, self.pose.y),
                           (self.goal.x, self.goal.y)) <= self.arrival_radius
                  and abs(wrap(self.pose.yaw-self.goal.yaw)) <=
                      self.arrival_heading):
                self.clear("arrived")
                state = "arrived"
            else:
                if now-self.last_plan > 450000000 or not self.result.points:
                    self.result = plan_track(self.pose, self.goal,
                                             self.cones, self.cfg)
                    self.last_plan = now
                state = "tracking" if self.result.points else self.result.reason
        self.state = state
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        if state == "tracking":
            for i, (x, y) in enumerate(self.result.points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = x, y
                neighbour = (self.result.points[i+1]
                             if i+1 < len(self.result.points)
                             else self.result.points[i-1])
                heading = math.atan2((neighbour[1]-y if i+1 < len(
                    self.result.points) else y-neighbour[1]),
                    (neighbour[0]-x if i+1 < len(self.result.points)
                     else x-neighbour[0]))
                pose.pose.orientation.z = math.sin(heading/2.)
                pose.pose.orientation.w = math.cos(heading/2.)
                path.poses.append(pose)
        self.path_pub.publish(path)
        marker = Marker()
        marker.header = path.header
        marker.ns, marker.id = "high_speed_track_goal", 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD if self.goal is not None else Marker.DELETE
        marker.pose.orientation.w = 1.
        marker.scale.x, marker.scale.y, marker.scale.z = 1., .15, .15
        marker.color.r, marker.color.g, marker.color.a = 1., .6, 1.
        if self.goal is not None:
            marker.pose.position.x, marker.pose.position.y = self.goal.x, self.goal.y
            marker.pose.orientation.z = math.sin(self.goal.yaw/2.)
            marker.pose.orientation.w = math.cos(self.goal.yaw/2.)
        self.marker_pub.publish(marker)
        self.status_pub.publish(String(data=json.dumps({
            "state": state, "valid_route": state == "tracking",
            "map_age_s": round((now-self.map_stamp)/1e9, 3) if self.map_stamp else None,
            "mapper_progress_age_s": (round((now-self.progress_stamp)/1e9, 3)
                                      if self.progress_stamp else None),
            "goal": ([round(self.goal.x, 2), round(self.goal.y, 2)]
                     if self.goal else None),
            "map_cones": len(self.cones),
            "confirmed_map_mode": bool(self.cfg.confirmed_map),
            "low_covariance_confidence_cones": sum(
                cone[3] < self.cfg.min_probability for cone in self.cones),
            "route_points": len(path.poses),
            "track_gates": self.result.gates_used,
            "lateral_lane_changes": self.result.lateral_changes,
            "cone_clearance_m": round(self.result.minimum_clearance_m, 3),
            "click_clock": self.click_source,
            "obstacle_map_scope": "SLAM_cones_only",
        }, sort_keys=True)))


def main(args=None):
    # Give SIGINT to Python's handler so ROS does not tear down a
    # subscription while rclpy.spin is still converting its last message.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TrackGoalPlanner()
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
