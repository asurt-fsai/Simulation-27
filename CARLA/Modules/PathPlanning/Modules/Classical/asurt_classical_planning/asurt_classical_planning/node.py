"""ROS2 adapter for the bounded, read-only classical centerline preview."""

import json
import math
from collections import deque
from heapq import nsmallest

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from asurt_msgs.msg import LandmarkArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import BLUE, YELLOW, Config, Result, centerline, in_map


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


class ClassicalPlanner(Node):
    def __init__(self, profile='classical', strategy=centerline,
                 config_type=Config, cone_types=(BLUE, YELLOW)):
        # Shared ROS input/timing/health contract, with independent output
        # topics and algorithms for each Formula Student planning discipline.
        super().__init__('asurt_' + profile + '_planner')
        self.profile = profile
        self.strategy = strategy
        self.cone_types = cone_types
        # rclpy's TimeSource already declares use_sim_time on Humble.
        self.declare_parameter('input_source', 'auto')  # auto | map | perception
        self.declare_parameter('global_cones_topic', '/map/global_cones')
        self.declare_parameter('perception_topic', '/perception_landmarks')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('max_age_s', 1.5)
        self.declare_parameter('teleport_distance_m', 8.0)
        self.declare_parameter('planning_period_s', 0.25)
        for key, default in vars(config_type()).items():
            self.declare_parameter(key, default)
        self.cfg = config_type(**{key: self.get_parameter(key).value
                                  for key in vars(config_type())})
        self.source = str(self.get_parameter('input_source').value)
        if self.source not in ('auto', 'map', 'perception'):
            raise ValueError('input_source must be auto, map or perception')
        self.max_age_ns = int(max(.1, float(self.get_parameter('max_age_s').value))*1e9)
        self.teleport_distance = max(1., float(self.get_parameter('teleport_distance_m').value))
        self.path_pub = self.create_publisher(Path, '/planning/' + profile + '/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planning/' + profile + '/markers', 10)
        self.status_pub = self.create_publisher(String, '/planning/' + profile + '/status', 10)
        self.pose = None
        self.odom_stamp = 0
        self.odom_history = deque(maxlen=96)
        self.cones = {}
        self.last_clock = 0
        self.had_path = False
        self.last_result = None
        self.create_subscription(Odometry, self.get_parameter('odometry_topic').value,
                                 self._on_odom, qos_profile_sensor_data)
        if self.source in ('auto', 'map'):
            self.create_subscription(LandmarkArray, self.get_parameter('global_cones_topic').value,
                                     lambda msg: self._on_cones(msg, 'map'), 10)
        if self.source in ('auto', 'perception'):
            self.create_subscription(LandmarkArray, self.get_parameter('perception_topic').value,
                                     lambda msg: self._on_cones(msg, 'perception'), 10)
        self.create_timer(max(.1, float(self.get_parameter('planning_period_s').value)),
                          self._tick)
        self.get_logger().info(
            'Visualization-only %s path planning; no vehicle command publisher' % profile)

    def _on_odom(self, msg):
        if msg.header.frame_id != 'map':
            self.pose = None
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not (.5 <= norm <= 1.5):
            self.pose = None
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = (float(p.x), float(p.y), yaw)
        if not all(math.isfinite(v) for v in pose):
            self.pose = None
            return
        timestamp = stamp_ns(msg.header.stamp)
        if self.odom_stamp and timestamp and timestamp < self.odom_stamp - 100_000_000:
            self._reset()
        if self.pose and math.hypot(pose[0]-self.pose[0], pose[1]-self.pose[1]) > self.teleport_distance:
            self.cones.clear()
            self._clear_path()
        self.pose = pose
        self.odom_stamp = timestamp or self.get_clock().now().nanoseconds
        self.odom_history.append((self.odom_stamp, pose))

    def _on_cones(self, msg, source):
        expected_frame = 'map' if source == 'map' else 'base_link'
        if msg.header.frame_id != expected_frame:
            self.cones.pop(source, None)
            return
        raw = []
        # A global map can contain many old landmarks, grouped by colour.
        # Preserve a bounded slice for EACH requested cone class.
        origin = self.pose[:2] if source == 'map' and self.pose else (0., 0.)
        budget = max(256, min(4096, self.cfg.max_cones*16))
        per_type = max(2, self.cfg.max_cones // len(self.cone_types))
        for kind in self.cone_types:
            eligible = (landmark for landmark in msg.landmarks[:budget]
                        if int(landmark.type) == kind
                        and math.isfinite(landmark.position.x)
                        and math.isfinite(landmark.position.y)
                        and math.isfinite(landmark.probability)
                        and landmark.probability >= self.cfg.min_probability)
            nearest = nsmallest(per_type, eligible,
                                key=lambda landmark: (
                                    (landmark.position.x-origin[0])**2
                                    + (landmark.position.y-origin[1])**2))
            raw.extend((float(l.position.x), float(l.position.y), int(l.type),
                        float(l.probability)) for l in nearest)
        t = stamp_ns(msg.header.stamp) or self.get_clock().now().nanoseconds
        # Store only complete snapshots. Never accumulate stale map landmarks.
        self.cones[source] = (t, tuple(raw))

    def _reset(self):
        self.pose = None
        self.odom_stamp = 0
        self.odom_history.clear()
        self.cones.clear()
        self._clear_path()

    def _recent(self, now, timestamp):
        return (timestamp > 0 and -200_000_000 <= now-timestamp <= self.max_age_ns)

    def _clear_path(self):
        if not self.had_path:
            return
        now = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = now
        self.path_pub.publish(msg)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = now
        marker.ns = 'asurt_' + self.profile + '_planner'
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.had_path = False

    def _publish(self, result):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for x, y in result.points:
            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose.position.x = float(x)
            stamped.pose.position.y = float(y)
            stamped.pose.orientation.w = 1.0
            path.poses.append(stamped)
        self.path_pub.publish(path)
        marker = Marker()
        marker.header = path.header
        marker.ns = 'asurt_' + self.profile + '_planner'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.16
        marker.color.r = 1.0 if self.profile == 'classical' else 0.2
        marker.color.g = 0.28 if self.profile == 'classical' else 0.9
        marker.color.b = 0.1 if self.profile != 'skidpad' else 0.95
        marker.color.a = 0.95
        marker.points = [pose.pose.position for pose in path.poses]
        marker.lifetime.sec = 1
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.had_path = True

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock and now < self.last_clock - 100_000_000:
            self._reset()
        self.last_clock = now
        reason = 'waiting_for_sim_clock'
        chosen = None
        result = Result(reason=reason)
        if now > 0:
            if self.pose is None or not self._recent(now, self.odom_stamp):
                result = Result(reason='waiting_for_fresh_map_odometry')
            else:
                sources = ['map', 'perception'] if self.source == 'auto' else [self.source]
                result = Result(reason='waiting_for_fresh_cones')
                for name in sources:
                    snapshot = self.cones.get(name)
                    if snapshot is None or not self._recent(now, snapshot[0]):
                        continue
                    cones = snapshot[1]
                    if name == 'perception':
                        # Perception positions were measured in base_link at
                        # capture time. Use the nearest past bridge odometry
                        # sample for that transform; avoid moving the old
                        # observation using the vehicle's newest position.
                        nearest = min(self.odom_history,
                                      key=lambda sample: abs(sample[0]-snapshot[0]),
                                      default=None)
                        if nearest is None or abs(nearest[0]-snapshot[0]) > 180_000_000:
                            result = Result(reason='missing_synchronized_odometry')
                            continue
                        cones = in_map(cones, nearest[1])
                    attempt = self.strategy(cones, self.pose, self.cfg)
                    chosen = name
                    result = attempt
                    if attempt.reason == 'ok':
                        break
        if result.reason == 'ok':
            self._publish(result)
        else:
            self._clear_path()
        self.last_result = result
        status = {
            'mode': self.profile,
            'state': 'preview' if result.reason == 'ok' else 'unavailable',
            'reason': result.reason,
            'source': chosen,
            'pairs': result.pairs,
            'points': len(result.points),
            'control_ready': False,
            'map_anchoring': 'bridge_odom',
            'visualization_only': True,
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = ClassicalPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
