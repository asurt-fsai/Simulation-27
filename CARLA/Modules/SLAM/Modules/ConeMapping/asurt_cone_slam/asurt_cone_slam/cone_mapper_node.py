"""Passive cone map from existing /perception_landmarks and /odom/TF.

Adapts the legacy camera cone_mapping algorithm without pyzed, duplicate
converters, ground-truth actor queries, static TF publishers or CARLA access.
This is odometry-aided landmark mapping; no loop-closure claim is made.
"""
import json
import math
import time

from asurt_msgs.msg import Landmark as LandmarkMsg, LandmarkArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray
import rclpy

from .association import ConeMap, transform_xy


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ConeMapperNode(Node):
    def __init__(self):
        super().__init__("asurt_cone_mapper")
        for key, default in (
            ("map_frame", "map"),
            ("perception_topic", "/perception_landmarks"),
            ("odometry_topic", "/odom"),
            ("max_detection_range", 45.0),
            ("min_detection_range", .25),
            ("association_gate_radius", 2.0),
            ("mahalanobis_threshold", 5.991),
            ("observations_for_confirmation", 3),
            ("maximum_landmarks", 1200),
            ("publish_rate_hz", 2.0),
            ("reset_on_pose_jump_m", 30.0),
            ("maximum_odom_age_s", .35),
        ):
            self.declare_parameter(key, default)
        p = lambda key: self.get_parameter(key).value
        self.frame = p("map_frame")
        self.reset_jump = float(p("reset_on_pose_jump_m"))
        self.max_odom_age = float(p("maximum_odom_age_s"))
        self.map = ConeMap(
            gate=p("association_gate_radius"),
            mahalanobis_gate=p("mahalanobis_threshold"),
            minimum_observations=p("observations_for_confirmation"),
            maximum_landmarks=p("maximum_landmarks"),
            max_detection_range=p("max_detection_range"),
            min_detection_range=p("min_detection_range"),
        )
        self.buffer = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.buffer, self)
        self.last_odom = None
        self.last_position = None
        self.last_stamp = None
        self.last_message_stamp = None
        self.rejected_frames = 0
        self.mapping_frames = 0
        self.reset_count = 0
        self.last_reason = "waiting for detections"
        self.create_subscription(Odometry, p("odometry_topic"), self.odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LandmarkArray, p("perception_topic"), self.cones_cb, 10)
        self.map_pub = self.create_publisher(LandmarkArray, "/map/global_cones", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/map/global_cones_markers", 10)
        self.health_pub = self.create_publisher(String, "/slam/cone_map_status", 10)
        self.create_timer(1.0 / max(.5, float(p("publish_rate_hz"))), self.publish)
        self.get_logger().info(
            f"SLAM cone mapping from {p('perception_topic')} + {p('odometry_topic')}; "
            f"frame={self.frame}; no CARLA actors or ticker owned"
        )

    def reset(self, reason):
        self.map.reset()
        self.last_message_stamp = None
        self.reset_count += 1
        self.last_reason = reason
        self.get_logger().warning(f"Resetting local cone map: {reason}")

    def odom_cb(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        pos = msg.pose.pose.position
        xyz = (float(pos.x), float(pos.y), float(pos.z))
        if (not all(math.isfinite(v) for v in xyz) or
            msg.header.frame_id.lstrip("/") != self.frame):
            return
        if self.last_stamp is not None:
            if stamp < self.last_stamp - .5:
                self.reset("simulation time moved backward / new world")
            elif self.last_position is not None and math.dist(xyz, self.last_position) > self.reset_jump:
                self.reset("vehicle teleported or map changed")
        self.last_position = xyz
        self.last_stamp = stamp
        self.last_odom = msg

    def lookup(self, source_frame, stamp):
        source = (source_frame or "").lstrip("/")
        if not source:
            raise ValueError("Landmark frame missing: cannot infer source coordinates")
        if source == self.frame:
            return (0., 0., 0.), (0., 0., 0., 1.)
        try:
            tf = self.buffer.lookup_transform(
                self.frame, source, Time.from_msg(stamp), timeout=Duration(seconds=.025))
            t, q = tf.transform.translation, tf.transform.rotation
            return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
        except TransformException:
            # Fallback to *real existing odometry*, never actor truth. Only
            # for base_link and with a recent timestamp, not arbitrary frames.
            if source != "base_link" or self.last_odom is None:
                raise
            if abs(stamp_seconds(stamp) -
                   stamp_seconds(self.last_odom.header.stamp)) > self.max_odom_age:
                raise
            p, q = self.last_odom.pose.pose.position, self.last_odom.pose.pose.orientation
            return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

    def cones_cb(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if self.last_message_stamp is not None and stamp <= self.last_message_stamp:
            if stamp < self.last_message_stamp - .5:
                self.reset("perception source timestamp rewound")
            else:
                return  # ROS relays must not double-count the same frame.
        self.last_message_stamp = stamp
        try:
            t, q = self.lookup(msg.header.frame_id, msg.header.stamp)
        except (TransformException, ValueError) as exc:
            self.rejected_frames += 1
            self.last_reason = f"missing frame transform: {exc}"
            if self.rejected_frames <= 3 or self.rejected_frames % 100 == 0:
                self.get_logger().warning(self.last_reason)
            return
        dets = []
        for lm in msg.landmarks:
            x, y, z = lm.position.x, lm.position.y, lm.position.z
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            distance = math.hypot(x, y)
            if not self.map.min_detection_range <= distance <= self.map.max_detection_range:
                continue
            position = transform_xy((x, y, z), t, q)
            dets.append((position[0], position[1], int(lm.type),
                         min(1., max(0., lm.probability)), distance))
        self.map.update(dets, stamp)
        self.mapping_frames += 1
        self.last_reason = "mapping"

    def publish(self):
        header_stamp = self.get_clock().now().to_msg()
        data = LandmarkArray()
        data.header.frame_id = self.frame
        data.header.stamp = header_stamp
        markers = MarkerArray()
        deleted = Marker()
        deleted.header = data.header
        deleted.action = Marker.DELETEALL
        markers.markers.append(deleted)
        palette = {0: (0., .3, 1.), 1: (1., 1., .05),
                   2: (1., .45, 0.), 3: (1., .15, 0.)}
        for lm in self.map.confirmed():
            point = LandmarkMsg()
            point.identifier = int(lm.uid)
            point.position.x, point.position.y = float(lm.x[0]), float(lm.x[1])
            point.type = int(lm.kind)
            point.probability = float(1.0 / (1.0 + lm.P.trace()))
            data.landmarks.append(point)
            marker = Marker()
            marker.header = data.header
            marker.ns = "slam_cones"
            marker.id = int(lm.uid)
            marker.action = Marker.ADD
            marker.type = Marker.CYLINDER
            marker.pose.position = point.position
            marker.pose.position.z = .24 if lm.kind == 3 else .15
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = .28 if lm.kind == 3 else .20
            marker.scale.z = .48 if lm.kind == 3 else .30
            marker.color.r, marker.color.g, marker.color.b = palette[lm.kind]
            marker.color.a = 1.0
            markers.markers.append(marker)
        self.map_pub.publish(data)
        self.marker_pub.publish(markers)
        status = {"state": self.last_reason, "map_frame": self.frame,
                  "detections_frames": self.mapping_frames,
                  "rejected_frames": self.rejected_frames,
                  "confirmed": len(data.landmarks), "total": len(self.map.landmarks),
                  "reset_count": self.reset_count}
        self.health_pub.publish(String(data=json.dumps(status, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = ConeMapperNode()
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
