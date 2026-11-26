"""Interchangeable ASURT LiDAR processor using unchanged ROS sensor contracts.

Input:  /velodyne_points          sensor_msgs/PointCloud2 (sensor-local)
Output: /filtered_points         sensor_msgs/PointCloud2
        /cluster_markers         visualization_msgs/MarkerArray
        /lidar/cones             asurt_msgs/LandmarkArray, UNKNOWN colour

The source LiDAR and this processor are separate ROS packages. Never publish
/perception_landmarks here: that belongs to the original ZED converter.
SLAM/fusion must consume both sources using TF and synchronized ROS stamps.
"""
from array import array as byte_array
import math
import time

import numpy as np
import rclpy
from asurt_msgs.msg import Landmark, LandmarkArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

from .fast_pipeline import FastConePipeline, Params
from .legacy_box_pipeline import LegacyBoxPipeline, LegacyBoxParams
from .legacy_lut_pipeline import LegacyLUTPipeline, LegacyLUTParams
from .visual_tracking import LidarVisualTracker


def read_xyz(msg):
    """Zero-copy PointCloud2 read for 12/16-byte little-endian layouts."""
    if msg.is_bigendian:
        raise ValueError("Big-endian PointCloud2 is unsupported")
    fields = {field.name: field for field in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        raise ValueError("Input must have x/y/z PointFields")
    if any(fields[k].datatype != PointField.FLOAT32 for k in ("x", "y", "z")):
        raise ValueError("x/y/z must be float32")
    if msg.point_step < 12 or not msg.width:
        return np.empty((0, 3), dtype=np.float32)
    if msg.height != 1 or msg.row_step != msg.width * msg.point_step:
        raise ValueError("Expected unorganized packed point cloud")
    buffer = memoryview(msg.data)
    if len(buffer) < msg.row_step:
        raise ValueError("Truncated PointCloud2 payload")
    spec = np.dtype({
        "names": ("x", "y", "z"),
        "formats": ("<f4", "<f4", "<f4"),
        "offsets": tuple(fields[k].offset for k in ("x", "y", "z")),
        "itemsize": msg.point_step,
    })
    xyz = np.frombuffer(buffer[:msg.row_step], dtype=spec)
    return np.column_stack((xyz["x"], xyz["y"], xyz["z"])).astype(
        np.float32, copy=False
    )


def make_xyz_cloud(points, header):
    data = np.ascontiguousarray(points, dtype="<f4")
    if data.ndim != 2 or data.shape[1] != 3:
        raise ValueError("Filtered cloud must be N x 3")
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(data)
    cloud.fields = [
        PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(("x", "y", "z"))
    ]
    cloud.is_bigendian = False
    cloud.is_dense = bool(np.isfinite(data).all())
    cloud.point_step = 12
    cloud.row_step = 12 * len(data)
    cloud.data = byte_array("B", data.tobytes())
    return cloud


def make_landmarks(cones, header):
    output = LandmarkArray()
    output.header = header
    for center, estimate in cones:
        landmark = Landmark()
        landmark.position.x = float(center[0])
        landmark.position.y = float(center[1])
        landmark.position.z = float(center[2])
        # LiDAR cannot distinguish blue/yellow without camera fusion.
        landmark.type = Landmark.CONE_TYPE_UNKNOWN
        landmark.identifier = -1
        landmark.probability = float(estimate)  # heuristic, NOT calibrated P
        output.landmarks.append(landmark)
    return output


def make_markers(cones, header, previous_ids=(), bootstrap_clear=False):
    """Publish stable, bounded RViz visuals independently of raw landmarks."""
    output = MarkerArray()
    if bootstrap_clear:
        deletion = Marker()
        deletion.action = Marker.DELETEALL
        output.markers.append(deletion)
    current_ids = {cone.id for cone in cones}
    for cone in cones:
        center = cone.center
        marker = Marker()
        marker.header = header
        marker.ns = "asurt_lidar_unknown_cones"
        marker.id = cone.id
        marker.action = Marker.ADD
        marker.type = Marker.CYLINDER
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2] + 0.16)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.23
        marker.scale.y = 0.23
        marker.scale.z = 0.325
        marker.color.r = 0.55
        marker.color.g = 0.35
        marker.color.b = 0.90
        marker.color.a = 0.85
        # A stalled input must not leave visual candidates indefinitely.
        marker.lifetime.sec = 1
        output.markers.append(marker)
    for missing_id in sorted(set(previous_ids) - current_ids):
        marker = Marker()
        marker.header = header
        marker.ns = "asurt_lidar_unknown_cones"
        marker.id = missing_id
        marker.action = Marker.DELETE
        output.markers.append(marker)
    return output


class LidarPerceptionNode(Node):
    def __init__(self):
        super().__init__("asurt_lidar_perception_node")
        defaults = {
            "pipeline_mode": "legacy_box",
            "input_topic": "/velodyne_points",
            "filtered_topic": "/filtered_points",
            "markers_topic": "/cluster_markers",
            "landmarks_topic": "/lidar/cones",
            "output_frame_id": "",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        for key, value in Params().__dict__.items():
            self.declare_parameter(key, value)
        for key, value in LegacyBoxParams().__dict__.items():
            self.declare_parameter("legacy_" + key, value)
        for key, value in LegacyLUTParams().__dict__.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p["pipeline_mode"] == "legacy_box":
            options = {
                key: self.get_parameter("legacy_" + key).value
                for key in LegacyBoxParams().__dict__
            }
            self.backend = LegacyBoxPipeline(LegacyBoxParams(**options))
        elif self.p["pipeline_mode"] == "fast":
            options = {
                key: self.get_parameter(key).value for key in Params().__dict__
            }
            self.backend = FastConePipeline(Params(**options))
        elif self.p["pipeline_mode"] == "legacy_lut":
            shared = {
                key: self.get_parameter("legacy_" + key).value
                for key in LegacyBoxParams().__dict__
            }
            lut_options = {
                key: self.get_parameter(key).value
                for key in LegacyLUTParams().__dict__
            }
            self.backend = LegacyLUTPipeline(
                LegacyLUTParams(**lut_options),
                LegacyBoxParams(**shared),
            )
        else:
            raise ValueError("pipeline_mode must be 'legacy_box', 'legacy_lut' or 'fast'")
        if len(set(self.p[k] for k in (
            "input_topic", "filtered_topic", "markers_topic", "landmarks_topic"))) != 4:
            raise ValueError("LiDAR source and outputs must have distinct ROS topics")
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.sub = self.create_subscription(
            PointCloud2, self.p["input_topic"], self.receive, cloud_qos
        )
        self.filtered_pub = self.create_publisher(
            PointCloud2, self.p["filtered_topic"], cloud_qos
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, self.p["markers_topic"], 3
        )
        self.marker_tracker = LidarVisualTracker()
        self.last_marker_ids = set()
        self.marker_bootstrap_clear = True
        self.landmarks_pub = self.create_publisher(
            LandmarkArray, self.p["landmarks_topic"], 3
        )
        self.last_input = None
        self.stale_cleared = False
        self.last_report = time.monotonic()
        self.count = 0
        self.total_cones = 0
        self.total_ms = 0.0
        self.max_ms = 0.0
        self.failures = 0
        self.create_timer(1.0, self.clear_stale_visuals)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"ASURT {self.p['pipeline_mode']} LiDAR processor: {self.p['input_topic']} -> "
            f"{self.p['landmarks_topic']} plus filtered cloud and markers; "
            f"original cone fit loaded={self.backend.legacy is not None}")

    def receive(self, msg):
        started = time.perf_counter()
        self.last_input = time.monotonic()
        self.stale_cleared = False
        if self.p["output_frame_id"] and self.p["output_frame_id"] != msg.header.frame_id:
            self.failures += 1
            self.get_logger().error("Cannot relabel a cloud into another TF frame")
            return
        try:
            cloud = read_xyz(msg)
            if self.p["pipeline_mode"] == "legacy_lut":
                stamp = msg.header.stamp
                stamp_s = stamp.sec + stamp.nanosec * 1e-9
                points, cones, metrics = self.backend.process(
                    cloud, stamp_s=stamp_s, frame_id=msg.header.frame_id
                )
            else:
                points, cones, metrics = self.backend.process(cloud)
            self.filtered_pub.publish(make_xyz_cloud(points, msg.header))
            self.landmarks_pub.publish(make_landmarks(cones, msg.header))
            # Only RViz receives the short visual track history; /lidar/cones
            # always consists exclusively of candidates from the current scan.
            visible = self.marker_tracker.update(
                cones, msg.header.frame_id, now=self.last_input)
            self.markers_pub.publish(make_markers(
                visible, msg.header, self.last_marker_ids,
                bootstrap_clear=self.marker_bootstrap_clear))
            self.marker_bootstrap_clear = False
            self.last_marker_ids = {item.id for item in visible}
            self.count += 1
            self.total_cones += len(cones)
            dt = (time.perf_counter() - started) * 1000
            self.total_ms += dt
            self.max_ms = max(self.max_ms, dt)
        except Exception as exc:
            self.failures += 1
            self.get_logger().error(f"LiDAR perception frame rejected: {exc}")

    def clear_stale_visuals(self):
        if self.stale_cleared or self.last_input is None:
            return
        if time.monotonic() - self.last_input > 2.0:
            clear = MarkerArray()
            m = Marker()
            m.action = Marker.DELETEALL
            clear.markers.append(m)
            self.markers_pub.publish(clear)
            self.marker_tracker.reset()
            self.last_marker_ids.clear()
            self.marker_bootstrap_clear = True
            self.stale_cleared = True

    def report(self):
        span = max(0.001, time.monotonic() - self.last_report)
        self.get_logger().info(
            f"LiDAR processing: {self.count/span:.1f}Hz; "
            f"avg_ms={self.total_ms/max(1,self.count):.2f}, "
            f"max_ms={self.max_ms:.1f}, cones={self.total_cones}, "
            f"invalid_frames={self.failures}, original_fit={self.backend.legacy is not None}")
        self.count = 0
        self.total_cones = 0
        self.total_ms = 0.0
        self.max_ms = 0.0
        self.last_report = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = LidarPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
