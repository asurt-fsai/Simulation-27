"""ROS2 adapter for map-relative camera landmark localization.

No CARLA connection, sensor creation, TF broadcaster or drive publisher.
Only corrected, geometrically observed camera poses are published.
"""

from collections import deque
import json
import math
import time

from asurt_msgs.msg import Landmark as LandmarkMsg, LandmarkArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .core import CameraMapLocalizer, Observation, Pose2, wrap


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec)*1e-9


def yaw_from_quaternion(q):
    values = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("invalid quaternion coordinates")
    norm = float(np.linalg.norm(values))
    if norm < 1e-7:
        raise ValueError("zero norm quaternion")
    x, y, z, w = values/norm
    sinpitch = 2*(w*y-z*x)
    if abs(sinpitch) > .35 or abs(2*(w*x+y*z)) > .35:
        raise ValueError("odometry orientation is not approximately planar")
    return wrap(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


class CameraSLAMNode(Node):
    def __init__(self):
        super().__init__('asurt_camera_slam')
        parameters = {
            'map_frame': 'map', 'base_frame': 'base_link',
            'camera_topic': '/perception_landmarks', 'odom_topic': '/odom',
            'map_identity_topic': '/slam/camera/map_id',
            'maximum_odom_age_s': .18, 'maximum_sensor_silence_s': 2.,
            'max_detections': 48, 'max_anchors': 400, 'max_range_m': 14.,
            'min_confidence': .45, 'association_gate_m': 1.7,
            'gate_chi2': 9.21, 'min_anchor_hits': 4,
            'min_distinct_views': 3, 'min_view_motion_m': .15,
            'min_baseline_m': .45, 'anchor_ttl_s': 240.,
            'max_step_m': .9, 'max_step_yaw_rad': .22,
            'max_residual_rms_m': .55, 'max_pose_jump_m': 10.,
            'max_pose_jump_yaw_rad': 1.1, 'map_publish_hz': 2.0,
        }
        for key, value in parameters.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.p = {key: p(key) for key in parameters}
        self.map_frame = str(p('map_frame')).lstrip('/')
        self.base_frame = str(p('base_frame')).lstrip('/')
        self.max_age = max(.02, float(p('maximum_odom_age_s')))
        core_keys = (
            'max_detections', 'max_anchors', 'max_range_m', 'min_confidence',
            'association_gate_m', 'gate_chi2', 'min_anchor_hits',
            'min_distinct_views', 'min_view_motion_m', 'min_baseline_m',
            'anchor_ttl_s', 'max_step_m', 'max_step_yaw_rad',
            'max_residual_rms_m', 'max_pose_jump_m', 'max_pose_jump_yaw_rad')
        self.engine = CameraMapLocalizer(
            **{key: self.p[key] for key in core_keys})
        self.odom_history = deque(maxlen=64)
        self.last_odom_stamp = None
        self.last_odom_pose = None
        self.last_camera_stamp = None
        self.last_camera_wall = None
        self.last_data_stamp = None
        self.map_identity = None
        self.reset_count = 0
        self.reset_reason = ''
        self.reject_count = 0
        self.last_state = 'waiting_for_repeated_cones'
        self.last_report = None
        self.corrected_path = deque(maxlen=300)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/slam/camera/pose', 10)
        self.map_pub = self.create_publisher(
            LandmarkArray, '/slam/camera/map', 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, '/slam/camera/map_markers', 10)
        self.path_pub = self.create_publisher(
            Path, '/slam/camera/path', 10)
        self.status_pub = self.create_publisher(
            String, '/slam/camera/status', 10)
        self.create_subscription(
            Odometry, str(p('odom_topic')), self.odom_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            LandmarkArray, str(p('camera_topic')), self.camera_cb,
            qos_profile_sensor_data)
        if str(p('map_identity_topic')):
            self.create_subscription(
                String, str(p('map_identity_topic')), self.map_id_cb, 10)
        self.create_timer(
            1.0/max(.2, float(p('map_publish_hz'))), self.publish_map)
        self.create_timer(1.0, self.health_tick)
        self.get_logger().info(
            'Camera landmark SE2 correction /slam/camera/pose '
            '(published only after geometric constraints); no control authority')

    def _status(self, state, report=None, pose_published_now=False):
        self.last_state = state
        self.last_report = report
        values = {
            'state': state, 'mode': 'map_relative_camera_landmark_pose',
            'pose_published': bool(pose_published_now and report and
                                   report.pose is not None),
            'global_absolute_localization': False,
            'map_frame': self.map_frame, 'input_frame': self.base_frame,
            'map_id': self.map_identity or '',
            'map_anchors': len(self.engine.anchors),
            'confirmed_anchors': len(self.engine.confirmed()),
            'corrected_frames': self.engine.corrected_frames,
            'reset_count': self.reset_count,
            'last_reset_reason': self.reset_reason,
            'rejected_inputs': self.reject_count,
            'matches': report.matches if report else 0,
            'inliers': report.inliers if report else 0,
            'accepted_detections': report.accepted_observations if report else 0,
            'residual_rms_m': report.residual_rms_m if report else None,
            'translation_innovation_m': report.innovation_m if report else None,
            'yaw_innovation_rad': report.innovation_yaw_rad if report else None,
            'covariance_xy_yaw': (report.covariance.tolist()
                                  if report and report.covariance is not None
                                  else None),
        }
        self.status_pub.publish(String(data=json.dumps(values, sort_keys=True)))

    def reset(self, reason):
        self.engine.reset()
        self.corrected_path.clear()
        self.odom_history.clear()
        self.last_odom_stamp = None
        self.last_odom_pose = None
        self.last_camera_stamp = None
        self.last_camera_wall = None
        self.last_data_stamp = None
        self.reset_count += 1
        self.reset_reason = reason
        self.clear_markers()
        self.path_pub.publish(self._path_message())
        self._status('reset:' + reason)
        self.get_logger().warning('Camera SLAM reset: ' + reason)

    def clear_markers(self):
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.markers_pub.publish(markers)

    def _path_message(self):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = list(self.corrected_path)
        return path

    def map_id_cb(self, msg):
        key = msg.data.strip()
        if not key:
            return
        if self.map_identity is not None and key != self.map_identity:
            self.map_identity = key
            self.reset('map_identity_changed')
        else:
            self.map_identity = key

    def odom_cb(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if not math.isfinite(stamp) or stamp <= 0:
            self.reject_count += 1
            self._status('invalid_odometry_stamp')
            return
        frame, child = msg.header.frame_id.lstrip('/'), msg.child_frame_id.lstrip('/')
        if frame != self.map_frame or child != self.base_frame:
            if self.odom_history:
                self.reset('odometry_frame_changed')
            self.reject_count += 1
            self._status('unexpected_odometry_frame')
            return
        try:
            pos = msg.pose.pose.position
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            pose = Pose2(float(pos.x), float(pos.y), yaw)
            if not all(math.isfinite(v) for v in (pose.x, pose.y)):
                raise ValueError('nonfinite odometry position')
        except ValueError:
            self.reject_count += 1
            self._status('invalid_odometry')
            return
        if self.last_odom_stamp is not None:
            if stamp < self.last_odom_stamp-.25:
                self.reset('odometry_time_rewind')
            elif stamp <= self.last_odom_stamp:
                return
            elif self.last_odom_pose is not None:
                travel = math.hypot(pose.x-self.last_odom_pose.x,
                                    pose.y-self.last_odom_pose.y)
                yaw_jump = abs(wrap(pose.yaw-self.last_odom_pose.yaw))
                if (travel > self.engine.max_pose_jump_m or
                        yaw_jump > self.engine.max_pose_jump_yaw_rad):
                    self.reset('odometry_teleport_or_world_change')
        self.last_odom_stamp, self.last_odom_pose = stamp, pose
        self.odom_history.append((stamp, pose))

    def _nearby_odom(self, stamp):
        if not self.odom_history:
            return None
        best = min(self.odom_history, key=lambda pair: abs(pair[0]-stamp))
        return best[1] if abs(best[0]-stamp) <= self.max_age else None

    def camera_cb(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if not math.isfinite(stamp) or stamp <= 0:
            self.reject_count += 1
            self._status('invalid_camera_stamp')
            return
        if self.last_camera_stamp is not None:
            if stamp < self.last_camera_stamp-.25:
                self.reset('camera_time_rewind')
                return  # wait for a new odometry message in the new epoch
            if stamp <= self.last_camera_stamp:
                return
        self.last_camera_stamp = stamp
        if msg.header.frame_id.lstrip('/') != self.base_frame:
            self.reject_count += 1
            self._status('unexpected_camera_frame')
            return
        odom_pose = self._nearby_odom(stamp)
        if odom_pose is None:
            self.reject_count += 1
            self._status('missing_time_aligned_odometry')
            return
        obs = [Observation(float(m.position.x), float(m.position.y),
                           float(m.position.z), int(m.type),
                           float(m.probability))
               for m in msg.landmarks[:int(self.engine.max_detections)]]
        report = self.engine.step(odom_pose, obs, stamp)
        if report.reset_reason:
            # A numeric-core reset must invalidate the ROS path/visuals too.
            # Discard this boundary frame; wait for fresh aligned odometry.
            self.reset(report.reset_reason)
            return
        self.last_data_stamp = msg.header.stamp
        self.last_camera_wall = time.monotonic()
        if report.pose is not None:
            self.publish_pose(report, msg.header.stamp)
        self._status(report.state, report, pose_published_now=True)

    def publish_pose(self, report, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(report.pose.x)
        msg.pose.pose.position.y = float(report.pose.y)
        msg.pose.pose.orientation.z = math.sin(report.pose.yaw/2.)
        msg.pose.pose.orientation.w = math.cos(report.pose.yaw/2.)
        # x,y,yaw estimated; z,roll,pitch have no camera-cone observability.
        cov = np.zeros((6, 6), dtype=float)
        cov[2, 2] = 1e4
        cov[3, 3] = cov[4, 4] = 1e3
        indices = (0, 1, 5)
        for i in range(3):
            for j in range(3):
                cov[indices[i], indices[j]] = report.covariance[i, j]
        msg.pose.covariance = cov.flatten().tolist()
        self.pose_pub.publish(msg)
        path_pose = PoseStamped()
        path_pose.header = msg.header
        path_pose.pose = msg.pose.pose
        self.corrected_path.append(path_pose)
        self.path_pub.publish(self._path_message())

    def publish_map(self):
        msg = LandmarkArray()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = (self.last_data_stamp if self.last_data_stamp
                            else self.get_clock().now().to_msg())
        markers = MarkerArray()
        clearing = Marker()
        clearing.header = msg.header
        clearing.action = Marker.DELETEALL
        markers.markers.append(clearing)
        palette = {
            0: (.05, .25, 1.0), 1: (1.0, .85, .05),
            2: (1.0, .40, .02), 3: (1.0, .16, .01)}
        for anchor in self.engine.confirmed():
            lm = LandmarkMsg()
            lm.identifier = int(anchor.uid)
            lm.type = int(anchor.color)
            lm.position.x, lm.position.y = (
                float(anchor.xy[0]), float(anchor.xy[1]))
            lm.probability = float(1./(1.+math.sqrt(
                max(0., np.trace(anchor.cov)))))
            msg.landmarks.append(lm)
            sphere = Marker()
            sphere.header = msg.header
            sphere.ns = 'asurt_camera_slam_confirmed'
            sphere.id = int(anchor.uid)
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(anchor.xy[0])
            sphere.pose.position.y = float(anchor.xy[1])
            sphere.pose.position.z = .18 if anchor.color != 3 else .25
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = (.26 if anchor.color != 3 else .36)
            sphere.scale.z = (.32 if anchor.color != 3 else .48)
            sphere.color.r, sphere.color.g, sphere.color.b = palette[anchor.color]
            sphere.color.a = 1.0
            markers.markers.append(sphere)
        self.map_pub.publish(msg)
        self.markers_pub.publish(markers)

    def health_tick(self):
        if (self.last_camera_wall is None or
                time.monotonic()-self.last_camera_wall >
                float(self.p['maximum_sensor_silence_s'])):
            self._status('stale_or_missing_camera_observations')
        else:
            self._status(self.last_state, self.last_report)


def main(args=None):
    rclpy.init(args=args)
    node = CameraSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
