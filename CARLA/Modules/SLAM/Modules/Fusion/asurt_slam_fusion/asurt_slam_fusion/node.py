"""Passive ROS2 camera/LiDAR observations with optional LiDAR-odom drift correction.

Never owns CARLA actors, /clock, /odom, TF transforms or driving commands.
All outputs use new /slam/fusion/* topics to leave Phase 1 untouched.
"""
from collections import deque
import json
import math

from asurt_msgs.msg import Landmark, LandmarkArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .core import Cone, LidarPoseCorrector, Pose2, fuse_cones, rebase_point, wrap


def seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def planar_pose(message):
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    values = (p.x, p.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('invalid pose values')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not .90 <= norm <= 1.10:
        raise ValueError('invalid odometry quaternion')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return Pose2(float(p.x), float(p.y), yaw)


def apply_tf(point, transform):
    t, q = transform.transform.translation, transform.transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < .9:
        raise ValueError('invalid extrinsic quaternion')
    x, y, z, w = quaternion / norm
    vec = np.asarray(point, dtype=float)
    twice = 2.0 * np.cross([x, y, z], vec)
    rotated = vec + w * twice + np.cross([x, y, z], twice)
    return tuple(rotated + [t.x, t.y, t.z])


def paired_cone_markers(landmarks, header):
    """Replace the current base_link detections with distinct RViz glyphs.

    Spheres identify paired measurements rather than Phase 1 mapped cylinders.
    The cyan ring marks dual-sensor confirmation even when cone colour is
    unknown. Marker IDs are frame-local; DELETEALL removes stale IDs first.
    """
    result = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    palette = {
        0: (.05, .55, 1.0),  # blue
        1: (1.0, .94, .15),  # yellow
        2: (1.0, .42, .08),  # orange
        3: (1.0, .08, .42),  # large orange / distinct magenta
        4: (.68, .68, .75), # unknown
    }
    for i, landmark in enumerate(landmarks):
        point = landmark.position
        radius = .19 if landmark.type != 3 else .27
        sphere = Marker()
        sphere.header = header
        sphere.ns = 'fusion_paired_cones'
        sphere.id = i
        sphere.action = Marker.ADD
        sphere.type = Marker.SPHERE
        sphere.pose.position = point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = radius * 2
        sphere.color.r, sphere.color.g, sphere.color.b = palette.get(
            int(landmark.type), palette[4])
        sphere.color.a = .90
        sphere.lifetime = Duration(seconds=.75).to_msg()
        result.markers.append(sphere)

        ring = Marker()
        ring.header = header
        ring.ns = 'fusion_paired_halo'
        ring.id = i
        ring.action = Marker.ADD
        ring.type = Marker.LINE_STRIP
        ring.pose.orientation.w = 1.0
        ring.scale.x = .045
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = (.05, 1., .85, .95)
        ring.lifetime = Duration(seconds=.75).to_msg()
        for step in range(17):
            angle = step * math.tau / 16
            ring.points.append(Point(x=point.x + (radius + .10) * math.cos(angle),
                                     y=point.y + (radius + .10) * math.sin(angle),
                                     z=point.z))
        result.markers.append(ring)
    return result


class FusionNode(Node):
    def __init__(self):
        super().__init__('asurt_slam_fusion')
        defaults = {
            'camera_topic': '/perception_landmarks',
            'lidar_cones_topic': '/lidar/cones',
            'bridge_odom_topic': '/odom',
            'lidar_odom_topic': '/slam/lidar/normalized_odom',
            'lidar_odom_frame': 'lidar_odom',
            'lidar_odom_child_frame': 'base_link',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'pair_tolerance_s': .12,
            'motion_negligible_s': .015,
            'maximum_odom_age_s': .18,
            'maximum_lidar_odom_age_s': .18,
            'reset_on_pose_jump_m': 30.0,
            'association_gate_m': 1.25,
            'association_chi2': 9.21,
            'max_cones_per_sensor': 64,
            'maximum_reference_anchors': 500,
            'map_gate_m': 1.40,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if not (0 < self.p['pair_tolerance_s'] <= .25 and
                0 < self.p['association_gate_m'] <= 3.0 and
                0 < self.p['maximum_odom_age_s'] <= .5 and
                0 < self.p['maximum_lidar_odom_age_s'] <= .5 and
                1 <= self.p['max_cones_per_sensor'] <= 64 and
                3 <= self.p['maximum_reference_anchors'] <= 1200):
            raise ValueError('Fusion parameters exceed bounded accepted limits')
        if self.p['lidar_odom_frame'] in ('map', 'camera_init', 'world'):
            self.get_logger().warning('Raw world/map/legacy LeGO-LOAM odometry is not '
                                      'assumed to use normalized local ENU axes; '
                                      'configure the independent lidar_odom_frame '
                                      'after validating coordinate conversion')
        self.tf = Buffer(cache_time=Duration(seconds=8.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.camera_queue = deque(maxlen=8)
        self.lidar_queue = deque(maxlen=8)
        self.odom_history = deque(maxlen=60)
        self.last_source = {'camera': None, 'lidar': None, 'bridge': None,
                            'lidar_odom': None}
        self.last_bridge_position = None
        self.estimator = LidarPoseCorrector(
            max_anchors=int(self.p['maximum_reference_anchors']),
            map_gate=float(self.p['map_gate_m']))
        self.frames = 0
        self.accepted_poses = 0
        self.rejected = 0
        self.reset_count = 0
        self.last_counts = {}
        self.reason = 'waiting_for_camera_and_lidar'
        self.pose_input_state = 'waiting_for_normalized_lidar_odom'
        self.last_rejected_lidar_frames = None
        self.last_pair_stamp = None
        self.create_subscription(LandmarkArray, self.p['camera_topic'],
                                 lambda msg: self.receive('camera', msg), 10)
        self.create_subscription(LandmarkArray, self.p['lidar_cones_topic'],
                                 lambda msg: self.receive('lidar', msg), 10)
        self.create_subscription(Odometry, self.p['bridge_odom_topic'],
                                 self.bridge_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['lidar_odom_topic'],
                                 self.lidar_odom_cb, qos_profile_sensor_data)
        self.landmarks_pub = self.create_publisher(
            LandmarkArray, '/slam/fusion/landmarks', 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, '/slam/fusion/markers', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/slam/fusion/pose', 10)
        # Map anchors are built only from accepted *paired* camera+LiDAR
        # observations. They remain odometry-anchored until a validated
        # joint map optimizer can correct loop closures globally.
        self.global_map_pub = self.create_publisher(
            LandmarkArray, '/slam/fusion/global_cones', 10)
        self.global_markers_pub = self.create_publisher(
            MarkerArray, '/slam/fusion/global_cones_markers', 10)
        self.status_pub = self.create_publisher(String, '/slam/fusion/status', 10)
        self.create_timer(.5, self.publish_status)
        self.create_timer(.5, self.publish_global_map)
        self.get_logger().info('Fusion passive: camera + LiDAR cones; optional '
                               'independent normalized LiDAR odometry for map-relative '
                               'pose; no control or global TF publications')

    def reset(self, reason):
        # An explicit DELETEALL is essential when the source resets without
        # another synchronized camera/LiDAR frame arriving.
        cleared = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.p['base_frame']
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        cleared.markers.append(marker)
        self.markers_pub.publish(cleared)
        self.camera_queue.clear()
        self.lidar_queue.clear()
        self.odom_history.clear()
        self.last_source = dict.fromkeys(self.last_source)
        self.last_bridge_position = None
        self.estimator.reset()
        self.publish_global_map()
        self.pose_input_state = 'waiting_for_normalized_lidar_odom'
        self.last_rejected_lidar_frames = None
        self.last_pair_stamp = None
        self.reset_count += 1
        self.reason = reason
        self.get_logger().warning('Fusion reset: ' + reason)

    def check_stamp(self, source, stamp):
        if not math.isfinite(stamp) or stamp <= 0:
            self.rejected += 1
            self.reason = 'invalid_' + source + '_stamp'
            return False
        previous = self.last_source[source]
        if previous is not None and stamp <= previous:
            if stamp < previous - .5:
                self.reset(source + '_time_rewind')
            else:
                self.rejected += 1
                return False
        self.last_source[source] = stamp
        return True

    def bridge_cb(self, msg):
        stamp = seconds(msg.header.stamp)
        if msg.header.frame_id.lstrip('/') != self.p['map_frame']:
            return
        try:
            pose = planar_pose(msg)
        except ValueError:
            self.rejected += 1
            return
        if not self.check_stamp('bridge', stamp):
            return
        if (self.last_bridge_position is not None and
                math.hypot(pose.x - self.last_bridge_position.x,
                           pose.y - self.last_bridge_position.y) >
                float(self.p['reset_on_pose_jump_m'])):
            self.reset('bridge_pose_teleport_or_map_change')
            self.last_source['bridge'] = stamp
        self.last_bridge_position = pose
        self.odom_history.append((stamp, pose, msg))

    def odom_at(self, stamp):
        maximum_age = float(self.p['maximum_odom_age_s'])
        if not self.odom_history:
            return None
        samples = list(self.odom_history)
        for (t0, a, _), (t1, b, _) in zip(samples, samples[1:]):
            if t0 <= stamp <= t1 and t1 - t0 <= 2 * maximum_age:
                k = (stamp - t0) / max(1e-9, t1 - t0)
                return Pose2(a.x + k*(b.x-a.x), a.y + k*(b.y-a.y),
                             wrap(a.yaw + k * wrap(b.yaw-a.yaw)))
        closest_t, closest_p, _ = min(samples, key=lambda entry: abs(entry[0] - stamp))
        return closest_p if abs(closest_t - stamp) <= maximum_age else None

    def lidar_odom_cb(self, msg):
        stamp = seconds(msg.header.stamp)
        # Raw LeGO-LOAM camera_init axes need a separately validated REP-103
        # conversion. Label checks prevent silently taking them as local XY.
        if (msg.header.frame_id.lstrip('/') != self.p['lidar_odom_frame'] or
                msg.child_frame_id.lstrip('/') != self.p['lidar_odom_child_frame']):
            self.last_rejected_lidar_frames = [msg.header.frame_id,
                                               msg.child_frame_id]
            self.pose_input_state = 'waiting_for_normalized_lidar_odom'
            self.reason = 'lidar_odom_frame_requires_normalization'
            # Drop any previously integrated prediction if a different
            # coordinate system begins arriving on the subscribed topic.
            self.estimator.prediction = None
            self.estimator.last_lidar = None
            self.estimator.last_lidar_stamp = None
            self.rejected += 1
            return
        try:
            pose = planar_pose(msg)
        except ValueError:
            self.rejected += 1
            return
        if not self.check_stamp('lidar_odom', stamp):
            return
        self.pose_input_state = 'normalized_frame_contract_received'
        bridge = self.odom_at(stamp)
        outcome = self.estimator.integrate(pose, stamp, bridge)
        if outcome in ('lidar_odometry_jump', 'lidar_timestamp_rewind'):
            self.reset(outcome)
            self.last_source['lidar_odom'] = stamp
        elif outcome != 'integrated' and outcome != 'anchored':
            self.reason = outcome

    def receive(self, source, msg):
        stamp = seconds(msg.header.stamp)
        if not self.check_stamp(source, stamp):
            return
        frame = msg.header.frame_id.lstrip('/')
        if not frame:
            self.reason = 'missing_' + source + '_frame'
            self.rejected += 1
            return
        data = (stamp, frame, msg)
        (self.camera_queue if source == 'camera' else self.lidar_queue).append(data)
        self.process_pairs()

    def process_pairs(self):
        while self.camera_queue and self.lidar_queue:
            cam = self.camera_queue[0]
            lidar_index = min(range(len(self.lidar_queue)),
                              key=lambda i: abs(self.lidar_queue[i][0] - cam[0]))
            laser = self.lidar_queue[lidar_index]
            if abs(cam[0] - laser[0]) <= self.p['pair_tolerance_s']:
                self.camera_queue.popleft()
                del self.lidar_queue[lidar_index]
                self.process_pair(cam, laser)
            elif cam[0] < laser[0] - self.p['pair_tolerance_s']:
                self.camera_queue.popleft()
                self.rejected += 1
                self.reason = 'unmatched_camera_stamp'
            elif laser[0] < cam[0] - self.p['pair_tolerance_s']:
                del self.lidar_queue[lidar_index]
                self.rejected += 1
                self.reason = 'unmatched_lidar_stamp'
            else:
                break

    def cones_in_base(self, frame, msg, stamp, target_stamp=None):
        if frame == self.p['base_frame']:
            tf = None
        else:
            tf = self.tf.lookup_transform(
                self.p['base_frame'], frame, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=.025))
        delta = abs(stamp - target_stamp) if target_stamp is not None else 0
        source_pose = target_pose = None
        if delta > self.p['motion_negligible_s']:
            source_pose = self.odom_at(stamp)
            target_pose = self.odom_at(target_stamp)
            if source_pose is None or target_pose is None:
                raise ValueError('missing timestamped motion compensation')
        cones = []
        for lm in msg.landmarks[:int(self.p['max_cones_per_sensor'])]:
            point = (lm.position.x, lm.position.y, lm.position.z)
            if not all(math.isfinite(v) for v in point):
                continue
            xyz = apply_tf(point, tf) if tf is not None else point
            if source_pose is not None:
                xyz = rebase_point(xyz, source_pose, target_pose)
            cones.append(Cone(*map(float, xyz), int(lm.type),
                              float(lm.probability), int(lm.identifier)))
        return cones

    def process_pair(self, camera, laser):
        cam_stamp, cam_frame, cam_msg = camera
        lidar_stamp, lidar_frame, lidar_msg = laser
        self.last_pair_stamp = cam_stamp
        try:
            cams = self.cones_in_base(cam_frame, cam_msg, cam_stamp)
            lasers = self.cones_in_base(lidar_frame, lidar_msg, lidar_stamp,
                                        target_stamp=cam_stamp)
        except (TransformException, ValueError) as exc:
            self.reason = 'transform_or_motion_compensation_failed: ' + str(exc)[:110]
            self.rejected += 1
            return
        fused, counts = fuse_cones(
            cams, lasers, gate_m=float(self.p['association_gate_m']),
            gate_chi2=float(self.p['association_chi2']),
            max_count=int(self.p['max_cones_per_sensor']))
        self.frames += 1
        self.last_counts = counts
        self.reason = 'paired_without_cone_matches' if not fused else 'fused_landmarks'
        output = LandmarkArray()
        output.header.stamp = cam_msg.header.stamp
        output.header.frame_id = self.p['base_frame']
        for cone in fused:
            lm = Landmark()
            lm.position.x, lm.position.y, lm.position.z = cone.x, cone.y, cone.z
            lm.type, lm.identifier, lm.probability = (cone.kind, cone.identifier,
                                                      cone.confidence)
            output.landmarks.append(lm)
        self.landmarks_pub.publish(output)
        self.markers_pub.publish(paired_cone_markers(output.landmarks, output.header))
        if not fused:
            return
        bridge = self.odom_at(cam_stamp)
        if bridge is None:
            self.reason = 'fused_without_recent_bridge_map_pose'
            return
        pose_result, outcome = self.estimator.correct(
            fused, cam_stamp, max_odom_age=float(self.p['maximum_lidar_odom_age_s']))
        # Learn *after* correction so a single frame cannot validate itself.
        self.estimator.learn(fused, pose_result[0] if pose_result else bridge,
                             cam_stamp)
        if pose_result is None:
            self.reason = outcome
            return
        corrected, covariance = pose_result
        self.publish_pose(corrected, covariance, cam_msg.header.stamp,
                          self.closest_bridge_msg(cam_stamp))
        self.accepted_poses += 1
        self.reason = 'corrected_with_lidar_odom_and_camera_lidar_landmarks'

    def closest_bridge_msg(self, stamp):
        if not self.odom_history:
            return None
        return min(self.odom_history, key=lambda e: abs(e[0] - stamp))[2]

    def publish_pose(self, corrected, covariance, stamp, bridge_message):
        out = PoseWithCovarianceStamped()
        out.header.frame_id = self.p['map_frame']
        out.header.stamp = stamp
        out.pose.pose.position.x, out.pose.pose.position.y = corrected.x, corrected.y
        out.pose.pose.position.z = (float(bridge_message.pose.pose.position.z)
                                    if bridge_message is not None else 0.)
        out.pose.pose.orientation.z = math.sin(corrected.yaw / 2)
        out.pose.pose.orientation.w = math.cos(corrected.yaw / 2)
        # Planar-only correction: z, roll and pitch explicitly unobserved.
        cov = [0.0] * 36
        for index, variance in ((0, covariance[0, 0]), (7, covariance[1, 1]),
                                (14, 100.0), (21, 100.0), (28, 100.0),
                                (35, covariance[2, 2])):
            cov[index] = float(variance)
        out.pose.covariance = cov
        self.pose_pub.publish(out)

    def publish_global_map(self):
        output = LandmarkArray()
        output.header.frame_id = self.p['map_frame']
        output.header.stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.header = output.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        palette = {0: (.04, .36, 1.), 1: (1., .88, .08),
                   2: (1., .38, .05), 3: (1., .16, .10)}
        for index, anchor in enumerate(self.estimator.anchors):
            if anchor.hits < 3 or anchor.kind not in palette:
                continue
            lm = Landmark()
            lm.identifier = index
            lm.type = anchor.kind
            lm.position.x = float(anchor.xy[0])
            lm.position.y = float(anchor.xy[1])
            # A conservative map confirmation score, not a calibrated
            # detection probability suitable for control.
            lm.probability = float(min(.95, .55 + .05 * anchor.hits))
            output.landmarks.append(lm)
            glyph = Marker()
            glyph.header = output.header
            glyph.ns = 'fusion_paired_global_map'
            glyph.id = index
            glyph.type = Marker.SPHERE
            glyph.action = Marker.ADD
            glyph.pose.position.x = lm.position.x
            glyph.pose.position.y = lm.position.y
            glyph.pose.position.z = .2
            glyph.pose.orientation.w = 1.
            glyph.scale.x = glyph.scale.y = .26 if anchor.kind != 3 else .38
            glyph.scale.z = .36 if anchor.kind != 3 else .45
            glyph.color.r, glyph.color.g, glyph.color.b = palette[anchor.kind]
            glyph.color.a = .8
            markers.markers.append(glyph)
        self.global_map_pub.publish(output)
        self.global_markers_pub.publish(markers)

    def publish_status(self):
        last_lidar = self.estimator.last_lidar_stamp
        recent_lidar = (last_lidar is not None and self.last_pair_stamp is not None and
                        abs(last_lidar - self.last_pair_stamp) <=
                        float(self.p['maximum_lidar_odom_age_s']))
        state = {
            'state': self.reason,
            'paired_frames': self.frames,
            'corrected_pose_frames': self.accepted_poses,
            'rejected_or_unmatched': self.rejected,
            'resets': self.reset_count,
            'sensor_matches': self.last_counts,
            'reference_anchors': len(self.estimator.anchors),
            'bridge_anchored_fused_global_cones': sum(
                a.hits >= 3 for a in self.estimator.anchors),
            'fused_global_map_loop_optimized': False,
            'pose_reference_matches': self.estimator.last_matches,
            'last_pose_fit_rms_m': self.estimator.last_residual,
            'lidar_odometry_initialized': self.estimator.prediction is not None,
            'lidar_odometry_recent_for_last_pair': recent_lidar,
            'lidar_odom_input_state': self.pose_input_state,
            'last_rejected_lidar_odom_frames': self.last_rejected_lidar_frames,
            'pose_correction_state': (
                'waiting_for_normalized_lidar_odom'
                if self.estimator.prediction is None else
                'waiting_for_recent_lidar_odom'
                if not recent_lidar else self.reason),
            'pose_covariance_xyyaw': list(map(float, self.estimator.prediction_cov.diagonal())),
            'pose_requires_normalized_lidar_odom': True,
            'map_anchored_to_bridge_odom': True,
        }
        self.status_pub.publish(String(data=json.dumps(state, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
