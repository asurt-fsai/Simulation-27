"""Calibrated, passive LeGO adjusted-odometry adapter; no /tf or /control."""
from collections import deque
import json
import math

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .core import Pair, Pose, TrajectoryCalibration, wrap


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def decode_pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(v).all():
        raise ValueError('nonfinite_pose')
    norm = float(np.linalg.norm(v[3:]))
    if not .90 <= norm <= 1.10:
        raise ValueError('invalid_quaternion_norm')
    x, y, z, w = v[3:] / norm
    up = 1. - 2. * (x * x + y * y)
    if math.acos(max(-1., min(1., up))) > .30:
        raise ValueError('nonplanar_or_misoriented_quaternion')
    yaw = math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))
    return Pose(float(p.x), float(p.y), yaw)


def covariance_variances(msg):
    c = np.asarray(msg.pose.covariance, dtype=float)
    if c.shape != (36,) or not np.isfinite(c).all():
        raise ValueError('invalid_covariance')
    xy = c[[0, 1, 6, 7]].reshape(2, 2)
    if not np.allclose(xy, xy.T, rtol=0., atol=1e-6):
        raise ValueError('asymmetric_xy_covariance')
    xy_eigenvalues = np.linalg.eigvalsh(xy)
    if xy_eigenvalues.min() < -1e-7 or c[35] < 0:
        raise ValueError('negative_covariance')
    if xy_eigenvalues.max() > 1. or c[35] > .25:
        raise ValueError('excessive_covariance')
    # The maximum XY eigenvalue upper-bounds variance under ANY unknown
    # calibrated planar rotation; do not ignore cross-covariance terms.
    worst_xy = float(max(0., xy_eigenvalues.max()))
    return worst_xy, worst_xy, float(c[35])


class NormalizationNode(Node):
    def __init__(self):
        super().__init__('asurt_lidar_odom_adapter')
        defaults = {
            'raw_topic': '/slam/lidar/mapped_adjusted',
            'bridge_topic': '/odom',
            'normalized_topic': '/slam/lidar/normalized_odom',
            'status_topic': '/slam/lidar/normalization_status',
            'raw_frame': 'slam_lidar_adjusted_map',
            'raw_child_frame': 'slam_lidar_adjusted',
            'bridge_frame': 'map',
            'bridge_child_frame': 'base_link',
            'output_frame': 'lidar_odom',
            'output_child_frame': 'base_link',
            'sync_tolerance_s': .07,
            'maximum_input_age_s': .35,
            'minimum_pairs': 40,
            'maximum_pairs': 96,
            'minimum_distance_m': 5.,
            'minimum_turn_rad': .48,
            'max_fit_rms_m': .20,
            'max_scale_error': .08,
            'max_postlock_difference_m': 1.,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if not (.015 <= self.p['sync_tolerance_s'] <= .12 and
                .08 <= self.p['maximum_input_age_s'] <= .8 and
                24 <= self.p['minimum_pairs'] <= self.p['maximum_pairs'] <= 192):
            raise ValueError('invalid_bounded_adapter_parameters')
        self.adapter = TrajectoryCalibration(
            min_pairs=int(self.p['minimum_pairs']),
            max_pairs=int(self.p['maximum_pairs']),
            min_distance=float(self.p['minimum_distance_m']),
            min_turn=float(self.p['minimum_turn_rad']),
            max_rms=float(self.p['max_fit_rms_m']),
            max_scale_error=float(self.p['max_scale_error']),
            max_disagreement=float(self.p['max_postlock_difference_m']))
        self.bridges = deque(maxlen=160)
        self.raws = deque(maxlen=24)
        self.last_raw = None
        self.last_bridge = None
        self.previous_output = None
        self.rejected = 0
        self.unspecified_covariance = 0
        self.last_rejection = None
        self.pub = self.create_publisher(
            Odometry, self.p['normalized_topic'], 10)
        self.status_pub = self.create_publisher(
            String, self.p['status_topic'], 10)
        self.create_subscription(
            Odometry, self.p['raw_topic'], self.raw_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.p['bridge_topic'], self.bridge_cb,
            qos_profile_sensor_data)
        self.create_timer(.5, self.publish_status)
        self.get_logger().info(
            'LeGO normalization passive; outputs gated by rigid calibration')

    def reset(self, reason):
        self.adapter.reset(reason)
        self.raws.clear()
        self.bridges.clear()
        self.last_raw = None
        self.last_bridge = None
        self.previous_output = None

    def _decode(self, msg, source):
        prefix = 'raw' if source == 'raw' else 'bridge'
        if (msg.header.frame_id.lstrip('/') != self.p[prefix + '_frame'] or
                msg.child_frame_id.lstrip('/') != self.p[prefix + '_child_frame']):
            raise ValueError(prefix + '_frame_contract_mismatch')
        t = seconds(msg.header.stamp)
        if not math.isfinite(t) or t <= 0:
            raise ValueError('invalid_timestamp')
        previous = self.last_raw if source == 'raw' else self.last_bridge
        if previous is not None and t <= previous:
            if t < previous - .5:
                self.reset(source + '_time_rewind')
            else:
                raise ValueError(source + '_out_of_order_stamp')
        if self.get_parameter('use_sim_time').value:
            now = seconds(self.get_clock().now().to_msg())
            if now > 0 and (now - t > self.p['maximum_input_age_s'] or
                            t - now > .12):
                raise ValueError(source + '_stale_or_future_sample')
        pose = decode_pose(msg)
        cov = covariance_variances(msg)
        if not any(cov):
            self.unspecified_covariance += 1
        if source == 'raw':
            self.last_raw = t
        else:
            self.last_bridge = t
        return t, pose, cov, msg

    def _receive(self, msg, source):
        try:
            data = self._decode(msg, source)
        except ValueError as exc:
            reason = str(exc)
            # Never preserve an old calibration after evidence of a
            # different map/sensor frame, stale clock epoch or corrupt pose.
            if any(part in reason for part in (
                    'frame_contract_mismatch', 'stale_or_future',
                    'nonplanar', 'quaternion', 'covariance', 'nonfinite')):
                self.reset('revoke_' + reason)
            self.rejected += 1
            self.last_rejection = reason
            self.adapter.state = reason
            return
        (self.raws if source == 'raw' else self.bridges).append(data)
        self.drain()

    def raw_cb(self, msg):
        self._receive(msg, 'raw')

    def bridge_cb(self, msg):
        self._receive(msg, 'bridge')

    def interpolate_bridge(self, stamp):
        if not self.bridges:
            return None
        buffer = list(self.bridges)
        tolerance = self.p['sync_tolerance_s']
        # A matching current bridge sample must win over a much older
        # bracketing sample (e.g. 10 Hz LiDAR/bridge in the isolated tests).
        close = min(buffer, key=lambda v: abs(v[0] - stamp))
        if abs(close[0] - stamp) <= min(.012, tolerance):
            return close[1], close[2]
        for a, b in zip(buffer, buffer[1:]):
            if a[0] <= stamp <= b[0]:
                if max(stamp - a[0], b[0] - stamp) > tolerance:
                    return None
                alpha = (stamp - a[0]) / max(1e-9, b[0] - a[0])
                pa, pb = a[1], b[1]
                pose = Pose(pa.x + alpha * (pb.x - pa.x),
                            pa.y + alpha * (pb.y - pa.y),
                            wrap(pa.yaw + alpha * wrap(pb.yaw - pa.yaw)))
                var = tuple(max(av, bv) for av, bv in zip(a[2], b[2]))
                return pose, var
        return None

    def drain(self):
        while self.raws:
            raw = self.raws[0]
            pair_bridge = self.interpolate_bridge(raw[0])
            if pair_bridge is None:
                if self.bridges and self.bridges[-1][0] > (
                        raw[0] + self.p['sync_tolerance_s']):
                    self.raws.popleft()
                    self.rejected += 1
                    self.last_rejection = 'no_timestamped_bridge_correspondence'
                    continue
                return
            self.raws.popleft()
            bridge, bridge_cov = pair_bridge
            previous_reset_count = self.adapter.resets
            result = self.adapter.feed(Pair(
                raw[0], raw[1], bridge, raw[2], bridge_cov))
            if self.adapter.resets != previous_reset_count:
                self.previous_output = None
            if result is not None:
                self.publish_normalized(result, raw[3])

    def publish_normalized(self, result, raw_msg):
        out = Odometry()
        out.header = raw_msg.header
        out.header.frame_id = self.p['output_frame']
        out.child_frame_id = self.p['output_child_frame']
        out.pose.pose.position.x = result.pose.x
        out.pose.pose.position.y = result.pose.y
        out.pose.pose.orientation.z = math.sin(result.pose.yaw / 2.)
        out.pose.pose.orientation.w = math.cos(result.pose.yaw / 2.)
        cov = [0.] * 36
        for index, variance in ((0, result.var_xyyaw[0]),
                                (7, result.var_xyyaw[1]),
                                (14, 100.), (21, 100.),
                                (28, 100.), (35, result.var_xyyaw[2])):
            cov[index] = float(variance)
        out.pose.covariance = cov
        # Legacy LeGO twist has unverified axis conventions; do NOT copy it.
        # Derive planar rates from consecutive *LiDAR-derived* normalized poses.
        twist_cov = [0.] * 36
        prev = self.previous_output
        if prev is not None and .01 <= result.t - prev.t <= .8:
            dt = result.t - prev.t
            vx = (result.pose.x - prev.pose.x) / dt
            vy = (result.pose.y - prev.pose.y) / dt
            c, s = math.cos(result.pose.yaw), math.sin(result.pose.yaw)
            out.twist.twist.linear.x = c * vx + s * vy
            out.twist.twist.linear.y = -s * vx + c * vy
            out.twist.twist.angular.z = wrap(
                result.pose.yaw - prev.pose.yaw) / dt
            rate_var = min(100., 2. * max(result.var_xyyaw[:2]) / dt ** 2)
            twist_cov[0] = twist_cov[7] = max(.1, rate_var)
            twist_cov[35] = max(.1, min(100., 2. * result.var_xyyaw[2] / dt ** 2))
        else:
            twist_cov[0] = twist_cov[7] = twist_cov[35] = 100.
        for i in (14, 21, 28):
            twist_cov[i] = 100.
        out.twist.covariance = twist_cov
        self.previous_output = result
        self.pub.publish(out)

    def publish_status(self):
        lock = self.adapter.lock
        calibration = None if lock is None else {
            'fit_rms_m': lock.train_rms,
            'heldout_rms_m': lock.heldout_rms,
            'yaw_rms_rad': lock.yaw_rms,
            'scale_ratio': lock.scale_ratio,
            'rotation_rad': lock.angle,
            'source_covariance_reported': lock.lidar_cov_reported,
        }
        self.status_pub.publish(String(data=json.dumps({
            'state': self.adapter.state,
            'validated': lock is not None,
            'correspondence_count': len(self.adapter.samples),
            'independent_lidar_output_frames': self.adapter.emitted,
            'resets': self.adapter.resets,
            'invalid_pairs': self.adapter.rejected,
            'rejected_inputs': self.rejected,
            'last_rejection': self.last_rejection,
            'source_covariance_unreported_messages': self.unspecified_covariance,
            'calibration': calibration,
            'bridge_only_for_calibration_and_consistency': True,
            'no_tf': True,
        }, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = NormalizationNode()
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
