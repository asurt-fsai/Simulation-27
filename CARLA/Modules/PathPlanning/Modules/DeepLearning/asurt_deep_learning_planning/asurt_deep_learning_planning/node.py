"""ROS2 visualization-only adapter for the original ASURT pretrained LSTM."""
import json
import math
from pathlib import Path as FilePath
import tempfile
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields

import rclpy
from asurt_msgs.msg import LandmarkArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from .core import (
    Limits, check_prediction, paired_gates, prepare_cones, raw_prediction_points,
    preview_age_allowed,
)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform_point(position, transform):
    """Apply timestamped 3-D TF, then project the landmark onto base_link XY."""
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z = float(position.x), float(position.y), float(position.z)
    qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = qx*qx + qy*qy + qz*qz + qw*qw
    if not .98 <= norm <= 1.02:
        raise ValueError('invalid_transform_rotation')
    # Unit-quaternion rotation R*v = v + 2*w*(q x v) + 2*(q x (q x v)).
    cx, cy, cz = qy*z-qz*y, qz*x-qx*z, qx*y-qy*x
    ccx, ccy = qy*cz-qz*cy, qz*cx-qx*cz
    return x + 2*(qw*cx+ccx) + t.x, y + 2*(qw*cy+ccy) + t.y


class ONNXModel:
    """One explicitly selected provider. Never silently call CPU a GPU backend."""
    PROVIDERS = {
        'cpu': 'CPUExecutionProvider',
        'cuda': 'CUDAExecutionProvider',
        'tensorrt': 'TensorrtExecutionProvider',
    }

    def __init__(self, path, backend):
        import numpy as np
        import onnxruntime as ort
        if backend not in self.PROVIDERS:
            raise ValueError('backend must be cpu, cuda or tensorrt')
        self.np = np
        provider = self.PROVIDERS[backend]
        if provider not in ort.get_available_providers():
            raise RuntimeError('provider_unavailable:' + provider)
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        if backend != 'cpu':
            options.enable_profiling = True
            options.profile_file_prefix = str(
                FilePath(tempfile.gettempdir()) / ('asurt_dl_provider_' + uuid.uuid4().hex))
        self.session = ort.InferenceSession(path, sess_options=options, providers=[provider])
        if provider not in self.session.get_providers():
            raise RuntimeError('provider_not_active:' + provider)
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1 or list(inputs[0].shape) != [1, 10, 4] or list(outputs[0].shape) != [1, 15, 2]:
            raise ValueError('legacy_model_shape_mismatch')
        self.input_name, self.output_name = inputs[0].name, outputs[0].name
        self.backend = backend
        self.providers = self.session.get_providers()
        self.profiled_op_providers = {}
        if backend != 'cpu':
            # ONNX Runtime may expose CUDA yet assign every operator to CPU.
            # Execute a private warmup and require actual GPU operator events
            # before making this backend available to ROS path publication.
            profile_path = None
            try:
                self.session.run(
                    [self.output_name],
                    {self.input_name: np.zeros((1, 10, 4), dtype=np.float32)})
            finally:
                profile_path = self.session.end_profiling()
            try:
                if not profile_path:
                    raise RuntimeError('gpu_operator_profile_missing')
                events = json.loads(FilePath(profile_path).read_text(encoding='utf-8'))
                for event in events:
                    if event.get('cat') != 'Node':
                        continue
                    name = event.get('args', {}).get('provider', 'unattributed')
                    self.profiled_op_providers[name] = (
                        self.profiled_op_providers.get(name, 0) + 1)
                if not self.profiled_op_providers.get(provider, 0):
                    raise RuntimeError('no_requested_gpu_operator_executed')
            finally:
                if profile_path:
                    FilePath(profile_path).unlink(missing_ok=True)

    def predict(self, rows):
        data = self.np.asarray(rows, dtype=self.np.float32).reshape((1, 10, 4))
        start = time.monotonic()
        result = self.session.run([self.output_name], {self.input_name: data})[0]
        elapsed = (time.monotonic() - start) * 1000
        return result.reshape((15, 2)), elapsed


class DeepLearningPlanner(Node):
    def __init__(self):
        super().__init__('asurt_deep_learning_planner')
        for name, value in (
            ('input_source', 'perception'),  # perception | map | auto
            ('perception_topic', '/perception_landmarks'),
            ('global_cones_topic', '/map/global_cones'),
            ('odometry_topic', '/odom'),
            ('model_path', ''),
            ('backend', 'cuda'),
            ('max_age_s', .7),
            ('max_result_age_s', .25),
            # Only the raw, read-only preview may accept completed inferences
            # that missed the tight safety-gated result deadline.
            ('raw_result_max_age_s', 3.0),
            # RViz-only retention of the LAST actual model prediction.
            # Never used for the strict safety-gated path or Control.
            ('raw_preview_hold_s', 8.0),
            ('teleport_distance_m', 8.),
            ('planning_period_s', .2),
        ):
            self.declare_parameter(name, value)
        for field in fields(Limits):
            self.declare_parameter(field.name, getattr(Limits(), field.name))
        p = lambda key: self.get_parameter(key).value
        self.limits = Limits(**{f.name: p(f.name) for f in fields(Limits)})
        self.source = str(p('input_source'))
        if self.source not in ('perception', 'map', 'auto'):
            raise ValueError('input_source must be perception, map or auto')
        self.max_age_ns = int(max(.1, float(p('max_age_s'))) * 1e9)
        self.max_result_ns = int(max(.05, float(p('max_result_age_s'))) * 1e9)
        self.raw_result_max_ns = int(max(self.max_age_ns / 1e9,
                                        float(p('raw_result_max_age_s'))) * 1e9)
        self.raw_hold_ns = int(max(float(p('max_age_s')),
                                   float(p('raw_preview_hold_s')), .2) * 1e9)
        self.teleport_distance_m = max(1., float(p('teleport_distance_m')))
        self.backend = str(p('backend'))
        self.model = None
        self.model_error = None
        try:
            path = str(p('model_path'))
            if not path:
                raise ValueError('model_path_not_configured')
            self.model = ONNXModel(path, self.backend)
        except Exception as exc:
            self.model_error = type(exc).__name__ + ':' + str(exc)[:220]
            self.get_logger().error('DeepLearning model unavailable: ' + self.model_error)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(Path, '/planning/deep_learning/path', 10)
        # Original prediction remains observable in RViz. A separate
        # experimental simulator controller may use it with its own source
        # timestamp/odometry checks; this planner never sends /drive.
        self.raw_pub = self.create_publisher(Path, '/planning/deep_learning/raw_path', 10)
        self.status_pub = self.create_publisher(String, '/planning/deep_learning/status', 10)
        self.latency_pub = self.create_publisher(Float32, '/planning/deep_learning/inference_ms', 10)
        self.snapshots = {}
        self.input_error = None
        self.last_input_stamps = {}
        self.last_clock = 0
        self.last_odom_stamp = 0
        self.last_pose = None
        self.generation = 0
        self.had_path = False
        self.valid_path_stamp = 0
        self.valid_path_source = None
        self.valid_path_points = 0
        self.valid_path_clipped = False
        self.had_raw_path = False
        self.raw_path_stamp = 0
        self.raw_path_source = None
        self.pending = None
        self.pending_meta = None
        self.executor_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='asurt_dl_onnx')
        self.last_inference_ms = None
        self.reason = 'waiting_for_cones'
        self.create_subscription(Odometry, str(p('odometry_topic')), self._on_odom,
                                 qos_profile_sensor_data)
        if self.source in ('perception', 'auto'):
            self.create_subscription(LandmarkArray, str(p('perception_topic')),
                                     lambda msg: self._on_cones(msg, 'perception'), 10)
        if self.source in ('map', 'auto'):
            self.create_subscription(LandmarkArray, str(p('global_cones_topic')),
                                     lambda msg: self._on_cones(msg, 'map'), 10)
        self.create_timer(max(.1, float(p('planning_period_s'))), self._tick)
        self.get_logger().info('Deep Learning publishes a validated planner path and the original LSTM preview')

    def _clear_path(self):
        if not self.had_path:
            return
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        self.path_pub.publish(message)
        self.had_path = False
        self.valid_path_stamp = 0
        self.valid_path_source = None
        self.valid_path_points = 0
        self.valid_path_clipped = False

    def _clear_raw_path(self):
        if not self.had_raw_path:
            return
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        self.raw_pub.publish(message)
        self.had_raw_path = False
        self.raw_path_stamp = 0
        self.raw_path_source = None

    def _reset(self, reason):
        self.snapshots.clear()
        self.last_input_stamps.clear()
        self.input_error = reason
        self.generation += 1
        self.reason = reason
        self._clear_path()
        self._clear_raw_path()

    def _on_odom(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if msg.header.frame_id.lstrip('/') != 'map' or not stamp:
            return
        point = msg.pose.pose.position
        pos = (float(point.x), float(point.y))
        if not all(math.isfinite(c) for c in pos):
            return
        if self.last_odom_stamp and stamp < self.last_odom_stamp - 100_000_000:
            self._reset('odometry_rewind')
        elif self.last_pose and math.dist(pos, self.last_pose) > self.teleport_distance_m:
            self._reset('vehicle_teleport_or_map_change')
        self.last_pose, self.last_odom_stamp = pos, stamp

    def _on_cones(self, msg, source):
        stamp = stamp_ns(msg.header.stamp)
        frame = msg.header.frame_id.lstrip('/')
        if not stamp or not frame:
            self.snapshots.pop(source, None)
            self.reason = 'missing_cone_stamp_or_frame'
            self.input_error = self.reason
            return
        last = self.last_input_stamps.get(source, 0)
        if last and stamp < last - 100_000_000:
            self._reset('cone_timestamp_rewind')
        elif last and stamp <= last:
            return
        self.last_input_stamps[source] = stamp
        now = self.get_clock().now().nanoseconds
        if now <= 0 or stamp > now + 200_000_000 or now-stamp > self.max_age_ns:
            self.snapshots.pop(source, None)
            self.reason = 'stale_or_future_cones'
            self.input_error = self.reason
            return
        if frame != 'base_link':
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', frame, Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=.025))
            except TransformException:
                self.snapshots.pop(source, None)
                self.reason = 'missing_timestamped_transform'
                self.input_error = self.reason
                return
        else:
            tf = None
        observations = []
        for lm in msg.landmarks[:2048]:
            xyzp = (lm.position.x, lm.position.y, lm.position.z, lm.probability)
            if not all(math.isfinite(float(v)) for v in xyzp):
                continue
            try:
                x, y = transform_point(lm.position, tf) if tf else (
                    float(lm.position.x), float(lm.position.y))
            except ValueError:
                self.reason = 'invalid_timestamped_transform'
                self.snapshots.pop(source, None)
                self.input_error = self.reason
                return
            observations.append((x, y, int(lm.type), float(lm.probability)))
        self.snapshots[source] = (stamp, tuple(observations))
        self.input_error = None

    def _recent(self, now, stamp):
        return stamp > 0 and -200_000_000 <= now-stamp <= self.max_age_ns

    def _make_path(self, points, source_stamp_ns=None):
        path = Path()
        # The raw local prediction is relative to the vehicle at INPUT time.
        # Keep source time even if CUDA returns late; RViz may retain it longer
        # than a moving controller may use it.
        path.header.stamp = (Time(nanoseconds=int(source_stamp_ns)).to_msg()
                             if source_stamp_ns is not None else
                             self.get_clock().now().to_msg())
        path.header.frame_id = 'base_link'
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _publish(self, points, source_stamp_ns):
        self.path_pub.publish(self._make_path(points, source_stamp_ns=source_stamp_ns))
        self.had_path = True

    def _publish_raw(self, points, stamp, source):
        self.raw_pub.publish(self._make_path(points, source_stamp_ns=stamp))
        self.had_raw_path = True
        self.raw_path_stamp = stamp
        self.raw_path_source = source

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock and now < self.last_clock - 100_000_000:
            self._reset('simulation_clock_rewind')
        self.last_clock = now
        source = None
        gates = 0
        finished = False
        if self.pending is not None and self.pending.done():
            finished = True
            request_stamp, request_source, request_cones, request_generation = self.pending_meta
            try:
                prediction, latency_ms = self.pending.result()
                self.last_inference_ms = latency_ms
                self.latency_pub.publish(Float32(data=float(latency_ms)))
                latest = self.snapshots.get(request_source)
                # The raw preview is evidence of what the LSTM actually
                # predicted. Never discard a bounded late result merely
                # because it is too old for the independent gated trajectory.
                # Generation changes (teleport/rewind) still invalidate it.
                raw_points = raw_prediction_points(prediction)
                raw_result_ok = (
                    preview_age_allowed(now, request_stamp, self.raw_result_max_ns,
                                        request_generation == self.generation)
                    and bool(raw_points)
                )
                if raw_result_ok:
                    self._publish_raw(raw_points, request_stamp, request_source)
                elif not raw_points:
                    self._clear_raw_path()
                if (request_generation != self.generation or not self._recent(now, request_stamp)
                        or latest is None or latest[0]-request_stamp > self.max_result_ns):
                    self.reason = 'stale_inference_result'
                else:
                    points, self.reason, gates = check_prediction(
                        prediction, request_cones, self.limits)
                    if self.reason == 'ok':
                        self._publish(points, request_stamp)
                        self.valid_path_stamp = request_stamp
                        self.valid_path_source = request_source
                        self.valid_path_points = len(points)
                        self.valid_path_clipped = (points[0][0] > raw_points[0][0] or
                                                   points[-1][0] < raw_points[-1][0])
            except Exception as exc:
                self.reason = 'inference_failed:' + type(exc).__name__
                self.get_logger().error('DeepLearning inference failed: ' + str(exc)[:200])
            self.pending = None
            self.pending_meta = None
        if finished:
            pass  # Preserve the completed result for this status cycle.
        elif now <= 0:
            self.reason = 'waiting_for_sim_clock'
        elif self.model is None:
            self.reason = 'model_unavailable'
        elif self.pending is not None:
            self.reason = 'inference_busy'
        else:
            sources = ('map', 'perception') if self.source == 'auto' else (self.source,)
            self.reason = self.input_error or 'waiting_for_fresh_cones'
            for candidate in sources:
                snapshot = self.snapshots.get(candidate)
                if snapshot is None or not self._recent(now, snapshot[0]):
                    continue
                rows, observed, reason = prepare_cones(snapshot[1], self.limits)
                self.reason = reason
                if rows is None:
                    continue
                gates_tuple, reason = paired_gates(observed, self.limits)
                self.reason = reason
                if reason != 'ok':
                    continue
                gates = len(gates_tuple)
                source = candidate
                self.pending_meta = (snapshot[0], candidate, observed, self.generation)
                self.pending = self.executor_pool.submit(self.model.predict, rows)
                self.reason = 'inference_pending'
                break
        valid_snapshot = self.snapshots.get(self.valid_path_source)
        can_display_previous = (
            self.had_path and self._recent(now, self.valid_path_stamp)
            and valid_snapshot is not None
            and valid_snapshot[0] - self.valid_path_stamp <= self.max_result_ns
            and self.reason in ('ok', 'inference_pending', 'inference_busy')
        )
        if not can_display_previous:
            self._clear_path()
        raw_is_fresh = (
            self.had_raw_path
            and self.raw_path_stamp > 0
            and -200_000_000 <= now - self.raw_path_stamp <= self.raw_hold_ns
        )
        if not raw_is_fresh:
            self._clear_raw_path()
        status = {
            'state': ('preview' if can_display_previous else
                      'raw_preview' if self.had_raw_path else 'unavailable'),
            'reason': self.reason, 'source': source,
            'gates': gates, 'backend_requested': self.backend,
            'backend_session': self.model.providers if self.model else [],
            'profiled_op_providers': (
                self.model.profiled_op_providers if self.model else {}),
            'model_error': self.model_error,
            'inference_ms': self.last_inference_ms,
            'control_ready': bool(can_display_previous), 'visualization_only': not can_display_previous,
            'control_path_points': self.valid_path_points,
            'control_path_clipped': self.valid_path_clipped,
            'raw_preview_active': bool(self.had_raw_path),
            'raw_preview_age_s': (round(max(0, now-self.raw_path_stamp)/1e9, 3)
                                  if self.had_raw_path else None),
            'raw_preview_result_max_age_s': round(self.raw_result_max_ns / 1e9, 3),
            'raw_preview_last_observation_may_be_stale': (
                bool(self.had_raw_path) and not self._recent(now, self.raw_path_stamp)),
            'raw_preview_is_safety_approved': False,
            'path_frame': 'base_link',
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def destroy_node(self):
        self.executor_pool.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeepLearningPlanner()
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
