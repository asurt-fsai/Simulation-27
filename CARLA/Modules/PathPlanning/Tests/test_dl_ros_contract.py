#!/usr/bin/env python3
"""Isolated ROS2 test of real pretrained ONNX inference and defensive state.

Requires an explicitly non-live ROS_DOMAIN_ID and existing planning overlay.
No CARLA, vehicle, odometry or perception publisher is created in the live domain.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from asurt_msgs.msg import Landmark, LandmarkArray
from builtin_interfaces.msg import Time
from nav_msgs.msg import Path as RosPath
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, String


ROOT = Path(os.environ.get('ASURT_ROOT', Path(__file__).resolve().parents[4]))
MODEL = (ROOT / 'Simulation/Systems/Legacy/fs-system-26-oldstack/Deeplearning'
         / 'src/planning_deep_learning/planning_deep_learning/Completed_Models'
         / 'best_model.onnx')


def stamp(seconds):
    integral = int(seconds)
    return Time(sec=integral, nanosec=int((seconds-integral)*1e9))


def markers(t, paired=True, frame='base_link'):
    msg = LandmarkArray()
    msg.header.frame_id = frame
    msg.header.stamp = stamp(t)
    for x in (2., 4., 6., 8., 10.):
        for kind, y in ((Landmark.BLUE_CONE, 2.), (Landmark.YELLOW_CONE, -2.)):
            if not paired and kind == Landmark.YELLOW_CONE:
                continue
            lm = Landmark()
            lm.type = kind
            lm.position.x, lm.position.y = x, y
            lm.probability = .96
            msg.landmarks.append(lm)
    return msg


def exercise():
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('Set isolated nonzero ROS_DOMAIN_ID for this test')
    if not MODEL.is_file():
        raise FileNotFoundError(MODEL)
    # CPU is an isolated functional test; the optional CUDA run must prove
    # that ORT actually executed at least one GPU operator during warmup.
    backend = os.environ.get('ASURT_DL_TEST_BACKEND', 'cpu').strip().lower()
    if backend not in ('cpu', 'cuda'):
        raise RuntimeError('ASURT_DL_TEST_BACKEND must be cpu or cuda')
    process = subprocess.Popen([
        sys.executable, '-m', 'asurt_deep_learning_planning.node', '--ros-args',
        '-p', 'use_sim_time:=true', '-p', 'backend:=' + backend,
        '-p', 'model_path:=' + str(MODEL),
        '-r', '/perception_landmarks:=/dl_contract/cones',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    rclpy.init()
    node = rclpy.create_node('asurt_dl_contract_test')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    cone_pub = node.create_publisher(LandmarkArray, '/dl_contract/cones', 10)
    paths, raw_paths, statuses, timing = [], [], [], []
    source_stamps = set()
    node.create_subscription(RosPath, '/planning/deep_learning/path',
                             lambda m: paths.append((m.header.frame_id,
                                 (m.header.stamp.sec, m.header.stamp.nanosec),
                                 tuple((p.pose.position.x, p.pose.position.y) for p in m.poses))), 10)
    node.create_subscription(RosPath, '/planning/deep_learning/raw_path',
                             lambda m: raw_paths.append(
                                 (m.header.frame_id,
                                  tuple((pose.pose.position.x, pose.pose.position.y)
                                        for pose in m.poses))), 10)
    node.create_subscription(String, '/planning/deep_learning/status',
                             lambda m: statuses.append(json.loads(m.data)), 10)
    node.create_subscription(Float32, '/planning/deep_learning/inference_ms',
                             lambda m: timing.append(m.data), 10)
    current = 20.
    try:
        def send(cones=True, paired=True, frame='base_link'):
            nonlocal current
            current += .06
            clk = Clock()
            clk.clock = stamp(current)
            clock_pub.publish(clk)
            if cones:
                message = markers(current, paired, frame)
                if paired and frame == 'base_link':
                    source_stamps.add((message.header.stamp.sec, message.header.stamp.nanosec))
                cone_pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.06)
            if process.poll() is not None:
                raise RuntimeError('deep-learning planner child terminated')

        # Missing one boundary must never invoke the model.
        # GPU provider initialization includes a real CUDA warmup/profile;
        # under an already-running CARLA renderer this can exceed CPU startup.
        deadline = time.monotonic() + (35 if backend == 'cuda' else 8)
        while time.monotonic() < deadline:
            send(paired=False)
            if statuses and statuses[-1]['reason'] == 'model_unavailable':
                raise AssertionError(
                    'ONNX model unavailable: ' + str(statuses[-1]['model_error']))
            if any(s['reason'] == 'need_two_cones_each_side' for s in statuses):
                break
        else:
            raise AssertionError(
                'one-sided cones were not rejected; last statuses: '
                + str(statuses[-4:]))
        assert not timing, 'model ran with only one color boundary'

        # An unknown landmark frame must not be treated as base_link.
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            send(frame='unmapped_test_frame')
            if any(s['reason'] == 'missing_timestamped_transform' for s in statuses):
                break
        else:
            raise AssertionError('missing TF did not invalidate landmark frame')
        assert not timing, 'model inferred without a valid transform'

        # The model predicts beyond the ten-metre fixture. Control receives
        # the validated overlap, and the raw prediction is still visible.
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            send()
            if timing and any(
                    ('CPUExecutionProvider' in s['backend_session'] if backend == 'cpu'
                     else (s['profiled_op_providers'].get('CUDAExecutionProvider', 0) > 0))
                    and s['control_ready'] and s['control_path_clipped']
                    for s in statuses) and any(poses for _, _, poses in paths) \
                    and any(len(poses) == 15 for _, poses in raw_paths):
                break
        else:
            raise AssertionError(
                'no clipped control path from paired cones. '
                f'Latest statuses: {statuses[-4:]}')
        if any(frame != 'base_link' for frame, _, _ in paths):
            raise AssertionError('unexpected output frame')
        if not any(frame == 'base_link' and len(poses) == 15 for frame, poses in raw_paths):
            raise AssertionError('no true 15-point raw LSTM preview')
        if not any(poses[-1][0] > 10.0 for _, poses in raw_paths if len(poses) == 15):
            raise AssertionError('raw model prediction was clipped at visible cones')
        for _, source_stamp, points in paths:
            if points:
                assert source_stamp in source_stamps, 'control path lost the input observation timestamp'
                assert 2 <= len(points) <= 15 and points[-1][0] <= 10., points
        assert node.count_publishers('/drive') == 0, 'planner published a drive command'

        # No fresh observation: the strictly gated path stays unavailable,
        # but the last actual raw preview may remain visible and explicitly
        # marked stale for the configured display-only hold period.
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            send(cones=False)
            if statuses and statuses[-1]['state'] == 'raw_preview' and not statuses[-1]['control_ready'] and (
                statuses[-1]['reason'] in ('waiting_for_fresh_cones', 'stale_inference_result')):
                break
        else:
            raise AssertionError('planner did not distinguish the raw-only stale preview')
        if not statuses[-1]['raw_preview_last_observation_may_be_stale']:
            # It may take additional cycles to age past max_age_s.
            for _ in range(16):
                send(cones=False)
        if not any(s['raw_preview_last_observation_may_be_stale'] for s in statuses):
            raise AssertionError('stale raw preview not explicitly labeled')
        assert any(not points for _, _, points in paths), 'stale control path was not cleared'
        # Raw display must also expire even if there are no new observations.
        previous_count = len(raw_paths)
        deadline = time.monotonic() + 14
        while time.monotonic() < deadline:
            send(cones=False)
            if any(len(poses) == 0 for _, poses in raw_paths[previous_count:]):
                break
        else:
            raise AssertionError('raw preview did not clear when its display hold expired')
        current = 1.
        for _ in range(10):
            send(cones=False)
        assert statuses[-1]['state'] == 'unavailable'
        print('PASS isolated ROS (' + backend + '): one-sided and TF gating, '
              'real ONNX inference (%d timings), observed-horizon clipping, '
              'base_link outputs, stale clearing, sim-clock rewind' % len(timing))
        print('Last status:', json.dumps(statuses[-1], sort_keys=True))
        print('Published non-empty gated paths:', sum(bool(points) for _, _, points in paths))
        print('Published 15-point raw previews:', sum(len(poses) == 15 for _, poses in raw_paths))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            _, errors = process.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            _, errors = process.communicate(timeout=4)
        if process.returncode not in (0, -15):
            print('Planner stderr:', errors.decode(errors='replace')[-2500:])


if __name__ == '__main__':
    exercise()
