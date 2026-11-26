"""Isolated ROS callbacks; no live sensors, simulator, CARLA, /clock, or TF."""
import json
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import rclpy

from asurt_lidar_odom_adapter.node import (
    NormalizationNode, decode_pose, covariance_variances)
from test.test_core import series


def stamp(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def odom(t, pose, lidar):
    msg = Odometry()
    msg.header.stamp = stamp(t)
    msg.header.frame_id = ('slam_lidar_adjusted_map' if lidar else 'map')
    msg.child_frame_id = ('slam_lidar_adjusted' if lidar else 'base_link')
    msg.pose.pose.position.x = pose.x
    msg.pose.pose.position.y = pose.y
    msg.pose.pose.orientation.z = math.sin(pose.yaw / 2)
    msg.pose.pose.orientation.w = math.cos(pose.yaw / 2)
    return msg


class Collector:
    def __init__(self):
        self.sent = []

    def publish(self, message):
        self.sent.append(message)


class NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = NormalizationNode()
        self.out, self.status = Collector(), Collector()
        self.node.pub = self.out
        self.node.status_pub = self.status

    def tearDown(self):
        self.node.destroy_node()

    def pairs(self, samples):
        for p in samples:
            self.node.bridge_cb(odom(p.t, p.bridge, False))
            self.node.raw_cb(odom(p.t, p.lidar, True))

    def test_undecided_silent_then_rep103_output_not_bridge_republication(self):
        samples = series()
        self.pairs(samples[:25])
        self.assertFalse(self.out.sent)
        self.node.publish_status()
        self.assertFalse(json.loads(self.status.sent[-1].data)['validated'])
        self.pairs(samples[25:])
        self.assertGreater(len(self.out.sent), 5)
        result = self.out.sent[-1]
        self.assertEqual((result.header.frame_id, result.child_frame_id),
                         ('lidar_odom', 'base_link'))
        self.assertEqual(result.header.stamp, stamp(samples[-1].t))
        self.assertAlmostEqual(result.pose.pose.position.x,
                               samples[-1].bridge.x, places=5)
        self.assertAlmostEqual(result.pose.pose.position.y,
                               samples[-1].bridge.y, places=5)
        self.assertGreater(result.pose.covariance[14], 90.)
        self.assertGreater(result.pose.covariance[0], 0.)
        self.node.publish_status()
        state = json.loads(self.status.sent[-1].data)
        self.assertTrue(state['validated'])
        self.assertTrue(state['bridge_only_for_calibration_and_consistency'])
        self.assertFalse(state['calibration']['source_covariance_reported'])
        self.assertGreater(state['source_covariance_unreported_messages'], 0)
        # Vary only LeGO with unchanged bridge: output tracks independent LeGO.
        previous = samples[-1]
        next_sample = odom(previous.t + .1, previous.lidar, True)
        next_sample.pose.pose.position.x += .21
        self.node.bridge_cb(odom(previous.t + .1, previous.bridge, False))
        self.node.raw_cb(next_sample)
        self.assertGreater(abs(self.out.sent[-1].pose.pose.position.x -
                               result.pose.pose.position.x), .18)

    def test_reject_wrong_frames_covariances_and_quaternions(self):
        p = series(1)[0]
        wrong = odom(p.t, p.lidar, True)
        wrong.header.frame_id = 'camera_init'
        self.node.raw_cb(wrong)
        self.assertEqual(self.node.last_rejection, 'raw_frame_contract_mismatch')
        bad_cov = odom(p.t, p.lidar, True)
        bad_cov.pose.covariance[0] = -1.
        self.node.raw_cb(bad_cov)
        self.assertEqual(self.node.last_rejection, 'negative_covariance')
        bad_q = odom(p.t, p.lidar, True)
        bad_q.pose.pose.orientation.x = 1.
        bad_q.pose.pose.orientation.w = 0.
        with self.assertRaisesRegex(ValueError, 'nonplanar'):
            decode_pose(bad_q)
        too_uncertain = odom(p.t, p.lidar, True)
        too_uncertain.pose.covariance[0] = 5.
        with self.assertRaisesRegex(ValueError, 'excessive'):
            covariance_variances(too_uncertain)
        correlated = odom(p.t, p.lidar, True)
        correlated.pose.covariance[0] = .30
        correlated.pose.covariance[1] = .20
        correlated.pose.covariance[6] = .20
        correlated.pose.covariance[7] = .30
        self.assertAlmostEqual(covariance_variances(correlated)[0], .5)
        self.assertFalse(self.out.sent)

    def test_frame_change_revokes_previously_validated_calibration(self):
        self.pairs(series())
        self.assertTrue(self.node.adapter.lock is not None)
        bad = odom(10., series()[-1].lidar, True)
        bad.header.frame_id = 'different_map_after_restart'
        self.node.raw_cb(bad)
        self.assertIsNone(self.node.adapter.lock)
        self.assertEqual(self.node.adapter.state, 'raw_frame_contract_mismatch')
        self.assertGreater(self.node.adapter.resets, 0)

    def test_interpolated_bridge_freshness_and_time_rewind(self):
        p = series(1)[0]
        self.node.bridge_cb(odom(p.t - .02, p.bridge, False))
        self.node.bridge_cb(odom(p.t + .02, p.bridge, False))
        self.node.raw_cb(odom(p.t, p.lidar, True))
        self.assertEqual(len(self.node.adapter.samples), 1)
        # Raw messages are not matched to an arbitrarily stale bridge stamp.
        q = series(1)[0]
        self.node.raw_cb(odom(q.t + 1., q.lidar, True))
        self.node.bridge_cb(odom(q.t + 1.2, q.bridge, False))
        self.assertEqual(len(self.node.adapter.samples), 1)
        self.assertEqual(self.node.last_rejection,
                         'no_timestamped_bridge_correspondence')
        self.node.bridge_cb(odom(.1, p.bridge, False))
        self.assertEqual(len(self.node.adapter.samples), 0)
        self.assertGreaterEqual(self.node.adapter.resets, 1)
        self.assertFalse(self.out.sent)

    def test_ros_clock_age_gate_is_fail_closed(self):
        p = series(1)[0]
        fake_clock = SimpleNamespace(now=lambda:
            SimpleNamespace(to_msg=lambda: stamp(5.)))
        with patch.object(self.node, 'get_parameter',
                          return_value=SimpleNamespace(value=True)):
            with patch.object(self.node, 'get_clock', return_value=fake_clock):
                self.node.raw_cb(odom(p.t, p.lidar, True))
        self.assertEqual(self.node.last_rejection, 'raw_stale_or_future_sample')
        self.assertFalse(self.out.sent)


if __name__ == '__main__':
    unittest.main()
