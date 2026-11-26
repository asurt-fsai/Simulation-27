"""Isolated synthetic ROS callbacks; no CARLA, /clock, TF or live sensors."""
import json
from types import SimpleNamespace
import unittest

from asurt_msgs.msg import Landmark, LandmarkArray
from nav_msgs.msg import Odometry
import rclpy
from visualization_msgs.msg import Marker, MarkerArray

from asurt_slam_fusion.node import FusionNode


class Collector:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeExtrinsics:
    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        if (target_frame, source_frame) != ('base_link', 'velodyne'):
            raise AssertionError('unexpected sensor TF lookup')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=.5, y=0., z=.2),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))


def stamp(time_s):
    from builtin_interfaces.msg import Time
    sec = int(time_s)
    return Time(sec=sec, nanosec=int(round((time_s - sec) * 1e9)))


def odom(time_s, x=0., lidar=False):
    msg = Odometry()
    msg.header.stamp = stamp(time_s)
    msg.header.frame_id = 'lidar_odom' if lidar else 'map'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.
    return msg


def cones(time_s, is_lidar=False):
    msg = LandmarkArray()
    msg.header.stamp = stamp(time_s)
    msg.header.frame_id = 'base_link'
    for x, y, kind in ((3., -1., 0), (5., 2., 1),
                       (7., -.5, 0), (9., 2.2, 1)):
        lm = Landmark()
        lm.position.x, lm.position.y, lm.position.z = x, y, 0.1
        lm.type = 4 if is_lidar else kind
        lm.probability = .9
        lm.identifier = -1 if is_lidar else 2
        msg.landmarks.append(lm)
    return msg


class FusionNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = FusionNode()
        self.fused = Collector()
        self.visuals = Collector()
        self.corrected = Collector()
        self.health = Collector()
        self.global_cones = Collector()
        self.global_visuals = Collector()
        self.node.landmarks_pub = self.fused
        self.node.markers_pub = self.visuals
        self.node.pose_pub = self.corrected
        self.node.status_pub = self.health
        self.node.global_map_pub = self.global_cones
        self.node.global_markers_pub = self.global_visuals

    def tearDown(self):
        self.node.destroy_node()

    def paired(self, t):
        self.node.receive('lidar', cones(t, is_lidar=True))
        self.node.receive('camera', cones(t))

    def test_only_cross_sensor_matches_and_independent_pose_correction(self):
        self.node.bridge_cb(odom(1.))
        self.paired(1.)
        self.assertEqual(len(self.fused.messages), 1)
        self.assertEqual(len(self.fused.messages[0].landmarks), 4)
        self.assertEqual(self.fused.messages[0].header.frame_id, 'base_link')
        self.assertFalse(self.corrected.messages)
        self.node.lidar_odom_cb(odom(1., lidar=True))
        self.node.bridge_cb(odom(1.1))
        self.node.lidar_odom_cb(odom(1.1, lidar=True))
        self.paired(1.1)
        self.assertFalse(self.corrected.messages)
        self.node.bridge_cb(odom(1.2))
        self.node.lidar_odom_cb(odom(1.2, x=.15, lidar=True))
        self.paired(1.2)
        self.assertEqual(len(self.corrected.messages), 1)
        pose = self.corrected.messages[0]
        self.assertEqual(pose.header.frame_id, 'map')
        self.assertLess(pose.pose.pose.position.x, .15)
        self.assertGreater(pose.pose.covariance[14], 50.)
        self.node.publish_status()
        state = json.loads(self.health.messages[-1].data)
        self.assertEqual(state['sensor_matches']['matched'], 4)
        self.assertEqual(state['corrected_pose_frames'], 1)
        self.assertGreaterEqual(state['pose_reference_matches'], 3)

    def test_wrong_lidar_odom_frame_and_reset(self):
        self.node.bridge_cb(odom(2.))
        raw = odom(2., lidar=True)
        raw.header.frame_id = 'camera_init'
        raw.child_frame_id = 'camera'
        self.node.lidar_odom_cb(raw)
        self.paired(2.)
        self.assertFalse(self.corrected.messages)
        self.assertFalse(self.node.estimator.prediction)
        self.node.publish_status()
        status = json.loads(self.health.messages[-1].data)
        self.assertEqual(status['pose_correction_state'],
                         'waiting_for_normalized_lidar_odom')
        self.assertEqual(status['last_rejected_lidar_odom_frames'],
                         ['camera_init', 'camera'])
        self.node.bridge_cb(odom(.5))
        self.assertEqual(self.node.reset_count, 1)
        self.assertFalse(self.node.estimator.anchors)
        self.assertEqual(len(self.visuals.messages[-1].markers), 1)
        self.assertEqual(self.visuals.messages[-1].markers[0].action,
                         Marker.DELETEALL)
        self.assertEqual(self.visuals.messages[-1].markers[0].header.frame_id,
                         'base_link')

    def test_paired_markers_have_local_frame_distinct_shapes_and_clear(self):
        self.paired(2.)
        self.assertEqual(len(self.visuals.messages), 1)
        current = self.visuals.messages[-1]
        self.assertIsInstance(current, MarkerArray)
        self.assertEqual(len(current.markers), 9)  # CLEAR + 4 spheres + 4 rings
        self.assertEqual(current.markers[0].action, Marker.DELETEALL)
        self.assertTrue(all(m.header.frame_id == 'base_link'
                            for m in current.markers))
        self.assertTrue(all(m.header.stamp == stamp(2.)
                            for m in current.markers))
        spheres = [m for m in current.markers if m.ns == 'fusion_paired_cones']
        rings = [m for m in current.markers if m.ns == 'fusion_paired_halo']
        self.assertEqual(len(spheres), 4)
        self.assertEqual(len(rings), 4)
        self.assertTrue(all(m.type == Marker.SPHERE for m in spheres))
        self.assertTrue(all(m.type == Marker.LINE_STRIP and len(m.points) == 17
                            for m in rings))
        self.assertNotEqual(spheres[0].color.b, spheres[1].color.b)
        self.assertTrue(all(r.color.g > .9 and r.color.b > .8 for r in rings))
        self.assertTrue(all(m.lifetime.sec == 0 and m.lifetime.nanosec > 0
                            for m in spheres + rings))
        # No paired landmarks on the next frame must clear old glyphs.
        empty = cones(2.1, is_lidar=True)
        empty.landmarks = []
        self.node.receive('lidar', empty)
        self.node.receive('camera', cones(2.1))
        self.assertEqual(len(self.visuals.messages[-1].markers), 1)
        self.assertEqual(self.visuals.messages[-1].markers[0].action,
                         Marker.DELETEALL)
        self.node.reset('unit_test_reset')
        self.assertEqual(len(self.visuals.messages[-1].markers), 1)
        self.assertEqual(self.visuals.messages[-1].markers[0].action,
                         Marker.DELETEALL)
        self.assertEqual(self.visuals.messages[-1].markers[0].header.frame_id,
                         'base_link')

    def test_timestamped_lidar_extrinsics_and_ego_motion(self):
        self.node.tf = FakeExtrinsics()
        self.node.bridge_cb(odom(1., x=0.))
        self.node.bridge_cb(odom(1.1, x=1.))
        frame = cones(1.08, is_lidar=True)
        frame.header.frame_id = 'velodyne'
        converted = self.node.cones_in_base('velodyne', frame, 1.08,
                                            target_stamp=1.)
        # LiDAR x3 + vehicle mount .5 + movement .8 = camera-time x4.3.
        self.assertAlmostEqual(converted[0].x, 4.3, places=6)
        self.assertAlmostEqual(converted[0].z, .3, places=6)
        self.node.odom_history.clear()
        with self.assertRaisesRegex(ValueError, 'motion compensation'):
            self.node.cones_in_base('velodyne', frame, 1.08,
                                    target_stamp=1.)

    def test_confirmed_global_map_requires_repeated_cross_sensor_pairs(self):
        self.node.bridge_cb(odom(1.))
        self.paired(1.)
        self.node.publish_global_map()
        self.assertEqual(len(self.global_cones.messages[-1].landmarks), 0)
        self.node.bridge_cb(odom(1.1))
        self.paired(1.1)
        self.node.publish_global_map()
        self.assertEqual(len(self.global_cones.messages[-1].landmarks), 0)
        self.node.bridge_cb(odom(1.2))
        self.paired(1.2)
        self.node.publish_global_map()
        out = self.global_cones.messages[-1]
        self.assertEqual(out.header.frame_id, 'map')
        self.assertEqual(len(out.landmarks), 4)
        self.assertEqual(len(self.global_visuals.messages[-1].markers), 5)
        self.assertEqual(self.global_visuals.messages[-1].markers[0].action,
                         Marker.DELETEALL)
        self.node.publish_status()
        state = json.loads(self.health.messages[-1].data)
        self.assertFalse(state['fused_global_map_loop_optimized'])
        self.assertEqual(state['bridge_anchored_fused_global_cones'], 4)
        self.node.reset('map_change')
        self.assertEqual(len(self.global_cones.messages[-1].landmarks), 0)
        self.assertEqual(len(self.global_visuals.messages[-1].markers), 1)


if __name__ == '__main__':
    unittest.main()
