"""ROS message adapter tests with no simulator, publishers, or active ROS spin."""
import json
import math
import os

os.environ['ROS_DOMAIN_ID'] = '194'

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('asurt_msgs')
from asurt_msgs.msg import Landmark, LandmarkArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from asurt_camera_slam.node import CameraSLAMNode
from asurt_camera_slam.core import Pose2
from test_core import WORLD, observe


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def stamp(message, seconds):
    message.header.stamp.sec = int(seconds)
    message.header.stamp.nanosec = int(round((seconds-int(seconds))*1e9))
    return message


def odom(pose, seconds):
    msg = stamp(Odometry(), seconds)
    msg.header.frame_id = 'map'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = pose.x
    msg.pose.pose.position.y = pose.y
    msg.pose.pose.orientation.z = math.sin(pose.yaw/2.)
    msg.pose.pose.orientation.w = math.cos(pose.yaw/2.)
    return msg


def camera(pose, seconds):
    msg = stamp(LandmarkArray(), seconds)
    msg.header.frame_id = 'base_link'
    for point in observe(pose):
        lm = Landmark()
        lm.position.x, lm.position.y, lm.position.z = point.x, point.y, point.z
        lm.type, lm.probability = point.color, point.confidence
        msg.landmarks.append(lm)
    return msg


@pytest.fixture
def node():
    rclpy.init()
    node = CameraSLAMNode()
    node.pose_pub, node.status_pub, node.map_pub = Capture(), Capture(), Capture()
    node.markers_pub, node.path_pub = Capture(), Capture()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_pose_emission_requires_repeated_multiview_camera_geometry(node):
    for k in range(6):
        pose = Pose2(.20*k, 0., 0.)
        t = 1.+k*.1
        node.odom_cb(odom(pose, t))
        node.camera_cb(camera(pose, t))
        if k < 4:
            assert not node.pose_pub.messages
    assert len(node.engine.confirmed()) >= 4
    prior_pose_publishes = len(node.pose_pub.messages)
    true_pose = Pose2(1.2, 0., 0.)
    node.odom_cb(odom(Pose2(1.42, .12, .065), 1.6))
    node.camera_cb(camera(true_pose, 1.6))
    assert len(node.pose_pub.messages) == prior_pose_publishes + 1
    msg = node.pose_pub.messages[-1]
    assert msg.header.frame_id == 'map'
    assert msg.pose.covariance[0] > 0
    assert msg.pose.covariance[35] > 0
    assert msg.pose.covariance[14] >= 1000
    status = json.loads(node.status_pub.messages[-1].data)
    assert status['state'] == 'corrected'
    assert status['inliers'] >= 3
    assert status['pose_published']
    node.publish_map()
    assert node.map_pub.messages[-1].header.frame_id == 'map'
    assert node.markers_pub.messages[-1].markers[0].action == Marker.DELETEALL
    assert len(node.markers_pub.messages[-1].markers) >= 4
    assert all(m.header.frame_id == 'map'
               for m in node.markers_pub.messages[-1].markers)
    assert len(node.path_pub.messages[-1].poses) == prior_pose_publishes + 1


def test_frame_guard_reset_and_world_identity_reset(node):
    t = 1.
    incoming = camera(Pose2(0., 0., 0.), t)
    incoming.header.frame_id = 'zed_camera'
    node.camera_cb(incoming)
    assert node.last_state == 'unexpected_camera_frame'
    assert not node.pose_pub.messages
    node.map_id_cb(String(data='silverstone'))
    for k in range(6):
        p = Pose2(.20*k, 0., 0.)
        node.odom_cb(odom(p, t+.1*k))
        node.camera_cb(camera(p, t+.1*k))
    assert node.engine.confirmed()
    before_reset = len(node.pose_pub.messages)
    node.map_id_cb(String(data='FlatAsphalt'))
    assert not node.engine.anchors
    assert not node.odom_history
    assert node.reset_count == 1
    assert node.last_state.startswith('reset:')
    assert len(node.pose_pub.messages) == before_reset
    assert node.markers_pub.messages[-1].markers[0].action == Marker.DELETEALL
    assert not node.path_pub.messages[-1].poses


def test_two_camera_cones_or_only_odometry_never_emit_pose(node):
    node.odom_cb(odom(Pose2(0., 0., 0.), 1.))
    assert not node.pose_pub.messages
    for k in range(7):
        p = Pose2(.20*k, 0., 0.)
        t = 1.+.1*k
        node.odom_cb(odom(p, t))
        msg = camera(p, t)
        msg.landmarks = msg.landmarks[:2]
        node.camera_cb(msg)
    assert not node.pose_pub.messages
    assert len(node.engine.confirmed()) <= 2
