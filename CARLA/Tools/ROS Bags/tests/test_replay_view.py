"""Replay view and QoS regressions for the Ubuntu/WSL ROS 2 runtime."""

from pathlib import Path
import sys
import tempfile
import unittest

import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bag_backend import read_bag, recorded_frame, replay_qos, rviz_config


class ReplayViewTests(unittest.TestCase):
    def make_bag(self, path, with_tf):
        odometry = Odometry()
        odometry.header.frame_id, odometry.child_frame_id = "map", "base_link"
        odometry.pose.pose.position.y = 847.0
        cloud = PointCloud2()
        cloud.header.frame_id = "velodyne"
        messages = [("/odom", "nav_msgs/msg/Odometry", odometry),
                    ("/velodyne_points", "sensor_msgs/msg/PointCloud2", cloud)]
        if with_tf:
            for topic, parent, child in (("/tf", "map", "base_link"), ("/tf_static", "base_link", "velodyne")):
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = parent, child
                messages.append((topic, "tf2_msgs/msg/TFMessage", TFMessage(transforms=[transform])))
        writer = SequentialWriter()
        writer.open(StorageOptions(uri=str(path), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
        for number, (topic, message_type, message) in enumerate(messages):
            writer.create_topic(TopicMetadata(name=topic, type=message_type, serialization_format="cdr"))
            writer.write(topic, serialize_message(message), 1_000_000_000 + number)

    def test_view_follows_the_car_in_a_global_frame(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "bag"
            self.make_bag(bag, with_tf=True)
            topics = read_bag(bag)[-1]
            fixed, target, warning = recorded_frame(bag, topics, "")
            self.assertEqual((fixed, target, warning), ("map", "base_link", ""))
            config = Path(temporary) / "view.rviz"
            rviz_config(config, topics, fixed, target)
            view = yaml.safe_load(config.read_text())["Visualization Manager"]
            self.assertEqual(view["Global Options"]["Fixed Frame"], "map")
            self.assertEqual(view["Views"]["Current"]["Target Frame"], "base_link")
            self.assertEqual(recorded_frame(bag, topics, "survey")[:2], ("survey", "<Fixed Frame>"))

    def test_cloud_only_frame_is_used_when_transforms_are_unavailable(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "bag"
            self.make_bag(bag, with_tf=False)
            fixed, target, warning = recorded_frame(bag, read_bag(bag)[-1], "")
            self.assertEqual((fixed, target), ("velodyne", "<Fixed Frame>"))
            self.assertIn("No recorded TF", warning)

    def test_only_best_effort_display_topics_get_replay_overrides(self):
        metadata = {"topics_with_message_count": [
            {"topic_metadata": {"name": name, "type": kind, "offered_qos_profiles": str(offers)}}
            for name, kind, offers in (
                ("/lidar", "sensor_msgs/msg/PointCloud2", [{"reliability": 1, "durability": 2}]),
                ("/image", "sensor_msgs/msg/Image", [{"reliability": 2, "durability": 2}]),
                ("/markers", "visualization_msgs/msg/MarkerArray", [{"reliability": 2, "durability": 1}]),
                ("/clock", "rosgraph_msgs/msg/Clock", [{"reliability": 2, "durability": 2}]),
                ("/unselected", "sensor_msgs/msg/Image", [{"reliability": 2, "durability": 2}]),
            )]}
        profiles = replay_qos(metadata, {"/lidar", "/image", "/markers", "/clock"})
        self.assertEqual(set(profiles), {"/image", "/markers"})
        self.assertEqual(profiles["/image"]["reliability"], "reliable")
        self.assertEqual(profiles["/image"]["durability"], "volatile")
        self.assertEqual(profiles["/markers"]["durability"], "transient_local")
