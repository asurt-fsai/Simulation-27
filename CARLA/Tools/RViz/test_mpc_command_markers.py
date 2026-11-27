"""Verify the read-only MPC command display using ROS message types."""
from math import sin
import unittest

from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from mpc_command_markers import command_markers


class MpcCommandTests(unittest.TestCase):
    def test_steering_arrow_and_caption_reflect_requested_command(self):
        command = AckermannDriveStamped()
        command.drive.speed = 2.0
        for angle, side in ((0.2, "left"), (-0.2, "right"), (0.0, "straight")):
            with self.subTest(angle=angle):
                command.drive.steering_angle = angle
                text, arrow = command_markers(command).markers
                self.assertEqual(text.type, Marker.TEXT_VIEW_FACING)
                self.assertEqual(text.header.frame_id, "base_link")
                self.assertIn("7.2 km/h", text.text)
                self.assertIn(side, text.text)
                self.assertAlmostEqual(arrow.points[-1].y, 1.7 * sin(angle))
                self.assertEqual(text.lifetime.nanosec, 500_000_000)

    def test_invalid_command_is_not_rendered(self):
        command = AckermannDriveStamped()
        command.drive.speed = float("nan")
        self.assertIsNone(command_markers(command))
