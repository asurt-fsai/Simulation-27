#!/usr/bin/env python3
"""Show the requested MPC speed and steering beside the vehicle in RViz."""

from math import cos, degrees, isfinite, sin

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


def command_markers(command):
    speed, angle = command.drive.speed, command.drive.steering_angle
    if not (isfinite(speed) and isfinite(angle)):
        return None
    markers = []
    for ident, kind in enumerate((Marker.TEXT_VIEW_FACING, Marker.ARROW)):
        marker = Marker(header=Header(frame_id="base_link"), ns="mpc_command", id=ident,
                        type=kind, action=Marker.ADD, frame_locked=True,
                        lifetime=Duration(nanosec=500_000_000),
                        color=ColorRGBA(r=1.0, g=0.73, b=0.26, a=1.0))
        marker.pose.orientation.w = 1.0
        markers.append(marker)

    text, arrow = markers
    steering = degrees(angle)
    direction = "left" if steering > 0.05 else "right" if steering < -0.05 else "straight"
    text.text = f"MPC request  {speed * 3.6:.1f} km/h\nSteer {abs(steering):.1f}° {direction}"
    text.pose.position.z = 1.35
    text.scale.z = 0.28
    arrow.points = [Point(x=1.0, z=0.35), Point(x=1.0 + 1.7 * cos(angle), y=1.7 * sin(angle), z=0.35)]
    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.15, 0.24
    return MarkerArray(markers=markers)


class MpcCommandView(Node):
    def __init__(self):
        super().__init__("asurt_mpc_command_view")
        self.markers = self.create_publisher(MarkerArray, "/asurt/rviz/mpc_commands", 5)
        self.create_subscription(AckermannDriveStamped, "/mpc/command_preview", self.receive,
                                 qos_profile_sensor_data)

    def receive(self, command):
        if markers := command_markers(command):
            self.markers.publish(markers)


if __name__ == "__main__":
    rclpy.init()
    node = MpcCommandView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
