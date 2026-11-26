"""Acceleration-specific ROS node sharing validated input contracts."""
import rclpy
from rclpy.executors import ExternalShutdownException
from asurt_classical_planning.node import ClassicalPlanner
from .geometry import AccelerationConfig, acceleration_path


def main(args=None):
    rclpy.init(args=args)
    node = ClassicalPlanner(profile='acceleration', strategy=acceleration_path,
                            config_type=AccelerationConfig)
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
