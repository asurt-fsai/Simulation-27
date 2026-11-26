"""Skidpad-specific ROS adapter, requiring mapped repeated observations."""
import rclpy
from rclpy.executors import ExternalShutdownException
from asurt_classical_planning.node import ClassicalPlanner
from .geometry import SkidpadConfig, skidpad_path


def main(args=None):
    rclpy.init(args=args)
    node = ClassicalPlanner(profile='skidpad', strategy=skidpad_path,
                            config_type=SkidpadConfig)
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
