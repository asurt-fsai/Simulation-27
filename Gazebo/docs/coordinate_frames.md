# Coordinate Frames

Use standard ROS body conventions throughout the simulator unless a documented conversion is required at an asset-import boundary.

    +X = forward
    +Y = left
    +Z = up

All distances are metres and angles are radians unless explicitly documented otherwise.

The intended frame tree is:

    map
    └── odom
        └── base_link
            ├── chassis
            ├── front_left_wheel
            ├── front_right_wheel
            ├── rear_left_wheel
            ├── rear_right_wheel
            ├── imu_link
            ├── lidar_link
            └── camera_link

Asset authors must document any source-coordinate conversion used by Blender, CAD, or map tooling. A committed asset should already appear at the correct scale and orientation in Gazebo.
