# Camera perception modules

The editable optics, stereo synchronizer, cone detector and tracker live in
the [ZED 2i ROS package](../../Adapters/CARLA/Camera/asurt_zed2i_sim).
The tests here run offline.

    /usr/bin/python3 -m unittest discover -s Simulation/Systems/Perception/Modules/Camera/Tests -v

The current camera detects cone colors using stereo images; it has not been refactored into a standalone ROS package for Gazebo. A future Gazebo adapter must implement its camera source and calibration without depending on CARLA APIs.

YOLO26 retains the distinct `large_orange` label from the FSOCO model across
right-eye stereo matching and tracking. Its 505 mm physical height informs
range gating and apparent-height depth estimation; regular cones use 325 mm.
Right-eye HSV matching supplies image evidence for orange hue without
overriding the left-eye YOLO cone-size classification. The HSV-only backend
remains limited to generic orange.
