# FS-System-26 simulator integration audit

The upstream `main` branch contains the repository scaffold. All 38 upstream
branch tips were inventoried for ROS packages and deployment entrypoints. The
restore source is the July 17 race snapshot `ff05c31688c9d974517c6d3cb043c60dc8316256`,
on `acceleration_17_7_10_50_AM`. The July 12 competition integration, July 15
LiDAR integration, and Xavier branches were compared to this snapshot. The
standalone `control` branch is an MPC development branch. The race snapshot
contains the Simple and Adaptive Pure Pursuit packages.

| Current CARLA module | Legacy source or simulator role |
| --- | --- |
| ZED 2i Stereo Camera | Keep the updated simulated camera and its current detector. |
| Velodyne LiDAR | Keep the simulated sensor that supplies `/velodyne_points`. The legacy LiDAR processor consumes this stream. |
| LiDAR Cone Detection | `Perception_Lidar/src/lidarpipeline`. The original publishes `/filtered_points` and `/cluster_markers`. |
| ZED Legacy Converter | Keep the message converter. It converts the updated camera's ZED object messages into `/perception_markers` (`MarkerArray`) and `/perception_landmarks` (`asurt_msgs/LandmarkArray`). The legacy DL planner and camera mapper respectively consume those topics. |
| Vehicle Trajectory, FakeMAP, FakeSLAM | Keep these CARLA diagnostics and test fixtures as explicitly requested. |
| Cone Landmark Mapper, Camera Landmark SLAM | The race snapshot supplies `SLAM_Camera/src/cone_mapping`. The mapped variant expects `pyzed` and the ZED pose and trajectory topics. The local variant has a different output frame. |
| LeGO-LOAM LiDAR SLAM, LiDAR Odometry Alignment | The LiDAR competition branch supplies its own `LeGO-LOAM`. The CARLA odometry alignment node is simulator-specific integration code. |
| Camera + LiDAR Fusion | CARLA integration. The branches contain separate July camera and LiDAR bringups. |
| Classical Cone Path Planner | `Centerline/src/planning_centerline_calc_MIX`. Local and global console entrypoints differ in frame and input topics. |
| Acceleration Path Planner | `Acceleration_Planning_NEW`. Preserve its original parameters and publish contract. |
| Skidpad Path Planner | `planning_skidpad`. The snapshot supplies `skidpad_hardcoded_planner`; the other skidpad console entrypoint refers to a missing `skidpad_node.py`. |
| Deep Learning Path Planner | `Deeplearning/src/planning_deep_learning`. The original main node consumes `/perception_markers` and publishes all predicted waypoints to `/path` in `zed_left_camera_frame`. The CARLA corridor gate was added later. |
| RViz Goal Pose, SLAM Track Goal | Both are CARLA features. The upstream classical package has no `/goal_pose` input. A single RViz goal workflow needs to consume the classical route and implement goal selection and cancellation outside the legacy planner. |
| Pure Pursuit (Goal Pose), Stanley High-Speed Track, Pure Pursuit Autonomous, Stanley High-Speed Autonomous | CARLA control implementations. The July packages are `control_simple/src/simple_pure_pursuit` and `control_adaptive/adaptivePurepursuit`. |

## Imported source and verification

The release adds the two July controllers and the camera-driven cone mapper.
The controller source and gains are preserved. The simulator input adapter
selects the planner topic and expresses its path in `base_link`. When a path
already uses that frame, Simple Pure Pursuit skips the legacy TF helper's
identity transform, which otherwise interprets ROS's quaternion in the wrong
order. Stanley and the former simulator Pure Pursuit entries are retired from
the bundled catalog. Selected local builds keep their launcher and settings.

The single RViz goal subscribes to the classical cone route. It uses only the
route up to the nearest projection of a goal on that route. Cancel Goal
clears the selected route. The imported planner and LiDAR sources are also
available in the integration workspace for the remaining migration.

The staged `LegacyFS26` directories contain the upstream ROS runtime source and
configuration. Original source files were checked against the upstream Git blob IDs.
The Deep Learning artifacts include both original engines and the original
ONNX and PyTorch weights. The ONNX SHA-256 is
`03f0d1aa9b3d36da5275d7acb74197df1a34834603974a99e8399fb11c8ece6d`.
The PyTorch SHA-256 is
`70129c5090ced6086e37897a1000a5806ff189fc217b057f0994d74be2ad0f16`.

Seven imported packages built on ROS Humble. Both original controllers
published commands in an isolated ROS domain using synthetic odometry and
paths, with only ROS topic remaps. The original global classical planner
published 33 map-frame waypoints from a synthetic two-sided cone course
after its saved parameter block and ROS topics were bound at launch. The
original ONNX file also produced all 15 waypoints in a CPU inference test.
Closed-loop parity needs the correct legacy frame, steering convention, and
VCU behavior. The legacy CAN bridge converts speed to motor RPM with a 3.5
gear ratio and sends a separate torque request. The CARLA bridge uses the
simulator's Ackermann actuator.

The exact main Deep Learning node needs PyCUDA and a compatible TensorRT
runtime and engine. The original node has no CPU inference backend. The
simulator copy of the original global camera mapper uses the converter's
base-link landmarks and CARLA odometry and drops the unused ZED SDK import.
The installer retirement mechanism keys removals
to individual former bundled package directories and the original seed
hashes. Workspace edits and selected local package paths are preserved.
