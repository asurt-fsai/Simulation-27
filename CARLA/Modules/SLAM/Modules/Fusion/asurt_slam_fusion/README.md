# ASURT passive camera/LiDAR fusion

This independent ROS2 Humble module consumes the existing **single** camera
converter `/perception_landmarks` (`asurt_msgs/LandmarkArray`, `base_link`),
LiDAR cone processor `/lidar/cones` (the same message, `velodyne`), `/odom`
(`nav_msgs/Odometry`, `map`), and existing extrinsic TF. The cone processor
publishes colour `4=UNKNOWN`; the camera supplies class/colour. Pairing uses
bounded timestamp skew, timestamp-specific TF and odometry-based ego-motion
compensation. Invalid transforms, missing motion compensation, or frames with
no paired cones produce no pose correction.

Outputs are deliberately separate from Phase 1:

| Topic | Message | Semantics |
|---|---|---|
| `/slam/fusion/landmarks` | `asurt_msgs/LandmarkArray` | `base_link`, matched camera AND LiDAR cones only. Colour from camera; confidence is heuristic. |
| `/slam/fusion/markers` | `visualization_msgs/MarkerArray` | `base_link`, local paired cone spheres coloured by camera class with cyan confirmation halos. Clears previous glyphs each paired frame (including empty sets) and sends DELETEALL on every reset. Markers have a 0.75-second lifetime. |
| `/slam/fusion/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map`, **only** when recent independent LiDAR odometry predicts pose and 3+ stable cross-sensor map matches support a bounded SE(2) correction. Vertical/roll/pitch covariance is deliberately large. |
| `/slam/fusion/status` | `std_msgs/String` | JSON counters, pairing/matching, pose correction state, covariance, map-size/reset counters and most recent gate result. |

Independent LiDAR odometry is subscribed at `/slam/lidar/odom` as
`nav_msgs/Odometry`; **the required header frame is `lidar_odom`, child
`base_link`, with conventional right-handed REP-103 XY/yaw orientation**.
LeGO-LOAM `/integrated_to_init` historically publishes `camera_init`/`camera`
in a nonstandard axis convention. It must be **validated and normalized**
before feeding this consumer. Remapping its topic alone does not satisfy the
coordinate contract, and frame rejection is intentional. The mapped `/aft_mapped_to_init`
is an alternative upstream pose to evaluate, but should not be used alongside
integrated odometry as if they were independent measurements. The local
historical implementation explicitly reorders LiDAR points `(x,y,z)` to
`(y,z,x)` in `featureAssociation.cpp:328-330`. It publishes mapped-adjusted
translation as `(internal_z,internal_x,internal_y)` in
`mapOptmization.cpp:1010-1012` with a separate quaternion component/sign
permutation in lines 1006-1009. `transformFusion.cpp:347-354` uses a
different raw-coordinate publication. No verified rigid-body REP-103
conversion and yaw sign have been established for these combined paths.
For this reason, neither `/slam/lidar/odom` nor `/slam/lidar/mapped_adjusted`
historical raw messages are treated as normalized solely by remapping.
Status explicitly reports `waiting_for_normalized_lidar_odom` and the last
rejected frame pair. Validate a future conversion against recorded stationary,
straight, and turning tracks before changing its source labels.

The local map of coloured cone anchors is bootstrapped from `/odom` and paired
measurements. The LiDAR odometry displacement is integrated relative to this
reference; the cross-sensor points provide a rigid-registration correction
when 3+ sufficiently spread landmarks reappear. A registration is rejected if
translation exceeds 0.85 m, yaw exceeds 0.15 rad, or RMS residual exceeds
0.35 m; outputs remain silent until accepted. Covariances are *engineering
floors*, not measured or statistically consistent localization uncertainty.
The map remains locally anchored to the bridge pose and is **not** an accepted
loop-closing SLAM solution or independent absolute localization. Do not use
this corrected pose for driving control without ground-truth-free testing,
calibration, and acceptance on both Silverstone and FlatAsphalt.

This module has **no automatic startup**, controls, additional TF broadcasters,
new actor or sensor instances, global map writer, or CARLA dependencies. Enable
it explicitly after the System Manager owner registers it; keep the existing
Phase 1 `/slam/pose` and `/map/global_cones` publishers unchanged.

To build from the module directory using the project ROS underlays:

```bash
source /opt/ros/humble/setup.bash
source /home/hassan/ASURT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
colcon build --base-paths asurt_slam_fusion --packages-select asurt_slam_fusion --symlink-install
```

To test the pure numeric part without starting sensors:

```bash
PYTHONPATH=asurt_slam_fusion:$PYTHONPATH ROS_DOMAIN_ID=189 python3 -m unittest discover -s asurt_slam_fusion/test -v
```
