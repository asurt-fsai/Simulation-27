# ASURT Formula Student path planning — cone/LSTM planners and RViz Goal Pose

The legacy baseline is at
`Simulation/Systems/Legacy/fs-system-26-oldstack/Centerline/src/planning_centerline_calc_MIX`.
Its original pipeline combines cone matching, path ordering and corner-case
generation, but its historical ROS adapter assumes MarkerArray inputs,
extra TF helpers and locally defined cone colors. Three independent ROS2
packages now adapt the old numerical approaches for different track layouts,
alongside a fourth isolated pretrained deep-learning path preview and a fifth
independent RViz Goal Pose experiment:

| Competition exercise | Legacy original | Current ROS2 package | Preview behavior |
| --- | --- | --- | --- |
| Autocross / Trackdrive | `Centerline/src/planning_centerline_calc_MIX` | `asurt_classical_planning` | Bounded blue/yellow gates, curved centerline |
| Acceleration | `Acceleration_Planning_NEW` | `asurt_acceleration_planning` | Straight heading and residual checks, at most 2 m extrapolation |
| Skidpad | `planning_skidpad/SkidPadPathPlanningClass.py` | `asurt_skidpad_planning` | Two observation-supported circles, experimental four-lap figure eight |
| Deep Learning | `Deeplearning/src/planning_deep_learning` | `asurt_deep_learning_planning` | Pretrained ONNX LSTM, strict timestamp and cone-boundary rejection gates |
| RViz Goal Pose (simulator) | Independent new module | `asurt_goal_pose_planning` | RViz /goal_pose to free-space forward Dubins map path; no Perception dependency |

The old Acceleration planner generates long straight paths even with sparse
or no cone observations. The old Skidpad planner starts from hardcoded circle
centers and includes lap-counter and Ackermann command behavior. For test
isolation these versions preserve their mission-specific geometric principles,
and reject unsupported fallback paths. The original source remains in Legacy
for fidelity comparisons; the complete historical runtime state machines
have not been reproduced.

The isolated `asurt_classical_planning` package tests the reusable classical
idea: match **blue-left and yellow-right** cones, compute gate midpoints,
order gates forward of the vehicle, then interpolate a bounded centerline.
It uses the installed `asurt_msgs/Landmark` enum (BLUE=0, YELLOW=1), not
the divergent historical acceleration-planner enum. It never subscribes
to ground-truth CARLA actors. It never publishes steering, speed, /drive, TF,
/clock or controller commands.

## ROS2 Humble interfaces

| Direction | Topic | Type / frame |
| --- | --- | --- |
| Input | `/map/global_cones` | `asurt_msgs/LandmarkArray`, `map` (SLAM) |
| Input | `/perception_landmarks` | `asurt_msgs/LandmarkArray`, `base_link` (ZED converter) |
| Input | `/odom` | `nav_msgs/Odometry`, `map` (existing CARLA bridge) |
| Output | `/planning/classical/path` | `nav_msgs/Path`, `map` |
| Output | `/planning/classical/markers` | `visualization_msgs/MarkerArray`, `map` |
| Output | `/planning/classical/status` | JSON `std_msgs/String` |
| DL output | `/planning/deep_learning/path` | `nav_msgs/Path`, `base_link` |
| DL output | `/planning/deep_learning/status` | JSON `std_msgs/String` (never control ready) |
| DL output | `/planning/deep_learning/inference_ms` | `std_msgs/Float32` |
| RViz Goal input | `/goal_pose` | `geometry_msgs/PoseStamped`, `map` or `base_link` |
| RViz Goal output | `/planning/goal_pose/path` | `nav_msgs/Path`, `map` |
| RViz Goal status | `/planning/goal_pose/status` | JSON `std_msgs/String`; tracking/arrived/cancelled |

For the classical outputs, substitute `acceleration` or `skidpad` for
`classical` to inspect the other independent algorithm. Each has its own
`Config/<mode>.yaml`, `Launch/run_<mode>_planner.sh` and System Manager
module switch. All three use the same bounded timestamp, frame/QoS and
rewind/teleport input checks. Acceleration accepts mapped or local cone
inputs; Skidpad explicitly uses `input_source: map` because a single short
field-of-view frame cannot establish two circles.

Config: `Config/classical.yaml`. The current three test configurations
use `input_source: map` to expose map association errors when comparing
the path with the exact global cone map shown by Planning RViz.
The shared adapter also supports `input_source: auto` (SLAM preferentially,
with time-aligned local Perception fallback) or `perception` for an explicitly
separate experiment, but neither is selected by the present presets.
Map landmarks remain odometry-anchored and experimental. Every status reports
`control_ready: false` and the input source used.

No path is emitted without fresh, geometry-consistent observations on both
boundaries. Expired inputs, time rewinds, large odometry jumps and blocked
geometry clear any previously rendered path. All three experiments remain
visualization and numerical behavior tests; they do not prove collision
avoidance, localization accuracy, track completion, or suitability for Control.
The Deep Learning module uses timestamped TF to `base_link`, reads current
Perception landmarks by default, and checks all 15 pretrained model waypoints
against paired cone boundaries. Its default ONNX backend is strict CUDA:
if the GTX 1050 CUDA provider is unavailable it reports `model_unavailable`.
The isolated CPU inference and ROS tests are diagnostics, and path rejection
is an expected safe result. The separate Jetson Xavier TensorRT engine is
preserved. See `Modules/DeepLearning/README.md` for its complete contracts,
validation limits and deployment details.
CARLA bridge `/odom` uses BEST_EFFORT QoS; the planner's odometry subscriber
uses the compatible sensor-data profile. Local cone coordinates are transformed
with nearby timestamped odometry samples, not a guessed static transform.

The read-only live sample on 22 November 2026 found Perception publishing
11 local cones (6 blue, 5 yellow), while the SLAM cone-map switch was OFF.
The bounded curved-corridor geometry accepted three paired gates and produced
a 17-point static centerline preview. This is a numerical candidate from
stereo detections, not a moving or collision-clearance acceptance test.
Use `Tests/inspect_live_inputs.py` with the same sourced ROS overlays to
repeat this check without starting a planner or changing system switches.

## Build and test without affecting the running simulator

From the ASURT root:

```bash
source /opt/ros/humble/setup.bash
source Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
source Simulation/Systems/Perception/Workspace/autonomy_integration/install/setup.bash
cd Simulation/Systems/PathPlanning/Workspace/planning_integration
colcon build --symlink-install --packages-select asurt_classical_planning asurt_acceleration_planning asurt_skidpad_planning asurt_deep_learning_planning asurt_goal_pose_planning

PYTHONPATH=../../Modules/Classical/asurt_classical_planning \
  python3 -m unittest discover -s ../../Modules/Classical/asurt_classical_planning/test -v

PYTHONPATH=../../Modules/Classical/asurt_classical_planning:../../Modules/Acceleration/asurt_acceleration_planning \
  python3 -m unittest discover -s ../../Modules/Acceleration/asurt_acceleration_planning/test -v

PYTHONPATH=../../Modules/Classical/asurt_classical_planning:../../Modules/Skidpad/asurt_skidpad_planning \
  python3 -m unittest discover -s ../../Modules/Skidpad/asurt_skidpad_planning/test -v

PYTHONPATH=../../Modules/DeepLearning/asurt_deep_learning_planning \
  python3 -m unittest discover -s ../../Modules/DeepLearning/asurt_deep_learning_planning/test -v
```

The canonical launchers are `Simulation/Systems/PathPlanning/Launch/`
`run_classical_planner.sh`, `run_acceleration_planner.sh`, and
`run_skidpad_planner.sh`, and `run_deep_learning_planner.sh`. In separate isolated ROS domains, with the overlay
sourced, test the real subscribers and publishers via:

```bash
ROS_DOMAIN_ID=193 python3 ../../Tests/test_ros_contract.py
ROS_DOMAIN_ID=194 python3 ../../Tests/test_mission_ros.py --mode acceleration
ROS_DOMAIN_ID=195 python3 ../../Tests/test_mission_ros.py --mode skidpad
# After adding the existing ONNX Runtime dependency to PYTHONPATH:
ROS_DOMAIN_ID=213 python3 ../../Tests/test_dl_ros_contract.py
```

Use the five independent, initially OFF PathPlanning switches in CARLA
System Manager. In its Tools area, select the **Planning RViz view**
(Overview/Classical/Acceleration/Skidpad/Deep Learning/Goal Pose), then open Planning RViz; each
dedicated view has map/vehicle toggles and a separate per-mode path/diagnostic
checkbox group. `tools/RViz/launch_planning.sh <mode>` supports the same
choices. The overall viewer displays four distinct colored path topics
when available. Keep one process per mode, and
retain the live vehicle under its existing control authority. All modes
publish `control_ready: false`; there is no /drive, /cmd, speed, steering,
TF or /clock publisher. Real moving-vehicle, clearance and full-course
acceptance remains pending. Consult the refreshed ASURT knowledge graph
for the latest interface and scientific validation constraints.

## Experimental simulator Control

The separate ROS 2 Pure Pursuit package in
`Simulation/Systems/Control/Modules/PurePursuit/asurt_pure_pursuit_control`
now defaults to `/planning/goal_pose/path` from the independent RViz Goal
Pose planner. The controller uses existing `/odom` and issues Ackermann
commands to `/drive` only in ROS control mode. The LSTM raw prediction
is still available with an explicit path_topic override; neither planner
publishes `/drive`. System Manager exposes initially OFF `planning_goal_pose`
and dependent `control_pure_pursuit`, without needing Perception or SLAM.
The dedicated Control RViz button displays the goal, orange Dubins route,
actual driven blue trajectory, steering target and live speed/steering text.
See `Simulation/Systems/Control/README.md` for enablement and testing.
