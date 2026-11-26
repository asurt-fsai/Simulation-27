# ASURT SLAM — Shared-Sensor LiDAR SLAM, Camera Mapping and Fusion

This ROS2 Humble package ports the probabilistic landmark-mapping approach
in Simulation/Systems/Legacy/fs-system-26-oldstack/SLAM_Camera/src/cone_mapping.
The original implementation depended on pyzed, its own Landmark messages,
an absent ZED pose topic and additional TF/converter publishers. The port
avoids those duplicate or unavailable components.

## Inputs (existing shared Perception / CARLA bridge)

- /perception_landmarks (asurt_msgs/LandmarkArray; ZED converter, base_link)
- /odom (nav_msgs/Odometry; the one FSAI telemetry bridge, map frame)
- Existing /tf and /tf_static for coordinate transforms

The perception-based SLAM nodes never connect to CARLA, spawn sensors or heroes, tick the
world, publish /clock or send /drive commands.

## Independent System Manager modules

**Vehicle Trajectory (slam_trajectory)** consumes /odom and publishes
/slam/path (nav_msgs/Path), /slam/pose (geometry_msgs/PoseStamped) and
/slam/distance (std_msgs/Float32). It remains usable with Perception off.

**Cone Landmark Mapper (slam_cone_map)** ports Kalman landmark state,
Mahalanobis-distance gating, Hungarian global assignment, confirmed/tentative
landmark lifecycle, and bounded CPU/memory use. It consumes existing
landmarks, TF and odometry, publishing /map/global_cones (asurt_msgs/LandmarkArray),
/map/global_cones_markers (MarkerArray), and /slam/cone_map_status (String).
It requires the existing zed_converter; when Perception is off the mapper
is blocked, without automatically enabling cameras.

These original Phase 1 modules are individually switchable. The new camera,
LiDAR, alignment and fusion modules appear in the same SLAM group with
independent controls and persistent intent. They do not duplicate camera,
LiDAR or world ticker processes.

## Historical LeGO-LOAM LiDAR SLAM

The historical team's ROS2 LeGO-LOAM and cloud_msgs were recovered from
asurt-fsai/fs-system-26, branch Lego-Loam-cone-mapping, commit
909b8f5626df2a8961d6c7a9a5dc6e3753dbf425. The source retains its
cone mapping and loop-closure extensions and builds on ROS2 Humble with
GTSAM unpacked into SLAM/ThirdParty.

The off-by-default slam_lidar module consumes the existing /velodyne_points
VLP-16 sweeps (16 lines, -15 to 15 degree vertical FOV, 10 Hz) and publishes
isolated /slam/lidar/* topics using private slam_lidar_* frames. Use the
canonical Launch/run_lidar_slam.sh; the original historical launch starts
conflicting TF statics and extra RViz and must not be used. Bridge time
rewinds and large pose discontinuities terminate the LOAM process, allowing
a fresh graph when the module is restarted.

LeGO's historic camera_init coordinates and quaternion conventions have
not yet been verified as a REP-103 normalized odometry frame. Its optional
RViz point cloud and odometry displays are initially disabled until proper
map alignment is established. This output is not yet suitable for Control.

## Camera landmark pose estimation

The optional slam_camera module consumes the existing /perception_landmarks
base_link detections and a time-aligned bridge /odom motion prior. It confirms
colour-consistent cone landmarks across distinct viewpoints, retains a
bounded reference map and computes a robust SE(2) correction only when
at least three non-collinear previously confirmed cones constrain vehicle
translation and heading. Rejected, sparse and stale observations do not
produce a corrected pose; a stationary-only history cannot establish a
confirmed landmark reference map. Outputs include /slam/camera/map,
/slam/camera/map_markers, /slam/camera/pose, /slam/camera/path and
/slam/camera/status. This corrects pose relative to an odometry-anchored
initial map; it cannot recover an independent absolute map origin.

## LeGO odometry coordinate normalization

The original LeGO map pose and quaternion components use historical
permutations. The optional slam_lidar_odom module reads the historic
/slam/lidar/mapped_adjusted and existing /odom reference and fits a proper
rigid planar transformation only after sufficient synchronized motion and
turning excitation. Held-out trajectory fit and yaw consistency must pass
before /slam/lidar/normalized_odom publishes odometry with
lidar_odom -> base_link frames. The /slam/lidar/normalization_status
reports missing excitation, failed calibration and health. Until these
requirements pass, the adapter publishes no normalized pose. It does
not publish TF or command the vehicle.

## Shared camera–LiDAR fusion

The independent off-by-default slam_fusion module consumes the existing
/perception_landmarks, /lidar/cones, and /odom topics and optionally verified
normalized LiDAR odometry. Matched two-sensor local cones appear on
/slam/fusion/landmarks (base_link) and /slam/fusion/markers (base_link).
Confirmed repeated cross-sensor cones also publish on
/slam/fusion/global_cones (map) and /slam/fusion/global_cones_markers
(map), using corrected pose when accepted and bridge odometry otherwise.
This bounded bridge-anchored map is experimental; historical LeGO loop
closures are not yet re-optimized across camera and LiDAR landmarks.
The /slam/fusion/status JSON topic reports freshness, gating and resets.
The /slam/fusion/pose map-frame estimate is only emitted when validated
LiDAR odometry and multiple stable cross-sensor map references support
a bounded pose correction.

Raw historical LeGO odometry does not meet the normalized input contract:
its legacy axes and quaternions require independent calibration/testing.
Thus fused local cones work independently, while global fused pose remains
gated until verified normalization. See Modules/Fusion/asurt_slam_fusion.

## SLAM RViz

Open from CARLA System Manager > Tools > SLAM RViz or from the desktop shortcut
ASURT SLAM RViz. The launcher is tools/RViz/launch_slam.sh and its preset
is tools/RViz/config/slam_mapping.rviz. The default view mirrors the
historical ASURT camera-mapper RViz: confirmed global cones in `map`,
the vehicle pose (now represented by a blue FSAI outline), and a grid.
It does not mix tentative detections, camera corrected poses, LiDAR scans
or fusion overlays into the baseline. The old experimental multi-layer
display is retained in tools/RViz/config/slam_diagnostics.rviz.

The viewer does not override any user-disabled module switch.
The dedicated diagnostic preset is available for fused local cone matches,
gated corrected poses and isolated LeGO outputs, but the baseline intentionally
contains only the earlier camera cone-mapping functions.

The SLAM view camera tracks `base_link` using the existing `map -> base_link`
TF. Its RobotModel loads `Visualization/fsai_vehicle_preview.urdf`, a
single-frame FSAI outline with measured chassis and wheel dimensions,
so the vehicle remains visible without spawning any additional CARLA
actors or TF publishers. The local grid follows the vehicle across both
maps; global landmark and trajectory topics continue to use `map`.
RViz visualizes robot pose and sensor maps, not CARLA's rendered road scene.
Both Perception presets also use the same passive blue FSAI outline.
See Docs/baseline_cone_diagnostics_20261122.md for a stage-by-stage
read-only investigation of missing and duplicated cone landmarks.

## Silverstone and FlatAsphalt

Both maps use the same ROS message topics, map frame and numerical algorithms.
Map history is reset on timestamp rewind, substantial pose discontinuity
or a new SLAM process. No hardcoded track origin or map geometry is used.
The FlatAsphalt launcher uses its own spawn profile and map; the Silverstone
launcher uses the corresponding Silverstone world. Switch runtime using
the existing quit/launch flow, never an extra CARLA server or ticker.

Offline numerical and isolated ROS-domain integration tests cover both
coordinate profiles, including a map switch/pose reset. This does **not**
substitute for live cone detection and moving-vehicle acceptance on both maps.

## Rebuild

From Simulation/Systems/SLAM/Workspace/slam_integration, source:

1. /opt/ros/humble/setup.bash
2. Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
3. Simulation/Systems/Perception/Workspace/autonomy_integration/install/setup.bash

The historical LeGO-LOAM requires GTSAM. The repository-local binary package
is located in `SLAM/ThirdParty/gtsam-prefix/opt/ros/humble`. Set
`CMAKE_PREFIX_PATH` to include that directory and
`LD_LIBRARY_PATH` to include its `lib/x86_64-linux-gnu` directory.
After sourcing the ROS underlays, build the complete SLAM overlay:

```bash
colcon build --symlink-install --packages-select \
  asurt_cone_slam cloud_msgs lego_loam_sr \
  asurt_camera_slam asurt_lidar_odom_adapter asurt_slam_fusion \
  --executor sequential
```

Run pure test suite:
PYTHONPATH=Simulation/Systems/SLAM/Modules/ConeMapping/asurt_cone_slam
python3 -m unittest discover -s
Simulation/Systems/SLAM/Modules/ConeMapping/asurt_cone_slam/test -v

After also sourcing SLAM/Workspace/slam_integration/install/setup.bash,
run the isolated ROS integration:
ROS_DOMAIN_ID=79 python3 Simulation/Systems/SLAM/Tests/test_ros_pipeline.py

The ROS test uses synthetic messages on a separate ROS domain and never
commands the live CARLA simulator.

The LeGO C++ executable has two additional isolated ROS tests in Tests:
test_lego_projection.py publishes an empty scan followed by a synthetic
VLP-16 cloud on ROS domain 187 and verifies the projected cloud topic and
velodyne frame. test_lego_reset.py publishes a timestamp rewind to verify
the process discards its old graph. Both copy the executable temporarily
within Tests so the live System Manager can continue enforcing the
disabled-by-default SLAM switch. These tests validate startup,
projection, and reset only; they do not validate moving SLAM accuracy
or real map loop closure.

## Acceptance limits

The original Phase 1 `/slam/pose` still reproduces bridge odometry. The
separate historical LeGO-LOAM now compiles and ingests VLP-16 sweeps in
isolated ROS tests, but has not established accuracy under real movement.
The new camera pose correction uses repeated cone geometry to correct drift
against an odometry-anchored reference map. The fusion estimator combines
camera and LiDAR cone measurements and corrects a LiDAR-derived pose only
when independently normalized odometry passes strict validation.
Its new global paired-cone output remains odometry-anchored. The system
does **not** yet implement a jointly optimized LiDAR–camera factor graph,
coherent global loop-closure updates or proven absolute localization.
The historical LeGO-LOAM and cone_a_lisa came from a separate Git branch;
the team's earlier launches did not contain that cross-sensor optimizer.
Do not feed these SLAM estimates to Control or Planning until validated
alignment, real moving-vehicle localization, drift correction, loop-closure
behavior and measured performance on both Silverstone and FlatAsphalt
are accepted.

CPU bounds: at most 64 detections processed per frame, 1200 stored landmarks,
2 Hz map output and at most 2400 trajectory poses. Confirmed cones require
repeated observations to reject single-frame false detections.

## FakeMAP and FakeSLAM for CARLA testing

Import a CSV or JSON course in **Track Importer** and enable a provider under
**System Manager → SLAM**.

- **FakeMAP** publishes the entire imported layout at its original placements.
  This is the provider previously called Fake SLAM. Its saved switch and local build carry over.
- **FakeSLAM** discovers imported cones within 4.5 m horizontally in front of the car.
  The map keeps each cone at its first observed position after you pass it, and revisiting
  it leaves that landmark unchanged. Camera and LiDAR modules can be off.

Both providers publish `/map/global_cones` and `/map/global_cones_markers` in the `map` frame.
Turning one on turns the other off and pauses the cone landmark mapper. Turning the fake provider
off restores the mapper's saved preference. Both simulated providers default off and are enabled
individually. **Clear** in Track Importer removes cones from the published map and RViz.

Set horizontal position noise for each provider under **Module Settings → SLAM Settings**.
Enter a standard deviation in metres per axis, such as `0.05` for 5 cm. Updates apply live.
The simulated position error is stable for each cone within an episode. `0` gives exact positions.
