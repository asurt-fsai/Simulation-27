# ASURT CARLA ROS 2 topic reference

These are the default topics for the bundled Windows and Ubuntu profiles.
Open **System Manager → Tools → ROS Bags → Record** and select **Refresh live topics**
to see the names, types and publisher counts in the active simulator's ROS domain.
Local builds, parameters and the enabled modules determine what appears. To inspect
the same graph from a shell, use the domain shown by ROS Bags in this command.

```bash
ASURT_ROS_DOMAIN_ID="$domain" ~/.local/share/asurt/current/bin/asurt exec ros2 topic list -t
```

## Follow an autonomous run

| Stage | Follow these topics |
| --- | --- |
| Timing and vehicle pose | The bridge publishes `/clock` (`rosgraph_msgs/Clock`), `/odom` (`nav_msgs/Odometry`) and `/tf` and `/tf_static` (`tf2_msgs/TFMessage`). Sensor timestamps and path frames use this context. |
| Camera and LiDAR | The ZED publishes left and right `image_rect_color` (`sensor_msgs/Image`) and `camera_info` (`sensor_msgs/CameraInfo`) under `/zed/zed_node`. Its Legacy Converter publishes `/perception_landmarks` (`asurt_msgs/LandmarkArray`). Velodyne publishes `/velodyne_points` (`sensor_msgs/PointCloud2`) and LiDAR Cone Detection publishes `/lidar/cones` (`asurt_msgs/LandmarkArray`). |
| Map | The selected map provider publishes `/map/global_cones` (`asurt_msgs/LandmarkArray`), `/map/global_cones_markers` and `/slam/cone_map_status`. The bundled geometric planners use the global cone map. The bundled Deep Learning planner uses the vehicle-relative camera landmarks. |
| Planner | The selected autonomous planner publishes `/planning/<planner>/path` (`nav_msgs/Path`) and `/planning/<planner>/status` (`std_msgs/String`). The bundled planner IDs are `classical`, `acceleration`, `skidpad` and `deep_learning`. Deep Learning also publishes `/planning/deep_learning/raw_path` for comparison with the validated path. |
| Drive control | The selected controller consumes the planner path, `/odom`, `/clock` and `/sim/feedback/autonomous_mode` (`std_msgs/Bool`). Follow `/control/stanley_auto/status` or `/control/pure_pursuit_auto/status` and its corresponding `command_preview`. The controller's `/drive` (`ackermann_msgs/AckermannDriveStamped`) is the command the bridge consumes in ROS driving mode. |
| Vehicle response | Capture `/sim/feedback/speed`, `/sim/feedback/steering_angle`, `/sim/feedback/imu` and `/odom` to compare actual motion against `/drive`. `/sim/feedback/collision` appears when a collision event occurs. |

The **Autonomous run** recording preset selects the active timing, landmark, map,
path, controller and feedback streams. Add the raw camera, LiDAR and calibration
topics if you intend to rerun perception or fusion. Add `/sim/feedback/collision`
if you need to capture impacts during the run.
For a team planner, add its configured path and status topics after checking the
graph. In playback, **All recorded topics** can replay `/drive` on the isolated
domain used by the player and RViz.

## Weather and tyre grip

Weather Editor, MCP `set_weather`, and CARLA API changes apply to an active
FlatAsphalt or Silverstone Split session. Open **System Manager → Weather Status**
or call MCP `weather_status` to read the current weather and wheel grip.
**Start Rain** uses the selected rainfall intensity. Enable **Timer** to choose
when rainfall stops. Light rain builds a water film slowly. Heavy rain wets
the road faster and can collect standing water. When rain
stops, the sky moves back toward the selected cloud cover. Clear Noon clears
the sky and leaves the road to dry. **Clear Rain** stops the rain and dries the
road immediately. Sunlight, higher asphalt temperature, and
wind speed up drying. The estimated rubber temperature follows air and asphalt
temperature and adjusts grip around a generic operating band. At zero wetness
and standing water, the vehicle returns to its temperature-adjusted dry grip.

The tyre solver resolves contact, suspension, slip, and the local physical
material at each wheel. Use `/sim/feedback/tire_forces` while rolling to observe
slip ratio, the effective coefficient, and wheel loads. The manual tyre-friction
service takes priority while its friction trigger is present. The thermal and
wet-grip curves approximate the FSAI tyres pending measured compound data.

Fog and dust reduce contrast with scene distance on both ZED image topics and
in the driving window. Wet
asphalt has darker diffuse shading and more sky reflection at shallow angles.
The adapter uses low-resolution CARLA depth for both effects and temporarily
falls back to uniform haze when depth is unavailable. Rain, fog and dust attenuate
LiDAR intensity, can drop weak returns and can produce sparse near-field returns
in `/velodyne_points`. These optical effects are approximate. The LiDAR topics,
timestamps and `PointCloud2` fields retain their usual format.

## Bridge and vehicle

| Topic | Publisher | Consumers or purpose |
| --- | --- | --- |
| `/clock` | CARLA telemetry bridge | Simulation time for ASURT modules, RViz and bag recording. ZED and Velodyne subscribe directly. |
| `/odom` | CARLA telemetry bridge | `nav_msgs/Odometry`, `map` → `base_link`. Required by the planners, controllers, cone mapper, trajectory, camera SLAM, LiDAR SLAM guard and alignment, fusion, and ZED camera adapter. |
| `/tf` | Bridge and enabled TF broadcasters | Dynamic transforms for RViz and ROS frame consumers. The bridge owns the vehicle pose. |
| `/tf_static` | Bridge/static TF broadcasters and ZED adapter | Sensor mounting transforms for RViz and ROS frame consumers. |
| `/drive` | One selected Control module, or the MCP reference driver in its own run | `ackermann_msgs/AckermannDriveStamped` consumed by the bridge in ROS driving mode. Speed is m/s and positive steering is left. |
| `/sim/feedback/autonomous_mode` | Bridge | ROS/manual mode (`std_msgs/Bool`) required by every drive controller. |
| `/sim/control/brake` | ROS operator or safety tools | `std_msgs/Bool` emergency brake consumed by the bridge. |
| `/sim/control/steering_angle_deg`, `/sim/control/velocity_rpm` | Legacy ROS control clients | Bridge inputs when configured for legacy vehicle-interface control. |
| `/sim/control/drag_coefficient` | Runtime tuning tools | Bridge input for drag coefficient. |
| `/sim/spawn_point`, `/sim/start`, `/sim/stop` | ROS operator tools | Bridge inputs for a spawn pose (`Pose2D`) and simulation start/stop (`Empty`). |
| `/sim/feedback/gps`, `/sim/feedback/gps_vel`, `/sim/feedback/imu` | Bridge | GPS fix, GPS velocity and IMU measurements for telemetry and recording. |
| `/sim/feedback/speed`, `/sim/feedback/steering_angle`, `/sim/feedback/steering_angles` | Bridge | Speed in m/s, steering in degrees and wheel joint angles. |
| `/sim/feedback/battery/state`, `/sim/feedback/motors` | Bridge | Battery and motor telemetry. |
| `/sim/feedback/tire_forces`, `/sim/feedback/vehicle_physics`, `/sim/feedback/vehicle_state` | Bridge | Wheel forces, physics settings and vehicle state for diagnostics. |
| `/sim/feedback/collision` | Bridge | `carla_msgs/CarlaCollisionEvent` consumed by tests and MCP bag scoring. |
| `/sim/ground_truth/boxes` | Bridge | Ground-truth marker boxes for visualization. |
| `/carla_system_manager_node/component_health`, `/ASU_RT_Carla_Telemetry_Node/transition_event` | Bridge lifecycle node | Component health and lifecycle events. |
| `/parameter_events`, `/rosout` | ROS 2 nodes | ROS parameter events and logging. |
| `/clicked_point`, `/initialpose` | RViz point and initial-pose tools | Point inspection and external localization integrations. |

The [bridge configuration](../../Simulator/Bridge/config/carla_interface_config.yaml) also
defines optional camera, LiDAR and feedback topics for sensor setups outside the
default ASURT profiles. Service names, including `/sim/control/force_manual_control`,
are listed separately there.

## Perception and SLAM

Inputs below are in addition to `/clock` for sim-time nodes. A map provider owns
`/map/global_cones`, `/map/global_cones_markers` and `/slam/cone_map_status` at any
one time. FakeMAP and FakeSLAM read the managed cone placements and the CARLA
world through the simulator API.

| Module | Inputs | Outputs and consumers |
| --- | --- | --- |
| ZED 2i Stereo Camera | CARLA stereo camera, `/odom` | `/zed/zed_node/left/image_rect_color`, `/zed/zed_node/right/image_rect_color`, `/zed/zed_node/left/camera_info`, `/zed/zed_node/right/camera_info` for recording and camera tools. `/zed/zed_node/rgb/cones_overlay`, `/zed/zed_node/cone_markers` for RViz. `/zed/zed_node/obj_det/objects` to the ZED Legacy Converter. Static camera TF on `/tf_static`. |
| Velodyne LiDAR | CARLA LiDAR | `/velodyne_points` (`PointCloud2`) to LiDAR Cone Detection and LeGO-LOAM. |
| LiDAR Cone Detection | `/velodyne_points` | `/filtered_points` and `/cluster_markers` for RViz, `/lidar/cones` (`LandmarkArray`) to Camera + LiDAR Fusion. |
| ZED Legacy Converter | `/zed/zed_node/obj_det/objects`, optional `/slam/distance` for orange-gate deduplication | `/perception_landmarks` (`LandmarkArray`, `base_link`) to Cone Landmark Mapper, Camera Landmark SLAM, Camera + LiDAR Fusion and Deep Learning. Map/perception configurations of the three geometric planners can also use it. `/perception_markers`, `/perception/isOrangeCone`, `/diagnostics/comp_time/perception`, `/status/perception_camera_node` for visualization and diagnostics. |
| Vehicle Trajectory (Odom) | `/odom` | `/slam/path`, `/slam/pose` to viewers, `/slam/distance` to the ZED Legacy Converter. |
| Cone Landmark Mapper | `/perception_landmarks`, `/odom` | `/map/global_cones` (`LandmarkArray`, `map`) to Classical, Acceleration, Skidpad and SLAM Track Goal. `/map/global_cones_markers` to RViz. `/slam/cone_map_status` to SLAM Track Goal and diagnostics. |
| FakeMAP | Imported cone placements and actor ownership | Same three map provider topics. |
| FakeSLAM | Imported cone placements, actor ownership and current CARLA hero pose | Same three map provider topics plus `/slam/fake_slam_status`. |
| Camera Landmark SLAM | `/perception_landmarks`, `/odom`, optional external `/slam/camera/map_id` | `/slam/camera/pose`, `/slam/camera/map`, `/slam/camera/map_markers`, `/slam/camera/path`, `/slam/camera/status` for localization diagnostics. |
| LeGO-LOAM LiDAR SLAM | `/velodyne_points`, `/odom` guard | `/slam/lidar/odom`, `/slam/lidar/mapped_adjusted` and the streams listed below. The alignment module consumes `/slam/lidar/mapped_adjusted`. |
| LiDAR Odometry Alignment | `/slam/lidar/mapped_adjusted`, `/odom` | `/slam/lidar/normalized_odom` to fusion, `/slam/lidar/normalization_status` to diagnostics. |
| Camera + LiDAR Fusion | `/perception_landmarks`, `/lidar/cones`, `/odom`, optional `/slam/lidar/normalized_odom` | `/slam/fusion/landmarks`, `/slam/fusion/markers`, `/slam/fusion/global_cones`, `/slam/fusion/global_cones_markers`, `/slam/fusion/pose`, `/slam/fusion/status` for diagnostics. |

LeGO-LOAM's default VLP-32C configuration uses the following pipeline topics.

| Producer | Inputs | Other outputs |
| --- | --- | --- |
| Image projection | `/velodyne_points` | `/slam/lidar/full_cloud_projected`, `/slam/lidar/full_cloud_info`, `/slam/lidar/ground_cloud`, `/slam/lidar/segmented_cloud`, `/slam/lidar/segmented_cloud_pure`, `/slam/lidar/segmented_cloud_info`, `/slam/lidar/outlier_cloud`, `/slam/lidar/cone_cloud`, `/slam/lidar/cone_cloud_marker` |
| Feature association | Segmented cloud, segmented cloud info and outlier cloud from projection | `/slam/lidar/laser_cloud_sharp`, `/slam/lidar/laser_cloud_less_sharp`, `/slam/lidar/laser_cloud_flat`, `/slam/lidar/laser_cloud_less_flat`, `/slam/lidar/laser_cloud_corner_last`, `/slam/lidar/laser_cloud_surf_last`, `/slam/lidar/outlier_cloud_last`, `/slam/lidar/odom_raw` |
| Map optimization | Last corner, surface and outlier clouds, cone cloud and raw odometry | `/slam/lidar/key_poses`, `/slam/lidar/local_map`, `/slam/lidar/mapped_odom`, `/slam/lidar/history_cloud`, `/slam/lidar/corrected_cloud`, `/slam/lidar/recent_cloud`, `/slam/lidar/mapped_adjusted`, `/slam/lidar/loop_closure`, `/slam/lidar/laps`, `/slam/lidar/cone_map`, `/slam/lidar/cone_map_marker`, `/loop_detected_odom` |
| Transform fusion | `/slam/lidar/odom_raw`, `/slam/lidar/mapped_odom` | `/slam/lidar/odom` |

See the [LeGO-LOAM topic configuration](../../Modules/SLAM/Modules/LiDARMapping/LeGO-LOAM/config/asurt_vlp32c.yaml).

## PathPlanning and Control

| Planner | Inputs in bundled profile | Outputs |
| --- | --- | --- |
| Classical Cone Path Planner | `/map/global_cones`, `/odom` | `/planning/classical/path`, `/planning/classical/markers`, `/planning/classical/status` |
| Acceleration Path Planner | `/map/global_cones`, `/odom` | `/planning/acceleration/path`, `/planning/acceleration/markers`, `/planning/acceleration/status` |
| Skidpad Path Planner | `/map/global_cones`, `/odom` | `/planning/skidpad/path`, `/planning/skidpad/markers`, `/planning/skidpad/status` |
| Deep Learning Cone Planner | `/perception_landmarks`, `/odom` | `/planning/deep_learning/path` contains the validated portion of the prediction through the last observed paired gate, stamped at the cone observation. `/planning/deep_learning/raw_path` is the full RViz prediction. `/planning/deep_learning/status` reports clipping and readiness. `/planning/deep_learning/inference_ms` reports model latency. |
| RViz Goal Pose (Dubins) | `/goal_pose` from RViz, `/odom`, `/planning/goal_pose/cancel` | `/planning/goal_pose/path`, `/planning/goal_pose/status`, `/planning/goal_pose/goal_marker` |
| SLAM Track Goal (Manual) | `/goal_pose` from RViz, `/map/global_cones`, `/slam/cone_map_status`, `/odom`, `/planning/track_goal/cancel` | `/planning/track_goal/path`, `/planning/track_goal/status`, `/planning/track_goal/goal_marker` |

System Manager's **Cancel Goal** publishes the two planner-specific cancel
topics. Autonomous controllers use the first four planner path topics or the
primary `nav_msgs/Path` topic of a registered team PathPlanning module. Choose
the source in **Module Settings → Autonomous Planner**. Both goal-based
planners remain available for the manual-control workflows.

| Controller | Inputs in addition to `/odom` and `/sim/feedback/autonomous_mode` | Outputs in addition to `/drive` |
| --- | --- | --- |
| Pure Pursuit (Goal Pose) | `/planning/goal_pose/path` | `/control/pure_pursuit/status`, `/control/pure_pursuit/command_preview`, `/control/pure_pursuit/target`, `/control/pure_pursuit/driven_path`, `/control/pure_pursuit/diagnostic_marker` |
| Stanley High-Speed Track | `/planning/track_goal/path`, `/planning/track_goal/status` | `/control/stanley/status`, `/control/stanley/command_preview`, `/control/stanley/target`, `/control/stanley/driven_path`, `/control/stanley/diagnostic_marker` |
| Pure Pursuit Autonomous | One active autonomous planner path | `/control/pure_pursuit_auto/status`, `/control/pure_pursuit_auto/command_preview` |
| Stanley High-Speed Autonomous | One active autonomous planner path | `/control/stanley_auto/status`, `/control/stanley_auto/command_preview` |

Path topics use `nav_msgs/Path`. Use `map` for fixed world points or `base_link`
with the observation timestamp for vehicle-relative points. The controllers
require fresh simulation time and odometry. A blank or stale path stops the
controller, and the status topic names the missing prerequisite. Status topics
use JSON in `std_msgs/String`. Command previews use the Ackermann message and
are available for diagnostics and recording.
