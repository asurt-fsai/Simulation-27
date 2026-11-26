# Control

## Autonomous planner following

1. Start CARLA, import a cone course, and position the FSAI car at the start.
2. Enable the inputs for your planner. The geometric planners use the global
   cone map. For an early test you can select **FakeMAP** or **FakeSLAM**.
   Deep Learning uses the ZED camera and ZED Legacy Converter.
3. Enable your **PathPlanning** module. In **Module Settings → Autonomous
   Planner**, select its name and click **Save**. Turn on either **Pure Pursuit
   Autonomous** or **Stanley High-Speed Autonomous**. The controller follows
   the selected planner's live path without a goal click. If the driving
   window shows manual mode, press **B** to enter ROS driving mode.

Both modes default off. Pure Pursuit retains the 0.8 m/s default target.
Stanley retains the 100 km/h target cap and reduces requested speed for corners,
tracking error and the usable path length. The simulator and available path
geometry determine the speed the car actually reaches. Tune `target_speed_mps`
on `/asurt_pure_pursuit_auto` or `/asurt_stanley_auto` in the current ROS domain.

System Manager shows the current path or the missing prerequisite. The
selection applies live to both autonomous controllers and persists across
restarts. Choose **Automatic (one live planner)** to let the controller pick
the sole live source. Automatic waits if several planners are publishing.
With a named planner selected, the controller waits for that planner to
publish. An explicit ROS `planner_topic` parameter overrides the saved choice.
Registered team planners appear in the picker when they declare their
`nav_msgs/Path` output as their primary topic. The two goal-click planners are
separate modes.

Disable Control or switch back to manual driving to stop autonomous commands.
The node sends zero on stale inputs and on graceful shutdown. The existing
bridge can retain the last drive command if the process is forcibly killed.
See the [ROS topic reference](../../Tools/RViz/ROS_TOPICS.md) for publishers,
subscribers and diagnostics.

---

**New independent high-speed SLAM track experiment:** see
[HIGH_SPEED_TRACK_TEST.md](HIGH_SPEED_TRACK_TEST.md) for the Stanley controller,
cone-aware manually selected goal planner, module switches, RViz viewer,
100 km/h target speed configuration and isolated test results.

---

## Final manual-goal track test: SLAM + Stanley (independent high-speed mode)

The original **Pure Pursuit (Goal Pose)** remains installed, unchanged, and
selectable in System Manager as a low-speed reference. The track test uses a
separate controller: **Stanley High-Speed Track** in
`Modules/Stanley/asurt_stanley_control`. The separate
`PathPlanning/Modules/TrackGoal/asurt_track_goal_planning` path planner reads
the genuine accumulated cone map (`/map/global_cones`, from SLAM), not CARLA
ground truth. It constructs blue/yellow track-boundary gates, searches
bounded alternate lateral lanes around mapped cones, checks clearance and
curvature, and publishes `/planning/track_goal/path`. The planner accepts
manual RViz `2D Goal Pose` clicks only: no autonomous destination selection.

**System Manager: enable these in dependency order**

1. **Perception / ZED 2i** and **ZED Legacy Converter** to supply landmark
   observations (`/perception_landmarks`).
2. **SLAM / Cone Landmark Mapper** (`slam_cone_map`); let it build a
   confirmed map with blue/yellow boundaries on the visible section.
3. **PathPlanning / SLAM Track Goal (Manual)** (`planning_track_goal`).
4. **Control / Stanley High-Speed Track** (`control_stanley`).

System Manager's declared dependency chain automatically handles the
converter and mapper, but both upstream input publishers must be active.
**Leave Control / Pure Pursuit (Goal Pose) OFF** during Stanley tests: both
controllers use `/drive`. Stanley detects another publisher and will decline
to issue competing commands, rather than arbitration by message timing.
The original **RViz Goal Pose (Dubins)** and Deep Learning planners can stay
OFF. No duplicate CARLA hero, sensors, tickers, odometry or TF are created.

Click **Tools / Track Control RViz** in Control Center (not the old Control
RViz). Its fixed-map view shows the one-meter grids, optional Perception
cone cylinders, accumulated SLAM cones, orange cone-aware trajectory, blue
driven trail, green Stanley projected target, and speed / steering / cross-
track-error status. Select **2D Goal Pose**, click *within the mapped track*
ahead of the car, drag to align the arrival direction with the track, and
observe `/planning/track_goal/status` and `/control/stanley/status`.
The planner requires a validated, fresh and sufficiently observed map; an
unmapped goal or blocked corridor produces an empty path and zero command.
Use the existing Pygame `B` toggle to enter ROS /drive mode for movement,
and **Cancel Goal** in Control Center to cancel the clicked track goal.

Stanley uses front-axle cross-track plus heading error and feedforward path
curvature. Its requested speed cap is **100 km/h** (`target_speed_mps:
27.7778`), matching the existing bridge command cap; this is *not* a
measured vehicle top speed or a promise to drive that fast on the mapped
track. A curvature-based lateral acceleration limit and an upcoming-corner
and goal braking envelope reduce requested speed where geometry demands it.
The CARLA hero remains subject to its configured 16-degree front steering.
To compare intermediate speed limits, use the Stanley node's dynamic ROS2
`target_speed_mps` parameter (e.g., 5.0, 10.0 or 27.7778); the original
Pure Pursuit parameters are not modified. The actual car speed, latency,
track width and localization quality must be measured during the live test.

Independent reproducible validation, without moving the live CARLA vehicle:
`ROS_DOMAIN_ID=217` with
`Simulation/Systems/Control/Tests/test_track_stanley_ros_contract.py`.
The contract checks a manual click, orange cone detour, >2.5 m/s command,
fully blocked track, stale map rejection, second /drive publisher detection,
and cancellation. Geometry tests are in the new Stanley and TrackGoal
packages. A mapped cone corridor does not represent arbitrary unknown
obstacles; this is a **track-cone-aware** planner, not a general occupancy-map
navigation or proven competition-ready autonomy stack.

---

Native ROS 2 Humble Pure Pursuit simulator Control is implemented in
Modules/PurePursuit/asurt_pure_pursuit_control. The geometry adapts the
legacy FS-System-26 simple and adaptive Pure Pursuit implementations,
without their historical duplicate TF/odometry/vehicle simulators.

The node reads a Path and bridge /odom, uses adaptive lookahead, publishes
AckermannDriveStamped at /drive (only while the bridge reports ROS mode), and
reports /control/pure_pursuit/status, /control/pure_pursuit/command_preview and
/control/pure_pursuit/target. No new CARLA actors, tickers or TF publishers.

System Manager module: control_pure_pursuit, Control group, OFF by default.
For the standalone goal-click test its dependency is planning_goal_pose,
which subscribes to RViz's /goal_pose (PoseStamped) and publishes
/planning/goal_pose/path in map coordinates. The independent controller now
uses that goal path by default. Both modules require the existing native
CARLA /clock and bridge /odom, but neither uses Perception, SLAM, or LSTM.

To test the previous model integration instead, explicitly override
path_topic:=/planning/deep_learning/raw_path and start its corresponding
planner; it continues to preserve the original camera observation timestamp.
Control rejects input older than 1.25 seconds, independently of RViz preview.

Config/pure_pursuit.yaml supports a different path_topic, for example
/planning/classical/path or /planning/deep_learning/raw_path. Frame handling:
map-path world coordinates are transformed using current /odom; base_link
paths are first anchored at the closest timestamped /odom sample and then
transformed to the vehicle's CURRENT frame. No arbitrary TF is broadcast.

The bridge already implements the correct ROS-positive-left to CARLA-positive-
right steering sign conversion, and native Ackermann speed handling. Its
Pygame B key is the ROS/manual switch. The controller watches
/sim/feedback/autonomous_mode and publishes zero speed when a previously
running reference goes stale or is cleared. It publishes a stop command on
shutdown. The bridge itself currently latches the last Ackermann command
and has no independent source-loss watchdog, which still needs testing or
a bridge-side timeout for abrupt process failure.

Build:
  source /opt/ros/humble/setup.bash
  source Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
  colcon --log-base Simulation/Systems/Control/Workspace/control_integration/log build --symlink-install --base-paths Simulation/Systems/Control/Workspace/control_integration/src --packages-select asurt_pure_pursuit_control --build-base Simulation/Systems/Control/Workspace/control_integration/build --install-base Simulation/Systems/Control/Workspace/control_integration/install

For observation only, launch with override:
  Simulation/Systems/Control/Launch/run_pure_pursuit_control.sh -p output_enabled:=false

No automated CARLA movement was initiated by installing this package.

## RViz goal-click Control experiment

The Control Center has **Control RViz** and **Cancel Goal** buttons. Control
RViz is a dedicated goal test bench with the standard **2D Goal Pose** toolbar
tool (publishes /goal_pose), a fixed map frame, the car, a yellow/orange
Dubins plan, a blue actual driven trajectory, an orange destination arrow, a
green Pure Pursuit lookahead target, and a vehicle-relative text marker with
current speed, commanded speed, steering, and control mode.

**Required System Manager modules (exactly two):**
  PathPlanning > RViz Goal Pose (Dubins) (planning_goal_pose)
  Control > Pure Pursuit (Goal Pose)   (control_pure_pursuit)

Turn both ON in that order (or let the manager resolve dependencies).
Keep ZED, YOLO, SLAM, Classical and Deep Learning OFF for this isolated test.
Only the existing CARLA server + ROS bridge must already be operating.

1. In Control Center, click **Control RViz**. Select **2D Goal Pose** in its
   toolbar, left-click a nearby point on an empty road, and drag to orient the
   arrival heading. RViz publishes /goal_pose; the planner locks any
   base_link-frame goal into map coordinates at click time.
2. Inspect the orange route and fixed destination. Before driving, check
   /planning/goal_pose/status for "tracking" and
   /control/pure_pursuit/status for "tracking" and fresh /odom. A click behind
   the car may generate a turn-around arc because this is forward-only Dubins.
3. Press B in the existing Pygame bridge if needed to select ROS /drive.
   The target speed is 0.8 m/s; monitor the actual blue trajectory beside the
   orange path and the green lookahead point. Steering sign conversion is
   already owned by the native CARLA bridge.
4. **Cancel Goal** in Control Center sends a single Bool cancellation event
   to the existing goal planner, which publishes an empty Path. The controller
   then commands zero speed. Alternatively choose MANUAL with B, or disable
   Control in System Manager. The planner also stops publishing a route upon
   arrival near the requested goal position AND heading.

No cone avoidance, map collision tests, reverse gear, parking, Nav2 or other
automation is part of this free-space click-to-go test. The demonstration
is intended for short, visually clear goals and no duplicate car ownership.

## Previous native CARLA LSTM driving experiment (optional)

1. Keep the existing CARLA hero and bridge running; do not start a second one.
   The existing Pygame bridge window stays open for control-mode selection.
2. From System Manager, enable ZED Perception and its zed_converter, then
   PathPlanning > Deep Learning. For LSTM Control, temporarily override
   the controller path_topic to /planning/deep_learning/raw_path and run
   its node independently of the Goal Pose module.
   SLAM is not required for this Deep Learning configuration.
3. Open Tools > Planning RViz view > Deep Learning. The cyan line shows
   all original 15 LSTM waypoints, the normal blue/yellow ZED cylinders
   show input detections, and the small green point is the Pure Pursuit
   lookahead target. The global SLAM cone layer remains disabled.
4. In Pygame, press B if needed to choose ROS /drive instead of keyboard
   control. Commands have REP-103 steering (+left) and are converted once
   by the existing bridge; default requested speed is 0.8 m/s.
5. Read /control/pure_pursuit/status for tracking, stale_path,
   missing_source_time_odometry or waiting_for_path. Observe command output
   on /control/pure_pursuit/command_preview before enabling Control when
   desired, or use output_enabled:=false on the command-line launcher.
6. Switch Pygame back to MANUAL or disable the Control module to end the
   simulator experiment. The node sends a zero Ackermann command on a normal
   shutdown. Abrupt process failure remains a bridge watchdog improvement.

The RViz prediction holds for eight seconds; the controller does not use
anything older than 1.25 seconds. It is possible to see a cyan path while
Control reports stale_path or waiting_for_path: that is intentional, not
a missing trajectory. If CUDA frequently misses the age threshold, measure
actual inference latency and optionally compare a CPU backend in a separate
experiment. Do not change LSTM weights to tune steering; tune the independent
Control configuration instead.

Validation performed with isolated ROS_DOMAIN_ID=215 (never live CARLA):
Ackermann positive-left steering, limited speed, empty-path stop, stale
observation rejection, manual-mode arbitration, normal SIGINT shutdown stop.
Ten independent controller geometry tests and System Manager/RViz regression
tests passed. The original LSTM CPU isolated ROS contract also passed after
source timestamp preservation. Native moving-course driving performance is
not yet measured.
