# High-Speed SLAM Track Test — Manually Selected RViz Destination

The new high-speed experiment is independent of the existing
Pure Pursuit (Goal Pose) system. The legacy FS-System-26 repository contains
Simple and Adaptive Pure Pursuit, not a preexisting Stanley/MPC controller.
The new native Stanley controller is a different front-axle
cross-track/heading tracker. The original Pure Pursuit packages and
their 0.8 m/s default are preserved for low-speed comparison.

## Processing path

    RViz 2D Goal Pose → /goal_pose (click and drag heading)
    ZED 2i → Perception Converter → /perception_landmarks
    SLAM Cone Landmark Mapper → /map/global_cones + /slam/cone_map_status
    SLAM Track Goal Planner → /planning/track_goal/path (map)
    Stanley High-Speed Track → /drive (Ackermann via existing bridge)

## Modules to enable

In CARLA System Manager, turn on these modules in order:

1. **Perception → ZED 2i Stereo Camera** (zed2i)
2. **Perception → ZED Legacy Converter** (zed_converter)
3. **SLAM → Cone Landmark Mapper** (slam_cone_map)
4. **PathPlanning → SLAM Track Goal (Manual)** (planning_track_goal)
5. **Control → Stanley High-Speed Track** (control_stanley)

The System Manager can resolve the declared module dependencies; confirm
that each upstream module is actually running and the mapper has accumulated
confirmed cones. The ZED modules may already be ON.

**Disable Control → Pure Pursuit (Goal Pose)** (control_pure_pursuit).
Both controllers can advertise the same /drive publisher interface. Stanley
detects a competing publisher and refuses to issue commands. Do not turn on
all Control modules at once. The independent RViz Goal Pose (Dubins) planner,
Classical Planner, and Deep Learning Planner are NOT needed for this test.

The new nodes are initially OFF and do not start another CARLA server,
vehicle, camera, odometry publisher, /clock publisher or world ticker.

## RViz workflow

1. Open **Tools → Track Control RViz** in Control Center. This is a distinct
   viewer from the existing **Control RViz** low-speed reference.
2. The view uses the CARLA simulation clock and global map frame. The 1 m
   grid and mapped SLAM cone markers are enabled.
3. Select **2D Goal Pose** in the RViz toolbar. Click a destination
   **inside the blue/yellow mapped corridor** and drag its arrow in the
   driving direction. This is manual goal selection; no automatic mission
   manager chooses destinations.
4. Watch the orange planned route, blue driven trail, Stanley projection
   marker and live requested/actual speed with steering diagnostics.
   The live Perception ZED cylinder layer is optional and OFF by default.
5. Inspect the Goal Planner status for state=tracking and valid_route=true.
   If it says missing map, no connected gates, goal outside corridor,
   stale mapping or blocked corridor, there is deliberately no driving path.
6. Once the mapped route appears, use the existing Pygame **B** key to
   select ROS /drive. Compare the desired route to the real CARLA car trail.
7. **Cancel Goal** in Control Center now requests cancellation from both
   goal-planning demonstrations. Returning Pygame to MANUAL or disabling
   Stanley also ends the test.

The map must already contain the relevant part of the track. The current
cone mapper only confirms landmarks observed by ZED: it cannot promise a
path through unknown parts of a course.

## What obstacle-aware means here

The planner uses confirmed blue and yellow mapped cones to define the track
boundary gates. It searches connected gate segments and chooses a limited
center/left/right lateral route where mapped cones obstruct the centerline.
It validates map freshness, observed mapper progress, orientation, cone
clearance and maximum path curvature.

It outputs an empty path when the corridor is incomplete or blocked. Orange
cones are included in clearance tests. This is **cone-aware route selection**,
not a full occupancy map: other objects, hidden obstacles, unmapped cones,
localization drift and vehicle swept-body dynamics have not been solved.
The current map is odometry-aided cone mapping, not a validated loop-closing
global SLAM solution.

## High-speed controller limits

Configuration file:
Simulation/Systems/Control/Config/stanley_track.yaml

| Parameter | Default | Interpretation |
| --- | --- | --- |
| target_speed_mps | 27.7778 | 100 km/h maximum requested speed |
| max_steer_rad | 0.27925 | Front wheels: 16 degrees |
| max_lateral_accel_mps2 | 4.0 | Lower speed for bends |
| max_accel_mps2 | 4.0 | Requested acceleration ramp |
| max_brake_mps2 | 7.0 | Future corner and destination braking constraint |
| cross_track_limit_m | 2.0 | Do not command travel far from valid path |
| update_period_s | 0.05 | Nominal 20 Hz control loop |

These are controller requests, not actual measured vehicle capabilities.
The original CARLA bridge has a configured 100 km/h speed cap, 16-degree
front steering limit and native Ackermann conversion. It is unchanged.

To reach 100 km/h from rest and then decelerate to zero at the defaults,
you need roughly 150 m of verified straight track (about 96 m for
acceleration and about 55 m for braking). The planner supports clicked
goals up to 180 m and 600 sampled points, but only inside the SLAM-mapped
track corridor. A freshly started mapper may know only nearby cones,
so manually building the map first is usually necessary before any full
speed attempt.

For incremental comparisons, the running Stanley node supports changing
the maximum speed without rebuilding. From a terminal with the ROS overlay
sourced:

    ros2 param set /asurt_stanley_control target_speed_mps 8.3333333    # 30 km/h
    ros2 param set /asurt_stanley_control target_speed_mps 16.6666667   # 60 km/h
    ros2 param set /asurt_stanley_control target_speed_mps 27.7777778   # 100 km/h

## ROS diagnosis

    ros2 topic echo /planning/track_goal/status --once
    ros2 topic echo /control/stanley/status --once

The planner status explains its gate/corridor and map-validity decisions.
Stanley status provides the actual versus requested km/h, steering degrees,
cross-track error, path age and command authority.

The independent end-to-end ROS contract runs only in isolated
ROS_DOMAIN_ID=217. It verifies a mapped blue/yellow track, orange-cone detour,
fully blocked corridor producing an empty route and zero command,
stale SLAM map stopping Control, duplicate /drive publisher arbitration and
manual goal cancellation. Pure mathematical and RViz/System Manager tests
are also included.

These tests do not move the live CARLA hero. Track performance, maximum
achievable speed and steering/tire response require a subsequent live
simulator run.
