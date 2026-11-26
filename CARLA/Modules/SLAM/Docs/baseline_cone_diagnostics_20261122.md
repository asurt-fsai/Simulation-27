# ASURT camera cone-map baseline: live inspection — 22 November 2026

## What belongs to the old baseline

The historical source is
`Simulation/Systems/Legacy/fs-system-26-oldstack/SLAM_Camera/src/cone_mapping`,
particularly `config/integrated_default.rviz` and
`cone_mapping/cone_mapping_node.py`. Its RViz preset shows the global cone
landmarks (`/map/global_cones_markers`), vehicle pose/odometry, and grid.
It accidentally includes two subscriptions to the same global marker topic;
the new default RViz uses one. Original `/zed/zed_node/pose` and
`/zed/zed_node/odom` are not the authoritative modern vehicle frames:
the existing CARLA bridge owns `map -> base_link`, which drives the
blue FSAI preview. This is a passive display, not a new localization estimate.

The historical `Perception_Camera/.../loop_closure_node.py` considers the
vehicle to have returned to the start when travelled distance >= 10 m
and its odometry pose is within a 1.5 m start radius. That is **a
start/finish proximity flag, not globally optimized landmark loop closure**.
It is not presented as proven SLAM on the modern baseline.

The default `tools/RViz/config/slam_mapping.rviz` loads only the grid,
vehicle preview, and `/map/global_cones_markers`. The previous rich view
is preserved at `tools/RViz/config/slam_diagnostics.rviz`.
System Manager was then set to the old camera-mapping baseline:
`slam_cone_map` and `slam_trajectory` ON (the trajectory process also
supplies the legacy /slam/distance input), and optional `slam_camera`,
`slam_lidar`, `slam_lidar_odom`, `slam_fusion` OFF. The snapshot below
was captured **before** disabling those extra processors; it is not
silently presented as a post-disable benchmark. The live mapper was not
restarted or cleared, so pre-existing map errors remain visible.

## Live, read-only evidence on FlatAsphalt

The active course actor receipt
`logs/native-linux/flatasphalt-YlNSyMy7/course-actors.json` names
`Assets/Maps/CSV/test1_3m_clear.csv` and records **100 placed cone
actors**. The CSV contains **50 blue and 50 yellow cones**; its closest
same-colour pair is **2.627 m** apart. The older importer receipt
`Simulation/CARLA/Bridge/Carla_ASU_Bridge/scripts/.cone_track_actors.json`
is from *Silverstone*, not this FlatAsphalt course. The current active
FlatAsphalt map should not be confused with that receipt.

At the first check the camera mapper held 112 confirmed landmarks. During
a later five-and-a-half-second passive diagnostic its reported map count
was **141 confirmed**, split into **68 blue and 73 yellow**. The saved
ROS snapshot is `logs/native-linux/slam_baseline_cone_diagnostic.json`.
The same 141 landmark positions have been exported for independent
inspection as `logs/native-linux/slam_baseline_cones_snapshot_20261122.csv`;
this file is a **raw mapper snapshot, not a ground-truth track or
validated loop-closed map**.
It contains eight same-colour landmark pairs closer than **1 m**,
even though the CSV track has **no same-colour pairs closer than 2 m**.
That is compelling evidence of *false landmarks and/or multiple
landmarks for the same physical cone*, not merely an RViz rendering problem.
The exact identity and cause of each mismatch need image/time-aligned
validation; a cone count is not localization accuracy.

In the same 5.5 s measurement, the existing ZED stereo node published
18 object messages with **0–9 objects per frame (median 3.5)**. The
legacy conversion node produced 18 `/perception_landmarks` messages with
**identical 0–9 counts, median 3.5** and the same last timestamp. It
did not cause count loss during that sample. Velodyne processing published
48 `/lidar/cones` messages with **1–4 detections per frame**.
The camera mapper status was `state=mapping`, `rejected_frames=0`,
`confirmed=141`, `total=142`. It confirms only repeated detections,
not ground truth or independent loop closure.

These measurements **do not** show that all 100 cones should be visible
in one image. Camera field of view, occlusion, stereo matching, detection
confidence, object tracking (minimum two hits), and limited range can
all reduce the count of *current* detections. To explain one particular
missing cone, inspect whether it was detected in the left image, matched
with the right image, emitted as a ZED object, passed to the converter,
and accepted by the global mapper. The 5.5 s sample establishes only the
observed stage counts; it does not yet assign specific missing cone IDs
to a particular stage.

## Existing constraints and recommended improvement order

1. Preserve the baseline RViz and source mapper as a measurable control,
   instead of hiding errors with marker filtering or lowering confirmation
   thresholds until the image-level evidence supports it.
2. Collect an aligned, read-only sample of actual ZED left/right images,
   YOLO candidate boxes, stereo matches, converter landmarks and map
   associations around the *same physical missing cone*.
3. Quantify landmark duplication versus reference cone positions offline,
   and separately measure stereo long-range errors, colour switches,
   odometry/time-alignment drift, gating, and confirmed-lifecycle errors.
4. Only then propose bounded perception/SLAM numerical changes with
   regression tests. Compare each revision with this preserved baseline
   on the same track and course, **without production actor-truth inputs**.

This diagnostic neither changed the 100-cone course nor commanded,
restarted, spawned or moved any CARLA actors or sensors.
