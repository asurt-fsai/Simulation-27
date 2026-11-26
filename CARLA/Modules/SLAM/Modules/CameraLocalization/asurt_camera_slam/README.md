# ASURT Camera Landmark Localization

Independent ROS2 Humble module for repeated ZED YOLO26 cone landmark
localization relative to a bounded odometry-initialized map. It is off by
default until the prime integrates it in the System Manager.

The package is **asurt_camera_slam**, its console executable is
**camera_slam_node**, and its standalone launch file is
**camera_slam.launch.py**. It creates no actors, converters, sensors,
world ticker, TF broadcaster, control publisher, or simulator connection.

## Inputs and outputs

| Direction | Topic | ROS type | Frame / semantics |
| --- | --- | --- | --- |
| Input | /perception_landmarks | asurt_msgs/msg/LandmarkArray | base_link; metric XYZ, known color classes 0–3, confidence 0–1, ROS timestamp |
| Input | /odom | nav_msgs/msg/Odometry | map → base_link; only initial map prior and incremental motion |
| Optional input | /slam/camera/map_id | std_msgs/msg/String | changing nonempty map identity resets everything |
| Output | /slam/camera/pose | geometry_msgs/msg/PoseWithCovarianceStamped | map; emitted ONLY when camera-geometry constraints succeed |
| Output | /slam/camera/map | asurt_msgs/msg/LandmarkArray | map; only confirmed, bounded, class-colored anchors |
| Output | /slam/camera/map_markers | visualization_msgs/msg/MarkerArray | map; stable class-colored SPHERE cones, DELETEALL on reset and redraw |
| Output | /slam/camera/path | nav_msgs/msg/Path | map; at most 300 real camera-corrected poses, cleared on reset |
| Output | /slam/camera/status | std_msgs/msg/String (JSON) | health, matches, inliers, RMS residual, covariance, reset reason, corrected-frame count |

Detection timestamps must match an already received odometry sample within
0.18 seconds. Invalid source frames, nonfinite coordinates, confidence below
0.45, bad quaternions and unknown cone classes are rejected. The ROS pose
covariance is a complete planar x,y,yaw 3×3 covariance embedded into a 6×6
matrix; unobservable z, roll and pitch use very large variances.

## Algorithm

1. Bootstrap a local camera cone map from synchronized metric detections
   transformed using the initial /odom map pose. The reference inherits
   odometry's global coordinate error.
2. Associate observations one-to-one to class-compatible stored cones, using
   confidence/range-dependent uncertainty, a hard Euclidean gate and
   Mahalanobis chi-square gate (Hungarian global assignment).
3. A cone becomes a *confirmed reference* only after at least four distinct
   frame timestamps, three viewpoints and a minimum 0.45 m viewpoint
   baseline. Stationary duplicate observations cannot confirm a cone.
4. Freeze confirmed world positions to avoid a map chasing drift. Keep at
   most 400 anchors for at most 240 seconds. Read at most 48 cones per frame.
5. Predict motion by integrating incremental SE2 /odom changes from the last
   estimate. Once three or more **previously confirmed noncollinear** cones
   match, solve a robust weighted Gauss–Newton planar pose adjustment with
   a motion prior. Check geometry (SVD), Huber residuals, second-pass outlier
   rejection, translation/yaw innovation, and RMS reprojection error.
6. Publish a pose only when the above independent camera-geometric update
   succeeds. Its covariance includes measurement range/anchor uncertainty
   and prior information; uncertainty floors prevent false precision from
   temporally correlated stereo detections.

Measured stereo depth beyond roughly 7 m was considerably less reliable
than near range; observation noise deliberately grows sharply beyond 7 m.
The robust optimizer is capped at two passes of nine iterations. Its
one-to-one matching matrix is at most 48 × 400. The path retains no more
than 300 corrected poses.

## Resets and map changes

The map and corrected path reset when a camera or odometry stamp rewinds,
an odometry jump indicates a teleport, the odometry frames change, or an
optional map identity String changes between Silverstone and FlatAsphalt.
The existing bridge and System Manager should publish the actual new map
identity when changing world. No algorithm can identify a silent world change
that retains identical poses, transforms, monotonic time, and map identity.
On resets /slam/camera/map_markers publishes DELETEALL and the path clears.
No stale estimate is republished as a camera pose when visual inputs vanish.

## Standalone build and tests

From the folder containing this README:

    source /opt/ros/humble/setup.bash
    source /home/hassan/ASURT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
    colcon --log-base log build --base-paths . --build-base build --install-base install --symlink-install --packages-select asurt_camera_slam
    source install/setup.bash
    PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q test/
    colcon --log-base log test --python-testing pytest --build-base build --install-base install --packages-select asurt_camera_slam
    colcon --log-base log test-result --test-result-base build --verbose

ROS adapter tests call synthetic callbacks with captured publishers and never
run a CARLA/ROS sensor producer. Numerical tests include corrected drift,
observability failure with two or collinear cones, low confidence, repeatable
FlatAsphalt-like origins versus large negative Silverstone-like coordinates,
covariance positive definiteness, bounded map state, stale-history resets
and anchor freezing.

## Scientific and operational limitations

The initial reference map is odometry-anchored, so this corrects **relative
drift against repeated visual cone observations**, not independent absolute
global position. It does not solve a global loop closure graph, guarantee
unique association on repeating same-color tracks, provide stereo depth,
or infer a true map transformation after a world change. Covariance is a
conservative engineering estimate until calibration with withheld *offline*
truth and live Silverstone/FlatAsphalt moving tests. Output is not authorized
for Control or Planning until independent acceptance.
