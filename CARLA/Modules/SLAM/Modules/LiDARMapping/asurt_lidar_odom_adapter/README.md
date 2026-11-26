# ASURT LiDAR odometry normalization adapter (optional, calibration-gated)

This independent ROS 2 Humble `ament_python` package is an **OFF-by-default**
passive consumer of historical LeGO-LOAM adjusted odometry. It neither starts
LeGO/CARLA nor publishes TF, sensors, `/clock`, vehicle commands, or a SLAM map.
Do not automatically start it from the LiDARMapping launch stack.

## Interfaces and prime registry contract

| Direction | Topic | Message | Contract |
|---|---|---|---|
| Input | `/slam/lidar/mapped_adjusted` | `nav_msgs/msg/Odometry` | Historical LeGO, header `slam_lidar_adjusted_map`, child `slam_lidar_adjusted`. Raw positions/quaternion are *not* accepted as REP-103 merely because they carry "adjusted" frames. |
| Input | `/odom` | `nav_msgs/msg/Odometry` | Existing bridge map → base_link. **Required** for timestamp pairing, finite offline-style trajectory calibration and continuous veto checks. No bridge poses are forwarded as output. |
| Output | `/slam/lidar/normalized_odom` | `nav_msgs/msg/Odometry` | **Silent until validated**. Header `lidar_odom`, child `base_link`, right-handed planar XY/yaw; local LiDAR-origin XY with axes aligned to the measured bridge ENU orientation. Pose and derived twist remain LiDAR-derived; no TF broadcaster. |
| Output | `/slam/lidar/normalization_status` | `std_msgs/msg/String` | JSON state, validation flag, sample count, residuals, scale ratio, yaw consistency, covariance visibility, resets, rejected samples and output count; emitted periodically even when odometry is suppressed. |

Executable: `asurt_lidar_odom_adapter lidar_odom_adapter_node`.
Launch: `ros2 launch asurt_lidar_odom_adapter adapter.launch.py`.
Config: `config/adapter.yaml` (`use_sim_time: true`). **System Manager
registration is an action for prime**, not this package; register disabled by
default, and start only when original LeGO and bridge are already running.
The optional Fusion `config/fusion.yaml` has been pointed at
`/slam/lidar/normalized_odom` (Fusion still rejects raw LeGO frame labels).

## Scientific constraints

Historical source evidence in
`../LeGO-LOAM/src/mapOptmization.cpp`:

- Lines 957–958: legacy quaternion built with
  `setRPY(transformAftMapped[0], [2], [1])`.
- Lines 1005–1012: the *adjusted* output reorders translation as
  `([5], [3], [4])` and negates selected quaternion components.
- The project's `config/asurt_vlp16.yaml` explicitly labels the adjusted
  frames `slam_lidar_adjusted_map` / `slam_lidar_adjusted` and topic
  `/slam/lidar/mapped_adjusted`.

None of that alone proves the position, quaternion, sensor origin or handedness
are REP-103 compatible. Rather than guessing a sign change or fixed quaternion
permutation, this adapter tests the actual XY position sequence and full
quaternion-derived yaw against real paired bridge motion. **No scale,
reflection or affine skew is fitted.** A proper SE(2) least-squares transform
(`det R=+1`) must pass all of these:

- At least 40 monotonic timestamp-matched pairs (bounded ring, at most 96).
  Bridge messages are interpolated only when bracketing samples are within
  70 ms of the LeGO stamp; an exact sample can be used within 12 ms. Reject
  stale ROS-clock timestamps older than 350 ms when simulation time is active.
- The first 65% of the pairs fit SE(2) without scale; the final 35% is held
  out. Calibration requires at least 5 m traversed, 0.48 rad turning and
  measurable lateral spread (not a straight line), plus fresh translating
  and turning motion in the held-out segment.
- Train RMS planar error at most 0.20 m, held-out error at most 0.27 m,
  scale-ratio discrepancy at most 8%, and no reflected trajectory outperforming
  a proper rotation. The *same* fitted rotation must also explain raw yaw
  orientation: RMS heading disagreement at most 0.12 rad and max error at
  most 0.25 rad. Negative handedness/reversed-yaw scenarios fail closed.
- Calibrated output is `R @ (p_lidar - p_lidar_first)` for XY and
  `wrap(yaw_lidar + theta_R)` for yaw. The bridge never supplies a replacement
  pose, origin or instantaneous yaw after calibration. Odom twist is derived
  from consecutive normalized LiDAR poses in the `base_link` child frame;
  legacy LeGO twist is deliberately ignored.
- On every subsequent paired sample, mismatch over 1 m or 0.33 rad against
  the bridge **revokes** calibration. Time rewind, time gaps, teleport,
  source discontinuity and frame changes reset it; no output is allowed again
  until a fresh trajectory has independently passed all gates.

ROS `pose.covariance` is checked for finite, positive-semidefinite XY
uncertainty, upper bounds (1 m² XY and 0.25 rad² yaw), and conservative
rotation of XY cross-covariance through its maximal eigenvalue. Historical
LeGO's pose covariance is usually all zeros (not a measured zero-variance
claim); in that case the adapter uses deliberately nonzero engineering floors
(at least 0.025 m² per planar axis and 0.012 rad² yaw) together with the
calibration residuals. Vertical/roll/pitch uncertainty is large (100).
`normalization_status` explicitly identifies when raw covariance was
unreported. **Those engineering floors are not calibrated probabilistic
confidence intervals**, and drift may exceed them between veto checks.

## Limitations: not yet field accepted

Synthetic tests prove that the guards work for known proper/mirrored/scaled
and misoriented trajectories. They do **not** establish that any actual
Silverstone or FlatAsphalt recording contains the geometric excitation and
calibration consistency required. The adapter may correctly produce *only
diagnostics* in practice. The historical adjusted odometry may represent a
LiDAR origin rather than the vehicle's `base_link`; no mounting lever arm
is assumed or inferred. If angular motion exposes a significant lever arm,
the rigid trajectory residual guard will refuse to normalize it. A proper
validated sensor-to-base rigid extrinsic would require separate, measured
evidence and a new scoped change. Sensor-independent absolute localization,
global accuracy, observability beyond tested planar excitation, and loop
closure are **not** claimed. Never feed this unaccepted pose to control.

If status remains `insufficient_translation_or_turn`,
`degenerate_almost_collinear_path`, `quaternion_yaw_inconsistent`,
`scale_mismatch`, `reflection_detected`, or residual failures, do **not**
force frame labels or loosen thresholds as a workaround. Capture independent
straight, left-turn and right-turn trajectories with original timestamps and
evaluate the actual source geometry offline.

## Isolated build and tests (do not start a simulator)

```bash
cd /home/hassan/ASURT/Simulation/Systems/SLAM/Modules/LiDARMapping/asurt_lidar_odom_adapter
source /opt/ros/humble/setup.bash
source /home/hassan/ASURT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
colcon build --base-paths . --packages-select asurt_lidar_odom_adapter --symlink-install
PYTHONPATH=.:$PYTHONPATH ROS_DOMAIN_ID=189 python3 -m unittest discover -s test -v
ROS_DOMAIN_ID=189 colcon test --packages-select asurt_lidar_odom_adapter --event-handlers console_direct+
```

These tests are isolated synthetic NumPy/ROS callback tests. They do not
launch LeGO, create world entities, broadcast TF or move the vehicle.
