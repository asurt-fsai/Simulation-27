# ASURT CARLA: live modular ZED 2i stereo simulation

## Current YOLO26 vision backend (2026-11-21)

The active ZED configuration now selects `vision_backend: yolo26`, with the
FSOCO YOLO26n end-to-end ONNX model from
`Simulation/Systems/Perception/Models/yolo26n_fsoco_e2e_opset17.onnx`.
It has fixed `1x3x640x640` RGB input and `1x300x6` end-to-end output
(`xyxy, confidence, class`). Classes are yellow, blue, orange, large_orange,
unknown. The ROS legacy contract maps large_orange to orange and ignores
unknown; both mappings are explicit. The model's embedded license metadata
identifies AGPL-3.0, relevant before redistributing its weights.

One model inference is performed on the **left eye**, with bounded HSV and
template/epipolar matching of **real right-eye pixels** for stereo range. It
uses no cone actor/CSV truth. The existing calibration, tracking, image,
ObjectsStamped and MarkerArray message types, topics, RViz preset, and
CARLA System Manager module `zed2i` are preserved. The name
`enable_legacy_detection` remains as a compatible detection-enable gate;
the actual algorithm is controlled by `vision_backend`.

The ONNX CPU provider is installed in the project-local
`Simulation/Systems/Perception/Dependencies/onnxruntime/` directory,
using `Dependencies/onnxruntime-requirements.txt`. Only `run_camera.sh`
exposes this path in its PYTHONPATH. To reinstall without replacing ROS's
system NumPy/OpenCV, run from ASURT:

    /usr/bin/python3 -m pip install --no-deps \
      --target Simulation/Systems/Perception/Dependencies/onnxruntime \
      -r Simulation/Systems/Perception/Dependencies/onnxruntime-requirements.txt

Inference runs in one bounded asynchronous worker so raw stereo camera and
ROS clock callbacks can continue while ONNX works. On a busy GTX 1050 host,
observed detection times were approximately 125-173 ms and camera stereo
pairs were roughly 3 Hz after the switch. This is not an assertion of 10 Hz
inference. Conservative CPU sampling favored two inference threads; three
threads created CPU contention. No movement/cone accuracy acceptance has
been completed yet: verify real detections and depth with cones in view.

For an immediate rollback: change `vision_backend: "hsv"` in the ZED YAML
and use CARLA System Manager to switch the ZED 2i module Off and then On.
Its dependent ZED converter is stopped before the camera and automatically
restored when the camera publishes again; the simulator and FSAI hero remain.
Only one camera-node publisher/actor owner should ever be running.

## Purpose and accepted environment

This is a self-contained ROS 2 Humble workspace alongside the existing
Carla_ASU_Bridge. Native CARLA remains on 127.0.0.1:2000 with one existing
vehicle.vehicle.asurt_fsai hero and one existing synchronous-world tick owner.
The camera node is **only an observer**. It does not publish /drive,
set weather, reload the map, move the car, spawn cones, or modify physics.

The ZED 2i body is simulated as a left and right RGB camera rigidly attached
to the same hero: two optical centers of a single stereo assembly.
Initial simulated baseline is 0.120 m, image size 640x360 per eye, wide
horizontal FOV assumption 110 degrees, and target output 10 stereo pairs/s.
The current camera configuration uses the Blender assembly origin
(x=0.45618349 m, y=+0.02084175 m left, z=0.73042095 m), with the two
simulated lenses placed 0.060 m to each side of this midpoint. Earlier
provisional camera positions (x=0.67866, z=0.95 m) were based on image
visibility and do not correspond to the measured source-model origin.
The precise real ZED 2i optical-center offset inside its housing remains to
be calibrated and the two CARLA eyes are not hardware synchronized.
ROS y points left; CARLA y points right. Hardware lens choice, exact position
and per-unit intrinsics remain calibration inputs, not known facts.

## Source-model sensor extrinsics and URDF convention — 2026-11-20

The model evidence is in
`fsai_simulation/fsai_description/urdf/fsaiurdf_manifest.json` and the
generated `fsaiurdf.urdf`. The manifest lists the **original Blender world**
LiDAR object origin at (0.37553552, +0.01003163, 0.69655180) metres and
the camera object origin at (0.45618349, +0.02084175, 0.73042095). Both
use +X forward, +Y left, +Z up. For CARLA sensor actor attachment, their
local Y is negated. The source LiDAR and camera rotations are zero.

The URDF exporter shifts its `base_link` by the original Blender world
coordinate (0.32656395, 0.02191870, 0) m, so its `lidar_joint` and
`camera_joint` offsets appear smaller. The CARLA FSAI hero's actor frame
tracks the original model coordinates, and `/odom`/bridge TF tracks the
CARLA actor. Do **not** apply the shifted URDF joint offsets directly to
CARLA attachments or to the static `base_link` TF of these observers.

The LiDAR configuration in `Adapters/CARLA/LiDAR/asurt_velodyne_sim/config/vlp32c.yaml`
and the stereo configuration in `src/asurt_zed2i_sim/config/zed2i.yaml`
hold these measured assembly origins. Their beam/emission centers may differ
slightly from the mesh object origins; the mesh is not a calibrated sensor
housing. Check the attached actors and ROS `/tf_static` **after restarting**
the observers because changing YAML does not change actors already alive.

## Stable interfaces: old system today, new system later

Producer: asurt_zed2i_sim/zed2i_sim_node

  /zed/zed_node/left/image_rect_color : sensor_msgs/Image (bgr8)
  /zed/zed_node/right/image_rect_color : sensor_msgs/Image (bgr8)
  /zed/zed_node/left/camera_info : sensor_msgs/CameraInfo
  /zed/zed_node/right/camera_info : sensor_msgs/CameraInfo
  /zed/zed_node/rgb/cones_overlay : sensor_msgs/Image (bounding boxes)
  /zed/zed_node/cone_markers : visualization_msgs/MarkerArray
  /zed/zed_node/obj_det/objects : zed_msgs/ObjectsStamped

Unmodified original ASURT consumer: perception_zed_pkg/conversion_node

  /perception_landmarks : asurt_msgs/LandmarkArray
  /perception_markers : visualization_msgs/MarkerArray
  /perception/isOrangeCone : std_msgs/Bool (when its original logic triggers)

Official Stereolabs zed_msgs definitions are vendored at
vendor/zed-ros2-interfaces (initially commit
8f86e35268881258cd3df98ab67afd2da5478e4d).
Original perception_zed_pkg is built unchanged from
/home/hassan/ASURT/fs-system-26-oldstack/Perception_Camera/src/perception_zed_pkg
and uses the existing tf_helper/asurt_msgs build.
Later detectors can publish the same ObjectsStamped contract; only one
producer must be active at a time.

Camera image, calibration, and object headers share each paired CARLA frame
timestamp mapped to the ROS clock. Object positions and detection markers
are in base_link (vehicle-relative metres: x forward, y left, z up).
The existing CARLA bridge publishes the dynamic map-to-base_link TF. This
node only adds static camera/optical TF links: no second map odometry source.
The fixed field-of-view is 110 degrees initially, to be replaced by actual
ZED 2i factory intrinsics for hardware calibration.

## Launch and inspect

Camera and original converter were started during acceptance. **Do not
start a duplicate**: check the running processes first.

  pgrep -af 'zed2i_sim_node|conversion_node'

If stopped, launch from separate terminals:

  cd /home/hassan/ASURT/autonomy_integration
  bash ./run_camera.sh

In a different terminal:

  cd /home/hassan/ASURT/autonomy_integration
  bash ./run_old_converter.sh

Stop each with Ctrl+C. On a normal Ctrl+C, the camera removes only its
two named CARLA sensor actors. If force-killed, inspect any leftover
asurt_zed2i_sim_left/right sensor actors before restarting; the process
refuses to create duplicates instead of deleting unknown actors.
Never restart CARLA or the ROS bridge to stop this observer.

To open RViz 2, in a fresh terminal:

  source /opt/ros/humble/setup.bash
  source /home/hassan/ASURT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
  source /home/hassan/ASURT/autonomy_integration/install/setup.bash
  rviz2

RViz configuration: Global Options -> Fixed Frame = base_link; add Image
display with /zed/zed_node/rgb/cones_overlay (Reliability = Best Effort);
add a MarkerArray display for /zed/zed_node/cone_markers. This stream draws
approximately physical-size cones. The original /perception_markers stream
draws 1-metre spheres and made three duplicate blue detections look like a
clump; its topic remains available but is not enabled in the saved layout.
The saved RViz orbit aims 6.5 m ahead of the hero rather than at its origin.
Optionally add raw
left/right Image displays or a TF display. Moving the hero manually moves
the two attached cameras. Because object positions are base_link-relative,
the cones' apparent positions change when the car moves; newly visible
cones appear and absent cones disappear (DELETEALL each frame). To see a
global view rather than car-relative view, choose Fixed Frame = map and
verify the map/base_link TF. Avoid a second global odometry publisher.

A ready-to-use RViz layout is included at rviz/zed2i_live.rviz. To open it
with the stereo images, old ASURT markers and TF preselected, run:

  rviz2 -d /home/hassan/ASURT/autonomy_integration/rviz/zed2i_live.rviz

To inspect message and QoS contracts:

  ros2 topic list -t | grep zed/zed_node
  ros2 topic info -v /zed/zed_node/obj_det/objects
  ros2 topic info -v /perception_landmarks

## Weather and optical fidelity

CARLA already provides rendered sunlight, rain, cloud cover and fog.
The camera process reads current CARLA weather approximately every 1.5 s.
It optionally layers independent lens water droplets/blur, sun-aligned glare,
low-contrast haze and image noise. CARLA camera blueprint also requests
postprocessing, bloom, and lens flare. All these are adjustable independently
through the camera config at:

  src/asurt_zed2i_sim/config/zed2i.yaml

The source of detected cones is the stereo image pair only. It uses HSV
color segmentation with simple epipolar left/right correspondence, disparity
triangulation and no access to source track CSV or CARLA cone positions.
The old converter receives these real pixel-derived detections. For
serious all-weather research, replace the HSV detector with a calibrated
model and evaluate observed precision/recall, 3D depth error, occlusions,
false positives, optical saturation and bad-weather effects against recorded
real ZED 2i data.

**Known fidelity limits:** This is a rectified pinhole stereo approximation,
not actual ZED 2i SDK output. It does not model per-unit lens distortion,
rolling shutter, hardware HDR/automatic exposure, ZED stereo neural depth,
dynamic lens water drainage, realistic ISP noise and exposure control, or
real detector domain shift with measured fidelity. The real stereo lens and
mount calibration are required before claiming hardware realism. Do not
substitute CSV ground truth into the detector: CSV is only offline scoring.

**Safety:** No autonomy driving is connected. Do not send commands to /drive
until command watchdog, emergency braking, manual override and tilt/roll
protection are separately verified. The current 100 production cone actors,
their ownership JSON and the hero remain outside this workspace.

## Build, test and performance

Built packages: official zed_msgs, asurt_zed2i_sim and the original
perception_zed_pkg, all installed in this workspace. To rebuild only this
sensor package:

  cd /home/hassan/ASURT/autonomy_integration
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  colcon build --base-paths src/asurt_zed2i_sim --packages-select asurt_zed2i_sim --symlink-install

Tests:

  /usr/bin/python3 -m unittest discover -s src/asurt_zed2i_sim/tests -v

This uses system Python (/usr/bin/python3) for ROS 2 Humble, OpenCV and
NumPy 1.x, appending the separate CARLA venv site-packages only to import
the matching CARLA API. Prepending that venv globally would load NumPy 2.x
and break the installed OpenCV binary.

## Cone detection and RViz correction — 2026-11-19

The initial HSV baseline missed sun-bleached yellow cones and treated the
separate blue bands on one nearby cone as three independent objects.
An actual ROS camera snapshot in evidence/baseline_left.png demonstrated
three near-identical blue positions at approximately 3.2–3.4 m and only
one yellow detection.

The isolated detection module now uses a lower-saturation yellow threshold,
vertical color-mask closing, overlapping-box suppression, and a stereo
correspondence check constrained by the approximate 0.325 m physical cone
height. For distant cones, apparent-height ranging is blended with stereo
disparity because 640-pixel imagery gives poor disparity precision. This
is an experimental approximation and MUST be validated against measured
camera/ground truth before using it for autonomous control.

In the **historical November 19 visual experiment**, the camera mount was
raised from 0.665524 to 0.95 m above the local vehicle origin to reduce nose
obstruction (see evidence/revised_left.png). That was a provisional position.
The November 20 Blender/URDF geometry check subsequently replaced it with
the measured assembly-origin coordinates in the sensor-extrinsics section.
The car geometry was not modified.

The existing old converter still publishes /perception_landmarks with types
blue=0 and yellow=1. The RViz preset now subscribes to the simulator's
/zed/zed_node/cone_markers instead of the converter's 1 m sphere markers,
and looks 6.5 m forward for a usable view of the corridor. This changes
visualization only; no downstream legacy message interface was altered.

During the live retest, six detections had distinct vehicle-relative locations:

  blue:   (3.30, +1.73), (6.57, +2.26), (12.02, +3.68) metres
  yellow: (3.64, -1.73), (7.32, -1.14), (10.72, -0.39) metres

Six local unit tests passed, including regression for low-saturation yellow
and striped blue. The ROS2 old converter received the same six landmarks.
The RViz screenshot is stored at evidence/rviz_revised.png. This is an
acceptance of static camera detection and correct topic delivery; a moving
vehicle/SLAM acceptance and depth-error measurement remain outstanding.

Before the ROS image publishing fix, the sensor delivered roughly 1–2 image
pairs/s. The Python ROS Image setter iterated over every byte of a 640x360
raw image. It now assigns a C-backed unsigned-byte array, cutting publish
latency by an order of magnitude; the live node's log showed approximately
10 stereo pairs/s with 50–65 ms total image-stage latency. Tune camera
size and update rate based on CARLA render load (GTX 1050 4 GB).

## Stereo tracking and weather performance — 2026-11-20

The camera remains a **passive observer** of the existing hero; no additional
world ticker, camera ground-truth oracle, vehicle control publisher, or second
TF/odometry authority was introduced. Sensor capture uses
`sensor_rate_hz: 10.0` and ROS publication targets 10 Hz. Left/right
images and CameraInfo are serialized only for active subscribers; the
original `zed_msgs/ObjectsStamped` and ASURT converter topics remain intact.

Temporal tracking in `tracking.py` confirms observations twice, uses
colour/spatial associations, compensates ego-motion when /odom is available,
assigns stable marker IDs, and explicitly deletes absent markers. Missed
observations are not republished as current detections. **Moving-vehicle
ghost-cone suppression is not yet accepted**: timestamped odometry-to-camera
alignment, TF conventions, stereo misassociation, and occlusion behavior
still need a live manual-drive test.

In the November 20 performance pass, `weather.py` stopped regenerating
the same sun-glare mesh and exponential on each left/right frame. It now
caches the glare field, invalidating on camera heading, sun position,
cloudiness, FOV, or image-size changes. It also reuses the OpenCV float32
Gaussian-noise allocation and uses in-place image arithmetic for glare,
fog, and rain blending. Glare, droplets, rain, fog, and independent
left/right noise are **still enabled**, not bypassed for performance.

Controlled offline microbenchmark: system Python 3, OpenCV, saved
`evidence/revised_left.png`, resized to 640x360, 18 trials per condition
(discarding the first three), median wall-clock times:

| Weather processing per eye | Before | After |
| --- | ---: | ---: |
| Sun facing camera, cloudiness 25 | 21.24 ms | 9.93 ms |
| Rain 90, fog 50, cloudiness 85 | 33.58 ms | 18.18 ms |

For the same deterministic frame/seed, the previous and updated pixel
outputs differed by at most one 8-bit intensity count (mean difference
rounded to 0.0000). Distinct weather and camera poses still update the
cached glare; noise remains independent by eye and reproducible. The
baseline weather module is preserved for comparison at
`evidence/weather_before_perf_20261120.py`.

All **14** ROS-independent tests passed after this optimization, including
new glare-cache invalidation and noise-resolution regressions:

    cd /home/hassan/ASURT/autonomy_integration
    /usr/bin/python3 -m unittest discover -s src/asurt_zed2i_sim/tests -v

These are **offline image-processing measurements**, not CARLA or RViz FPS
claims. At the measurement time CARLA port 2000 was not listening, while
two abandoned observer/converter processes still existed; those process
groups were stopped without touching the simulator, map, hero, cone assets,
or cone ownership JSON. Do not launch duplicate observer/converter
processes after restarting CARLA.

Previous live perception was roughly 6.3–6.9 Hz against a 10 Hz target;
the latest optimized code has **not yet been retested against live CARLA**.
Before acceptance, profile the same stationary scene with CARLA+bridge+
Pygame alone, then stereo+converter, then marker-only RViz, then the
annotated image. Collect world/server/client FPS, paired camera rate,
processing latency, ROS rates, CPU/GPU/VRAM and dropped frames. Test manual
driving separately, confirming identity persistence, sane relative
coordinates and absence of ghost cones. Keep the 100-cone restored course
and original FSAI hero untouched unless an inspected, fresh CARLA world
has lost them; inspect ownership and actor IDs before any recovery action.

At 640x360 and 110° horizontal FOV, small distant cone contours and
subpixel stereo disparity remain possible range limits, **not proven ZED
2i hardware limitations**. Investigate candidate, stereo-match, tracking
and RViz rejection stage separately before changing sensor resolution.
Real ZED optical-center/intrinsic calibration and adverse-weather validation
remain outstanding. The simulated mount now matches the source FSAI housing.

## Live stereo message-stall diagnosis and correction — 2026-11-20

The original observer grouped camera frames by **identical CARLA world-frame
number** and discarded unmatched frames more than five ticks old. These are
separate CARLA RGB actors with independently phased `sensor_tick=0.1`
capture schedules, not two outputs of a hardware-triggered ZED 2i. Even
though each eye was streaming 7.2–9.0 frames/s, simultaneous frame numbers
were rare. Four-second callback diagnostics measured only 0–0.3 exact
pairs/s while dropping roughly **56–69 unpaired frames per interval**.
This explains the multi-second gaps in ZED objects and RViz markers.

**Fix:** `stereo_sync.py` now matches left/right exposures by **CARLA
simulation timestamp** (not wall-clock receive time) within a strict
`max_stereo_skew_s: 0.05` tolerance. Each image is consumed at most once;
the newer eligible pair wins, stale images are dropped, and buffers hold at
most ten pending frames per eye. No synthetic frames or oracle detections
are produced. On this simulator the observed pair skew is typically one
world tick, **0.033 seconds**. This is an *approximate-time* stereo pair,
NOT truly simultaneous or hardware-calibrated stereo. Evaluate motion
and disparity/depth errors during manual driving before using these
measurements for localization or autonomous planning. A future synchronized
stereo render/capture path would remove this approximation.

The node now logs four-second receive/pair-rate windows and counts unmatched
frames and callback gaps. A live run after the change measured:

| Stage / condition | Old exact-frame sync | Updated timestamp sync |
| --- | --- | --- |
| Each RGB eye, no RViz | ~7–9 Hz | ~7–8 Hz |
| Accepted stereo pairs, no RViz | **0–0.3 Hz** in diagnostic windows | **7.2–7.8 Hz** |
| Unpaired drops per 4 seconds | ~56–69 | **0–2** (typical) |
| Accepted stereo pairs, RViz open | Not measured with same diagnostics | **6.2–6.9 Hz** |
| Typical accepted inter-eye time skew | Identical frame only | **0.033 s** |

A bounded ten-second ROS subscription test *before reopening RViz* observed
64 `zed_msgs/ObjectsStamped`, 63 diagnostic `MarkerArray`, and 51 original
`asurt_msgs/LandmarkArray` messages. Maximum observed gap in those
object/marker streams was ~0.41 seconds. In a later 12-second
**RViz-loaded** test: 77 objects, 76 marker messages and 63 original ASURT
landmark messages; maximum observed gaps were respectively 1.30, 0.65 and
1.32 seconds. The 10 Hz goal is still unmet due to CARLA/render/ROS load;
do not claim that every system-level gap has been eliminated.

**RViz marker handling:** Observations are no longer automatically deleted
by a one-second marker lifetime. Lifetime zero plus stable track IDs and
explicit per-ID `Marker.DELETE` prevents temporary image stalls from
causing mass disappearance; the first publish from a new observer also
clears stale markers on its dedicated topic. These are visualization fixes,
not fabricated detections. Graceful shutdown is designed to release only
the two observer-owned CARLA cameras even if ROS has already invalidated
the marker publisher. During this work a previous cleanup attempt failed
after ROS invalidation: the two identified orphan observer cameras were
verified by role and removed without touching the hero, Pygame view
camera, course, bridge, or world ticker.

Verification: **19 of 19** tests passed, including five new stereo-sync
regressions; the `asurt_zed2i_sim` package rebuilt successfully. Live
original `/perception_landmarks` compatibility and active markers were
confirmed after restarting the observer. The native server stayed on
`127.0.0.1:2000`, with one existing FSAI hero and the restored 100-cone
course. Only observer-owned sensors were restarted. To monitor live
continuity with the complete ROS overlays sourced:

    ros2 topic hz /zed/zed_node/obj_det/objects
    ros2 topic hz /perception_landmarks
    ros2 topic info -v /zed/zed_node/cone_markers
    tail -f /tmp/asurt_zed2i_approx_sync_20261120.log

For the next performance pass, profile CARLA server and Pygame FPS, GPU
render time, and RViz image rendering with the same scene. Raising CARLA
world rate closer to 30 FPS is needed to reach 10 Hz camera capture with
the unchanged `sensor_tick=0.1`. Do not change world tick ownership,
physics, course placement, or weather fidelity to satisfy a benchmark.

## Previous VLP-16 simulation measurements — 2026-11-20

The independent `asurt_velodyne_sim` ROS 2 package attaches one CARLA
`sensor.lidar.ray_cast` actor to the existing FSAI hero. The source is
configurable in `src/asurt_velodyne_sim/config/vlp16.yaml` and can be
launched separately with `bash ./run_velodyne.sh` after checking existing
processes and CARLA LiDAR actors. It does not spawn the vehicle, tick the
world, edit weather or cones, or publish control. A real Velodyne driver can
replace this one source node, provided only one publisher and static TF
authority remain active.

The historical **provisional VLP-16-like profile** was 16 channels, 300,000
configured points/s, 10 rotations/s, 100 m range, and ±15° elevation.
CARLA ray casting provides actual scene intersections, intensity and noise;
it is **not** a calibrated physical VLP-16 return, reflectivity or firing
timing model. The output `/velodyne_points` is standard
`sensor_msgs/PointCloud2` with float32 x/y/z/intensity in `velodyne`
(+X forward, +Y left, +Z up). The single static transform
`base_link -> velodyne` is owned by this adapter. It never publishes
vehicle odometry or another world-frame TF.

CARLA provides a rotating sector every world tick. The
`RevolutionAccumulator` in `scan.py` merges sectors on fixed simulation-time
rotation boundaries, with a bounded queue. An earlier algorithm reset the
period after every completed scan and therefore discarded about one world
tick of potential scan phase per rotation. The corrected implementation
retains phase and rejects a scan if a full period of source segments is
missing, so timing and point-cloud continuity are not fabricated.

Two read-only **20-second live ROS measurements** were recorded while the
native CARLA server, bridge, camera observer and original camera converter
were running. The second sample used the corrected source mounts and scan
phase, without the LiDAR cone processor active:

| Measurement | Before LiDAR scan correction | Calibrated sensors and fixed scan phase |
| --- | ---: | ---: |
| Sim seconds in 20 wall seconds | 9.60 | 10.41 |
| CARLA real-time factor | 0.48 | 0.52 |
| LiDAR scans / simulated second | 6.04 | **9.90** |
| LiDAR scans / wall second | 2.89 | **5.14** |
| Mean points per published cloud | 27,948 | 17,277 |
| ZED object messages / simulated second | 9.58 | 9.80 |
| ZED object messages / wall second | 4.59 | 5.09 |

The mean points per LiDAR scan decrease because fewer sectors are incorrectly
combined into a single 360° scan. This is not an intended reduction in the
sensor's configured points-per-second parameter. Both measurements observed
monotonically increasing sensor message stamps. The observed differences
between sensor stamps and the latest `/clock` sample still require careful
acquisition-versus-processing latency analysis before SLAM synchronization.

Benchmark command (read-only; leaves running nodes in place):

```bash
cd /home/hassan/ASURT/autonomy_integration
source /opt/ros/humble/setup.bash
source /home/hassan/ASURT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
source install/setup.bash
/usr/bin/python3 scripts/benchmark_perception.py --seconds 20
/usr/bin/python3 scripts/profile_resources.py --seconds 12
```

Saved initial measurements: `evidence/perception_benchmark_20261120.json`,
`evidence/perception_sensor_calibrated_20261120.json`, and
`evidence/resource_profile_20261120.json`. The first resource sampler
assumed exactly one second between /proc readings; it slightly overestimates
CPU percentages because NVIDIA measurements add wall time. The corrected
resource sampler and current concurrent results are documented below. With
the 0.033-s fixed delta and an
observed ~0.5 real-time factor, a simulated 10 Hz sensor stream produces
around 5 messages per wall second. Reaching strict real-time wall-rate
operation remains unaccepted.

The original LiDAR processor integration is tracked separately from the
CARLA source. Do not use the old rosbag playback launch to consume live
LiDAR. SLAM, fusion, planning and autonomous control remain deferred until
the independent camera and LiDAR outputs, TF, and timestamps are accepted.

## Original ASURT LiDAR perception as a replaceable processor

The standalone `asurt_lidar_perception` ROS 2 package is a live consumer of
`/velodyne_points`; it is **not** a CARLA sensor actor. It publishes
`/filtered_points` (sensor-local PointCloud2), `/cluster_markers`
(MarkerArray), and `/lidar/cones` (ASURT LandmarkArray). All output headers
retain the original scan's capture timestamp and `velodyne` frame. The
LiDAR cone label is **UNKNOWN** because laser geometry alone cannot assign
blue/yellow colours. Stereo detection remains a separate publisher of
`/perception_landmarks`; sensor fusion can later associate them using TF.

The default `legacy_box` processing profile reproduces the historical
ASURT box ROI, vehicle exclusion, horizontal-ground RANSAC deletion, voxel
centroid filter, Euclidean clustering, and cone fitting. Its
`legacy_cone_classifier.py` matches the original old-stack ConeClassifier
source exactly (SHA-256
`5d0469ccff7c286eb25a8d794dd1a18b028e532d15a40c5b8a4e37af5fa9a300`).
It ports Open3D/Pandas operations to system NumPy/SciPy to limit memory
and dependency cost. The numeric RANSAC and voxel results can differ from
Open3D. This is a faithful CPU-oriented implementation of the historical
**BOX mode**. The original old-stack node actually defaults to the
stateful **LUT mode**, which is now available as a separate optional port
(`legacy_lut`) with the original algorithms and additional bounded state.
The `fast` profile is a separate, materially reworked alternative.

Before starting, verify no `lidar_perception_node` exists. Run the default
historical BOX profile with:

```bash
cd /home/hassan/ASURT/autonomy_integration
bash ./run_lidar_perception.sh
```

To select the faster alternative explicitly:

```bash
ASURT_LIDAR_PROFILE=fast bash ./run_lidar_perception.sh
```

The stateful historical LUT corridor profile is selectable separately:

```bash
ASURT_LIDAR_PROFILE=legacy_lut bash ./run_lidar_perception.sh
```

The LUT port keeps the original radial/z fallback, car exclusion, ground
RANSAC, voxel centroids, Euclidean clusters, **strict LUT cone classifier**,
paired left/right corridor update, EMA and width gating, and separate final
cone classifier. It replaces unbounded floating-point LUT keys with
quantized 0.1-metre cells, caps storage at 512 entries, and expires state
after 80 frames or six simulated seconds. It clears state on timestamp
rewind or frame-ID change. These protective changes, and the NumPy/SciPy
RANSAC implementation, mean it is not bit-identical to old Open3D.
An isolated live ROS smoke test published nonempty unknown-colour cones and
markers on separate test topics without creating a second production
publisher. The recorded 28,194-point scan took a median **20.83 ms** per
scan (10 runs), although that scan did not contain enough strict cone
pairs to form a corridor; synthetic multi-frame tests verified LUT building
and filtering. Stateful geometry is held in the **sensor-local** frame:
ego-motion compensation is still needed before accepting LUT persistence
while driving, or the LUT must be cleared as the vehicle moves. Therefore
the verified BOX profile remains the default live processor.

Only one LiDAR processor should own the three downstream output topics.
The input subscriber uses BEST_EFFORT with a single message in the ROS
queue, so overloaded processing discards stale frames instead of building
unbounded delay. Neither processor publishes CARLA commands or TF.

The 28,194-point recorded-scan microbenchmark measured median
`legacy_box` processing around **11.15 ms** and `fast` around **7.67 ms**
(ten trials). Six package-specific tests passed, the ROS package built,
and both profiles were smoke-tested with live point clouds. The original
source repository was not edited.

With the calibrated LiDAR and ZED camera and the **legacy_box processor**
running concurrently, a separate 20.03 wall-second measurement observed
8.48 simulated seconds (real-time factor **0.42**). It received 78 raw
LiDAR clouds, 81 filtered clouds, 81 `/lidar/cones` messages, 81 ZED objects
and 84 original ZED-converter landmark messages. These correspond to roughly
**9.2–9.9 messages per simulated second**, with no observed non-monotonic
sensor header stamps. The original short-range box filters the roughly
17,254 points per raw scan to about 65 points per output, and classified an
average 2.1 geometric cones per message. This number is limited by its
historical 8-m forward ROI and does not measure course-wide recall.

A concurrent 15-sample process profile observed the `legacy_box` process
using approximately **7.5% of one CPU core** on average (78 MiB RSS), with
the initial sampler's timing caveat described above. The whole simulator
remains slower than wall-clock real time, and the samples were taken under
varying desktop background loads; the change in real-time factor between
measurements cannot be attributed solely to adding the LiDAR processor.
Detailed evidence: `evidence/perception_full_legacy_box_20261120.json`
and `evidence/resource_full_legacy_box_20261120.json`.

## Final concurrent sensor-clock calibration and geometry audit — 2026-11-20

Both simulator observers now use a bounded one-time alignment against the
existing bridge's epoch-based simulation clock. Rather than anchoring each
sensor to the **first arbitrary** `/clock` and CARLA snapshot pair,
`ClockOffsetCalibrator` takes up to 21 distinct-frame pairs, spaced at least
95 ms apart, and freezes their median (or at least 15 samples after 3 s).
The bridge's `SimClock` uses an extrapolated `/clock`, so samples are not
truly instantaneous. The robust median removes most frame-phase bias.
The adapter does not keep polling CARLA snapshots after calibration.
During the brief calibration warm-up, it withholds sensor publications
instead of assigning an invented current timestamp to older measurements;
queues stay bounded. If `/clock` stops or the simulation resets, the source
must be restarted after restoring a valid clock. The bridge remains the
only tick and world-odometry authority.

The **live offset lock** was LiDAR
`1795186301.407806` s (21 samples) and camera
`1795186301.407494` s (17 samples), differing by about **0.000312 s**.
The 30.01-wall-second measurement with all four perception stages active
found 14.03 simulated seconds (real-time factor **0.468**):

| ROS topic | Messages | Hz per simulated second | Mean `/clock` minus stamp |
| --- | ---: | ---: | ---: |
| `/velodyne_points` | 135 | 9.62 | 55 ms |
| `/filtered_points` | 140 | 9.98 | 56 ms |
| `/lidar/cones` | 140 | 9.98 | 56 ms |
| `/zed/zed_node/obj_det/objects` | 135 | 9.62 | 78 ms |
| `/perception_landmarks` | 139 | 9.90 | 85 ms |

All observed header stamps advanced monotonically; no backward `/clock`
samples occurred. Mean age includes real sensor acquisition, full LiDAR
scan interval, stereo processing and ROS delays. A zero mean age is neither
possible nor desirable for a real capture timestamp. Relative to the previous
single-sample clock alignment, the mean raw LiDAR and stereo-object ages fell
from approximately 177 to 55 ms and 298 to 78 ms respectively. Largest
observed wall-clock stream gap was approximately 1.05 s for LiDAR and
0.91 s for ZED objects, so hard real-time continuity remains unaccepted.

The corrected resource sampler, which accounts for actual elapsed sample
intervals, observed: CARLA **332% of one CPU core** on average, ZED observer
44%, LiDAR source 10%, historical LiDAR BOX processor 6.8%, and ZED
converter 13%. GPU utilization averaged 28%, with **1812 MiB VRAM** of
4096 MiB reported in use. On this 8-logical-CPU i7-7700HQ, the renderer
and other desktop processes compete for CPU time. Sensor and processor
throughput are reasonable per simulated second, but full simulation still
does **not** run at wall-clock speed. No physics, weather, render-quality,
cone, map or CARLA fixed-delta settings were altered to improve a number.

The read-only `scripts/evaluate_perception_geometry.py` tool subscribes to
the existing detections and compares their transformed XY positions against
the 100 physical cone actors **for OFFLINE validation only**. It refuses to
score a moving vehicle and never publishes, feeds or changes perception
data. In its stationary 12-message-per-source run, the nearest-actor
horizontal errors were:

| Output | Observations | Mean XY error | Max XY error |
| --- | ---: | ---: | ---: |
| Historical LiDAR BOX cones | 26 | 0.0335 m | 0.0496 m |
| Stereo cones up to 7 m forward | 36 | 0.0244 m | 0.0342 m |
| Stereo cones beyond 7 m forward | 35 | 1.0329 m | 1.4177 m |

These repeated samples are correlated and cover only one stationary car
pose. The original short-range BOX mode classifies only about 2–3 of the
nearby cones at once; this audit measures positional agreement of reported
detections, **not detection recall**. Stereo nearest-cone blue/yellow labels
matched 71/71 stationary observations, which likewise does not establish
all-weather accuracy. The marked stereo range error beyond 7 m is a real
remaining blocker before using long-distance camera depth in fusion;
it probably requires better stereo matching, calibration and synchronized
exposures. The LiDAR source still represents a VLP-16-like ray-cast
approximation rather than a verified hardware firing model.

To repeat this entirely observational geometry audit, with the stationary
hero, 100 cones, calibrated sensors, and both processors already running:

```bash
cd /home/hassan/ASURT/autonomy_integration
source /opt/ros/humble/setup.bash
source /home/hassan/ASURT/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash
source install/setup.bash
/usr/bin/python3 scripts/evaluate_perception_geometry.py --seconds 12
```

Final evidence: `evidence/perception_full_clockaligned_20261120.json`,
`evidence/resource_full_clockaligned_corrected_20261120.json`, and
`evidence/perception_geometry_stationary_20261120.json`. Unit tests passed:
**16 Velodyne**, **23 ZED**, and **13 independent LiDAR processor** tests.
All ROS packages rebuilt successfully. A moving-car accuracy experiment,
faithful hardware firing/ring timing, hard real-time performance under
full autonomous compute, and future SLAM/fusion/planning/control acceptance
remain to be completed. The saved SLAM work remains deferred.

The optional combined RViz configuration at
`rviz/perception_combined.rviz` shows camera cone overlays and markers,
the lightweight `/filtered_points` cloud, and the unknown-colour LiDAR
cone candidates in one `base_link` view. Raw `/velodyne_points` drawing
and TF overlays are included but initially disabled to limit display load.
It is separate from the existing ZED-only RViz desktop shortcut. With the
correct ROS overlays sourced, open manually only if another RViz instance
is not already active:

```bash
rviz2 -d /home/hassan/ASURT/autonomy_integration/rviz/perception_combined.rviz
```

The combined RViz YAML parsed successfully; a fresh RViz window was not
launched during the performance measurement to avoid adding rendering load.

## Live sensor replacement contract

The read-only `scripts/check_perception_contract.py` verifies the current FSAI
hero, original 100-cone course, single Velodyne source, paired ZED RGB
sensor actors, physically calibrated mount positions, and all five ROS
perception topics. It also checks sensor timestamp order, cloud fields,
single `/clock` and `/odom` publishers, and the static camera/LiDAR TF.
It never creates an actor, publishes control, or ticks CARLA. Source
replacement requires preserving those topics, ownership and TF, or an
explicitly versioned update to the consumers and this checker.

Run after sourcing the ROS and ASURT overlays with:

`/usr/bin/python3 scripts/check_perception_contract.py --seconds 8`

The November 20 live check passed with zero failures and the report is
in `evidence/perception_contract_live_20261120.json`. A subsequent
physical sensor upgrade must update the expected mount coordinates used
by this checker and regression tests at the same time.

### Moving-vehicle acceptance blocked by control stopping

An experimental low-speed ROS `/drive` trial was attempted on November 20.
It did not meet its stop threshold within the test window, and the hero
subsequently appeared at a different position. A separate explicit brake
and zero-speed command brought the hero to rest. Five one-second CARLA
checks verified zero velocity and unchanged pose; the 100-cone course
remained present. The temporary command runner was deleted, and no further
motion testing is authorized by this evidence. The exact original car pose
was not saved, so it has not been restored. See
`evidence/moving_perception_trial_failed_20261120.md` for exact observations.
Bridge command/brake receipt and timeout behavior require instrumentation
and a verified fail-safe before repeating a dynamic test or starting SLAM.
