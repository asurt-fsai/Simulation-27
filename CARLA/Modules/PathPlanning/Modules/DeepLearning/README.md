# ASURT deep-learning path planning

This is an isolated ROS 2 Humble adapter for the **existing** legacy FS-System-26
pretrained Seq2Seq LSTM. It reads existing landmark messages, runs the original
ONNX model, and publishes its **complete 15-waypoint raw LSTM prediction** to a
read-only RViz preview. The independently validated `nav_msgs/Path` is the
input to the autonomous controllers in CARLA. Drive commands come from the
selected controller.

## Model and hardware contracts

Original model is kept in-place at
`Simulation/Systems/Legacy/fs-system-26-oldstack/Deeplearning/src/planning_deep_learning/planning_deep_learning/Completed_Models/best_model.onnx`.
Input is float32 `[1,10,4]`: `[x_right, y_forward, blue_flag, yellow_flag]`
for up to ten nearest forward cones, padded to ten. The 15 predicted model
waypoints are converted to REP-103 `base_link`: `(x_forward,y_left) =
(model_y_forward,-model_x_right)`. No blue/yellow substitution for small
orange (type 2), large orange (type 3), or other classes is permitted.

The canonical simulation target is **GTX 1050, SM 6.1, 4 GB**. The competition
target is a **separate NVIDIA Jetson Xavier**. On 2026-11-22 the original
pretrained ONNX model executed on the GTX 1050 with ONNX Runtime GPU 1.20.2,
CUDA 12/cuDNN 9, and a privately installed NVIDIA cuRAND 10.3.9.55 runtime.
The benchmark profile confirmed actual `CUDAExecutionProvider` operator
execution, not just provider discovery. **CUDA was slower than CPU** for
batch-one, small-model inference on this GTX 1050; see measurements below.
`backend: cuda` is strict:
if this process has no ONNX Runtime CUDA provider, it reports
`model_unavailable` and publishes no path. To run an explicit non-control
CPU diagnostic, pass `-p backend:=cpu` to the launcher. The previous
vendored ONNX Runtime 1.20.1 package supports CPU and Azure only; a different
compatible GPU runtime must be installed to reproduce the verified GTX 1050
result on another machine. Provider presence alone does not prove that ONNX
operators execute on GPU; the adapter independently profiles CUDA warmup and
refuses a session with no GPU operator events. For Xavier, inspect its
JetPack/CUDA/TensorRT stack and deserialize or regenerate the legacy
`best_model_xavier.engine` **on the Xavier itself**. This ROS adapter does
not directly load standalone serialized TensorRT engine files.

### Measured laptop inference (2026-11-22)

The GTX 1050 was also serving an existing CARLA renderer during this benchmark.
The original model was run 100 times per provider with ONNX Runtime profiling
enabled (plus ten warmups per provider). This is a bounded diagnostic, not a
real-time latency guarantee or an end-to-end ROS throughput measurement.
One separate isolated ROS CUDA repetition reported a 752.6 ms inference,
further underscoring that GPU tail latency is not yet bounded for Control.

| Metric | CPUExecutionProvider | CUDAExecutionProvider |
|---|---:|---:|
| Median inference | 6.6115 ms | 21.9918 ms |
| p95 inference | 8.2051 ms | 88.0383 ms |
| Profiled operator events | 26,950 CPU | 26,950 CUDA |
| 100-call elapsed inference | 0.6758 s | 3.0580 s |

CUDA session GPU memory rose from 342 MiB to 414 MiB (approximately 72 MiB,
including CUDA initialization). CPU/CUDA output maximum absolute difference
across 12 test inputs was 5.722046e-6, within the 1e-2 comparison tolerance.
The earlier whole-path validator rejected all 12 synthetic examples by observed-track-extent checks,
not published as apparently valid paths. The isolated ROS CUDA contract test
also passed: its startup warmup recorded 245 CUDA operator events, with model
inference, stale clearing, simulated-time rewind, and disabled control
authority. Evidence is saved in
`Modules/DeepLearning/GPU/evidence/gtx1050_onnx_gpu.json`.

These numbers favor CPU for latency on **this** laptop, despite the GPU being
functional. The default remains `backend: cuda` to keep GPU validation
explicit; use the documented `-p backend:=cpu` diagnostic override for CPU.
Any choice for a real deployment needs repeatable end-to-end measurements.

## Interfaces

| Direction | Topic | Message | Frame / behavior |
|---|---|---|---|
| Input | `/perception_landmarks` | `asurt_msgs/LandmarkArray` | Usually `base_link`, timestamped, metric; TF transformed if needed |
| Optional input | `/map/global_cones` | `asurt_msgs/LandmarkArray` | Set `input_source: map` or `auto`; requires timestamped TF to `base_link` |
| Reset reference | `/odom` | `nav_msgs/Odometry` | Map frame; detects clock rewind / large pose jumps |
| **Raw model output** | `/planning/deep_learning/raw_path` | `nav_msgs/Path` | `base_link`; 15 finite, unmodified ONNX waypoints, including those past the last visible cone; timestamp is the source camera observation (not CUDA completion); separate experimental simulator Control may read it with independent freshness checks |
| Safety-gated output | `/planning/deep_learning/path` | `nav_msgs/Path` | `base_link`, stamped at cone observation time. The retained portion ends at or before the last observed paired gate. An empty Path clears an expired or rejected reference. |
| Output | `/planning/deep_learning/status` | `std_msgs/String` | JSON with model/provider, gating reason, `control_ready`, `control_path_points`, and `control_path_clipped`. |
| Output | `/planning/deep_learning/inference_ms` | `std_msgs/Float32` | Measured elapsed model inference, milliseconds |

Default `input_source: perception` depends on the existing ZED converter,
and the System Manager switch `planning_deep_learning` depends on
`zed_converter`. This module must be explicitly enabled; the saved default
is OFF. RViz views can be opened independently:
`tools/RViz/launch_planning.sh deep_learning`.
The RViz **Deep Learning** profile enables the raw model prediction by default
(bright cyan) and leaves the optional safety-gated path disabled (pink).
Unlike the Overview, Classical, Acceleration and Skidpad presets, the dedicated
Deep Learning profile shows **live ZED stereo cone cylinders** from
`/zed/zed_node/cone_markers` in `base_link`. Its accumulated
`/map/global_cones_markers` SLAM display is OFF by default and labeled
COMPARISON ONLY. The planner itself still consumes the converted
`/perception_landmarks` array; camera marker drawings are the visualization
companion, not an alternative landmark source.
Unlike the gated path, the raw preview is **not** rejected just because its
waypoints extrapolate beyond the observed cones. Raw previews are still
cleared when the 8-second `raw_preview_hold_s` display-only hold expires,
the map/clock rewinds, or the original model yields an invalid/nonfinite
tensor. The status includes `raw_preview_age_s` and
`raw_preview_last_observation_may_be_stale` to distinguish the **last true
prediction** from a fresh one during short perception gaps. The retention
does not affect the safety-gated path or any Control authority. It contains no
straight-line fallback, clipped waypoint, or fabricated track centreline.
On the GTX 1050 the async CUDA backend can miss the gated 0.25-second result
deadline, especially when CARLA and YOLO are active. The raw visualization
may show a completed result up to `raw_result_max_age_s: 3.0` old even when
the safety-gated path rejects it for being stale; status reports that fact.

## Safety and limitations

Input records are bounded to 2048; the model sees ten nearest, finite,
confidence-filtered blue/yellow cones within 15 m and 60 degrees. At least
two cones of each color must survive selection. Gates pair disjoint nearby
blue/yellow boundaries with 2–7 m width. The validated path keeps the
continuous overlap with the observed corridor, including the existing
bounded lead-in of up to 2.5 m before the first paired gate. It interpolates
the cut points on the original prediction. The retained path needs at least
1.5 m of forward progress, bounded waypoint spacing, and a 0.35 m margin
from interpolated cone boundaries. Each crossed gate is checked even when
it lies between waypoints. Raw model prediction is
always visually available when a valid model inference completed recently,
irrespective of whether it passes these geometric checks. A missing or
stale frame/TF, insufficient input cones, invalid model, stale asynchronous
result, teleport or simulated clock rewind still invalidate stale output.
There is no straight-line or odometry-only fallback.

These are *rejection gates*, not a proof of track safety, model calibration,
closed-loop stability, or race readiness. The guarded path is available for
CARLA testing. Competition use requires independent validation of perception,
planner timing, path geometry and controller behavior on the vehicle.

## Build and validate

With ROS 2 Humble and the ASURT message underlay sourced:

```bash
cd /home/hassan/ASURT
colcon build --packages-select asurt_deep_learning_planning --symlink-install \
  --base-paths Simulation/Systems/PathPlanning/Workspace/planning_integration/src \
  --build-base Simulation/Systems/PathPlanning/Workspace/planning_integration/build \
  --install-base Simulation/Systems/PathPlanning/Workspace/planning_integration/install

PYTHONPATH=Simulation/Systems/PathPlanning/Modules/DeepLearning/asurt_deep_learning_planning \
  python3 -m unittest discover -s \
  Simulation/Systems/PathPlanning/Modules/DeepLearning/asurt_deep_learning_planning/test -v
```

For an isolated ROS-only contract test, source the common ASURT underlay
and planning overlay, prepend the existing vendored CPU ONNX Runtime to
`PYTHONPATH`, set a non-live `ROS_DOMAIN_ID`, and run
`Simulation/Systems/PathPlanning/Tests/test_dl_ros_contract.py`.
This publishes simulated cones and clock **only in that isolated domain**,
invokes the original ONNX model, checks denial of one-sided input, stale
output clearing, rewind behavior, and never commands a vehicle.

For the normal attach-only simulation launcher, after the workspace build:
`Simulation/Systems/PathPlanning/Launch/run_deep_learning_planner.sh`.
The launcher reads the original model in place and does not start CARLA,
a ticker, a hero, sensors or Control.

## Reproduce the isolated GTX 1050 CUDA test

The pinned wheel downloader verifies the exact SHA256 published by PyPI. The
libraries are installed only into this module's virtual environment:

```bash
cd /home/hassan/ASURT
GPU=Simulation/Systems/PathPlanning/Modules/DeepLearning/GPU
python3 -m venv --system-site-packages "$GPU/.venv"
python3 "$GPU/download_ort_gpu.py"
python3 "$GPU/download_curand.py"
"$GPU/.venv/bin/python" -m pip install --no-index --no-deps \
  "$GPU"/.wheelhouse/onnxruntime_gpu-1.20.2-*.whl \
  "$GPU"/.wheelhouse/nvidia_curand_cu12-10.3.9.55-*.whl
python3 "$GPU/link_local_cuda.py"
"$GPU/run_benchmark.sh"
ROS_DOMAIN_ID=214 bash "$GPU/run_isolated_ros_test.sh"
```

The current laptop's additional existing CUDA 12/cuDNN 9 libraries come from
`~/.local/opt/resolve/libs`; `link_local_cuda.py` links these into a
**private** `.cuda_libs` directory without changing their originals. That
host path is a machine-specific dependency and **not** a portable CUDA
installation recipe. On another Ubuntu laptop use an independently verified
compatible CUDA/cuDNN 9 installation, and check `ldd`, driver compatibility,
CUDA operator profiling, and model parity before enabling the GPU module.
Do not copy these Linux GPU libraries or this serialized runtime to Xavier.

The isolated ROS test requires a nonzero `ROS_DOMAIN_ID`; it starts a
visual-only planner and synthetic test publisher in that domain and never
joins the live CARLA/ROS control graph. Evidence is a JSON file, not a model
weight replacement or competition acceptance certificate.
