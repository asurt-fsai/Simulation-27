# Perception packages and workspace

Camera, LiDAR and future fusion algorithms are organized under Modules. CARLA sensor publishers live under Adapters/CARLA, with a separate Adapters/Gazebo directory reserved for implementations against Gazebo. The live colcon workspace is Workspace/autonomy_integration; its src links map to one authoritative source per package.

The ZED 2i package is the active CARLA stereo observer. Its ROS package contains
the optics, detection, timing and tracking code. The active vision backend is the FSOCO YOLO26n ONNX model in
Models/yolo26n_fsoco_e2e_opset17.onnx. The former OpenCV HSV detector remains
available via vision_backend=hsv. Evidence contains validation records, and
Rosbags is reserved for recordings. Camera intrinsics, FSAI hero, CARLA ticker,
the stereo baseline, ROS topics and tool controls are unchanged by the swap.

## Windows simulator profiles

Native Windows, WSLg, and noVNC start the same ROS Humble perception workspace
in Ubuntu-22.04. The shared session launcher passes the selected CARLA endpoint
to the module scripts registered in `tools/RViz/modules.conf`. The stereo
observer, legacy ROS converter, LiDAR observer, and LiDAR processing each have
their own `run_*.sh` entry point in `Workspace/autonomy_integration`.

Camera settings live in the ZED adapter's `config/zed2i.yaml`. An empty
`yolo_model_path` selects the bundled FSOCO model. For a different model, set
an absolute path in that YAML or set `ASURT_YOLO_MODEL_PATH` before launching.
Set `ASURT_VISION_BACKEND=hsv` to compare with HSV. The LiDAR source density
uses `ASURT_VELODYNE_PROFILE` and the downstream algorithm uses
`ASURT_LIDAR_PROFILE`. Restart the relevant module or simulator session to
apply configuration changes.

The camera script adds the pinned CPU ONNX Runtime under `Dependencies` only
to the camera process. Update the perception packages with
`colcon build --symlink-install` from the ROS workspace. Model and settings
updates require no CARLA cook.

The PC release under `Simulation/Systems/Perception/Releases` contains the
perception source, model and inference dependencies. On an existing PC setup,
run `Simulation/CARLA/Launchers/stage-wsl-parity.sh perception` from WSL to
deploy just that release and rebuild the perception packages. The optional
`ASURT_PERCEPTION_RELEASE` path selects a different bundle.

## Velodyne VLP-32C Ultra Puck

The default adapter traces all 32 nominal VLP-32C elevations and azimuth
offsets from table 9-2 of the [manufacturer's manual](https://data.ouster.io/downloads/velodyne/user-manual/vlp-32c-revd-user-manual.pdf).
The beams cover 360° horizontally and −25° to +15° vertically. Each ray
uses a 200 m limit. The adapter combines the banks into one XYZI stream
on `/velodyne_points` at 10 rotations per simulated second, with one
`base_link → velodyne` transform at the existing mount.

The default `reference` profile requests 600,000 rays/s for the VLP-32C.
Set `ASURT_VELODYNE_PROFILE=balanced` for 360,000 rays/s or `performance`
for 240,000 rays/s. Scene intersections and simulated dropoff determine
how many points reach `/velodyne_points`. Beam timing, intensity, and range
noise use CARLA's ray-cast approximation.

Use `ASURT_VELODYNE_MODEL=vlp16` before launching CARLA for the previous
scan pattern. The SLAM launcher selects the matching projection profile.
