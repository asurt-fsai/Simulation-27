# ASURT PC System Manager and RViz

Open ASURT CARLA System Manager from the Windows ASURT CARLA folder, the
existing Tools panel, or the noVNC desktop. The manager reports the profile,
RPC endpoint and modules owned by the existing PC session supervisor. It
does not start a simulator when it opens. A switch change is picked up by
the current supervisor and retained for future sessions.

The group arrows and individual arrows only affect presentation. Layout
preferences live in ~/.config/asurt/control-center-view.json. Module switches
live in ~/.config/asurt/modules.json. Disabling a source pauses only its
dependent modules while retaining their individual switch preferences.

The four Perception entries keep their existing defaults. All six SLAM
entries start disabled in a fresh PC configuration. Enable Vehicle Trajectory
and Cone Landmark Mapper for the original baseline. Mapper results come from
the existing ZED converter and bridge. Restart the mapper with its own off/on
switch after its status shows it stopped to clear its in-memory map.

| View | Launcher | Displays |
| --- | --- | --- |
| Combined Perception | launch.sh | Stereo views, LiDAR and the blue FSAI vehicle |
| ZED-only Perception | launch.sh --zed | Stereo views and the same FSAI vehicle |
| SLAM baseline | launch_slam.sh | Ground grid, FSAI vehicle and confirmed cone markers |
| Advanced SLAM Diagnostics | launch_slam.sh --diagnostics | Separate 16-display diagnostic view |

The same single-link URDF is loaded from a file in all four presets. RViz
uses the existing map and base_link frames from the active session. The
standalone view commands open RViz without starting or enabling a simulator
or a SLAM module.

The PC registry is shared by the System Manager and session supervisor.
Perception-only bundle installs preserve it once the PC Manager is present.
Register new modules deliberately and keep them off until accepted.
