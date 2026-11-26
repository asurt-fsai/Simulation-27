# ASURT desktop tools

Each tool has its own folder and desktop entry:

| Folder | Function | Canonical code |
|---|---|---|
| `Trackdraw/` | Design Formula Student cone tracks and export positions | `Trackdraw/fs-trackdraw/` |
| `Track Importer/` | Preview and import course coordinates in CARLA | `Track Importer/cone_track_gui.py` and `import_cone_track.py` |
| `Weather Editor/` | Live CARLA sun and weather controls | `Weather Editor/carla_weather_gui.py` |
| `RViz/` | Combined ZED + Velodyne perception display | `RViz/config/perception_combined.rviz` |

Each application's real source files and the RViz preset files live in its corresponding tool directory. Legacy source locations in the ROS workspace and bridge have compatibility links. The `.desktop` entries use this host's absolute ASURT paths; adjust those when deploying to another machine.

The actual perception workspace is at `../Simulation/Systems/Perception/Workspace/autonomy_integration`; `../autonomy_integration` remains a compatibility link for live processes and installed ROS paths. The Track Importer cone-actor ownership JSON stays inside `../Simulation/CARLA/Bridge/Carla_ASU_Bridge/scripts`.

The CARLA Silverstone start and quit desktop entries are owned by `../Simulation/CARLA/Launchers/`. The quit script offers a read-only `--check` mode. During the November 2026 desktop audit Trackdraw, Weather Editor, Track Importer and combined RViz were observed as actual windows. RViz's Bash nounset initialization bug was repaired; repeat clicks reuse the same RViz process.
