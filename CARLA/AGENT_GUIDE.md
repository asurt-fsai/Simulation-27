# ASURT CARLA agent guide

Use this guide when maintaining `asurt-fsai/Simulation-27` or working on a team's installed simulator. Confirm the active profile, map and module source before making a change.

## How the application starts

- The release launcher lives at `~/.local/share/asurt/current/bin/asurt` inside Ubuntu or WSL. Run it with `workspace` to print the editable workspace path. Run it with `exec` to use the same ROS Humble and CARLA Python environment as the application.
- [session.py](Simulator/Launchers/session.py) starts the selected CARLA server, ROS bridge, driving window and managed modules. Developer profiles include native Windows, WSLg and noVNC. The packaged Ubuntu profile runs natively. Stop the active session before changing profiles or updating.
- The bridge exposes the simulated car to ROS. The camera and Velodyne adapters take sensor data from CARLA. Perception, SLAM, Planning and Control run as separate ROS processes and communicate over the topics and message types in their builds.
- [modules.conf](Tools/RViz/modules.conf) defines shipped modules, their launchers, ROS readiness topics, dependencies and default switches. [module_control.py](Tools/RViz/module_control.py) resolves overrides, checks dependencies and reports live process status. Camera and LiDAR implementations live under [Modules/Perception](Modules/Perception).
- The [ROS topic reference](Tools/RViz/ROS_TOPICS.md) maps the default topics to their producers and consumers. Autonomous controllers follow one registered autonomous planner and require fresh `/clock`, `/odom`, and ROS driving mode.

## Work on a local module

1. Open **System Manager → Module Settings** and check whether the team is using **Bundled** or **Local**. The installed shortcuts resolve through `current`. The editable source is in **ASURT Workspace → Modules**. The workspace is normally `~/.local/share/asurt/workspace`.
2. For a shipped ROS package, select **Build and use workspace source**. For work in a separate repository, build the team's package with ROS Humble and the ASURT message underlay. Choose its installed executable and `install/local_setup.bash` through **Save local build**. The existing shipped entry supplies its normal launch arguments. Reopen Module Settings and confirm the recorded paths.
3. Use **Add module** for a new independent process. Give it a stable ID, label, group, an absolute executable path, a unique output topic and any required managed-module dependencies. Set the optional overlay setup script and arguments. Enable the new entry in System Manager and verify the topic publishes in the same ROS domain as the session. Keep a standalone test outside the manager if it has its own launcher and lifecycle.
4. Use **Use bundled build** to revert a shipped module. **Remove module** removes a custom entry from the manager. Module files and builds remain on disk.

**ASURT Workspace → Settings** links to `$XDG_CONFIG_HOME/asurt` or `~/.config/asurt`. `module-sources.json` holds paths and custom entries, `modules.json` holds switches, and `tool-paths.json` holds tool entry overrides. Build and test from the selected source. Preserve all three files when upgrading. If an incoming release also changed a locally edited file, review the incoming copy in the workspace's `.asurt-updates/` directory.

Use the session's ROS environment when testing a module. For example, with the simulator running, inspect the LiDAR output with this command in Ubuntu or WSL.

```bash
~/.local/share/asurt/current/bin/asurt exec ros2 topic info /lidar/cones
```

## Work on maps, props and tools

- **ASURT Workspace → tools** is editable. Use **Settings → tool-paths.json** to attach a different entry file to an existing shortcut. Reopen the tool after editing.
- The [local MCP server](Tools/MCP/README.md) generates Trackdraw cases, starts isolated CARLA test runs, and records ROS bags. A track report captures the rulebook hash and cited overrides. Keep dedicated test runs in their own ROS domain and let the existing bridge own world ticks.
- `Assets/Maps` contains layouts and OpenDRIVE source. Silverstone Split is an optional map pack with the ID `silverstone-section`. The pack manifest records its spawn pose, course origin, cooked files and base compatibility. See the [map cook instructions](Simulator/Launchers/MAP_PACKS.md) and root [map catalog](../asurt-maps.json).
- Keep asphalt, curbs, sand, grass, barriers and structures collidable on Silverstone Split. Set decorative trees, tents and track markings to **NoCollision**. Confirm collision in the cooked Windows and Linux maps at known coordinates.
- [Track Importer](Tools/Track%20Importer/import_cone_track.py) uses the stock coloured cone meshes and Unreal rigid-body physics. The smaller cones use 0.45 kg and the large orange cone uses 1.05 kg, matching the event specifications. The importer owns actors through a persisted ID, type and role record. Preserve this contract and keep preview read-only. The installer enables `SensorTrace` for Unreal's `PhysicsActor` profile so CARLA LiDAR scans include movable cones. Test for actual cone detections in the camera and LiDAR output. [Formula Student Germany 2026 Event Handbook, table 2](https://www.formulastudent.de/fileadmin/user_upload/all/2026/rules/FSG26_Event_Handbook_v1.1.pdf)

## Verify and release

Test through the installed shortcut or launcher. Check the selected paths, process status, ROS output, map and car spawn. For importer changes, test preview, spawn, status and clear with a live track. For compatible map revisions, exercise an update over the last published pack as well as a clean install. Keep team-local sources and paths, credentials, logs and builds out of commits. Use the [release workflow](Release/README.md) to publish the relevant stack or map asset.
