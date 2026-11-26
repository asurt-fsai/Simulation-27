# ASURT FlatAsphalt installer and releases

The installer downloads the cooked simulator, the compatible CARLA client, the ROS bridge and modules, model weights, and desktop tools. A launch opens FlatAsphalt. The system manager, RViz views, Weather Editor, Track Importer, Trackdraw, and Map Packs get shortcuts.

## Install

Use Ubuntu 22.04 x86-64 with a working desktop and Vulkan driver, or Windows x64. The Windows installer provisions WSL2 and Ubuntu 22.04 when needed and checks the distro's version and architecture. A first Windows setup can require a reboot and creation of the Ubuntu user. It installs native Windows CARLA and the WSLg runtime. The isolated Dozen driver is used by WSLg CARLA. Windows firewall authorization is restricted to the native executable, its three RPC/streaming ports, and the current WSL address. If that address changes and connection fails, rerun the installer.

```bash
bash Install-ASURT.sh
```

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Install-ASURT.ps1
```

The release installers default to `asurt-fsai/Simulation-27`. GitHub CLI sign-in gives the installer and System Manager access to the release and optional map catalog. Linux installs under `~/.local/share/asurt`. Windows stores the native simulator under `%LOCALAPPDATA%\ASURT` and the WSLg runtime, ROS stack and tools under the Linux user’s installation directory.

If a CUDA-capable NVIDIA GPU is available, the installer downloads a pinned CUDA 12.8 and cuDNN 9.8 runtime plus ONNX Runtime GPU. Both YOLO26 and the deep-learning planner use CUDA. The installer runs both models and verifies that operators ran on the GPU. The first GPU installation downloads about 1.94 GB of additional wheels. Reruns reuse the installed CUDA runtime while the requirements stay the same. An existing local CUDA runtime can also be reused when the previous installation has matching requirements, the expected ONNX Runtime version, and both models pass GPU verification. If a wheel download fails, rerun the installer to reuse completed downloads.

Install a compatible NVIDIA host driver before running GPU inference. Windows users install the Windows driver so WSL2 can access the GPU. If a machine has no CUDA device, the installer uses the CPU models and skips the CUDA download. When an available CUDA device fails validation, the installer stops the update and reports the error. [NVIDIA CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/index.html) · [cuDNN 9.8 support matrix](https://docs.nvidia.com/deeplearning/cudnn/backend/v9.8.0/reference/support-matrix.html)

Mission and controller modules remain opt-in. Use **ASURT CARLA System Manager** for module switches and **ASURT CARLA Map Packs** for map loading.

Use **System Manager → Tools → ROS Bags** to record selected live topics into one bag or replay an existing bag with RViz. The tool opens a visible console for recorder and player output. Compressed SQLite bags are expanded into temporary disk or RAM storage for replay. See the [autonomous ROS topic reference](../Tools/RViz/ROS_TOPICS.md) for the active sensor-to-control streams and recording choices.

The bundled VLP-32C uses the `reference` profile at 600,000 rays per simulated second. An explicit `ASURT_VELODYNE_PROFILE` still selects the requested density.

On native Ubuntu, the installer and launcher check for a working dedicated Vulkan GPU. The launcher uses the verified driver for the CARLA server. It honors explicit Vulkan or PRIME settings and uses the system Vulkan default when it cannot isolate a dedicated GPU. The installer prints the selected renderer.

## Edit a module or tool

Open the **ASURT Workspace** desktop shortcut or **System Manager → Module Settings → Open workspace**.
On Windows the shortcut is inside the **ASURT CARLA** desktop and Start menu folder. The path is
`~/.local/share/asurt/workspace` in Ubuntu or WSL. Use an editor running in Ubuntu or connected to
WSL when building a ROS package.

The default WSL install is also accessible from Windows Explorer at
`\\wsl.localhost\Ubuntu-22.04\home\<linux-user>\.local\share\asurt\workspace`.

| Workspace folder | Contents |
| --- | --- |
| `Modules/` | Editable Perception, SLAM, Planning, Control, and shared ROS source |
| `tools/` | Desktop tools and RViz configurations |
| `Assets/Maps/` | Cone CSV and JSON presets, reference paths, and road descriptions |
| `Assets/Map Packs/` | Optional maps installed through Map Packs |
| `Assets/Props/` | Editable prop exports and material revision notes |
| `Settings/` | Module and tool path overrides |

System Manager writes module overrides to `Settings/module-sources.json`. The stock module
catalog is `tools/RViz/modules.conf`. Edit `Settings/tool-paths.json` to point an existing tool
shortcut at a different entry file. Paths can be absolute or relative to the workspace.
For example:

```json
{
  "track": "tools/Track Importer/cone_track_gui.py",
  "planning-rviz": "/home/me/ros_ws/settings/planning.rviz"
}
```

Tool keys include `track`, `weather`, `trackdraw`, `manager`, `map-packs`, `ros-bags`, `tools`, `rviz`,
`zed-rviz`, and `planning-rviz`. Reopen the tool after saving a path change.

- Edit a tool under `tools/` or in `Simulation/CARLA/Launchers/`. Reopen the tool to load the edits.
- In **Module Settings**, select a module and click **Open selected source**. After editing, click
  **Build and use workspace source**. The supervisor switches that module and restarts any active
  dependent modules. Select **Use bundled build** to switch back.
- To use a team's separate colcon workspace, select its executable under
  `install/<package>/lib/<package>/` and its `install/local_setup.bash`, then click
  **Save local build**. **Add module** registers a custom executable, ROS topic, and optional
  dependencies. **Remove module** removes the entry from System Manager.
- Set up a development machine with
  `sudo apt install python3-colcon-common-extensions build-essential`. LeGO-LOAM also needs the
  PCL and Boost development packages. The release supplies the compatible ROS interfaces and
  GTSAM prefix. Changes to a shared ROS message require rebuilding its dependent modules.

You can also build and select the Classical Planner from Ubuntu:

```bash
~/.local/share/asurt/current/bin/asurt dev-build asurt_classical_planning planning_classical
```

The workspace uses symlink installs for development builds. Saving the same local build again in
Module Settings restarts the module after a Python edit. Rebuild after changing compiled code
or package metadata. Updates preserve workspace edits. If both the workspace and release changed
the same file, review the incoming version under `workspace/.asurt-updates/`.
Installer updates keep module selections and switches in `Settings/` and
leave local build directories in place. A module selected as **Local**
continues to use its selected executable after an upgrade.

## Publish and update

Attach `Install-ASURT.ps1`, `Install-ASURT.sh`, `asurt-release.json`, `asurt_update.py`, both `ASURT-FlatAsphalt-*.tar.gz` runtimes, and `ASURT-Stack-Ubuntu-22.04-x64.tar.gz` to the same GitHub Release in `asurt-fsai/Simulation-27`. Also attach the Linux x86-64 CARLA wheel for teammates writing Python 3.10 clients. The installed stack includes that client. Keep `asurt-maps.json` at the Git repository root. A maps-only release should leave the installer release marked Latest.

Each simulator runtime and the module/tool stack has a separate SHA-256 in `asurt-release.json`. Updating the stack reuses an unchanged simulator. Existing settings and registered maps live outside versioned installations. Stop the simulator before activating an update.

A small stack update can pin the previous stack asset by release tag and supply a verified
`stack_patch` archive. The installer applies it to a new stack before activating the version.
Use `--wsl-graphics` when assembling a release that includes the pinned Mesa Dozen WSL
archive. Publish it alongside the manifest, or reference its existing production
release tag in `wsl_graphics`.

## Make a new map

1. In the editable Unreal project, duplicate **FlatAsphalt** into `/Game/ASURTMaps/<pack_id_with_underscores>/`. Give the new level a unique asset name. Keep shared CARLA assets referenced from their original paths. Keep any new map-specific meshes and materials in the pack's folder.
2. Edit the geometry and barriers. Keep the weather actors from FlatAsphalt and verify that the driving surface has collision and the intended physical material. Choose a clear FSAI spawn position. Unreal positions are in centimetres, while the pack needs CARLA coordinates in metres and yaw in degrees. Verify the pose in CARLA.
3. Prepare a matching `.xodr` road description for the level. The simulator uses it for road topology. Cone Track Importer still loads your cone layout from CSV or JSON.
4. Run `cook-map-pack.ps1` from the [map-pack authoring workflow](../Simulator/Launchers/MAP_PACKS.md) with `-Workspace` set to the ASURT Unreal build workspace. For a released map, point `-WindowsBase` at the `WindowsNoEditor` folder extracted from the release and `-LinuxBase` at the extracted Linux runtime directory. The pack records the executable and shared asset hashes from those bases. Test the ZIP with **ASURT CARLA Map Packs** before listing it.

Attach the finished map ZIP to a release, then update the root `asurt-maps.json` on the repository’s `main` branch.

```text
python CARLA/Release/add_map_to_catalog.py silverstone-national.zip --tag maps-2026-11 --catalog asurt-maps.json
```

Rerun the installer to see compatible additions. Choose each map interactively, or select `--maps all` on Linux or `-Maps all` on Windows. The installer checks the SHA-256 of the installed map, its download cache, and ZIPs in the current directory or `~/Downloads` before downloading. New downloads show transfer progress. Imported maps appear in **Map Packs**. A new map pack reuses the matching base.

## Update maps and props

- A new map or prop placed only in that level can ship in a map ZIP. Keep its new meshes and materials inside the map folder. For a compatible fix to a published map, keep the pack ID and publish the ZIP with a new release tag. Run `add_map_to_catalog.py` with `--replace` to record the previous checksums. Installer reruns update known releases and preserve custom pack ZIPs. The launcher replaces existing cooked files only when they match a known previous release. Use a new ID for a separate layout or a different set of base builds.
- A change to shipped FlatAsphalt, a shared prop such as a cone, or a new globally spawnable CARLA prop goes in the next simulator runtime archives. For a new spawnable prop, register it in the CARLA prop factory and cook its dependencies for Windows and Linux. Rerunning the installer picks up the new runtime archive by its SHA-256.
- A map pack validates its shared asset hashes on load. If a shared prop update changes a file listed by an existing pack, recook that pack against the new release base. A CSV or JSON cone-layout change can ship with the stack or be loaded directly in Track Importer.

## Assemble

Run `CARLA/Release/build_release.py` on the Ubuntu 22.04 build host or WSL distro. Pass `--source` for the ASURT workspace containing the built ROS overlays and model weights, `--launchers CARLA/Simulator/Launchers`, and `--output` for a new output directory. Supply `--linux` and `--windows` for the staged runtime directories, or use `--reuse-runtimes-from /path/to/previous/asurt-release.json` to preserve the simulator archives through module and tool updates. Set `--tag` to the release tag. The builder embeds the Bash bootstrap in the downloadable Windows installer.

For offline checks, keep the artifacts next to their manifest and pass `--manifest /path/to/asurt-release.json` or `-Manifest C:\path\asurt-release.json`. The host still needs the platform prerequisites for a first installation.
