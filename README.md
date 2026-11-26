# Simulation-27

ASURT Formula Student AI simulators.

| Simulator | Getting started |
| --- | --- |
| **Gazebo Harmonic** | Follow the [Gazebo guide](Gazebo/README.md) and build the ROS packages from `Gazebo/`. |
| **CARLA** | Download the installer for your OS from [Simulation-27 Releases](https://github.com/asurt-fsai/Simulation-27/releases/latest). The installer opens FlatAsphalt by default. |

## Install CARLA

On **Windows x64**, run the release copy of `Install-ASURT.ps1` in PowerShell. It provisions WSL2 and Ubuntu 22.04 as needed. Approve the Windows setup prompt and rerun after restarting if Windows requires it. Complete Ubuntu's first-run user setup when prompted.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Install-ASURT.ps1
```

On **Ubuntu 22.04 x86-64**, run the release copy of `Install-ASURT.sh`.

```bash
bash Install-ASURT.sh
```

The Simulation-27 installer downloads the public release over HTTPS without a GitHub sign-in. It verifies the release files and offers compatible optional maps on reruns.

## Run and update CARLA

1. Open **ASURT FlatAsphalt** to start the driving window and ROS bridge. On Windows, the **WSLg** shortcut uses the Linux simulator. Stop the current session before switching profiles.
2. Open **ASURT CARLA System Manager** to start modules and tools. The manager reports module status and live memory usage. Enable only the modules you need for your test.
3. Open **Map Packs** from System Manager, select a runtime profile and **Silverstone Split** or **FlatAsphalt**, then click **Load Map**. If you haven't installed Silverstone, rerun the installer and accept it, or install all available maps with one of the commands below.
4. Open **Cone Track Importer**, choose a CSV or JSON from **ASURT Workspace → Assets → Maps**, preview the layout, and spawn the cones. Use **Clear** and spawn again to reset a course after a cone strike.

For planning and control tests, enable **FakeMAP** for the full imported course or **FakeSLAM** to build a map as the car encounters cones ahead within 4.5 m. Both are under **System Manager → SLAM**. Adjust position noise under **Module Settings → SLAM Settings**. [FakeMAP and FakeSLAM workflow](CARLA/Modules/SLAM/README.md#fakemap-and-fakeslam-for-carla-testing)

**Pure Pursuit Autonomous** and **Stanley High-Speed Autonomous** follow a running planner's path without a goal click. [Autonomous control setup](CARLA/Modules/Control/README.md#autonomous-planner-following) · [ROS topic reference](CARLA/Tools/RViz/ROS_TOPICS.md)

Connect a local agent to generate rule-based Trackdraw cases, run them in CARLA, or record training and regression data. [ASURT MCP and ROS bag guide](CARLA/Tools/MCP/README.md)

To update, stop the simulator and rerun the latest release installer. A normal rerun updates installed release maps when a compatible revision is available. The following commands also install any compatible maps you haven't selected yet.

```bash
bash Install-ASURT.sh --maps all
```

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Install-ASURT.ps1 -Maps all
```

Starting with v0.1.26-flat, System Manager checks `asurt-fsai/Simulation-27` for new releases and compatible maps each time it opens, without a GitHub sign-in. On Windows and Ubuntu, accepting the prompt closes the managed ASURT simulator and tools, then opens the installer in a terminal with all compatible maps selected. Declining dismisses the prompt until the next time System Manager opens.

Older installations can accept the transition update if their existing GitHub credentials still work. If System Manager offers no update, run the latest Simulation-27 installer once to switch the installation to public updates.

## Edit team modules and tools

Open **ASURT Workspace** after installing. Inside it, **Modules** contains the editable ROS
source, **tools** contains the desktop tools and RViz configurations, and **Assets → Maps**
contains the track CSVs, JSON layouts, and road descriptions. **Assets → Map Packs** opens the
installed optional maps.
**Settings** opens the files for module and tool path overrides.

- For a bundled module, open **System Manager → Module Settings**, select it, and choose **Open selected source** or **Build and use workspace source**. For a separate team build, use **Save local build** and choose its executable and ROS setup script. **Add module** registers a new executable with a unique ROS output topic.
- Installer reruns preserve local selections, custom module entries, and editable workspace changes. When an incoming release changes a file you've edited, check `.asurt-updates/` inside the editable workspace for the incoming copy.
- [Architecture and handoff for coding agents](CARLA/AGENT_GUIDE.md)
- [CARLA source layout and prerequisites](CARLA/README.md)
- [Release assembly and publication](CARLA/Release/README.md)
- [Cook and register an add-on map](CARLA/Simulator/Launchers/MAP_PACKS.md)
