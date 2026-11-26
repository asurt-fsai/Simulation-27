# CARLA

The distributable simulator is the ASURT CARLA 0.9.16 build with FlatAsphalt, the FSAI car, and the cone assets. See the [root README](../README.md) for installation.

| Directory | Contents |
| --- | --- |
| `Simulator/` | CARLA launcher, map-pack tools, ROS bridge source and configuration, and the isolated WSL graphics dependency. |
| `Tools/` | Cone Track Importer, Weather Editor, Trackdraw, ROS Bags, and RViz tools and displays. The launcher also provides System Manager and Map Packs. |
| `Assets/Maps/` | CSV and JSON cone layouts, competition courses, and source road descriptions. |
| `Assets/Props/` | Editable prop exports and material revision records. |
| `Modules/` | Perception, SLAM, path planning, control and shared ROS code. |
| `Release/` | Build recipe, updater, dependency definitions, and optional-map catalog utility. |

The binary release supplies the cooked simulator, CARLA client, compiled ROS workspaces, and model weights. The release builder consumes the existing ASURT authoring and ROS build workspace. Its installed paths match the existing launcher and module paths so that existing runtime configurations load correctly.

The installer also provides an [editable module and tools workspace](Release/README.md#edit-a-module-or-tool).
Use System Manager to switch between shipped and local module builds.

The local [Trackdraw and CARLA MCP server](Tools/MCP/README.md) exposes course validation, test runs, and ROS bag recording to agents.

## Windows and Ubuntu

- The Windows installer supplies **native CARLA**, with the ROS bridge and tools inside **Ubuntu 22.04 on WSL2**. It also installs the Linux runtime for the **WSLg** shortcut. Stop the current session before switching between the two runtime profiles.
- The installer downloads the pinned [Mesa Dozen v0.1.0 kit](https://github.com/hassanradwannn/Mesa-Dozen-Custom-WSL-driver-for-Carla/releases/tag/v0.1.0) inside WSL, verifies its SHA-256, and selects its isolated Vulkan driver for the WSLg CARLA process. The Windows host's GPU driver must expose GPU access to WSL2. The native Windows CARLA server uses its Windows graphics driver, and the ROS bridge connects over the local network.
- On Ubuntu 22.04 x86-64, CARLA and the tools run natively. Where a CUDA-capable NVIDIA device is available, the installer verifies and enables the pinned CUDA inference stack for YOLO and the deep-learning planner. If no CUDA device is visible, both use CPU inference.
