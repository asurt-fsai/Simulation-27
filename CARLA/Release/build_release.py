"""Assemble ASURT release assets on the validated Ubuntu/WSL build host."""

import argparse
import base64
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
from urllib.request import urlopen
import zipfile


HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[1]
PRODUCTION_REPOSITORY = "asurt-fsai/Simulation-27"
WSL_GRAPHICS_FILE = "asurt-mesa-dozen-wsl-team-kit-20261024.tar.gz"
WSL_GRAPHICS_SHA256 = "f729e89f8e069b4e717a3b38d5ce89311789a38b81e450d4b6014e08b3204200"
OVERLAYS = (
    "Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build",
    "Simulation/CARLA/Bridge/Carla_ASU_Bridge",
    "Simulation/Systems/Perception/Workspace/autonomy_integration",
    "Simulation/Systems/SLAM/Workspace/slam_integration",
    "Simulation/Systems/PathPlanning/Workspace/planning_integration",
    "Simulation/Systems/Control/Workspace/control_integration",
)
IGNORE = shutil.ignore_patterns(".git", "__pycache__", ".pytest_cache", "Saved", "Logs",
    "install", "Installed", "build", "log", "evidence", "Evidence", "build-*", "log-*", "install-*",
    ".venv", ".wheelhouse", ".cuda_libs", "ThirdParty", "Dependencies", "thirdparty_lib",
    "*.pyc", "*.pdb", "*.log", "*.egg-link", "direct_url.json")
MODULES = (
    "tools", "Assets/Maps",
    "Simulation/Systems/Perception", "Simulation/Systems/SLAM",
    "Simulation/Systems/PathPlanning", "Simulation/Systems/Control",
)
PYTHON_ADAPTERS = {
    "asurt_zed2i_sim": "Camera",
    "asurt_velodyne_sim": "LiDAR",
}
DRIVING_SOURCE = Path("CARLA/Simulator/Bridge/src/ros_apps/carla_telemetry/carla_telemetry/manual_control.py")
BRIDGE = Path("Simulation/CARLA/Bridge/Carla_ASU_Bridge")
BRIDGE_CONFIG = Path("CARLA/Simulator/Bridge/config/carla_interface_config.yaml")
BRIDGE_FEEDBACK = Path("CARLA/Simulator/Bridge/src/ros_apps/carla_telemetry_cpp/src/ros2_backend.cpp")
BRIDGE_EXECUTABLE = BRIDGE / "install/carla_telemetry_cpp/lib/carla_telemetry_cpp/carla_telemetry_node"
DRIVING_INSTALL = BRIDGE / "install/carla_telemetry/lib/python3.10/site-packages/carla_telemetry/manual_control.py"
ADAPTERS = Path("Adapters/CARLA")
PERCEPTION = Path("Simulation/Systems/Perception")
PERCEPTION_WORKSPACE = PERCEPTION / "Workspace/autonomy_integration"
DEP_PKG = "Simulation/Systems/Perception/Dependencies/onnxruntime"
LEGACY = "Simulation/Systems/Legacy/fs-system-26-oldstack/Deeplearning/src/planning_deep_learning/planning_deep_learning/Completed_Models/best_model.onnx"
FS26_PACKAGES = (
    ("Control", "simple_pure_pursuit", "simple_pp_node", "simple_pure_pursuit.simple_pp_node"),
    ("Control", "adaptive_pure_pursuit", "adaptive_pp_node", "adaptive_purepursuit.adaptive_pp_node"),
    ("SLAM", "cone_mapping", "cone_mapping_node", "cone_mapping.cone_mapping_node"),
)
FS26_LAUNCHERS = {
    "Control": ("run_fs26_control.sh", "run_fs26_simple.sh", "run_fs26_adaptive.sh"),
    "SLAM": ("run_fs26_cone_mapping.sh",),
    "PathPlanning": ("run_classical_goal.sh", "classical_goal.py"),
}
FS26_RETIRED = tuple("Simulation/Systems/" + package for package in (
    "Control/Modules/PurePursuit/asurt_pure_pursuit_control",
    "Control/Modules/Stanley/asurt_stanley_control",
    "Control/Modules/Autonomous/asurt_autonomous_control",
    "SLAM/Modules/CameraLocalization/asurt_camera_slam",
    "SLAM/Modules/Fusion/asurt_slam_fusion",
    "SLAM/Modules/LiDARMapping/asurt_lidar_odom_adapter",
    "PathPlanning/Modules/GoalPose/asurt_goal_pose_planning",
    "PathPlanning/Modules/TrackGoal/asurt_track_goal_planning",
))
TRANSFORMATIONS_WHEEL = (
    "https://files.pythonhosted.org/packages/7e/46/8d70ff706699b44107f1c649ca4af7c9665da313b706cf5f4bb1ec4db1f4/"
    "transformations-2025.1.1-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
)
TRANSFORMATIONS_HASH = "2ed30be8eed56680dd1405117dfae58f2ebba99cff1333503b8cdac46c85f976"


def digest(path):
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(chunk)
    return sha.hexdigest()


def item(path):
    return {"file": path.name, "sha256": digest(path), "bytes": path.stat().st_size}


def copy(source, target, ignore=IGNORE, symlinks=False):
    if source.is_dir():
        def skip(folder, names):
            omitted = set(ignore(folder, names))
            for name in names:
                path = Path(folder) / name
                if path.is_symlink() and not path.exists():
                    omitted.add(name)
            return omitted

        shutil.copytree(source, target, dirs_exist_ok=True, ignore=skip, symlinks=symlinks)
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def copy_overlay(source, target):
    copy(source, target)
    for link in source.rglob("*.egg-link"):
        location = Path(link.read_text().splitlines()[0])
        site = target / link.parent.relative_to(source)
        for package in location.iterdir():
            if package.name.endswith(".egg-info") or (package.is_dir() and (package / "__init__.py").is_file()):
                copy(package, site / package.name)
    for package in target.glob("*/share/*"):
        if not package.is_dir():
            continue
        for hook in package.glob("package.*"):
            if hook.suffix in {".dsv", ".sh", ".ps1", ".bash"}:
                lines = hook.read_text().splitlines(keepends=True)
                hook.write_text("".join(line for line in lines if "pythonpath_develop" not in line))
    (target / "setup.bash").write_text('source "$(dirname -- "${BASH_SOURCE[0]}")/local_setup.bash"\n')


def copy_python_adapter_file(package, filename, stack):
    adapter = ADAPTERS / PYTHON_ADAPTERS[package] / package / package / filename
    source = REPOSITORY / "CARLA/Modules/Perception" / adapter
    install = PERCEPTION_WORKSPACE / "install" / package / "lib/python3.10/site-packages" / package
    build_source = PERCEPTION_WORKSPACE / "src" / package / package
    for destination in (PERCEPTION / adapter, build_source / filename, install / filename):
        copy(source, stack / destination)


def copy_driving_window(stack):
    editable = BRIDGE.joinpath(*DRIVING_SOURCE.parts[3:])
    for destination in (editable, DRIVING_INSTALL):
        copy(REPOSITORY / DRIVING_SOURCE, stack / destination)


def stage_fs26(stack):
    """Package the original Python nodes for the shared Ubuntu/WSL ROS stack."""
    source = REPOSITORY / "CARLA/Modules"
    systems = stack / "Simulation/Systems"
    runtime = stack / "libexec/asurt-fs26"
    (stack / "retired-modules.json").write_text(json.dumps(FS26_RETIRED, indent=2) + "\n", encoding="utf-8")
    for group, package, executable, module in FS26_PACKAGES:
        relative = Path(group) / "Modules/LegacyFS26" / package
        copy(source / relative, systems / relative)
        destination = runtime / package
        python_package = module.split(".", 1)[0]
        copy(source / relative / python_package, destination / python_package)
        copy(source / relative / "config", destination / "config")
        entry = destination / "lib" / package / executable
        entry.parent.mkdir(parents=True, exist_ok=True)
        if executable == "simple_pp_node":
            entry.write_text((source / "Control/Launch/fs26_simple_bootstrap.py").read_text(
                encoding="utf-8"), encoding="utf-8", newline="\n")
        else:
            entry.write_text(f"#!/usr/bin/python3\nfrom {module} import main\n\nmain()\n", encoding="utf-8")
        entry.chmod(0o755)
    for group, scripts in FS26_LAUNCHERS.items():
        for script in scripts:
            relative = Path(group) / "Launch" / script
            copy(source / relative, systems / relative)
    control_pair = runtime / "control_pair"
    control_pair.write_text((source / "Control/Launch/fs26_control_pair.py").read_text(
        encoding="utf-8"), encoding="utf-8", newline="\n")
    control_pair.chmod(0o755)
    goal = runtime / "classical_goal"
    goal.write_text((source / "PathPlanning/Launch/classical_goal.py").read_text(
        encoding="utf-8"), encoding="utf-8", newline="\n")
    goal.chmod(0o755)
    dependencies = runtime / "dependencies"
    with tempfile.TemporaryDirectory() as temp:
        wheel = Path(temp) / "transformations.whl"
        with urlopen(TRANSFORMATIONS_WHEEL, timeout=25) as response, wheel.open("wb") as output:
            shutil.copyfileobj(response, output)
        if digest(wheel) != TRANSFORMATIONS_HASH:
            raise ValueError("The FS-System-26 transformations wheel failed verification.")
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(dependencies)


def adapt_launchers(source, stack):
    launch = stack / "Simulation/CARLA/Launchers"
    launch.mkdir(parents=True, exist_ok=True)
    for name in ("map_packs.py", "map_pack_panel.py", "managed_modules.py", "native_vulkan.py", "release_update.py",
                 "world_selection.py", "weather_grip.py", "road_weather.py", "windows_interop.py", "process_ownership.py",
                 "flat-asphalt.json", "MAP_PACKS.md", "author-wet-road-materials.py", "Icons"):
        copy(source / name, launch / name)
    copy(HERE / "asurt", stack / "bin/asurt")
    copy(HERE / "dev-build", stack / "bin/dev-build")
    (stack / "bin/asurt").chmod(0o755)
    copy(HERE / "windows-server.ps1", launch / "windows-server.ps1")
    copy(HERE / "novnc_viewer.py", launch / "novnc_viewer.py")
    command = (source / "session.py").read_text()
    command = command.replace('ROOT = Path.home() / "ASURT"', 'ROOT = Path(os.environ["ASURT_ROOT"])')
    command = command.replace('LOG_ROOT = ROOT / "log" / "carla"', 'LOG_ROOT = Path(os.environ["ASURT_STATE_ROOT"]) / "log/carla"')
    command = command.replace('PYTHON = ROOT / ".venvs/carla/bin/python"', 'PYTHON = Path(sys.executable)')
    command = command.replace('PROFILES = {"wslg": "WSLg", "novnc": "noVNC", "windows": "Native Windows"}', '''PROFILES = {"linux": "Ubuntu"}
if os.environ["ASURT_PROFILE"] == "windows":
    PROFILES = {"windows": "Native Windows"}
    if (CARLA_ROOT / "Runtime/Active/asurt-linux-flat").is_dir():
        PROFILES["wslg"] = "WSLg"
        if (LOG_ROOT.parents[2] / ".private-viewer").is_file():
            PROFILES["novnc"] = "noVNC"''')
    command = command.replace('MAPS = {"silverstone": "Carla/Maps/silverstone/silverstone", "flat-asphalt": "Carla/Maps/FlatAsphalt"}',
                              'MAPS = {"flat-asphalt": "Carla/Maps/FlatAsphalt"}')
    command = command.replace('MAP_LABELS = {"silverstone": "Silverstone", "flat-asphalt": "FlatAsphalt"}',
                              'MAP_LABELS = {"flat-asphalt": "FlatAsphalt"}')
    command = command.replace('map_path=MAPS["silverstone"]', 'map_path=MAPS["flat-asphalt"]')
    command = command.replace('display = ":1" if profile == "novnc" else ":0"',
                              'display = ":1" if profile == "novnc" else os.environ.get("DISPLAY", ":0")')
    command = command.replace('profile = data["profile"] if data else "wslg"', 'profile = data["profile"] if data else os.environ["ASURT_PROFILE"]')
    command = command.replace('    if args.profile == "auto" and args.action not in', '''    if args.profile == "auto" and args.action in ("stop", "status"):
        active = running_session()
        args.profile = active["profile"] if active else os.environ["ASURT_PROFILE"]
        if active and args.action == "stop":
            args.session_state = args.session_state or active["state"]
    if args.profile == "auto" and args.action not in''')
    command = command.replace('else (":1" if profile == "novnc" else ":0")',
                              'else (":1" if profile == "novnc" else os.environ.get("DISPLAY", ":0"))')
    command = command.replace('        window_env.pop("WAYLAND_DISPLAY", None)',
                              '        window_env.pop("WAYLAND_DISPLAY", None)\n'
                              '        window_env.pop("XAUTHORITY", None)')
    command = command.replace('pace_world_to_wall_time=profile in ("wslg", "novnc")',
                              'pace_world_to_wall_time=profile in ("linux", "wslg", "novnc")')
    a = command.index("def wake_novnc():")
    b = command.index("def stop_process(", a)
    command = command[:a] + '''def wake_novnc():
    from novnc_viewer import desktop
    return desktop(LOG_ROOT)


''' + command[b:]
    command = command.replace('        wake_novnc()\n    elif args.action in ("stop", "status"):',
                              '        from novnc_viewer import open_viewer\n        open_viewer(LOG_ROOT)\n'
                              '    elif args.action in ("stop", "status"):')
    command = command.replace('def start(name, args, *, environment=env):', 'def start(name, args, *, environment=env, cwd=None):')
    command = command.replace('proc = subprocess.Popen(args, env=environment, stdout=output, stderr=subprocess.STDOUT,',
                              'proc = subprocess.Popen(args, cwd=cwd, env=environment, stdout=output, stderr=subprocess.STDOUT,')
    a = command.index('            else:', command.index('        def server():'))
    b = command.index('        def shutdown(', a)
    command = command[:a] + '''            else:
                runtime = (CARLA_ROOT / "Runtime/Active/asurt-linux-flat").resolve(strict=True)
                if pack is not None:
                    install_map(pack, "LinuxNoEditor", runtime)
                session["runtime"] = str(runtime)
                binary = runtime / "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
                world = f"/Game/{map_path}"
                section = "[/Script/EngineSettings.GameMapsSettings]"
                defaults = f"-ini:Engine:{section}:GameDefaultMap={world},{section}:ServerDefaultMap={world}"
                launch = [binary, world, defaults, "-RenderOffScreen", "-quality-level=Low",
                          "-nosound", "-vulkan", "-carla-rpc-port=2000"]
                server_env = env
                if profile == "novnc":
                    env.update(wake_novnc())
                    env.pop("WAYLAND_DISPLAY", None)
                    env.pop("XAUTHORITY", None)
                kernel = os.uname().release.casefold()
                linux_on_wsl = profile == "linux" and ("microsoft-standard" in kernel or "wsl2" in kernel)
                if profile == "linux" and not linux_on_wsl:
                    from native_vulkan import native_vulkan_environment
                    server_env, renderer = native_vulkan_environment(env)
                    print(f"ASURT Ubuntu Vulkan renderer: {renderer}", flush=True)
                elif profile in ("wslg", "novnc") or linux_on_wsl:
                    dozen = Path.home() / ".local/opt/asurt-dozen-20261024/scripts/run-dozen.sh"
                    if not dozen.is_file():
                        raise RuntimeError("Mesa Dozen is missing. Rerun the ASURT installer in WSL.")
                    launch[:0] = [dozen, "--variant", "descriptor-reset-candidate"]
                    # Use a session-local Engine.ini for Shipping builds that ignore -ini.
                    cvars = {"r.RHICmdBypass": 0, "r.RHICmdUseThread": 1,
                             "r.RHICmdUseParallelAlgorithms": 1, "r.RHICmdUseDeferredContexts": 1,
                             "r.RHICmdWidth": 4, "r.MeshDrawCommands.ParallelPassSetup": 1,
                             "r.Vulkan.RHIThread": 2}
                    config_home = run_dir / "config"
                    config = config_home / "Epic/CarlaUE4/Saved/Config/LinuxNoEditor/Engine.ini"
                    config.parent.mkdir(parents=True, exist_ok=True)
                    values = "".join(f"{key}={value}\\n" for key, value in cvars.items())
                    config.write_text(f"{section}\\nGameDefaultMap={world}\\nServerDefaultMap={world}\\n"
                                      f"[ConsoleVariables]\\n{values}")
                    shader_cache = LOG_ROOT.parent.parent / "cache/dozen-v0.1.0"
                    shader_cache.mkdir(parents=True, exist_ok=True)
                    launch += ["-ForceRes", "-ResX=640", "-ResY=360", "-USEALLAVAILABLECORES",
                               "-ini:Engine:" + ",".join(f"[ConsoleVariables]:{k}={v}" for k, v in cvars.items())]
                    server_env = dict(env, XDG_CONFIG_HOME=str(config_home), SDL_VIDEODRIVER="x11",
                                      MESA_VK_WSI_DEBUG="noshm", MESA_SHADER_CACHE_DIR=str(shader_cache),
                                      DZN_CACHE_IMAGE_DESCRIPTORS="1", LD_LIBRARY_PATH="")
                start("server", launch, environment=server_env, cwd=runtime)
                try:
                    yield
                finally:
                    stop_process(processes["server"])

''' + command[b:]
    a = command.index('                        while time.monotonic() < deadline:')
    b = command.index('                    try:\n                        client, world, world_map', a)
    command = command[:a] + command[b:]
    compile(command, "session.py", "exec")
    (launch / "session.py").write_text(command)

    pack = launch / "map_packs.py"
    pack.write_text(pack.read_text().replace('Path.home() / "ASURT/Simulation/CARLA/Maps/Installed"',
                                             'Path(os.environ.get("ASURT_STATE_ROOT", Path.home() / ".local/share/asurt/state")) / "maps"'))
    shell = (HERE / "asurt").read_text().split('action="', 1)[0]
    (launch / "ros-parity-env.sh").write_text(shell + 'exec "$@"\n')


def relocate(stack, source):
    for path in stack.rglob("*"):
        if path.is_symlink() or not path.is_file() or path.suffix not in {
            ".py", ".yaml", ".yml", ".rviz", ".sh", ".urdf", ".dsv",
        }:
            continue
        raw = path.read_bytes()
        raw = raw.replace(b"/mnt/e/ASURT/Simulation/CARLA/Launchers", b"@ASURT_ROOT@/Simulation/CARLA/Launchers")
        raw = raw.replace(os.fsencode(source), b"@ASURT_ROOT@").replace(b"/home/hassan/ASURT", b"@ASURT_ROOT@")
        raw = raw.replace(b'$HOME/ASURT', b'$ASURT_ROOT')
        raw = raw.replace(b'Path.home() / "log/carla/tools"', b'Path(os.environ["ASURT_STATE_ROOT"]) / "log/carla/tools"')
        path.write_bytes(raw)


def adapt_inference(stack):
    for name in ("inference.py", "cuda-requirements.txt"):
        copy(HERE / name, stack / "bin" / name)
    systems = stack / "Simulation/Systems"
    planner = systems / "PathPlanning/Launch/run_deep_learning_planner.sh"
    text = "".join(line for line in planner.read_text().splitlines(keepends=True)
                   if not line.startswith(("ORT_DEPS=", "GPU_SITE=", "GPU_LIBS=")))
    start = text.index('if [[ -d "$GPU_SITE/onnxruntime" ]]; then')
    runner = '\n${ASURT_MODULE_RUNNER:-exec} ros2 run'
    end = text.index(runner if runner in text else '\nexec ros2 run', start)
    planner.write_text(text[:start] + text[end:].replace('"$@"', '-p "backend:=${ASURT_INFERENCE_BACKEND:-cpu}" "$@"'))
    perception = systems / "Perception"
    camera = perception / "Workspace/autonomy_integration/run_camera.sh"
    if "camera-inference-env.sh" not in camera.read_text():
        camera.write_text(camera.read_text().replace('export PYTHONPATH="$PERCEPTION',
                                                    'export PYTHONPATH="${ASURT_CUDA_SITE:+$ASURT_CUDA_SITE:}$PERCEPTION'))
    for detector in perception.rglob("yolo26.py"):
        text = detector.read_text()
        if "ASURT_INFERENCE_BACKEND" in text:
            continue
        text = text.replace('import math\n', 'import math\nimport os\n', 1)
        cpu = 'providers=["CPUExecutionProvider"])'
        if text.count(cpu) != 1:
            raise ValueError(f"Could not locate YOLO provider selection in {detector}")
        text = text.replace('        self.session = ort.InferenceSession', '''        provider = ("CUDAExecutionProvider" if os.environ.get("ASURT_INFERENCE_BACKEND") == "cuda"
                    else "CPUExecutionProvider")
        self.session = ort.InferenceSession''')
        detector.write_text(text.replace(cpu, '''providers=[provider])
        if provider not in self.session.get_providers():
            raise RuntimeError(f"YOLO26 could not initialize {provider}")'''))


def build_stack(args, output, workspace):
    source = args.source
    stack = workspace / "stack"
    stack.mkdir()
    for folder in MODULES:
        copy(source / folder, stack / folder)
    for tool in ("ROS Bags", "Control Center/control_center.py", "RViz/ROS_TOPICS.md"):
        copy(HERE.parent / "Tools" / tool, stack / "tools" / tool)
    copy(HERE.parent / "Tools/MCP", stack / "tools/MCP")
    copy(HERE.parent / "Tools/MCP", stack / "libexec/asurt-mcp")
    copy(HERE.parent / "Tools/Track Importer/import_cone_track.py", stack / "libexec/asurt-mcp/import_cone_track.py")
    if (source / "Assets/Props").is_dir():
        copy(source / "Assets/Props", stack / "Assets/Props")
    for folder in ("config", "scripts", "src"):
        bridge = "Simulation/CARLA/Bridge/Carla_ASU_Bridge"
        copy(source / bridge / folder, stack / bridge / folder)
    copy(REPOSITORY / BRIDGE_CONFIG, stack / BRIDGE / "config" / BRIDGE_CONFIG.name)
    for folder in OVERLAYS:
        copy_overlay(source / folder / "install", stack / folder / "install")
    for package in PYTHON_ADAPTERS:
        for name in ("node.py", "weather.py"):
            copy_python_adapter_file(package, name, stack)
    copy_driving_window(stack)
    common = Path(OVERLAYS[0]) / "src"
    for package in ("asurt_msgs", "tf_helper"):
        copy(source / common / package, stack / common / package)
    copy(source / DEP_PKG, stack / DEP_PKG)
    copy(source / LEGACY, stack / LEGACY)
    gtsam = "Simulation/Systems/SLAM/ThirdParty/gtsam-prefix"
    copy(source / gtsam, stack / gtsam)
    adapt_launchers(args.launchers, stack)
    for name in ("Install-ASURT.sh", "Install-ASURT.ps1"):
        copy(output / name, stack / "bin" / name)
    adapt_inference(stack)
    stage_fs26(stack)
    from asurt_update import prune_retired_packages
    prune_retired_packages(stack)
    client = source / ".venvs/carla/lib/python3.10/site-packages"
    for name in ("carla", "carla-0.9.16.dist-info"):
        copy(client / name, stack / "client" / name)
    # Include a standalone wheel for users writing additional CARLA clients.
    subprocess.run(["/usr/bin/python3", "-m", "wheel", "pack", str(stack / "client"), "-d", str(output)], check=True)
    relocate(stack, source)
    pack_directory(stack, output / "ASURT-Stack-Ubuntu-22.04-x64.tar.gz")


def pack_directory(directory, output, prefix=""):
    def normalize(info):
        if set(info.name.split("/")) & {"Saved", "Logs", "CrashReportClient"}:
            return None
        info.uid = info.gid = info.mtime = 0
        info.uname = info.gname = ""
        return info

    with output.open("wb") as raw, gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0, compresslevel=3) as zipped:
        with tarfile.open(fileobj=zipped, mode="w", dereference=True) as archive:
            archive.add(directory, arcname=prefix or ".", filter=normalize)


def write_installers(output, repository=None, branch="main"):
    shell = (REPOSITORY / "Install-ASURT.sh").read_text()
    windows = (REPOSITORY / "Install-ASURT.ps1").read_text()
    if repository:
        import re
        if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repository):
            raise ValueError("Expected a GitHub owner/repository.")
        shell = shell.replace(PRODUCTION_REPOSITORY, repository, 1)
        windows = windows.replace(PRODUCTION_REPOSITORY, repository, 1)
    if branch == "testing":
        shell = shell.replace("\nbranch=main\n", "\nbranch=testing\n", 1)
        windows = windows.replace("[string]$Branch='main'", "[string]$Branch='testing'", 1)
    payload = shell.encode()
    (output / "Install-ASURT.sh").write_bytes(payload)
    embedded = base64.b64encode(payload).decode()
    windows = windows.replace("__ASURT_EMBEDDED_LINUX_INSTALLER__", embedded, 1)
    (output / "Install-ASURT.ps1").write_text(windows, encoding="utf-8-sig")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--launchers", type=Path, required=True)
    parser.add_argument("--linux", type=Path)
    parser.add_argument("--windows", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tag", default="v0.1.0-flat")
    parser.add_argument("--repository", help="Bind both installers to a GitHub OWNER/REPO.")
    parser.add_argument("--branch", choices=("main", "testing"), default="main")
    parser.add_argument("--wsl-graphics", type=Path, help="Include the pinned Mesa Dozen WSL archive.")
    parser.add_argument("--reuse-runtimes-from", type=Path,
                        help="Use the simulator downloads from an earlier release manifest.")
    args = parser.parse_args()
    if not args.reuse_runtimes_from and (args.linux is None or args.windows is None):
        parser.error("Supply both staged runtimes or --reuse-runtimes-from.")
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    write_installers(output, args.repository, args.branch)
    with tempfile.TemporaryDirectory() as temporary:
        build_stack(args, output, Path(temporary))
    linux = output / "ASURT-FlatAsphalt-Ubuntu-22.04-x64.tar.gz"
    windows = output / "ASURT-FlatAsphalt-Windows-x64.tar.gz"
    if args.reuse_runtimes_from:
        previous = json.loads(args.reuse_runtimes_from.read_text())
        for key, target in (("linux", linux), ("windows", windows)):
            asset = previous["runtime"][key]
            source = args.reuse_runtimes_from.parent / asset["file"]
            if digest(source) != asset["sha256"]:
                raise ValueError(f"Previous {key} simulator archive failed verification.")
            shutil.copy2(source, target)
        runtimes = {key: dict(item(path), shipping_sha256=previous["runtime"][key]["shipping_sha256"])
                    for key, path in (("linux", linux), ("windows", windows))}
    else:
        pack_directory(args.linux, linux)
        pack_directory(args.windows / "WindowsNoEditor", windows, "WindowsNoEditor")
        runtimes = {"linux": dict(item(linux), shipping_sha256=digest(args.linux / "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping")),
                    "windows": dict(item(windows), shipping_sha256=digest(args.windows / "WindowsNoEditor/CarlaUE4/Binaries/Win64/CarlaUE4-Win64-Shipping.exe"))}
    copy(HERE / "asurt_update.py", output / "asurt_update.py")
    data = {"format": 1, "tag": args.tag, "updater": item(output / "asurt_update.py"),
            "linux_installer": item(output / "Install-ASURT.sh"),
            "stack": item(output / "ASURT-Stack-Ubuntu-22.04-x64.tar.gz"),
            "ubuntu_packages": (HERE / "ubuntu-packages.txt").read_text().splitlines(),
            "runtime": runtimes}
    if args.branch == "testing":
        data["channel"] = "testing"
    if args.wsl_graphics:
        driver = item(args.wsl_graphics)
        if driver["file"] != WSL_GRAPHICS_FILE or driver["sha256"] != WSL_GRAPHICS_SHA256:
            raise ValueError("The Mesa Dozen archive does not match the installer pin.")
        copy(args.wsl_graphics, output / WSL_GRAPHICS_FILE)
        data["wsl_graphics"] = driver
    (output / "asurt-release.json").write_text(json.dumps(data, indent=2) + "\n")
    (output / "asurt-maps.json").write_text(json.dumps({"format": 1, "maps": []}, indent=2) + "\n")


if __name__ == "__main__":
    main()
