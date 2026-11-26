#!/usr/bin/env python3
"""Own one CARLA episode and its independently replaceable ROS modules."""

import argparse
import contextlib
import fcntl
import json
import math
import os
from pathlib import Path
from queue import Queue
import re
import signal
import subprocess
import sys
import time

import carla
import yaml

from map_packs import catalog as map_catalog, check_base as check_map_base, install as install_map
from process_ownership import process_start, recover_orphaned_processes
from road_weather import RoadWeather
from weather_grip import WeatherGrip
from windows_interop import powershell_exe
from world_selection import select_world


ROOT = Path.home() / "ASURT"
CARLA_ROOT = ROOT / "Simulation/CARLA"
BRIDGE = CARLA_ROOT / "Bridge/Carla_ASU_Bridge"
TOOLS = ROOT / "tools"
LAUNCH = Path(__file__).resolve().parent
WSL_LAUNCH = CARLA_ROOT / "Launchers"
EVIDENCE = CARLA_ROOT / "Tests/Parity-20261121"
LOG_ROOT = ROOT / "log" / "carla"
PYTHON = ROOT / ".venvs/carla/bin/python"
PROFILES = {"wslg": "WSLg", "novnc": "noVNC", "windows": "Native Windows"}
RVIZ_PRESETS = {
    "rviz": "perception_combined.rviz",
    "zed-rviz": "zed2i_live.rviz",
    "slam-rviz": "slam_mapping.rviz",
    "slam-diagnostics": "slam_diagnostics.rviz",
    "planning-rviz": "planning_classical.rviz",
    "planning-classical-rviz": "planning_classical_mode.rviz",
    "planning-acceleration-rviz": "planning_acceleration_mode.rviz",
    "planning-skidpad-rviz": "planning_skidpad_mode.rviz",
    "planning-deep-learning-rviz": "planning_deep_learning_mode.rviz",
    "planning-goal-pose-rviz": "planning_goal_pose_mode.rviz",
    "control-rviz": "control_goal_pose.rviz",
    "track-control-rviz": "control_track_mode.rviz",
}
GOAL_RVIZ = {"planning-goal-pose-rviz", "control-rviz", "track-control-rviz"}
SESSION_REQUIRED_TOOLS = ("track", "weather", "weather-status", "cancel-goal")
LIVE_TOOLS = (*SESSION_REQUIRED_TOOLS, *RVIZ_PRESETS)
MAPS = {"flat-asphalt": "Carla/Maps/FlatAsphalt"}
MAP_LABELS = {"flat-asphalt": "FlatAsphalt"}
MAP_PACKS = map_catalog()
MAPS.update({key: pack.map_path for key, pack in MAP_PACKS.items()})
MAP_LABELS.update({key: pack.label for key, pack in MAP_PACKS.items()})
HERO = "vehicle.vehicle.asurt_fsai"
CONE_TYPES = {
    "static.prop.asurtbluecone", "static.prop.asurtyellowcone",
    "static.prop.asurtorangecone", "static.prop.asurtbigorangecone",
}


class RendererStartupError(RuntimeError):
    """The owned Linux renderer exited before the session reached ready."""


def windows_host():
    with contextlib.suppress(OSError, subprocess.SubprocessError):
        mode = subprocess.check_output(["wslinfo", "--networking-mode"], text=True, timeout=5).strip()
        if mode == "mirrored":
            return "127.0.0.1"
    routes = subprocess.check_output(["ip", "-j", "route", "show", "default", "dev", "eth0"],
                                     text=True, timeout=5)
    gateway = next((entry["gateway"] for entry in json.loads(routes) if "gateway" in entry), None)
    if not gateway:
        raise RuntimeError("WSL cannot find the Windows host. Check WSL networking before launching Native Windows.")
    return gateway


def windows_flat_pointer():
    return (LAUNCH.parent / "Runtime/Active/asurt-win64-flat.txt").read_text().strip()


def windows_runtime_directory(path):
    return Path(subprocess.check_output(["wslpath", "-u", path], text=True).strip()) / "WindowsNoEditor"


def linux_port_is_listening(port):
    for path in ("/proc/net/tcp", "/proc/net/tcp6"):
        for line in Path(path).read_text().splitlines()[1:]:
            fields = line.split()
            if fields[3] == "0A" and int(fields[1].split(":")[1], 16) == port:
                return True
    return False


def foreign_linux_stack():
    for name in Path("/proc").glob("[0-9]*/cmdline"):
        with contextlib.suppress(OSError, IndexError):
            command = name.read_bytes().split(b"\0", 1)[0]
            if command.endswith((b"CarlaUE4-Linux-Shipping", b"carla_telemetry_node", b"UE4Editor")):
                return True
    return any(linux_port_is_listening(port) for port in (2000, 2001, 2002))


def session_file():
    return LOG_ROOT / "active-session.json"


def running_session():
    try:
        data = json.loads(session_file().read_text())
        if data["start"] and data["start"] == process_start(data["pid"]):
            return data
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return None


def last_session_failure():
    try:
        return json.loads((LOG_ROOT / "session-start-error.json").read_text())
    except (OSError, ValueError):
        return None


def launch_error(log, offset, exit_code):
    try:
        with log.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(offset, stream.tell() - 4096))
            lines = stream.read().decode(errors="replace").strip().splitlines()
        if lines:
            message = re.sub(r"^[A-Za-z]+Error: ", "", lines[-1])
            return f"{message}\nLogs: {log}"
    except OSError:
        pass
    return f"Launch exited with code {exit_code}. Logs: {log}"


def record_session(data):
    path = session_file()
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)


def wake_novnc():
    sys.path.insert(0, str(WSL_LAUNCH / "Dozen"))
    from scoped_vnc_compositor import desktop
    _, desktop_env = desktop()
    env = dict(os.environ, **desktop_env)
    # ICC device registration is unnecessary for the virtual Xvnc display.
    # Stop its per-display color client so it cannot leave a polkit prompt
    # over the simulator. The system color and authentication services stay up.
    for proc in Path("/proc").glob("[0-9]*"):
        with contextlib.suppress(OSError, ValueError):
            if (proc / "exe").resolve(strict=True) == Path("/usr/bin/xiccd"):
                environ = (proc / "environ").read_bytes().split(b"\0")
                if b"DISPLAY=:1" in environ or b"DISPLAY=:1.0" in environ:
                    os.kill(int(proc.name), signal.SIGTERM)
    subprocess.run(["xset", "s", "reset"], env=env, timeout=5, check=True)
    subprocess.run(["xfce4-screensaver-command", "--deactivate"], env=env, timeout=5, check=True)
    return desktop_env


def stop_process(proc):
    if proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 10), (signal.SIGTERM, 8), (signal.SIGKILL, 5)):
        with contextlib.suppress(ProcessLookupError):
            os.killpg(proc.pid, sig)
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue


def run_windows(action, runtime, identity="", *, map_path=MAPS["flat-asphalt"]):
    script = subprocess.check_output(["wslpath", "-w", LAUNCH / "windows-server.ps1"], text=True).strip()
    args = [powershell_exe(), "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", script,
            "-Action", action, "-Runtime", runtime, "-Map", f"/Game/{map_path}"]
    if identity:
        args += ["-Identity", identity]
    result = subprocess.run(args, capture_output=True, text=True, timeout=35)
    if result.returncode:
        raise RuntimeError(f"Windows CARLA {action} failed: {result.stderr or result.stdout}")
    return result.stdout.strip()


def load_world(host, port, deadline, ensure_alive):
    last_error = ""
    while time.monotonic() < deadline:
        ensure_alive()
        try:
            client = carla.Client(host, port)
            client.set_timeout(4)
            client.get_server_version()
            client.set_timeout(15)
            world = client.get_world()
            world_map = world.get_map()
            return client, world, world_map
        except RuntimeError as exc:
            last_error = str(exc)
            time.sleep(1)
    raise TimeoutError(f"CARLA {host}:{port} did not reach world readiness: {last_error}")


def check_empty_episode(world):
    props = {blueprint.id for blueprint in world.get_blueprint_library()}
    missing = CONE_TYPES - props
    if missing:
        raise RuntimeError(f"Missing FSAI cone registrations: {sorted(missing)}")
    actors = world.get_actors()
    if any(a.attributes.get("role_name", "").startswith("asurt_cone_")
           or a.attributes.get("role_name") == "hero" for a in actors):
        raise RuntimeError("Expected a new CARLA episode without a hero or imported cones")


def call(log, args, *, env=None, timeout=150):
    with log.open("a") as output:
        subprocess.run(args, stdout=output, stderr=subprocess.STDOUT, check=True, env=env, timeout=timeout)


def configure_bridge(run_dir, host, port, blueprint, map_path, flat, *, pace_world_to_wall_time):
    path = run_dir / "carla_interface_config.yaml"
    cfg = yaml.safe_load((BRIDGE / "config/carla_interface_config.yaml").read_text())
    cfg["carla"].update(host=host, port=port, timeout=30.0 if port == 2100 else 10.0,
                        open_manual_control=False, pace_world_to_wall_time=pace_world_to_wall_time)
    cfg["world"]["town"] = map_path.rsplit("/", 1)[-1]
    if blueprint.has_attribute("generation"):
        cfg["vehicle"]["generation"] = str(blueprint.get_attribute("generation").as_int())
    if flat:
        cfg["world"]["spawn_point_coords"] = flat["spawn"]
    elif port == 2100:
        # Preserve the parked spawn supported by the PC's native Silverstone cook.
        cfg["world"]["spawn_point_coords"].update(x=-384.3957214355469, y=234.75315856933594, yaw=-57.332)
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return path, cfg["vehicle"]["generation"]


def cone_type_id(actor):
    if actor.type_id == "static.prop.mesh":
        mesh = actor.attributes.get("mesh_path", "")
        if mesh.startswith("/Game/ASURTCones/"):
            return "static.prop." + mesh.rsplit(".", 1)[-1].lower()
    return actor.type_id


def validate_course(world, state):
    ref = json.loads((EVIDENCE / "laptop-world-transforms-reference.json").read_text())["cones"]
    registered = json.loads(state.read_text())["actors"]
    actors = [a for a in world.get_actors()
              if a.attributes.get("role_name", "").startswith("asurt_cone_")]
    if len(actors) != 100 or len(registered) != 100:
        raise RuntimeError(f"Expected 100 actors and 100 owned IDs, got {len(actors)} and {len(registered)}")
    gaps = []
    for actor in actors:
        pose = actor.get_transform()
        kind = cone_type_id(actor)
        same_type = [cone for cone in ref if cone["type_id"] == kind]
        matched = min(same_type, key=lambda cone: (cone["transform"]["x"] - pose.location.x)**2
                      + (cone["transform"]["y"] - pose.location.y)**2)
        ref.remove(matched)
        nearest = matched["transform"]
        dx, dy, dz = (abs(nearest[key] - getattr(pose.location, key)) for key in ("x", "y", "z"))
        gaps.append((dx, dy, dz))
    largest = tuple(max(values) for values in zip(*gaps))
    print(f"COURSE_MAX_ABSOLUTE_DELTA_M={largest}", flush=True)
    if largest[0] > 0.01 or largest[1] > 0.01 or largest[2] > 0.03:
        raise RuntimeError(f"Course differs from laptop transform reference: {largest}")
    return largest


def capture_fixture(world, run_dir):
    actors = [a for a in world.get_actors() if cone_type_id(a) in CONE_TYPES]
    if {cone_type_id(a) for a in actors} != CONE_TYPES or len(actors) != 4:
        raise RuntimeError("Expected one physical actor of each FSAI cone type")
    collision = {}
    for actor in actors:
        point = actor.get_location()
        center = point.z + 0.10
        top = point.z + 2 * actor.bounding_box.extent.z + 0.08
        spans = []
        for z in (center, top):
            hits = world.cast_ray(carla.Location(point.x - 0.24, point.y, z),
                                  carla.Location(point.x + 0.24, point.y, z))
            spans.append([(str(hit.label), hit.location.x - point.x) for hit in hits])
        collision[cone_type_id(actor)] = {"through": spans[0], "above": spans[1]}
    (run_dir / "fixture-rays.json").write_text(json.dumps(collision, indent=2))
    if any(not item["through"] or item["above"] for item in collision.values()):
        raise RuntimeError(f"Cone collision ray check failed. See {run_dir / 'fixture-rays.json'}")
    points = [a.get_location() for a in actors]
    center = carla.Location(**{axis: sum(getattr(point, axis) for point in points) / len(points)
                               for axis in ("x", "y", "z")})
    line = max(points, key=lambda p: p.distance(center)) - center
    length = math.hypot(line.x, line.y)
    direction_x, direction_y = -line.y / length, line.x / length
    camera_location = carla.Location(center.x + direction_x * 3.4, center.y + direction_y * 3.4,
                                     center.z + 0.95)
    yaw = math.degrees(math.atan2(-direction_y, -direction_x))
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "960")
    bp.set_attribute("image_size_y", "540")
    bp.set_attribute("sensor_tick", "0.2")
    camera = world.spawn_actor(bp, carla.Transform(camera_location, carla.Rotation(pitch=-12, yaw=yaw)))
    frames = Queue()
    try:
        camera.listen(frames.put)
        for _ in range(3):
            frame = frames.get(timeout=20)
        frame.save_to_disk(str(run_dir / "fixture-cones.png"))
    finally:
        camera.stop()
        camera.destroy()


def install_course(session, world, log, acceptance, env, flat):
    endpoint = ["--host", session["host"], "--port", str(session["port"])]
    importer = [PYTHON, TOOLS / "Track Importer/import_cone_track.py", *endpoint]
    verifier = [PYTHON, EVIDENCE / "verify_fsai_runtime_cones.py", *endpoint]
    origin = flat["course_origin"] if flat else {"x": -138.4876, "y": -22.2655, "z": -1.4869, "yaw": -162.4316}
    coords = ["--anchor", "coords", "--origin-x", str(origin["x"]), "--origin-y", str(origin["y"]),
              "--origin-z", str(origin["z"]), "--yaw", str(origin["yaw"]), "--invert-y", "--ground", "project"]
    if acceptance:
        call(log, [*importer, "--mode", "blueprints"])
        fixture_state = Path(session["log"]) / "fixture-csv-actors.json"
        fixture = [*importer, EVIDENCE / "asurt_cone_acceptance.csv", *coords, "--state", fixture_state]
        call(log, [*fixture, "--mode", "preview", "--marker-seconds", "0.2"])
        call(log, [*fixture, "--mode", "spawn"])
        world.wait_for_tick(12)
        call(log, [*verifier, "--expected-course-count", "4"])
        capture_fixture(world, Path(session["log"]))
        call(log, [*importer, "--mode", "clear", "--state", fixture_state])
        world.wait_for_tick(12)
        call(log, [*verifier, "--expected-course-count", "0"])
        gui_test = LAUNCH.parent / "Tests/Parity-20261121/exercise_importer_gui.py"
        gui_state = Path(session["log"]) / "fixture-json-actors.json"
        call(log, [PYTHON, gui_test, EVIDENCE / "asurt_cone_acceptance.json", gui_state], env=env)
        world.wait_for_tick(12)
        call(log, [*verifier, "--expected-course-count", "0"])
    if flat:
        return None
    course = [*importer, ROOT / "Assets/Maps/CSV/test1_3m_clear.csv", *coords, "--forward-offset", "12",
              "--state", session["state"]]
    call(log, [*course, "--mode", "preview", "--marker-seconds", "0.2"])
    call(log, [*course, "--mode", "spawn"])
    world.wait_for_tick(12)
    call(log, [*verifier, "--expected-course-count", "100"])
    return validate_course(world, Path(session["state"]))


def supervisor(profile, candidate, acceptance, packed_candidate=False, map_key="flat-asphalt"):
    map_path = MAPS[map_key]
    pack = MAP_PACKS.get(map_key)
    if pack is not None:
        flat = pack.placement
    elif map_key == "flat-asphalt":
        flat = json.loads((LAUNCH / "flat-asphalt.json").read_text())
    else:
        flat = None
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    with (LOG_ROOT / "session.lock").open("a") as lock, contextlib.ExitStack() as locks:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            running = running_session()
            if running and running["profile"] == profile and running.get("map_key", "silverstone") == map_key:
                print(f"ASURT {profile} {MAP_LABELS[map_key]} already running. Logs: {running['log']}")
                return
            raise RuntimeError("An ASURT simulator is already starting or running. Use its Stop shortcut first.")
        if running_session() is not None:
            raise RuntimeError("An ASURT simulator is already running. Stop it before starting a new session.")
        recovered = recover_orphaned_processes(LOG_ROOT)
        if recovered:
            print(f"Stopped {recovered} ASURT process(es) left by a previous session.", flush=True)
        if foreign_linux_stack():
            raise RuntimeError("Another CARLA server or ROS bridge is running. Close it through its launcher before starting ASURT.")
        domain = int(os.environ.get("ROS_DOMAIN_ID", "46"))
        domain_lock = locks.enter_context((LOG_ROOT / f"ros-domain-{domain}.lock").open("a"))
        try:
            fcntl.flock(domain_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f"ROS domain {domain} is in use by ASURT bag playback. Stop playback or choose a different domain.") from None

        sys.path.insert(0, str(TOOLS / "RViz"))
        from module_control import CONFIG, module_plan
        from managed_modules import ManagedModules

        modules, _, enabled = module_plan()
        module_names = {module.name for module in modules}
        if module_names & {"server", "bridge", "driving", "viewer-inhibitor"}:
            raise ValueError("Module registry conflicts with a core simulator process")
        perception_modules = [
            module for module in modules
            if module.script is not None and module.group == "Perception" and enabled[module.name]
        ]
        run_dir = LOG_ROOT / (time.strftime("%Y%m%d-%H%M%S") + f"-{profile}-{map_key}-{os.getpid()}")
        run_dir.mkdir()
        display = ":1" if profile == "novnc" else ":0"
        host, port = "127.0.0.1", 2100 if profile == "windows" else 2000
        env = dict(os.environ, DISPLAY=display, ROS_DOMAIN_ID=str(domain),
                   SDL_VIDEODRIVER="x11", SDL_AUDIODRIVER="dummy",
                   ASURT_CARLA_HOST=host, ASURT_CARLA_PORT=str(port), ASURT_CLIENT_BUSY_WAIT="0")
        session = dict(profile=profile, map_key=map_key, host=host, port=port, display=display, pid=os.getpid(),
                       ros_domain_id=domain,
                       module_config=str(CONFIG.resolve()),
                       start=process_start(os.getpid()), log=str(run_dir),
                       state=str(run_dir / "course-actors.json"), phase="starting")
        env.update(ASURT_CONE_STATE=session["state"],
                   ASURT_SESSION_LOG=str(run_dir),
                   ASURT_ROAD_WEATHER_DIR=str(run_dir),
                   ASURT_LEGACY_CONE_STATE=str(BRIDGE / "scripts/.cone_track_actors.json"))
        if flat:
            session["course_origin"] = flat["course_origin"]
            env["ASURT_COURSE_ORIGIN"] = json.dumps(flat["course_origin"])
        session["perception_modules"] = [module.name for module in perception_modules]
        record_session(session)
        (run_dir / "session.json").write_text(session_file().read_text())
        processes = {}
        windows_identity = ""
        windows_probe_after = 0.0
        windows_runtime = ""

        def start(name, args, *, environment=env):
            with (run_dir / f"{name}.log").open("ab") as output:
                proc = subprocess.Popen(args, env=environment, stdout=output, stderr=subprocess.STDOUT,
                                        start_new_session=True)
            processes[name] = proc

        def ensure_alive():
            nonlocal windows_probe_after
            if windows_identity and time.monotonic() >= windows_probe_after:
                windows_probe_after = time.monotonic() + 4.0
                try:
                    run_windows("ready", windows_runtime, windows_identity)
                except RuntimeError as exc:
                    raise RuntimeError(f"Native Windows CARLA exited. See {run_dir} for startup details.") from exc
            for name, proc in processes.items():
                if name in module_names:
                    continue
                if proc.poll() is not None:
                    log = run_dir / f"{name}.log"
                    message = (f"{name} exited with {proc.returncode}. {log}\n"
                               f"{log.read_text(errors='replace')[-2500:]}")
                    if name == "server" and session["phase"] == "starting":
                        raise RendererStartupError(message)
                    raise RuntimeError(message)

        @contextlib.contextmanager
        def server():
            nonlocal windows_identity
            if profile == "windows":
                if pack is not None:
                    install_map(pack, "WindowsNoEditor", windows_runtime_directory(windows_runtime))
                windows_identity = run_windows("start", windows_runtime, map_path=map_path)
                try:
                    yield
                finally:
                    run_windows("stop", windows_runtime, windows_identity)
            else:
                sys.path.insert(0, str(WSL_LAUNCH / "Dozen"))
                scope = contextlib.nullcontext()
                if profile == "novnc":
                    env.update(wake_novnc(), QT_QPA_PLATFORM="xcb")
                    env.pop("WAYLAND_DISPLAY", None)
                    from scoped_vnc_compositor import scoped
                    scope = scoped(run_dir)
                old = Path.home() / "asurt"
                if flat:
                    selection = "Active/asurt-linux-flat"
                elif packed_candidate:
                    selection = "Candidates/asurt-linux-packed-20261121"
                elif candidate:
                    selection = "Candidates/asurt-linux-20261121"
                else:
                    selection = "Active/asurt-linux"
                runtime = CARLA_ROOT / "Runtime" / selection
                if pack is not None:
                    install_map(pack, "LinuxNoEditor", runtime)
                session["runtime"] = str(runtime.resolve())
                gpu = dict(env, ASURT_LINUX_RUNTIME=str(runtime), ASURT_SESSION_LOGS=str(run_dir),
                           CARLA_DRIVER_PREFIX=str(
                               old / "mesa-prefixes/descriptor-reset-candidate-20261024"),
                           CARLA_SHADER_CACHE_DIR=str(old / "cache/descriptor-reset-candidate-20261024"),
                           CARLA_UNIFORM_SNAPSHOTS="1", CARLA_STAGE_DESCRIPTORS="1",
                           CARLA_CACHED_UNIFORMS="1", CARLA_POOL_UPLOADS="1",
                           CARLA_POOL_CPU_DESCRIPTOR_HEAPS="1", CARLA_UPLOAD_POOL_MAX_MB="512",
                           CARLA_PARK_SPECTATOR="1", CARLA_QUIESCE_MAP_LOAD="1")
                # Use the same image descriptor cache for Silverstone on both displays.
                if profile == "wslg" or pack is not None:
                    gpu["CARLA_CACHE_IMAGE_DESCRIPTORS"] = "1"
                with scope:
                    try:
                        if profile == "novnc":
                            start("viewer-inhibitor", ["xfce4-screensaver-command", "--inhibit",
                                  "--application-name=ASURT CARLA",
                                  "--reason=Keep the active simulator visible"])
                        startup_map = MAPS["flat-asphalt"] if pack is not None else map_path
                        start("server", ["/usr/bin/python3", WSL_LAUNCH / "Dozen/launch_gpu.py", "asurt",
                                         "vnc" if profile == "novnc" else "wslg", "--map", f"/Game/{startup_map}",
                                         "--offscreen", "--no-panel",
                                         "--x11-transport", "copy", "--threading", "parallel", "--rhi-width", "4"],
                              environment=gpu)
                        yield
                    finally:
                        for name in ("server", "viewer-inhibitor"):
                            if name in processes:
                                stop_process(processes[name])

        def shutdown(_sig, _frame):
            raise KeyboardInterrupt

        for sig in (signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, shutdown)
        startup_error = LOG_ROOT / "session-start-error.json"
        startup_error.unlink(missing_ok=True)

        def report_failure(exc):
            session["phase"] = "failed"
            record_session(session)
            temporary = startup_error.with_suffix(".next")
            temporary.write_text(json.dumps({"message": str(exc).splitlines()[0],
                                             "log": str(run_dir), "map_key": map_key, "profile": profile}))
            temporary.replace(startup_error)

        try:
            if profile == "windows":
                host = windows_host()
                env["ASURT_CARLA_HOST"] = session["host"] = host
                windows_runtime = windows_flat_pointer()
                session["runtime"] = windows_runtime
                record_session(session)
            print(f"ASURT {profile} at {host}:{port}. Logs: {run_dir}", flush=True)
            with server():
                try:
                    deadline = time.monotonic() + (100 if profile == "windows" else 420)
                    if profile != "windows":
                        while not linux_port_is_listening(port) and time.monotonic() < deadline:
                            ensure_alive()
                            time.sleep(1)
                        while time.monotonic() < deadline:
                            ensure_alive()
                            if "READY — Simulator server running." in (run_dir / "server.log").read_text(errors="replace"):
                                break
                            time.sleep(1)
                    try:
                        client, world, world_map = load_world(host, port, deadline, ensure_alive)
                    except TimeoutError as exc:
                        if profile == "windows":
                            raise RuntimeError("Native CARLA started but WSL could not reach its RPC port. "
                                               "Rerun the installer to repair Windows firewall access.") from exc
                        raise
                    world, world_map = select_world(client, world, world_map, map_path)
                    print(f"ROAD_READY map={world_map.name} client={client.get_client_version()} "
                          f"server={client.get_server_version()}", flush=True)
                    check_empty_episode(world)
                    cfg, generation = configure_bridge(
                        run_dir, host, port, world.get_blueprint_library().find(HERO),
                        map_path, flat, pace_world_to_wall_time=profile in ("wslg", "novnc"),
                    )
                    env["ASURT_BRIDGE_CONFIG"] = str(cfg)
                    call(run_dir / "hero.log", [PYTHON, BRIDGE / "scripts/spawn_asurt_hero.py", "--config", cfg], env=env)
                    start("bridge", ["ros2", "launch", "carla_telemetry_cpp", "carla_telemetry.launch.py",
                                     f"config_file:={cfg}", "open_manual_control:=false", "auto_start:=true"])
                    deadline = time.monotonic() + (125 if profile == "windows" else 70)
                    while time.monotonic() < deadline:
                        ensure_alive()
                        try:
                            state = subprocess.run(["ros2", "lifecycle", "get", "--no-daemon",
                                                   "/ASU_RT_Carla_Telemetry_Node"],
                                                   capture_output=True, text=True, timeout=8)
                            if "active [3]" in state.stdout:
                                break
                        except subprocess.TimeoutExpired:
                            ensure_alive()
                        time.sleep(1)
                    else:
                        raise RuntimeError(f"The C++ bridge did not become active. See {run_dir / 'bridge.log'}")
                    settings = world.get_settings()
                    if not (settings.synchronous_mode and abs(settings.fixed_delta_seconds - 0.033) < 1e-6):
                        raise RuntimeError(f"Unexpected world settings: {settings}")
                    hero = next(actor for actor in world.get_actors().filter(HERO)
                                if actor.attributes.get("role_name") == "hero")
                    weather_grip = WeatherGrip(world, hero)
                    road_weather = RoadWeather(run_dir)
                    road_weather.update(world)
                    delta = install_course(session, world, run_dir / "course.log", acceptance, env, flat)
                    managed = ManagedModules(modules, processes, start, stop_process)
                    for module in perception_modules:
                        try:
                            start(module.name, module.command())
                        except OSError as exc:
                            managed.failed(module.name, f"Launch failed: {exc}", time.monotonic())
                    start("driving", [PYTHON, "-m", "carla_telemetry.manual_control", "--host", host,
                                      "-p", str(port), "--rolename", "hero", "--filter", HERO, "--generation",
                                      generation, "--res", "800x450", "--render-rate", "24", "--attach"])
                    deadline = time.monotonic() + 65
                    while True:
                        ensure_alive()
                        world.wait_for_tick(12)
                        if (run_dir / "driving-ready").is_file():
                            break
                        if time.monotonic() >= deadline:
                            raise RuntimeError(f"The driving window received no camera frame. See {run_dir / 'driving.log'}")
                        time.sleep(0.4)
                    session.update(phase="ready", client=client.get_client_version(),
                                   server=client.get_server_version(), map=world_map.name, course_delta=delta)
                    record_session(session)
                    (run_dir / "session.json").write_text(session_file().read_text())
                    print(f"ASURT {profile} ready. Close the driving window or use Stop to end this session.", flush=True)
                    report = run_dir / "module-status.json"
                    previous = None
                    config_error = None
                    weather_error = None
                    while True:
                        if processes["driving"].poll() == 0:
                            return
                        ensure_alive()
                        try:
                            weather = road_weather.update(world)
                            grip = weather_grip.update(weather, road_weather.tire_c)
                            weather_error = None
                            if grip is not None:
                                print(f"Road weather grip multiplier: {grip:.2f}", flush=True)
                        except (RuntimeError, OSError, ValueError) as exc:
                            if str(exc) != weather_error:
                                weather_error = str(exc)
                                print(f"Road weather grip update failed: {exc}", flush=True)
                        try:
                            latest_modules, _, updated = module_plan()
                            managed.reconfigure(latest_modules)
                            module_names = {module.name for module in latest_modules}
                            enabled = updated
                            config_error = None
                        except (OSError, ValueError, TypeError, KeyError) as exc:
                            if str(exc) != config_error:
                                config_error = str(exc)
                                print(f"Keeping previous module switches: {config_error}", flush=True)
                        managed.reconcile(enabled)
                        current = managed.snapshot()
                        if current != previous:
                            temporary = report.with_suffix(".tmp")
                            temporary.write_text(json.dumps(current, indent=2) + "\n")
                            temporary.replace(report)
                            previous = current
                        time.sleep(1)
                except (RuntimeError, OSError, subprocess.SubprocessError, ValueError) as exc:
                    report_failure(exc)
                    raise
                finally:
                    for name, proc in reversed(tuple(processes.items())):
                        if name != "server":
                            stop_process(proc)
        except KeyboardInterrupt:
            print("ASURT stopped.", flush=True)
        except (RuntimeError, OSError, subprocess.SubprocessError, ValueError) as exc:
            if session["phase"] != "failed":
                report_failure(exc)
            raise
        finally:
            try:
                recover_orphaned_processes(LOG_ROOT, run_dir=run_dir)
            finally:
                session_file().unlink(missing_ok=True)


def tool_path(action, default):
    config_home = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    settings = config_home / "asurt/tool-paths.json"
    try:
        overrides = json.loads(settings.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return Path(default)
    if not isinstance(overrides, dict):
        raise ValueError(f"Tool paths must be a JSON object in {settings}.")
    if action not in overrides:
        return Path(default)
    value = overrides[action]
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Tool {action} needs a file path in {settings}.")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(os.environ.get("ASURT_WORKSPACE", ROOT)) / path
    if not path.is_file():
        raise FileNotFoundError(f"Configured entry for {action} does not exist: {path}")
    return path


def tool(profile, action, arguments, expected_session=None):
    standalone = action == "trackdraw"
    data = None if standalone else running_session()
    automatic = profile == "auto"
    if automatic:
        if expected_session is not None and (not data or data["state"] != expected_session):
            raise RuntimeError("The simulator changed. Open the tool again to use the current session.")
        profile = data["profile"] if data else "wslg"
    display = data["display"] if data and data["profile"] == profile else (":1" if profile == "novnc" else ":0")
    if action in SESSION_REQUIRED_TOOLS:
        if not data or data["phase"] != "ready":
            raise RuntimeError("Start a simulator and wait until it is ready.")
        if data["profile"] != profile:
            raise RuntimeError(f"Use the {data['profile']} tool shortcut for the running simulator.")
    if standalone:
        display = os.environ.get("DISPLAY", display)
    window_env = dict(os.environ, DISPLAY=display)
    if profile == "novnc":
        window_env.pop("WAYLAND_DISPLAY", None)
        window_env["QT_QPA_PLATFORM"] = "xcb"
    if data and data["profile"] == profile:
        window_env.update(ASURT_CARLA_HOST=data["host"], ASURT_CARLA_PORT=str(data["port"]))
        if "course_origin" in data:
            window_env["ASURT_COURSE_ORIGIN"] = json.dumps(data["course_origin"])
    if action == "track":
        script = tool_path(action, TOOLS / "Track Importer/cone_track_gui.py")
        if not arguments:
            arguments = [str(ROOT / "Assets/Maps/CSV/test1_3m_clear.csv")]
        if "--state" not in arguments:
            arguments += ["--state", data["state"]]
        cmd = [PYTHON, script, *arguments]
    elif action in ("weather", "weather-status"):
        stock = TOOLS / "Weather Editor/carla_weather_gui.py"
        cmd = [PYTHON, stock, "--status"] if action == "weather-status" else [PYTHON, tool_path(action, stock)]
        if data and data["profile"] == profile:
            window_env["ASURT_ROAD_WEATHER_DIR"] = data["log"]
        window_env["PYTHONPATH"] = str(LAUNCH) + os.pathsep + window_env.get("PYTHONPATH", "")
    elif action in RVIZ_PRESETS:
        cmd = ["rviz2", "-d", tool_path(action, TOOLS / "RViz/config" / RVIZ_PRESETS[action])]
        if action in GOAL_RVIZ:
            cmd += ["--ros-args", "-p", "use_sim_time:=true"]
    elif action == "cancel-goal":
        cmd = ["bash", tool_path(action, TOOLS / "RViz/cancel_goal.sh")]
    elif action == "manager":
        cmd = [PYTHON, tool_path(action, TOOLS / "Control Center/control_center.py")]
        if os.environ.get("ASURT_PORTABLE"):
            cmd = [PYTHON, LAUNCH / "release_update.py", "--manager", *cmd]
    elif action == "map-packs":
        cmd = [PYTHON, tool_path(action, LAUNCH / "map_pack_panel.py")]
    elif action == "ros-bags":
        window_env.update(ASURT_LAUNCHERS=str(LAUNCH))
        cmd = [PYTHON, tool_path(action, TOOLS / "ROS Bags/ros_bags.py")]
    elif action == "trackdraw":
        trackdraw = TOOLS / "Trackdraw/fs-trackdraw"
        os.chdir(trackdraw)
        portable = bool(os.environ.get("ASURT_PORTABLE"))
        interpreter = PYTHON if portable else trackdraw / ".venv/bin/python"
        cmd = [interpreter, tool_path(action, trackdraw / "src/main.py")]
        if portable:
            for variable in ("QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH", "LD_LIBRARY_PATH"):
                window_env.pop(variable, None)
            window_env.update(PYTHONPATH="/usr/lib/python3/dist-packages", PYTHONNOUSERSITE="1")
            if profile in ("windows", "wslg", "novnc"):
                window_env["QT_QPA_PLATFORM"] = "xcb"
    else:
        raise ValueError(action)
    directory = LOG_ROOT / "tools"
    directory.mkdir(parents=True, exist_ok=True)
    bound = action in LIVE_TOOLS and data and data["profile"] == profile and data["phase"] == "ready"
    lock_name = action if action in (*RVIZ_PRESETS, "manager", "ros-bags", "trackdraw") else f"{action}-{profile}"
    if bound and action not in RVIZ_PRESETS:
        lock_name = f"{lock_name}-{Path(data['state']).parent.name}"
    with (directory / f"{lock_name}.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(f"{action} is already open for {profile}.")
            return
        with (directory / f"{action}-{profile}.log").open("ab") as out:
            run_session_tool(cmd, window_env, out, data["state"] if bound else None, action)


def run_session_tool(cmd, env, output, state, action):
    """Track launched tool windows and close tools bound to an ended session."""
    directory = LOG_ROOT / "tools/running"
    directory.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(cmd, env=env, stdout=output, stderr=subprocess.STDOUT,
                               start_new_session=True)
    marker = directory / f"{process.pid}.json"
    temporary = marker.with_suffix(".next")
    try:
        temporary.write_text(json.dumps({"action": action, "start": process_start(process.pid)}))
        temporary.replace(marker)
        while True:
            if state is not None:
                active = running_session()
                if not active or active["state"] != state:
                    return
            try:
                code = process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                continue
            if code:
                raise RuntimeError(f"The tool exited with code {code}. See {output.name}.")
            return
    finally:
        stop_process(process)
        temporary.unlink(missing_ok=True)
        marker.unlink(missing_ok=True)


def stop_for_update():
    active = running_session()
    if active:
        with contextlib.suppress(ProcessLookupError):
            os.kill(active["pid"], signal.SIGTERM)
        deadline = time.monotonic() + 45
        while (current := running_session()) is not None:
            if current["state"] != active["state"]:
                raise RuntimeError("A different simulator session started during shutdown.")
            if time.monotonic() >= deadline:
                raise TimeoutError("The ASURT simulator is still shutting down.")
            time.sleep(.25)

    def owned_tools(*, bag_windows=False):
        for path in (LOG_ROOT / "tools/running").glob("*.json"):
            try:
                pid = int(path.stem)
                entry = json.loads(path.read_text())
                if (entry["action"] == "manager" or (entry["action"] == "ros-bags") != bag_windows
                        or not entry["start"] or pid <= 1
                        or path.stat().st_uid != os.getuid()
                        or Path(f"/proc/{pid}").stat().st_uid != os.getuid()
                        or process_start(pid) != entry["start"]
                        or os.getpgid(pid) != pid):
                    continue
                yield pid
            except (OSError, ValueError, KeyError, TypeError):
                continue

    for pid in owned_tools(bag_windows=True):
        with contextlib.suppress(ProcessLookupError):
            os.killpg(pid, signal.SIGTERM)
    deadline = time.monotonic() + 45
    while list(owned_tools(bag_windows=True)):
        if time.monotonic() >= deadline:
            raise RuntimeError("ROS Bags is still finalizing. Wait for it to finish before updating.")
        time.sleep(.25)

    for sig, grace in ((signal.SIGINT, 4), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        for pid in owned_tools():
            with contextlib.suppress(ProcessLookupError):
                os.killpg(pid, sig)
        deadline = time.monotonic() + grace
        while list(owned_tools()) and time.monotonic() < deadline:
            time.sleep(.15)
        if not list(owned_tools()):
            return
    raise RuntimeError("An ASURT tool is still closing. Wait for it before updating.")


def stop_for_map_switch(profile, map_key, expected_session):
    active = running_session()
    if active is None:
        return
    if expected_session is None or active["state"] != expected_session or active["profile"] != profile:
        raise RuntimeError("Select the current simulator before switching maps.")
    if active["phase"] != "ready":
        raise RuntimeError("Wait for the current simulator to finish starting.")
    if active["map_key"] == map_key:
        return
    pack = MAP_PACKS.get(map_key)
    if pack:
        if profile == "windows":
            check_map_base(pack, "WindowsNoEditor", windows_runtime_directory(windows_flat_pointer()))
        else:
            check_map_base(pack, "LinuxNoEditor", CARLA_ROOT / "Runtime/Active/asurt-linux-flat")
    os.kill(active["pid"], signal.SIGTERM)
    deadline = time.monotonic() + 45
    while True:
        current = running_session()
        if current is not None and current["state"] != expected_session:
            raise RuntimeError("A different simulator session started while switching maps.")
        # The receipt is removed before the supervisor releases its launch lock.
        if process_start(active["pid"]) != active["start"]:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("The current simulator is still shutting down.")
        time.sleep(0.25)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", choices=(*PROFILES, "auto"))
    parser.add_argument("action", choices=("start", "switch-map", "open-map", "stop", "status", *LIVE_TOOLS,
                                           "trackdraw", "viewer", "manager", "map-packs", "ros-bags"))
    parser.add_argument("--candidate", action="store_true")
    parser.add_argument("--acceptance", action="store_true")
    parser.add_argument("--packed-candidate", action="store_true")
    parser.add_argument("--session-state")
    parser.add_argument("--map", choices=MAPS, default="flat-asphalt")
    args, extra = parser.parse_known_args()
    if args.action == "open-map":
        active = running_session()
        if args.profile == "auto":
            args.profile = active["profile"] if active else os.environ.get("ASURT_PROFILE", "wslg")
        if active and active["profile"] == args.profile and not args.session_state:
            args.session_state = active["state"]
        args.action = "switch-map"
    if args.profile == "auto" and args.action not in (*LIVE_TOOLS, "trackdraw", "manager", "map-packs", "ros-bags"):
        parser.error("Select a simulator profile for this action")
    if args.action in ("start", "switch-map"):
        if args.packed_candidate and args.profile == "windows":
            parser.error("The Linux pak candidate requires WSLg or noVNC")
        if args.action == "switch-map":
            stop_for_map_switch(args.profile, args.map, args.session_state)
        try:
            supervisor(args.profile, args.candidate, args.acceptance, args.packed_candidate, args.map)
        except RendererStartupError as exc:
            print(f"{exc}\nRetrying the owned Linux simulator once with a new episode.", flush=True)
            time.sleep(1)
            supervisor(args.profile, args.candidate, args.acceptance, args.packed_candidate, args.map)
    elif args.action == "viewer":
        if args.profile != "novnc":
            parser.error("The browser viewer is available only for noVNC")
        wake_novnc()
    elif args.action in ("stop", "status"):
        active = running_session()
        if active and active["profile"] == args.profile:
            if args.action == "stop":
                if args.session_state is not None and active["state"] != args.session_state:
                    raise RuntimeError("The simulator changed before the stop request.")
                os.kill(active["pid"], signal.SIGTERM)
            else:
                print(json.dumps(active, indent=2))
        else:
            print(f"No owned ASURT {args.profile} session is active.")
    else:
        tool(args.profile, args.action, extra, args.session_state)


if __name__ == "__main__":
    main()
