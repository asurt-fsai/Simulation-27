"""Run owned CARLA scenarios and record ROS 2 bags without changing team settings."""

from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
from threading import Event
import time
from uuid import uuid4

import yaml

from track_tools import json_write, sha256_stream


ROOT = Path(os.environ["ASURT_ROOT"]).resolve()
STATE = Path(os.environ["ASURT_STATE_ROOT"]).resolve()
LAUNCHERS = ROOT / "Simulation/CARLA/Launchers"
MCP_BIN = ROOT / "libexec/asurt-mcp"
IMPORTER = MCP_BIN / "import_cone_track.py"
OUTPUT = STATE / "mcp"
sys.path[:0] = [str(LAUNCHERS), str(ROOT / "tools/RViz")]

from module_control import CONFIG, definitions, live_states, module_plan, module_sources
from session import MAP_LABELS, PROFILES, process_start, running_session


CORE = ("/clock", "/tf", "/tf_static", "/odom", "/drive", "/sim/feedback/collision")
TOPICS = {
    "planning": CORE + ("/map/global_cones", "/goal_pose", "/perception_landmarks",
                        "/planning/track_goal/path", "/planning/track_goal/status", "/control/stanley/status"),
    "lidar": CORE + ("/velodyne_points", "/filtered_points", "/lidar/cones", "/map/global_cones"),
    "camera": CORE + tuple(f"/zed/zed_node/{eye}/{suffix}" for eye in ("left", "right")
                            for suffix in ("image_rect_color", "camera_info")) + ("/zed/zed_node/obj_det/objects",),
}
TOPICS["training"] = tuple(dict.fromkeys(topic for group in TOPICS.values() for topic in group))


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def current_runtime(profile, session=None):
    install = STATE.parent
    if (install / "current").resolve() != ROOT or ROOT.parent != install / "installations":
        raise RuntimeError("Reconnect MCP through the current installed ASURT launcher.")
    active = LAUNCHERS / "../Runtime/Active"
    if profile == "windows":
        pointer = (active / "asurt-win64-flat.txt").read_text().strip()
        convert = lambda value: Path(subprocess.check_output(
            ["wslpath", "-u", value], text=True, timeout=5).strip()).resolve(strict=True)
        runtime = convert(pointer)
        binary = runtime / "WindowsNoEditor/CarlaUE4/Binaries/Win64/CarlaUE4-Win64-Shipping.exe"
        platform = "WindowsNoEditor"
    elif profile in ("linux", "wslg"):
        convert = lambda value: Path(value).resolve(strict=True)
        runtime = (active / "asurt-linux-flat").resolve(strict=True)
        if runtime.parent != install / "runtimes":
            raise RuntimeError("The Linux CARLA runtime is outside the current ASURT installation.")
        binary = runtime / "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
        platform = "LinuxNoEditor"
    else:
        raise ValueError("Choose a packaged CARLA profile.")
    if (runtime.parent.name != "runtimes" or not re.fullmatch(r"[0-9a-f]{64}", runtime.name)
            or not binary.is_file()):
        raise RuntimeError("The simulator must use an installer-managed CARLA runtime.")
    if session is not None and convert(session["runtime"]) != runtime:
        raise RuntimeError("The active simulator uses a different CARLA runtime. Restart from the current shortcut.")
    receipt = STATE / "release-installation.json"
    release = read_json(receipt) if receipt.is_file() else {}
    if release:
        with binary.open("rb") as source:
            digest = sha256_stream(source)
        if digest != release["base_sha256"].get(platform):
            raise RuntimeError("The CARLA executable does not match the installed release receipt.")
    return {"release": release.get("tag"), "runtime": str(runtime), "runtime_archive_sha256": runtime.name}


def cone_ownership(session, *, enforce=True):
    path = Path(session["state"])
    ownership = read_json(path) if path.is_file() else {"actors": []}
    if enforce and ownership["actors"] and ownership.get("map_ready") is not True:
        raise RuntimeError("The cone track was loaded by an older importer or is incomplete. "
                           "Clear it and spawn it with MCP track_importer before recording.")
    return ownership


def item_path(kind, ident):
    if not re.fullmatch(r"[0-9a-f]{32}", ident):
        raise ValueError(f"Invalid {kind} ID.")
    path = OUTPUT / kind / ident
    if path.is_symlink() or not path.is_dir():
        raise ValueError(f"Unknown {kind} ID.")
    return path


def simulator_status(probe=False):
    active = running_session()
    states = live_states(active)
    modules = [dict(item, name=item["label"], local=item["source"] == "Local") for item in states["modules"]]
    running = [m for m in modules if m["pids"]]
    if active:
        active = dict(active, perception_modules=[m["id"] for m in running if m["group"] == "Perception"])
    result = {"profiles": PROFILES, "maps": MAP_LABELS, "session": active, "modules": modules,
              "running_modules": [m["id"] for m in running]}
    if probe and states["carla_listening"]:
        from live_probe import probe_carla, probe_ros
        result["health"] = {"carla": probe_carla(active), "ros": probe_ros(active, running)}
    return result


def selected_modules(requested, driver):
    entries = definitions()
    names = {m.name for m in entries}
    selected = set(requested if requested is not None else ("slam_fake",))
    if driver == "stanley":
        selected.add("control_stanley")
    controllers = {module.name for module in entries if module.group.casefold() == "control"}
    if driver == "stanley":
        controllers.discard("control_stanley")
    if selected.intersection(controllers):
        raise ValueError("Choose a single scenario driver without additional control modules.")
    if selected - names:
        raise ValueError(f"Unknown modules: {', '.join(sorted(selected - names))}.")
    if {"slam_fake", "slam_fake_slam"} <= selected:
        raise ValueError("Select one fake map provider.")
    switches = {name: name in selected for name in names}
    plan, _, _ = module_plan(entries, switches)
    for module in reversed(plan):
        if switches[module.name]:
            switches.update(dict.fromkeys(module.dependencies, True))
    _, _, effective = module_plan(entries, switches)
    if selected - {name for name, enabled in effective.items() if enabled}:
        raise ValueError("The requested modules have conflicting map providers or missing dependencies.")
    builds = []
    for module in entries:
        if effective[module.name]:
            path = Path(module.binary)
            if not path.is_file():
                raise ValueError(f"Module {module.name} has no installed executable at {path}.")
            with path.open("rb") as source:
                digest = sha256_stream(source)
            builds.append({"id": module.name, "local": module.local, "entrypoint": str(path), "sha256": digest})
    topics = [m.topic for m in entries if effective[m.name] and m.group != "Perception"]
    return {"enabled": switches, "sources": module_sources(), "effective": [item["id"] for item in builds],
            "builds": builds, "topics": topics}


def start_job(spec):
    ident = uuid4().hex
    directory = OUTPUT / "runs" / ident
    directory.mkdir(parents=True)
    json_write(directory / "job.json", spec)
    json_write(directory / "report.json", {"run_id": ident, "state": "queued", "kind": spec["kind"]})
    with (directory / "worker.log").open("wb") as log:
        process = subprocess.Popen([sys.executable, MCP_BIN / "runner.py", ident], stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    json_write(directory / "worker.json", {"pid": process.pid, "start": process_start(process.pid)})
    return {"run_id": ident, "state": "queued", "directory": str(directory)}


def run_scenario(track_id, map_id="flat-asphalt", profile=None, duration_s=35, driver="reference",
                 modules=None, bag="planning", speed_mps=3.0, origin=None, goal=None):
    track = item_path("tracks", track_id)
    profile = profile or os.environ["ASURT_PROFILE"]
    if profile not in PROFILES or map_id not in MAP_LABELS:
        raise ValueError("Choose an installed map and profile from simulator_status.")
    if running_session():
        raise RuntimeError("Stop the active simulator before starting a dedicated scenario.")
    if driver == "stanley" and goal is None:
        raise ValueError("Stanley requires a [x, y, yaw_degrees] goal in the exported track frame.")
    simulator = current_runtime(profile)
    plan = selected_modules(modules, driver)
    return start_job({"kind": "scenario", "track_id": track.name, "profile": profile, "map_id": map_id,
                      "duration_s": duration_s, "driver": driver, "speed_mps": speed_mps, "plan": plan,
                      "bag": bag, "origin": origin, "goal": goal, "release": simulator["release"],
                      "simulator": simulator})


def record_bag(duration_s=30, preset="planning", topics=None):
    active = running_session()
    if not active or active["phase"] != "ready":
        raise RuntimeError("Launch ASURT CARLA and wait for a ready session first.")
    simulator = current_runtime(active["profile"], active)
    ownership = cone_ownership(active)
    requested = list(dict.fromkeys(("/clock", *(topics if topics is not None else TOPICS[preset]))))
    if not all(re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*", topic) for topic in requested):
        raise ValueError("Use fully qualified ROS topic names.")
    return start_job({"kind": "recording", "session": active, "duration_s": duration_s, "topics": requested,
                      "simulator": simulator, "cone_count": len(ownership["actors"])})


def get_run(run_id):
    directory = item_path("runs", run_id)
    report = read_json(directory / "report.json")
    if report["state"] in ("queued", "starting", "running") and (directory / "worker.json").is_file():
        owner = read_json(directory / "worker.json")
        if owner["start"] is None or process_start(owner["pid"]) != owner["start"]:
            report = read_json(directory / "report.json")
            if report["state"] in ("queued", "starting", "running"):
                report.update(state="failed", error="The scenario worker exited early. See worker.log.")
    return dict(report, directory=str(directory), log=str(directory / "worker.log"))


def cancel_run(run_id):
    directory = item_path("runs", run_id)
    report = get_run(run_id)
    if report["state"] in ("queued", "starting", "running"):
        owner = read_json(directory / "worker.json")
        if owner["start"] and process_start(owner["pid"]) == owner["start"]:
            os.kill(owner["pid"], signal.SIGTERM)
            return {"run_id": run_id, "state": "cancelling"}
    return report


def same_session(identity):
    active = running_session()
    return active and (active["pid"], active["start"]) == (identity["pid"], identity["start"])


def stop_child(process, first_signal, timeout):
    if process.poll() is not None:
        return
    process.send_signal(first_signal)
    try:
        process.wait(timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(6)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


@contextmanager
def bag_record(directory, topics, env):
    with (directory / "rosbag.log").open("wb") as log:
        process = subprocess.Popen(["ros2", "bag", "record", "--use-sim-time", "-o", directory / "bag", *topics],
                                   env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15
            while "Subscribed to topic '/clock'" not in (directory / "rosbag.log").read_text(errors="replace"):
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("ROS bag did not subscribe to /clock. See rosbag.log.")
                time.sleep(.1)
            yield process
        finally:
            stop_child(process, signal.SIGINT, 20)


def bag_summary(directory, requested):
    path = directory / "bag/metadata.yaml"
    if not path.is_file():
        raise RuntimeError("ROS bag has no metadata. See rosbag.log.")
    info = yaml.safe_load(path.read_text())["rosbag2_bagfile_information"]
    counts = {row["topic_metadata"]["name"]: row["message_count"] for row in info["topics_with_message_count"]}
    if not counts.get("/clock"):
        raise RuntimeError("ROS bag received no simulation clock samples.")
    return {"path": str(path.parent), "messages": info["message_count"], "topics": counts,
            "missing_topics": [topic for topic in requested if not counts.get(topic)]}


def collision_metrics(bag, ownership):
    topic = "/sim/feedback/collision"
    total = bag["topics"].get(topic, 0)
    hit_ids = set()
    if total:
        from carla_msgs.msg import CarlaCollisionEvent
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions

        reader = SequentialReader()
        reader.open(StorageOptions(uri=bag["path"], storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
        reader.set_filter(StorageFilter(topics=[topic]))
        owned_ids = {cone["id"] for cone in ownership["actors"]}
        while reader.has_next():
            _, data, _ = reader.read_next()
            actor = deserialize_message(data, CarlaCollisionEvent).other_actor_id
            if actor in owned_ids:
                hit_ids.add(actor)
    return {"collision_messages": total, "distinct_cones_hit": len(hit_ids), "hit_cone_actor_ids": sorted(hit_ids)}


def capture_existing(directory, spec, stop, status):
    session = spec["session"]
    if not same_session(session):
        raise RuntimeError("The selected simulator is no longer running.")
    simulator = current_runtime(session["profile"], session)
    ownership = cone_ownership(session)
    if simulator != spec["simulator"]:
        raise RuntimeError("The ASURT installation changed before recording started.")
    env = dict(os.environ, ROS_DOMAIN_ID=str(session.get("ros_domain_id", os.environ.get("ROS_DOMAIN_ID", "46"))))
    with bag_record(directory, spec["topics"], env) as process:
        status("running", map_id=session["map_key"], ros_domain_id=int(env["ROS_DOMAIN_ID"]),
               simulator=simulator, cone_count=len(ownership["actors"]))
        deadline = time.monotonic() + spec["duration_s"]
        while not stop.wait(.25) and time.monotonic() < deadline:
            if process.poll() is not None or not same_session(session):
                raise RuntimeError("ROS bag recording or its simulator ended early.")
    return {"bag": bag_summary(directory, spec["topics"])}


def scenario_environment(directory, plan):
    config_home = directory / "config"
    config = config_home / "asurt"
    config.mkdir(parents=True)
    json_write(config / "modules.json", {"version": 1, "enabled": plan["enabled"]})
    json_write(config / "module-sources.json", plan["sources"])
    if (CONFIG / "slam-settings.json").is_file():
        shutil.copyfile(CONFIG / "slam-settings.json", config / "slam-settings.json")
    domain = str(80 + int(directory.name[:4], 16) % 100)
    return dict(os.environ, XDG_CONFIG_HOME=str(config_home), ASURT_ROS_DOMAIN_ID=domain, ROS_DOMAIN_ID=domain)


def session_ready(process, stop):
    deadline = time.monotonic() + 240
    while not stop.wait(.25) and time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("The simulator launcher exited. See worker.log and the session logs.")
        session = running_session()
        if session and session["pid"] == process.pid and session["phase"] == "ready":
            return session
    raise RuntimeError("The owned simulator did not finish starting.")


def import_command(mode, session, track=None, origin=None, invert_y=True):
    if not same_session(session):
        raise RuntimeError("The simulator changed before using Track Importer.")
    if IMPORTER.resolve(strict=True) != IMPORTER:
        raise RuntimeError("The bundled Track Importer must come from this ASURT installation.")
    command = [sys.executable, IMPORTER, "--mode", mode, "--host", session["host"],
               "--port", str(session["port"]), "--state", session["state"]]
    if track is not None:
        x, y, z, yaw = origin or [session["course_origin"][key] for key in ("x", "y", "z", "yaw")]
        command += ["--anchor", "coords", "--origin-x", str(x), "--origin-y", str(y),
                    "--origin-z", str(z), "--yaw", str(yaw)]
        if invert_y:
            command.append("--invert-y")
        command += ["--", track]
    return command


def track_importer(action, track_id=None, track_path=None, origin=None, invert_y=True):
    session = running_session()
    if not session or session["phase"] != "ready":
        raise RuntimeError("Launch the current ASURT CARLA simulator first.")
    simulator = current_runtime(session["profile"], session)
    track = None
    if action in ("clear", "status"):
        if track_id or track_path or origin is not None:
            raise ValueError("Status and clear use the active session's managed cone track.")
    else:
        if bool(track_id) == bool(track_path):
            raise ValueError("Provide exactly one generated track_id or absolute CSV/JSON track_path.")
        track = item_path("tracks", track_id) / "cones.csv" if track_id else Path(track_path).expanduser()
        if not track.is_absolute() or track.suffix.lower() not in {".csv", ".json"} or not track.is_file():
            raise ValueError("Choose an existing absolute CSV or JSON track path.")
    OUTPUT.mkdir(exist_ok=True)
    with (OUTPUT / "scenario.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("A dedicated MCP scenario owns the simulator.") from None
        command = import_command(action, session, track, origin, invert_y)
        result = subprocess.run(command, capture_output=True, text=True, timeout=100)
        if result.returncode:
            raise RuntimeError((result.stderr + result.stdout).strip()[-10000:])
        ownership = cone_ownership(session, enforce=action == "spawn")
    return {"action": action, "map_id": session["map_key"], "simulator": simulator,
            "managed_cones": len(ownership["actors"]), "output": result.stdout.strip()[-10000:]}


def run_importer(track, directory, session, placement, env):
    with (directory / "track-import.log").open("wb") as log:
        for mode in ("spawn", "place-car"):
            command = import_command(mode, session, track / "cones.csv", placement)
            result = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=100)
            if result.returncode:
                raise RuntimeError(f"Track Importer failed during {mode}. See track-import.log. "
                                   "For Silverstone, supply coordinates where the full track fits.")
    return cone_ownership(session)


def start_simulator(profile, map_id, env):
    action = "start-wslg" if profile == "wslg" else "start"
    if profile != os.environ["ASURT_PROFILE"] and profile != "wslg":
        raise ValueError("Use one of this installation's packaged CARLA profiles.")
    return subprocess.Popen([ROOT / "bin/asurt", action, "--map", map_id],
                            env=dict(env, ASURT_ROOT=str(ROOT)), stdin=subprocess.DEVNULL,
                            start_new_session=True)


def run_scenario_worker(directory, spec, stop, status):
    from drive import drive_scenario

    track = item_path("tracks", spec["track_id"])
    shutil.copytree(track, directory / "track")
    env = scenario_environment(directory, spec["plan"])
    with (OUTPUT / "scenario.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if running_session():
            raise RuntimeError("Another CARLA session is active. It was left running.")
        simulator = current_runtime(spec["profile"])
        if simulator != spec["simulator"]:
            raise RuntimeError("The ASURT installation changed before the scenario started.")
        launcher = start_simulator(spec["profile"], spec["map_id"], env)
        session = None
        try:
            status("starting", map_id=spec["map_id"], track_id=spec["track_id"], simulator=simulator,
                   release=simulator["release"], modules=spec["plan"]["effective"],
                   ros_domain_id=int(env["ROS_DOMAIN_ID"]))
            session = session_ready(launcher, stop)
            current_runtime(spec["profile"], session)
            origin = spec["origin"] or [session["course_origin"][key] for key in ("x", "y", "z", "yaw")]
            ownership = run_importer(track, directory, session, origin, env)
            json_write(directory / "cone-placements.json", ownership)
            status("running", simulator_log=session["log"], origin=origin)
            topics = tuple(dict.fromkeys((*TOPICS[spec["bag"]], *spec["plan"]["topics"])))
            with bag_record(directory, topics, env):
                metrics = drive_scenario(directory, spec, session, origin, ownership, env, stop)
            bag = bag_summary(directory, topics)
            metrics.update(collision_metrics(bag, ownership))
            return {"bag": bag, "metrics": metrics, "ground_truth": str(directory / "ground-truth.jsonl")}
        finally:
            stop_child(launcher, signal.SIGTERM, 35)
            if session:
                logs = directory / "simulator-logs"
                logs.mkdir(exist_ok=True)
                for path in Path(session["log"]).glob("*.log"):
                    with path.open("rb") as source:
                        source.seek(max(0, path.stat().st_size - 1024 * 1024))
                        (logs / path.name).write_bytes(source.read())


def worker(run_id):
    directory = item_path("runs", run_id)
    spec = read_json(directory / "job.json")
    report = read_json(directory / "report.json")
    stop = Event()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stop.set())

    def status(state, **details):
        report.update(state=state, **details)
        json_write(directory / "report.json", report)

    status("starting")
    handler = run_scenario_worker if spec["kind"] == "scenario" else capture_existing
    try:
        result = handler(directory, spec, stop, status)
        status("cancelled" if stop.is_set() else "complete", **result)
    except Exception as error:
        status("cancelled" if stop.is_set() else "failed", error=str(error))
        raise


if __name__ == "__main__":
    worker(sys.argv[1])
