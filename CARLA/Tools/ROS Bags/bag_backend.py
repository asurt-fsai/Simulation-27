"""ROS 2 discovery, bag metadata, and owned recording/playback jobs."""

from concurrent.futures import CancelledError
from contextlib import ExitStack, closing, contextmanager, suppress
from copy import deepcopy
import fcntl
from itertools import chain
import os
from pathlib import Path
import pty
from queue import Queue
import random
import re
import shlex
import shutil
import signal
import sqlite3
import subprocess
import tempfile
import threading
import time
from uuid import uuid4

import yaml

MIB = 1024 ** 2
DISK_RESERVE = 512 * MIB
RAM_RESERVE = 1024 * MIB
TOPIC = re.compile(r"/[A-Za-z_][A-Za-z_0-9]*(?:/[A-Za-z_][A-Za-z_0-9]*)*")
DISPLAYS = {
    "sensor_msgs/msg/PointCloud2": "PointCloud2",
    "sensor_msgs/msg/Image": "Image",
    "visualization_msgs/msg/MarkerArray": "MarkerArray",
    "visualization_msgs/msg/Marker": "Marker",
    "nav_msgs/msg/Path": "Path",
    "nav_msgs/msg/Odometry": "Odometry",
    "sensor_msgs/msg/LaserScan": "LaserScan",
}
TF_TOPICS = ("/tf", "/tf_static")
REPLAY_SUPPORT = {*TF_TOPICS, "/clock"}
SPATIAL_DISPLAYS = DISPLAYS.keys() - {"sensor_msgs/msg/Image"}
STATIC_TF_QOS = {"reliability": "reliable", "durability": "transient_local",
                 "history": "keep_all", "depth": 10}


def required_tf(selected, available):
    if not any(available.get(topic, ("", 0))[0] in SPATIAL_DISPLAYS for topic in selected):
        return ()
    return tuple(topic for topic in TF_TOPICS if topic not in selected and available.get(topic, ("", 0))[1])


def live_domain(session):
    if not session or session.get("phase") != "ready":
        raise ValueError("Start CARLA and wait for its session to be ready.")
    try:
        return int(session["ros_domain_id"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("The simulator did not report a ROS domain. Restart it from System Manager.") from None


def ros_environment(domain):
    return dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
                RMW_IMPLEMENTATION="rmw_fastrtps_cpp", PYTHONUNBUFFERED="1")


@contextmanager
def ros_node(domain):
    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions

    # The packaged launcher exports the local-only DDS settings before importing ROS.
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node("asurt_bags_" + uuid4().hex[:10], context=context,
                                 enable_rosout=False, start_parameter_services=False)
        try:
            yield node
        finally:
            node.destroy_node()
    finally:
        context.try_shutdown()


def discover(domain):
    with ros_node(domain) as node:
        time.sleep(.65)
        return {name: (", ".join(types), node.count_publishers(name))
                for name, types in sorted(node.get_topic_names_and_types())
                if TOPIC.fullmatch(name) and name not in ("/parameter_events", "/rosout")}


@contextmanager
def exclusive(path):
    with path.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError(f"Another ASURT process owns {path.stem}.") from None
        yield lock.fileno()


@contextmanager
def replay_domain(log_root, current_session):
    active = current_session()
    excluded = {int(os.environ.get("ROS_DOMAIN_ID", "0"))}
    if active and "ros_domain_id" in active:
        excluded.add(int(active["ros_domain_id"]))
    choices = sorted(set(range(1, 102)) - excluded)
    random.shuffle(choices)
    for domain in choices:
        with (log_root / f"ros-domain-{domain}.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                continue
            with ros_node(domain) as node:
                time.sleep(.65)
                if any(name != node.get_name() for name in node.get_node_names()):
                    continue
            yield domain, lock.fileno()
            return
    raise ValueError("No unused local ROS domain is available for playback.")


def read_bag(path):
    path = Path(path).expanduser().resolve(strict=True)
    if path.name == "metadata.yaml":
        path = path.parent
    if not path.is_dir():
        raise ValueError("Choose a bag directory or its metadata.yaml file.")
    try:
        metadata = yaml.safe_load((path / "metadata.yaml").read_text(encoding="utf-8"))
        info = metadata["rosbag2_bagfile_information"]
        files = [(path / name).resolve(strict=True) for name in info["relative_file_paths"]]
        topics = {item["topic_metadata"]["name"]:
                  (item["topic_metadata"]["type"], int(item["message_count"]))
                  for item in info["topics_with_message_count"]}
        int(info["message_count"])
    except FileNotFoundError as error:
        raise ValueError(f"The bag is missing {Path(error.filename).name}.") from error
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise ValueError(f"Invalid ROS bag metadata in {path}.") from error
    if not files or any(not p.is_relative_to(path) or not p.is_file() for p in files):
        raise ValueError("The bag's segments must be files inside the selected folder.")
    return path, metadata, info, files, topics


def available_space(path, ram=False):
    free = shutil.disk_usage(path).free
    if ram:
        line = next(line for line in Path("/proc/meminfo").read_text().splitlines()
                    if line.startswith("MemAvailable:"))
        free = min(free, int(line.split()[1])*1024)
    return free


def stage_bag(details, target, ram, job):
    """Expand file compression into a disposable validated copy, leaving source files read-only."""
    original, metadata, info, files, _ = details
    file_compressed = str(info.get("compression_mode", "")).lower() == "file"
    if not ram and not file_compressed:
        return original
    if (info.get("storage_identifier") != "sqlite3"
            or (file_compressed and info.get("compression_format") != "zstd")
            or (ram and str(info.get("compression_mode", "")).lower() == "message")):
        raise ValueError("Staging supports plain sqlite3 or file-compressed sqlite3/zstd. Try Disk for other uncompressed formats.")
    if file_compressed and not shutil.which("zstd"):
        raise ValueError("Install the Ubuntu zstd package to play file-compressed bags.")
    reserve = RAM_RESERVE if ram else DISK_RESERVE
    size = sum(p.stat().st_size for p in files)
    if available_space(target, ram) - reserve < size * (1.6 if file_compressed else 1):
        raise ValueError("Insufficient free RAM or tmpfs. Choose Disk." if ram else
                         "Insufficient free disk space for temporary expanded bag segments.")
    output = deepcopy(metadata)
    out_info = output["rosbag2_bagfile_information"]
    entries = out_info.get("files", [])
    total = 0
    names = []
    for index, source in enumerate(files):
        job.check_cancel()
        job.state(f"Preparing {'RAM' if ram else 'Disk'} · segment {index+1}/{len(files)}")
        name = f"bag_{index}.db3"
        destination = target / name
        if file_compressed:
            process = subprocess.Popen(["zstd", "-dc", "--", source], stdout=subprocess.PIPE)
            stream = process.stdout
        else:
            process, stream = None, source.open("rb")
        try:
            with stream, destination.open("wb") as out:
                while data := stream.read(MIB):
                    job.check_cancel()
                    if available_space(target, ram) - reserve < len(data):
                        raise ValueError("Stopped staging to preserve available memory or disk space.")
                    out.write(data)
            if process and process.wait(timeout=10):
                raise ValueError(f"Could not decompress {source.name}.")
        finally:
            if process and process.poll() is None:
                process.terminate()
                process.wait()
        with closing(sqlite3.connect(destination.as_uri() + "?mode=ro", uri=True)) as db:
            count = db.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        total += count
        if index < len(entries):
            if count != int(entries[index]["message_count"]):
                raise ValueError(f"Message count mismatch in {source.name}.")
            entries[index]["path"] = name
        names.append(name)
        job.log(f"Verified {source.name} · {count:,} messages")
    if total != int(info["message_count"]):
        raise ValueError("Staged bag message count differs from the source metadata.")
    out_info.update(compression_mode="", compression_format="", relative_file_paths=names)
    (target / "metadata.yaml").write_text(yaml.safe_dump(output, sort_keys=False))
    return target


def recorded_frame(path, topics, manual):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    _, _, info, segments, _ = read_bag(path)
    frames = {}
    edges = {}
    dynamic_targets = []
    odom_child = ""
    if info.get("storage_identifier") == "sqlite3" and not info.get("compression_mode"):
        for segment in segments[:2]:
            with closing(sqlite3.connect(segment.as_uri() + "?mode=ro", uri=True)) as db:
                first, last = db.execute(
                    "SELECT (SELECT timestamp FROM messages ORDER BY timestamp LIMIT 1), "
                    "(SELECT timestamp FROM messages ORDER BY timestamp DESC LIMIT 1)").fetchone()
                middle = first + (last - first) // 2 if first is not None else 0
                for topic in topics:
                    kind = topics[topic][0]
                    if kind not in DISPLAYS and topic not in TF_TOPICS:
                        continue
                    if topic in frames and topic not in TF_TOPICS:
                        continue
                    marker_array = kind == "visualization_msgs/msg/MarkerArray"
                    limit = 32 if topic in TF_TOPICS or marker_array else 3
                    query = ("SELECT messages.data FROM messages JOIN topics ON topics.id=messages.topic_id "
                             "WHERE topics.name=?")
                    rows = db.execute(query + " ORDER BY timestamp LIMIT ?", (topic, limit))
                    if marker_array:
                        # A recording can start before any cones come into view.
                        rows = chain(rows,
                                     db.execute(query + " AND timestamp>=? ORDER BY timestamp LIMIT ?",
                                                (topic, middle, limit)),
                                     db.execute(query + " ORDER BY timestamp DESC LIMIT ?", (topic, limit)))
                    for (data,) in rows:
                        try:
                            msg = deserialize_message(data, get_message(kind))
                            if kind == "tf2_msgs/msg/TFMessage":
                                for transform in msg.transforms:
                                    parent = transform.header.frame_id.lstrip("/")
                                    child = transform.child_frame_id.lstrip("/")
                                    if parent and child:
                                        edges.setdefault(parent, set()).add(child)
                                        edges.setdefault(child, set()).add(parent)
                                        if topic == "/tf" and child not in dynamic_targets:
                                            dynamic_targets.append(child)
                                        frames.setdefault(topic, parent)
                                continue
                            elif kind == "visualization_msgs/msg/MarkerArray":
                                frame = next((marker.header.frame_id for marker in msg.markers
                                              if marker.action == marker.ADD and marker.header.frame_id), "")
                            else:
                                frame = msg.header.frame_id
                                if kind == "nav_msgs/msg/Odometry":
                                    odom_child = msg.child_frame_id.lstrip("/")
                            if frame:
                                frames[topic] = frame.lstrip("/")
                                break
                        except (AttributeError, ImportError, RuntimeError, ValueError):
                            break
    def reachable(frame):
        visited, pending = set(), [frame]
        while pending:
            current = pending.pop()
            if current not in visited:
                visited.add(current)
                pending.extend(edges.get(current, ()))
        return visited

    spatial = {topic: frame for topic, frame in frames.items() if topics[topic][0] in SPATIAL_DISPLAYS}
    clouds = [frame for topic, frame in spatial.items()
              if topics[topic][0] in ("sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/LaserScan")]
    odom_frame = next((frame for topic, frame in spatial.items()
                       if topics[topic][0] == "nav_msgs/msg/Odometry" and frame in edges), "")
    preferred = (odom_frame, frames.get("/tf"), frames.get("/tf_static"),
                 "base_link" if "base_link" in spatial.values() else "", *clouds, *spatial.values())

    def coverage(frame):
        connected = reachable(frame)
        return sum(value in connected for value in spatial.values())

    fixed = manual.lstrip("/") or max((frame for frame in dict.fromkeys(preferred) if frame),
                                     key=coverage,
                                     default=next(iter(frames.values()), "map"))
    connected = reachable(fixed)
    target = next((frame for frame in (odom_child, "base_link", *dynamic_targets, *clouds)
                   if frame in connected), fixed)
    warnings = []
    if not edges and any(kind in SPATIAL_DISPLAYS for kind, _ in topics.values()):
        warnings.append("No recorded TF. Displays in different frames cannot be overlaid.")
    disconnected = [topic for topic, frame in spatial.items() if frame not in connected]
    if disconnected:
        warnings.append(f"No recorded transform to {fixed} for: {', '.join(disconnected)}.")
    return fixed, target if target != fixed else "<Fixed Frame>", " ".join(warnings)


def rviz_config(path, topics, fixed_frame, target_frame):
    displays = []
    for topic, (kind, _) in topics.items():
        if plugin := DISPLAYS.get(kind):
            topic_config = {"Value": topic, "Reliability Policy": "Best Effort",
                            "Durability Policy": "Volatile", "History Policy": "Keep Last", "Depth": 5}
            display = {"Class": f"rviz_default_plugins/{plugin}", "Name": topic,
                       "Enabled": True, "Value": True, "Topic": topic_config}
            if plugin == "PointCloud2":
                display.update({"Style": "Points", "Size (Pixels)": 2, "Color Transformer": "Intensity"})
            displays.append(display)
    if any(t in topics for t in ("/tf", "/tf_static")):
        displays.append({"Class": "rviz_default_plugins/TF", "Name": "TF", "Enabled": False})
    config = {"Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"},
                         {"Class": "rviz_common/Time", "Name": "Time"}],
              "Visualization Manager": {"Class": "", "Displays": displays, "Enabled": True,
                                        "Global Options": {"Fixed Frame": fixed_frame},
                                        "Tools": [{"Class": "rviz_default_plugins/Interact"}],
                                        "Views": {"Current": {"Class": "rviz_default_plugins/Orbit", "Distance": 20,
                                                               "Target Frame": target_frame}}}}
    path.write_text(yaml.safe_dump(config, sort_keys=False))


def replay_qos(info, topics):
    """Make recorded best-effort display topics compatible with RViz's initial subscriptions."""
    profiles = {}
    for item in info["topics_with_message_count"]:
        meta = item["topic_metadata"]
        name = meta["name"]
        if name == "/tf_static" and name in topics:
            profiles[name] = STATIC_TF_QOS
            continue
        if name not in topics or meta["type"] not in DISPLAYS:
            continue
        offers = meta.get("offered_qos_profiles") or []
        if isinstance(offers, str):
            offers = yaml.safe_load(offers) or []
        if not isinstance(offers, list) or not any(isinstance(q, dict) and
                str(q.get("reliability")).lower() in ("2", "best_effort") for q in offers):
            continue
        durable = all(isinstance(q, dict) and str(q.get("durability")).lower() in ("1", "transient_local")
                      for q in offers)
        profiles[name] = {"reliability": "reliable", "durability": "transient_local" if durable else "volatile",
                          "history": "keep_last", "depth": 5}
    return profiles


class Job:
    def __init__(self, emit):
        self.emit = emit
        self.stop = threading.Event()
        self.commands = Queue()
        self.readers = []

    def state(self, text):
        self.emit("state", text)

    def log(self, text):
        self.emit("log", text)

    def check_cancel(self):
        if self.stop.is_set():
            raise CancelledError()

    def start(self, args, env, locks):
        self.log("$ " + shlex.join(map(str, args)))
        master, slave = pty.openpty()
        try:
            process = subprocess.Popen(args, env=env, stdin=subprocess.DEVNULL,
                                       stdout=slave, stderr=slave, start_new_session=True, pass_fds=locks)
        except BaseException:
            os.close(master)
            raise
        finally:
            os.close(slave)

        def reader():
            with os.fdopen(master, "r", errors="replace") as stream:
                with suppress(OSError):
                    for line in stream:
                        self.log(line.rstrip())
        thread = threading.Thread(target=reader, daemon=True)
        thread.start()
        self.readers.append(thread)
        return process

    def flush_output(self):
        for reader in self.readers:
            reader.join(timeout=1)


def stop_process(process, timeout=15):
    if process is None or process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        os.killpg(process.pid, signal.SIGINT)
    process.wait(timeout=timeout)


def inspect_bag(path, env=None):
    details = read_bag(path)
    subprocess.run(["ros2", "bag", "info", details[0]], env=env, check=True, capture_output=True, text=True, timeout=35)
    return details


def record_job(job, log_root, session, current_session, folder, name, topics, sim_time):
    domain = live_domain(session)
    if not topics or any(not TOPIC.fullmatch(topic) for topic in topics):
        raise ValueError("Select at least one valid ROS topic.")
    if not re.fullmatch(r"[A-Za-z0-9_-][A-Za-z0-9_.-]{0,127}", name):
        raise ValueError("Use a bag name made of letters, digits, underscores, dots or hyphens.")
    if not folder.strip():
        raise ValueError("Choose a destination folder for the recording.")
    folder = Path(folder).expanduser().resolve()
    if folder.is_relative_to(Path("/dev/shm")):
        raise ValueError("Choose a persistent disk folder for the recording.")
    folder.mkdir(parents=True, exist_ok=True)
    filesystem = subprocess.check_output(["stat", "-f", "-c", "%T", folder], text=True).strip()
    if filesystem in {"tmpfs", "ramfs"} or not os.access(folder, os.W_OK):
        raise ValueError("Choose a writable folder on persistent storage.")
    destination = folder / name
    if destination.exists() or destination.is_symlink():
        raise ValueError("A bag with that name already exists.")
    if available_space(folder) < DISK_RESERVE:
        raise ValueError("At least 512 MiB of free disk space is required.")
    env = ros_environment(domain)
    with exclusive(log_root / "bag-recorder.lock") as lock, tempfile.TemporaryDirectory(prefix="asurt-record-") as temporary:
        job.state(f"Checking live topics · ROS domain {domain}")
        graph = discover(domain)
        missing = [name for name in topics if not graph.get(name, ("", 0))[1]]
        if sim_time and not graph.get("/clock", ("", 0))[1]:
            missing.append("/clock" if "/clock" in topics else "/clock (simulation time)")
        if missing:
            raise ValueError("No active publisher for " + ", ".join(dict.fromkeys(missing)))
        support = required_tf(topics, graph)
        if sim_time and "/clock" not in topics:
            support += ("/clock",)
        if support:
            topics = [*topics, *support]
            job.log("Recording replay support: " + ", ".join(support))
        active = current_session()
        if not active or active["state"] != session["state"] or live_domain(active) != domain:
            raise ValueError("The simulator session changed. Refresh its topics and try again.")
        job.check_cancel()
        node_name = "asurt_bag_recorder"
        command = ["ros2", "bag", "record", "--node-name", node_name, "--compression-mode", "file",
                   "--compression-format", "zstd", "--compression-threads", "1",
                   "--max-cache-size", "67108864", "--max-bag-size", "1500000000",
                   "-o", destination]
        if sim_time:
            command.append("--use-sim-time")
        if "/tf_static" in topics:
            qos_file = Path(temporary) / "record-qos.yaml"
            qos_file.write_text(yaml.safe_dump({"/tf_static": STATIC_TF_QOS}))
            command.extend(("--qos-profile-overrides-path", qos_file))
        process = job.start([*command, *topics], env, (lock,))
        try:
            job.state(f"Starting recorder · domain {domain}")
            start = time.monotonic()
            with ros_node(domain) as node:
                while process.poll() is None and not job.stop.wait(.4):
                    active = current_session()
                    if not active or active["state"] != session["state"]:
                        job.log("The simulator session ended.")
                        break
                    free = available_space(folder)
                    if free < DISK_RESERVE:
                        job.log("Low disk space. Finalizing the bag.")
                        break
                    seconds = time.monotonic() - start
                    ready = node_name in node.get_node_names() and destination.is_dir()
                    if ready:
                        size = 0
                        for segment in destination.iterdir():
                            # Compression can replace a segment between listing and stat.
                            with suppress(FileNotFoundError):
                                size += segment.stat().st_size
                        job.state(f"Recording · {seconds:.0f}s · {size/MIB:.1f} MiB · {free/MIB:.0f} MiB free"
                                  f" · ~{size/MIB/seconds:.1f} MiB/s on disk")
                    elif seconds > 20:
                        raise RuntimeError("The recorder did not appear on the live ROS graph. Check its terminal.")
        finally:
            job.state("Finalizing and verifying the recording")
            # File compression may take time. Allow a graceful interrupt to finish.
            stop_process(process, timeout=None)
        if job.stop.is_set() and not destination.exists():
            raise CancelledError()
        if process.returncode not in (0, -signal.SIGINT):
            raise RuntimeError(f"Recorder exited with code {process.returncode}. Check the terminal and {destination}.")
        _, _, info, _, contents = inspect_bag(destination, env)
        empty = [topic for topic in topics if not contents.get(topic, ("", 0))[1]]
        if not info["message_count"]:
            raise ValueError(f"Bag retained at {destination}. No messages were recorded.")
        for topic in topics:
            job.log(f"{topic} · {contents.get(topic, ('', 0))[1]:,} messages")
        warning = f" · no messages: {', '.join(empty)}" if empty else ""
        job.emit("saved", destination)
        job.state(f"Saved {info['message_count']:,} messages · {destination}{warning}")


def play_job(job, log_root, current_session, path, ram, rate, start_paused, requested, frame):
    from rclpy.executors import SingleThreadedExecutor
    from rosbag2_interfaces.srv import GetRate, IsPaused, Pause, Resume, SetRate

    details = inspect_bag(path)
    selected = {name: value for name, value in details[-1].items()
                if value[1] and (requested is None or name in requested)}
    if not selected:
        raise ValueError("Select at least one topic containing messages or enable All recorded topics.")
    support = required_tf(selected, details[-1])
    selected.update((name, details[-1][name]) for name in support)
    duration = details[2].get("duration", {}).get("nanoseconds", 0) / 1e9
    with exclusive(log_root / "bag-player.lock") as play_lock:
        cache_parent = Path("/dev/shm") if ram else log_root
        with tempfile.TemporaryDirectory(prefix="asurt-bag-", dir=cache_parent) as temporary:
            target = Path(temporary)
            job.log(f"Source bag: {details[0]}")
            if support:
                job.log("Replaying transforms for RViz: " + ", ".join(support))
            source = stage_bag(details, target, ram, job)
            fixed_frame, target_frame, warning = recorded_frame(source, selected, frame)
            if warning:
                job.log(warning)
            has_clock = "/clock" in selected
            job.log(f"RViz fixed frame: {fixed_frame}. " +
                    ("Using the bag's recorded clock." if has_clock else "Using wall time (bag has no recorded clock)."))
            if target_frame != "<Fixed Frame>":
                job.log(f"RViz view follows {target_frame}.")
            config = target / "replay.rviz"
            rviz_config(config, selected, fixed_frame, target_frame)
            qos = replay_qos(details[2], selected)
            qos_file = target / "replay-qos.yaml"
            if qos:
                qos_file.write_text(yaml.safe_dump(qos, sort_keys=False))
            with replay_domain(log_root, current_session) as (domain, domain_lock):
                job.check_cancel()
                env = ros_environment(domain)
                locks = (play_lock, domain_lock)
                job.state(f"Starting RViz and player · isolated ROS domain {domain}")
                view = player = None
                try:
                    view_name = "asurt_bag_rviz"
                    view = job.start(["rviz2", "-d", config, "--ros-args", "-r",
                                      f"__node:={view_name}", "-p",
                                      f"use_sim_time:={'true' if has_clock else 'false'}"], env, locks)
                    # The recorded clock is replayed as-is. Never synthesize a second clock.
                    command = ["ros2", "bag", "play", source, "--start-paused", "--rate", str(rate),
                               "--disable-keyboard-controls", "--read-ahead-queue-size", "256"]
                    if qos:
                        command.extend(("--qos-profile-overrides-path", qos_file))
                    command.extend(("--topics", *selected))
                    player = job.start(command, env, locks)
                    with ros_node(domain) as node, ExitStack() as resources:
                        executor = SingleThreadedExecutor(context=node.context)
                        resources.callback(executor.shutdown)
                        executor.add_node(node)
                        services = {name: node.create_client(kind, "/rosbag2_player/" + name)
                                    for name, kind in {"is_paused": IsPaused, "resume": Resume, "pause": Pause,
                                                       "set_rate": SetRate, "get_rate": GetRate}.items()}

                        def call(name, request):
                            future = services[name].call_async(request)
                            deadline = time.monotonic() + 8
                            try:
                                while not future.done() and player.poll() is None and time.monotonic() < deadline:
                                    job.check_cancel()
                                    executor.spin_once(timeout_sec=.2)
                                if not future.done() or (reply := future.result()) is None:
                                    raise TimeoutError(f"Player did not confirm {name} yet.")
                                return reply
                            finally:
                                if not future.done():
                                    services[name].remove_pending_request(future)

                        def change_pause(requested, previous):
                            name, request = ("pause", Pause.Request()) if requested else ("resume", Resume.Request())
                            try:
                                call(name, request)
                                return requested
                            except TimeoutError as error:
                                job.log(str(error))
                                try:
                                    return call("is_paused", IsPaused.Request()).paused
                                except TimeoutError:
                                    return previous

                        def change_rate(requested, previous):
                            try:
                                if call("set_rate", SetRate.Request(rate=requested)).success:
                                    return requested
                                job.log(f"The player refused speed {requested:g}×.")
                            except TimeoutError as error:
                                job.log(str(error))
                                try:
                                    return call("get_rate", GetRate.Request()).rate
                                except TimeoutError:
                                    pass
                            return previous

                        deadline = time.monotonic() + 75
                        expected = [topic for topic, (kind, _) in selected.items() if kind in DISPLAYS]
                        while True:
                            job.check_cancel()
                            if player.poll() is not None or view.poll() is not None:
                                raise RuntimeError("The player or RViz exited while starting. Check the playback terminal.")
                            graph_ready = (view_name in node.get_node_names()
                                           and all(node.count_subscribers(topic) for topic in expected)
                                           and all(node.count_publishers(topic) for topic in selected))
                            if graph_ready and all(client.service_is_ready() for client in services.values()):
                                break
                            if time.monotonic() >= deadline:
                                raise RuntimeError("Player or RViz topic discovery timed out. Check the playback terminal.")
                            job.stop.wait(.2)
                        paused = True if start_paused else change_pause(False, True)
                        job.emit("paused", paused)
                        offset, last = 0., time.monotonic()
                        while player.poll() is None:
                            if job.stop.wait(.15):
                                break
                            if view.poll() is not None:
                                raise RuntimeError("RViz closed. Playback stopped.")
                            now = time.monotonic()
                            if not paused:
                                offset = min(duration, offset + (now - last)*rate)
                            last = now
                            while not job.commands.empty():
                                command, value = job.commands.get_nowait()
                                if command == "rate":
                                    rate = change_rate(value, rate)
                                else:
                                    paused = change_pause(command == "pause", paused)
                                    job.emit("paused", paused)
                            job.state(f"{'Paused' if paused else 'Playing'} · {rate:g}× · "
                                      f"~{offset:.1f}/{duration:.1f}s · domain {domain}")
                        if player.poll() not in (None, 0):
                            raise RuntimeError("The bag player exited with an error. Check the playback terminal.")
                        job.state("Playback stopped" if job.stop.is_set() else "Playback complete")
                finally:
                    for process in (player, view):
                        try:
                            stop_process(process)
                        except subprocess.TimeoutExpired:
                            # Leave the job's lock held by the child if it has not exited.
                            # Do not remove its staged source while it can still be reading it.
                            job.state("Waiting for the playback process to exit")
                            process.wait()
