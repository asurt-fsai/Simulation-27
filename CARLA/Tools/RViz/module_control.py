"""Shared module registry, persistent switches, and bounded ROS process control.

Each registered executable is owned by its node, not by CARLA. Only exact
user-owned executable paths from the registry may receive a stop signal.
"""
from dataclasses import dataclass, replace
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import tempfile
import time

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("ASURT_ROOT", HERE.parent.parent))
WORKSPACE = ROOT / "Simulation/Systems/Perception/Workspace/autonomy_integration"
REGISTRY = HERE / "modules.conf"
CONFIG = Path(os.environ.get("XDG_CONFIG_HOME", str(Path.home() / ".config"))) / "asurt"
STATE = CONFIG / "modules.json"
STATE_LOCK = CONFIG / "modules.lock"
SOURCES = CONFIG / "module-sources.json"
RESERVED = {"server", "bridge", "driving", "viewer-inhibitor"}
# The existing ID keeps saved switches and local builds when renamed to FakeMAP.
FAKE_PROVIDERS = ("slam_fake_slam", "slam_fake")
AUTONOMOUS_CONTROLLERS = ("control_pure_pursuit_auto", "control_stanley_auto")
DRIVE_CONTROLLERS = (*AUTONOMOUS_CONTROLLERS, "control_pure_pursuit", "control_stanley",
                     "control_fs26_simple", "control_fs26_adaptive")
MANUAL_PLANNERS = {"planning_goal_pose", "planning_track_goal"}


@dataclass(frozen=True)
class Module:
    name: str
    binary: str
    script: Path
    topic: str
    dependency: str
    group: str
    label: str
    default_enabled: bool
    setup: str = ""
    local: bool = False
    arguments: tuple = ()
    revision: int = 0
    retired: bool = False

    @property
    def dependencies(self):
        return () if self.dependency == "-" else tuple(self.dependency.split(","))

    def command(self):
        if not self.local:
            return ["bash", str(self.script)]
        return ["bash", str(HERE / "module-entry.sh"), self.setup, self.binary,
                str(self.script or ""), *self.arguments]


def shipped_modules(*, include_retired=False):
    """Validate the whole registry before performing a potentially live action."""
    result = []
    seen = set()
    topics = set()
    for raw in REGISTRY.read_text(encoding="utf-8").splitlines():
        raw = raw.strip()
        if not raw or raw.startswith("#"):
            continue
        fields = raw.split("|")
        if len(fields) not in (5, 8):
            raise ValueError(f"Expected 8 module fields: {raw}")
        name, binary, script, topic, dependency = fields[:5]
        group, label, default = (fields[5:] if len(fields) == 8
                                 else ("Perception", name, "off"))
        if not re.fullmatch(r"[a-zA-Z0-9_-]+", name) or name in seen or name in RESERVED:
            raise ValueError(f"Invalid or duplicate module name: {name}")
        if any(not part or Path(part).is_absolute() or ".." in Path(part).parts
               for part in (binary, script)):
            raise ValueError(f"Invalid path for module {name}")
        if topic in topics or not re.fullmatch(r"/[a-zA-Z0-9_/]+", topic):
            raise ValueError(f"Invalid or duplicated topic for module {name}: {topic}")
        dependencies = [] if dependency == "-" else dependency.split(",")
        for dep in dependencies:
            if dep not in seen:
                raise ValueError(f"Dependency {dep} must be registered before {name}")
        if not group.strip() or not label.strip() or default not in ("on", "off", "retired"):
            raise ValueError(f"Invalid UI/default values for module {name}")
        full_script = ROOT / script
        if not full_script.is_file():
            raise FileNotFoundError(f"Module {name} launcher missing: {full_script}")
        # Root-relative installed entrypoints allow modules in independent
        # SLAM/Control/Planning workspaces without moving their source files.
        executable = (ROOT / binary[5:] if binary.startswith("root/")
                      else WORKSPACE / "install" / binary)
        result.append(Module(
            name, str(executable), full_script, topic,
            dependency, group.strip(), label.strip(), default == "on", retired=default == "retired"
        ))
        seen.add(name)
        topics.add(topic)
    return result if include_retired else [module for module in result if not module.retired]


def module_sources(path=None):
    try:
        return json.loads((path or SOURCES).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}


def local_path(value, label, validate):
    path = Path(value).expanduser()
    if not path.is_absolute() or (validate and not path.is_file()):
        raise ValueError(f"Choose an absolute path to an existing {label}: {path}")
    return str(path)


def definitions(sources=None, *, validate=False):
    sources = module_sources() if sources is None else sources
    if not isinstance(sources, dict):
        raise ValueError("Module settings must be a JSON object.")
    shipped = {m.name: m for m in shipped_modules(include_retired=True)}
    modules = shipped.copy()
    hidden = {name for name, module in shipped.items() if module.retired}
    for name, settings in sources.items():
        if not re.fullmatch(r"[a-zA-Z0-9_-]+", name) or name in RESERVED or not isinstance(settings, dict):
            raise ValueError(f"Invalid module ID: {name}")
        if settings.get("hidden") is True:
            if name not in shipped:
                raise ValueError(f"Unknown bundled module: {name}")
            hidden.add(name)
            continue
        binary = local_path(settings["binary"], "executable", validate)
        if validate and not os.access(binary, os.X_OK):
            raise ValueError(f"File is not executable: {binary}")
        setup = settings.get("setup", "")
        setup = local_path(setup, "ROS setup script", validate) if setup else ""
        if name in shipped:
            modules[name] = replace(shipped[name], binary=binary, setup=setup, local=True,
                                    revision=settings.get("revision", 0))
            hidden.discard(name)
            continue
        label, group, topic = (settings[key].strip() for key in ("label", "group", "topic"))
        depends = tuple(value.strip() for value in settings.get("dependencies", "").split(",") if value.strip())
        if not label or not group or not re.fullmatch(r"/[a-zA-Z0-9_/]+", topic):
            raise ValueError("Custom modules require a label, group, and absolute ROS topic.")
        arguments = settings.get("arguments", [])
        if not isinstance(arguments, list) or not all(isinstance(arg, str) for arg in arguments):
            raise ValueError("Module arguments must be a list of strings.")
        modules[name] = Module(name, binary, None, topic, ",".join(depends) or "-",
                               group, label, False, setup, True, tuple(arguments),
                               settings.get("revision", 0))
    order, visiting, seen = [], set(), set()

    def visit(name):
        if name in seen:
            return
        if name in visiting or name not in modules:
            raise ValueError(f"Missing or circular module dependency: {name}")
        visiting.add(name)
        module = modules[name]
        for dependency in module.dependencies:
            visit(dependency)
            if dependency in hidden:
                hidden.add(name)
        order.append(module)
        visiting.remove(name)
        seen.add(name)

    for name in modules:
        visit(name)
    active = [module for module in order if module.name not in hidden]
    for field in ("topic", "binary"):
        values = [getattr(module, field) for module in active]
        if len(set(values)) != len(values):
            raise ValueError(f"Each active module needs a unique {field}.")
    return active


def autonomous_planners(*, include_explicit=False):
    entries = definitions()
    topics = {
        module.topic: module.label
        for module in entries if module.group.casefold() in {"pathplanning", "path planning", "planning"}
        and (module.name not in MANUAL_PLANNERS or (include_explicit and module.name == "planning_goal_pose"))
    }
    if include_explicit and any(module.name == "planning_deep_learning" for module in entries):
        topics["/planning/deep_learning/raw_path"] = "Deep Learning (raw LSTM)"
    return topics


def autonomous_planner():
    try:
        settings = json.loads((CONFIG / "autonomous-control.json").read_text(encoding="utf-8"))
    except FileNotFoundError:
        return ""
    if not isinstance(settings, dict) or not isinstance(settings.get("planner_topic", ""), str):
        raise ValueError("Invalid autonomous planner preference.")
    return settings.get("planner_topic", "")


def save_autonomous_planner(topic):
    if topic and topic not in autonomous_planners(include_explicit=True):
        raise ValueError("Choose a registered autonomous planner.")
    CONFIG.mkdir(parents=True, exist_ok=True)
    path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=CONFIG, delete=False) as output:
            path = Path(output.name)
            json.dump({"planner_topic": topic}, output)
            output.write("\n")
        path.replace(CONFIG / "autonomous-control.json")
    finally:
        if path is not None:
            path.unlink(missing_ok=True)


def save_module_sources(sources, *, expected=None):
    CONFIG.mkdir(parents=True, exist_ok=True)
    with STATE_LOCK.open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if expected is not None and module_sources() != expected:
            raise ValueError("Module settings changed in another window. Reopen Module Settings and retry.")
        definitions(sources, validate=True)
        descriptor, temporary = tempfile.mkstemp(prefix=".module-sources-", dir=CONFIG)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as target:
                json.dump(sources, target, indent=2)
                target.write("\n")
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary, SOURCES)
        finally:
            Path(temporary).unlink(missing_ok=True)


def select_developer_build(name, install):
    existing = module_sources()
    bundled = next((module for module in shipped_modules(include_retired=True)
                    if module.name == name and (not module.retired or existing.get(name, {}).get("binary"))), None)
    if bundled is None:
        raise ValueError(f"Unknown bundled module: {name}")
    package, executable = Path(bundled.binary).parts[-2:]
    sources = existing.copy()
    sources[name] = {
        "binary": str(install / package / "lib" / package / executable),
        "setup": str(install / "local_setup.bash"),
    }
    save_module_sources(sources, expected=existing)


def saved_overrides(path=None):
    try:
        data = json.loads((path or STATE).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("enabled"), dict):
            raise ValueError("Expected an object with enabled overrides")
        for key, value in data["enabled"].items():
            if not isinstance(key, str) or type(value) is not bool:
                raise ValueError("Invalid module enablement value")
        return data["enabled"]
    except FileNotFoundError:
        return {}


def module_plan(entries=None, overrides=None):
    entries = definitions() if entries is None else entries
    overrides = saved_overrides() if overrides is None else overrides
    desired = {module.name: overrides.get(module.name, module.default_enabled) for module in entries}
    fake = next((name for name in FAKE_PROVIDERS if desired.get(name)), None)
    controller = next((name for name in DRIVE_CONTROLLERS if desired.get(name)), None)
    if fake:
        entries = [
            replace(module, dependency=",".join(
                fake if dep == "slam_cone_map" else dep for dep in module.dependencies
            ))
            if "slam_cone_map" in module.dependencies else module
            for module in entries
        ]
    effective = {}
    for module in entries:
        effective[module.name] = (
            desired[module.name] and not (fake and module.name != fake
                                          and module.name in (*FAKE_PROVIDERS, "slam_cone_map"))
            and not (controller and module.name in DRIVE_CONTROLLERS and module.name != controller)
            and all(effective[dep] for dep in module.dependencies)
        )
    return entries, desired, effective


def effective_states(entries=None, overrides=None):
    _, desired, effective = module_plan(entries, overrides)
    return desired, effective


def save_switches(changes):
    entries = definitions()
    names = {m.name for m in entries}
    if any(name not in names or type(value) is not bool
           for name, value in changes.items()):
        raise ValueError("Unknown module or invalid switch value")
    CONFIG.mkdir(parents=True, exist_ok=True)
    with STATE_LOCK.open("a+") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        overrides = saved_overrides()
        overrides.update(changes)
        for exclusive in (FAKE_PROVIDERS, DRIVE_CONTROLLERS):
            selected = next((name for name in exclusive if changes.get(name)), None)
            if selected:
                overrides.update({name: name == selected for name in exclusive if name in names})
        descriptor, temp = tempfile.mkstemp(prefix=".modules-", dir=CONFIG)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as target:
                json.dump({"version": 1, "enabled": overrides}, target,
                          indent=2, sort_keys=True)
                target.write("\n")
                target.flush()
                os.fsync(target.fileno())
            os.replace(temp, STATE)
        finally:
            if os.path.exists(temp):
                os.unlink(temp)
    return effective_states(entries, overrides)


def node_pids(module, *, group=None):
    """Return only exact user-owned installed entrypoints, never shell parents."""
    expected = os.path.realpath(module.binary)
    found = []
    uid = os.getuid()
    for path in Path("/proc").iterdir():
        if not path.name.isdecimal():
            continue
        try:
            if path.stat().st_uid != uid:
                continue
            if group is not None and os.getpgid(int(path.name)) != group:
                continue
            argv = [os.fsdecode(a) for a in (path / "cmdline").read_bytes().split(b"\0") if a]
            # The Python ROS2 console entrypoint is argv[1]. Do not match
            # arbitrary command text or a process with the same name.
            if any(arg.startswith("/") and os.path.realpath(arg) == expected
                   for arg in argv[:2]):
                found.append(int(path.name))
        except (OSError, PermissionError):
            continue
    return sorted(found)


def stop_module(module, deadline_seconds=8):
    """Request ROS shutdown so CARLA actors are released by node cleanup."""
    targets = node_pids(module)
    for pid in targets:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    end = time.monotonic() + deadline_seconds
    while targets and time.monotonic() < end:
        remaining = set(node_pids(module))
        if not remaining.intersection(targets):
            break
        time.sleep(0.15)
    remaining = sorted(set(node_pids(module)).intersection(targets))
    return {"signaled": targets, "still_running": remaining}


def live_states(session=None):
    """Combine module preferences and process status for the active session."""
    if session and session.get("module_config"):
        config = Path(session["module_config"])
        entries = definitions(module_sources(config / "module-sources.json"))
        entries, desired, effective = module_plan(entries, saved_overrides(config / "modules.json"))
    else:
        entries, desired, effective = module_plan()
    fake_label = next((module.label for module in entries if module.name in FAKE_PROVIDERS
                       and effective[module.name]), None)
    controller = next((name for name in DRIVE_CONTROLLERS if desired.get(name)), None)
    labels = {module.name: module.label for module in entries}
    ready = bool(session and session["phase"] == "ready")
    report = {}
    if ready:
        try:
            report = json.loads((Path(session["log"]) / "module-status.json").read_text())
        except FileNotFoundError:
            pass
    output = []
    status_by_id = {}
    for module in entries:
        running = report.get(module.name, {})
        group = running.get("group")
        pids = node_pids(module, group=group) if group else []
        ram_kib = 0
        for pid in pids:
            try:
                for line in (Path("/proc") / str(pid) / "status").read_text().splitlines():
                    if line.startswith("VmRSS:"):
                        ram_kib += int(line.split()[1])
                        break
            except (OSError, ValueError):
                pass
        prerequisites = [labels[dep] for dep in module.dependencies]
        waiting_for = [f"{labels[dep]} ({status_by_id[dep]['status']})" for dep in module.dependencies
                       if not effective[dep] or (ready and not status_by_id[dep]["pids"])]
        status = ("running" if pids and effective[module.name] else
                  "stopping" if pids else
                  "disabled" if not desired[module.name] else
                  f"using {fake_label}" if fake_label and module.name in (*FAKE_PROVIDERS, "slam_cone_map")
                  and not effective[module.name] else
                  f"using {labels[controller]}" if controller and module.name in DRIVE_CONTROLLERS
                  and module.name != controller else
                  f"waiting for {', '.join(waiting_for)}" if waiting_for else
                  "simulator offline" if not ready else
                  running.get("error") or "starting")
        path_topic = None
        if pids and effective[module.name] and module.name in AUTONOMOUS_CONTROLLERS:
            try:
                info = json.loads((Path(session["log"]) / f"{module.name}-readiness.json").read_text())
                if info["pid"] in pids:
                    path_topic = info.get("path_topic")
                    waiting_for = info["waiting_for"]
                    if waiting_for:
                        status = f"waiting for {', '.join(waiting_for)}"
                    elif info["state"] == "tracking":
                        status = f"tracking {path_topic}"
                    elif info["state"] == "end_of_path":
                        status = "at end of planner path"
            except (OSError, ValueError, KeyError, TypeError):
                pass
        item = {
            "id": module.name, "label": module.label, "group": module.group,
            "topic": module.topic, "dependency": module.dependency,
            "source": "Local" if module.local else "Bundled",
            "enabled": desired[module.name], "effective": effective[module.name],
            "pids": pids, "memory_mib": round(ram_kib / 1024, 1),
            "prerequisites": prerequisites, "waiting_for": waiting_for if desired[module.name] else [],
            "status": status,
        }
        if path_topic:
            item["path_topic"] = path_topic
        output.append(item)
        status_by_id[module.name] = item
    return {"carla_listening": ready, "session": session, "modules": output}
