"""Install or update a versioned ASURT release, including optional map packs."""

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import time
from urllib.parse import quote
from urllib.request import urlopen
from urllib.error import URLError
import zipfile


NAME = re.compile(r"[a-zA-Z0-9][a-zA-Z0-9._-]*")
SHA = re.compile(r"[a-f0-9]{64}")
REPO = re.compile(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+")
PRODUCTION_REPOSITORY = "asurt-fsai/Simulation-27"
LEGACY_REPOSITORY = "hassanradwannn/ASURTSIM"
ASSET_ROOT = "Simulation/CARLA/Runtime/Active"
ICON_ROOT = "Simulation/CARLA/Launchers/Icons"
SYSTEMS = Path("Simulation/Systems")
EDITABLE = (
    Path("tools"),
    Path("Assets/Maps"),
    Path("Assets/Props"),
    Path("Simulation/CARLA/Launchers/map_pack_panel.py"),
    Path("Simulation/CARLA/Launchers/map_packs.py"),
    *(SYSTEMS / group / folder for group in ("SLAM", "PathPlanning", "Control")
      for folder in ("Config", "Launch", "Modules")),
    *(SYSTEMS / "Perception" / folder for folder in (
        "Adapters", "Models", "Modules/LiDAR", "Modules/Fusion",
        "Modules/Camera/Tests", "Modules/Camera/README.md")),
    SYSTEMS / "Common/ROSUnderlays/fs-system-26-test-build/src",
)
TEXT_TYPES = {".py", ".sh", ".rviz", ".yaml", ".yml", ".urdf", ".dsv"}
SHIP = {
    "linux": "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping",
    "windows": "WindowsNoEditor/CarlaUE4/Binaries/Win64/CarlaUE4-Win64-Shipping.exe",
}
ACTIONS = {
    "ASURT FlatAsphalt": "start",
    "ASURT Stop Simulator": "stop",
    "ASURT CARLA System Manager": "manager",
    "ASURT CARLA Map Packs": "map-packs",
    "ASURT Cone Track Importer": "track",
    "ASURT Weather Editor": "weather",
    "ASURT Trackdraw": "trackdraw",
    "ASURT Perception RViz": "rviz",
    "ASURT ZED RViz": "zed-rviz",
    "ASURT SLAM RViz": "slam-rviz",
    "ASURT SLAM Diagnostics": "slam-diagnostics",
    "ASURT Planning RViz": "planning-rviz",
    "ASURT Control RViz": "control-rviz",
    "ASURT Track Control RViz": "track-control-rviz",
}


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def open_source(source):
    if source.startswith("https://"):
        return urlopen(source, timeout=90)
    if "://" in source:
        raise ValueError("Use an HTTPS URL or a local file.")
    return Path(source).open("rb")


def read_json(source):
    with open_source(source) as file:
        return json.load(file)


def artifact_source(asset, args):
    name = asset["file"]
    if not NAME.fullmatch(name) or not SHA.fullmatch(asset["sha256"]):
        raise ValueError(f"Invalid asset in release manifest: {name!r}")
    if args.manifest and not args.manifest.startswith("https://"):
        return str(Path(args.manifest).resolve().parent / name)
    tag = asset.get("tag", args.tag)
    if not NAME.fullmatch(tag):
        raise ValueError("Invalid GitHub release tag.")
    return f"https://github.com/{args.repository}/releases/download/{quote(tag)}/{name}"


def show_download_progress(received, total, *, finished=False):
    terminal = sys.stdout.isatty()
    amount = f"{received / (1024 * 1024):.1f} MiB"
    if total:
        percent = min(100, received * 100 // total)
        filled = percent // 5
        bar = "#" * filled + "-" * (20 - filled)
        amount = f"[{bar}] {percent:3d}%  {amount} / {total / (1024 * 1024):.1f} MiB"
    print(("\r" if terminal else " ") + amount, end="\n" if finished or not terminal else "", flush=True)


def copy_download(source, output, total):
    received = 0
    last_update = time.monotonic()
    interval = 1 if sys.stdout.isatty() else 10
    read = getattr(source, "read1", source.read)
    for chunk in iter(lambda: read(1024 * 1024), b""):
        output.write(chunk)
        received += len(chunk)
        now = time.monotonic()
        if now - last_update >= interval:
            show_download_progress(received, total)
            last_update = now
    show_download_progress(received, total, finished=True)


def download_github(asset, args, partial, total):
    command = ["gh", "release", "download", asset.get("tag", args.tag), "--repo", args.repository,
               "--pattern", asset["file"], "--dir", str(partial.parent)]
    interval = 1 if sys.stdout.isatty() else 10
    with subprocess.Popen(command) as download:
        try:
            while True:
                try:
                    result = download.wait(timeout=interval)
                except subprocess.TimeoutExpired:
                    try:
                        received = partial.stat().st_size
                    except FileNotFoundError:
                        received = 0
                    show_download_progress(received, total)
                    continue
                if result:
                    raise subprocess.CalledProcessError(result, command)
                break
        finally:
            if download.poll() is None:
                download.kill()
    show_download_progress(partial.stat().st_size, total, finished=True)


def fetch(asset, args, cache, *, search_dirs=()):
    source = artifact_source(asset, args)
    name, checksum = asset["file"], asset["sha256"]
    target = cache / f"{checksum}-{name}"
    for candidate in (target, *(directory / name for directory in search_dirs)):
        if candidate.is_file() and digest(candidate) == checksum:
            print(f"Using verified download {name} from {candidate.parent}.", flush=True)
            return candidate
    cache.mkdir(parents=True, exist_ok=True)
    print(f"Downloading {name}…", flush=True)
    total = asset.get("bytes")
    total = total if type(total) is int and total > 0 else None
    with tempfile.TemporaryDirectory(dir=cache) as temporary:
        partial = Path(temporary) / name
        if args.github_cli:
            download_github(asset, args, partial, total)
        else:
            with open_source(source) as response, partial.open("wb") as output:
                copy_download(response, output, total)
        if digest(partial) != checksum:
            raise ValueError(f"SHA-256 check failed for {name}.")
        partial.replace(target)
    return target


def extract(archive, destination):
    destination.mkdir()
    with tarfile.open(archive, "r:gz") as bundle:
        for member in bundle:
            if member.isdir() and member.name in {".", "./", ""}:
                continue
            path = PurePosixPath(member.name)
            if (path.is_absolute() or not path.parts or ".." in path.parts
                    or any("\\" in part or ":" in part for part in path.parts)):
                raise ValueError(f"Invalid archive path: {member.name}")
            target = destination.joinpath(*path.parts)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                with bundle.extractfile(member) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output, length=1024 * 1024)
                target.chmod(0o755 if member.mode & 0o111 else 0o644)
            else:
                raise ValueError(f"Unsupported archive entry: {member.name}")


def replace_directory(staged, target):
    backup = None
    if os.path.lexists(target):
        backup = Path(tempfile.mkdtemp(prefix=target.name + ".repair-", dir=target.parent)) / "previous"
        target.rename(backup)
    try:
        staged.rename(target)
    except OSError:
        if backup is not None:
            backup.rename(target)
        raise
    if backup is not None:
        print(f"Previous files saved at {backup}.", flush=True)


def install_archive(archive, target, *, repair=False, validate=None):
    if target.is_dir() and not repair:
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=target.parent) as temporary:
        extracted = Path(temporary) / "stage"
        extract(archive, extracted)
        if validate is not None and not validate(extracted):
            raise ValueError(f"Released archive did not contain the expected files: {archive.name}")
        replace_directory(extracted, target)


def ensure_artifact(asset, args, cache, target, label, *, shipping=None):
    if not SHA.fullmatch(asset["sha256"]) or target.name != asset["sha256"]:
        raise ValueError(f"Invalid archive fingerprint for {label}.")
    checks = dict(asset.get("startup_files", {}))
    if shipping:
        checks[shipping] = asset["shipping_sha256"]
    for name, expected in checks.items():
        path = PurePosixPath(name)
        if (path.is_absolute() or not path.parts or ".." in path.parts or
                any("\\" in part or ":" in part for part in path.parts) or not SHA.fullmatch(expected)):
            raise ValueError(f"Invalid startup file checksum for {label}.")

    def complete(directory):
        if not directory.is_dir() or directory.is_symlink():
            return False
        try:
            if shipping and asset.get("startup_files"):
                prefix = "WindowsNoEditor/" if shipping.startswith("WindowsNoEditor/") else ""
                engine = directory / (prefix + "CarlaUE4/Config/DefaultEngine.ini")
                if b"[/Script/Engine.CollisionProfile]" not in engine.read_bytes():
                    return False
            return all((directory / path).resolve().is_relative_to(directory) and digest(directory / path) == expected
                       for path, expected in checks.items())
        except OSError:
            return False

    if complete(target):
        print(f"{label} is up to date.")
        return False
    if os.path.lexists(target):
        print(f"Repairing {label} from its verified release archive.", flush=True)
    install_archive(fetch(asset, args, cache), target, repair=True, validate=complete)
    return True


def activate(source, link):
    temporary = link.with_name(link.name + ".next")
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(source, target_is_directory=True)
    temporary.replace(link)


def close_previous_manager(current, previous):
    """Release the packaged manager's window lock after activating a different release."""
    relative = Path("tools/Control Center/control_center.py")
    roots = {os.fsencode(current), os.fsencode(previous)}
    entrypoints = {
        os.fsencode(current / relative), os.fsencode(previous / relative),
        os.fsencode(previous / "libexec/asurt-stock-tools/Control Center/control_center.py"),
    }
    for process in Path("/proc").iterdir():
        if not process.name.isdecimal():
            continue
        try:
            if process.stat().st_uid != os.getuid():
                continue
            argv = (process / "cmdline").read_bytes().split(b"\0")
            if len(argv) < 2 or argv[1] not in entrypoints:
                continue
            markers = (process / "environ").read_bytes().split(b"\0")
            if any(b"ASURT_ROOT=" + root in markers for root in roots):
                os.kill(int(process.name), signal.SIGTERM)
        except OSError:
            continue


def patch_text(root, active_path):
    for path in root.rglob("*"):
        if path.is_symlink() or not path.is_file() or path.suffix not in TEXT_TYPES:
            continue
        raw = path.read_bytes()
        if b"@ASURT_ROOT@" in raw:
            path.write_bytes(raw.replace(b"@ASURT_ROOT@", os.fsencode(active_path)))


def workspace_paths(stack):
    perception = SYSTEMS / "Perception/Workspace/autonomy_integration"
    return (
        *EDITABLE,
        perception / "src/perception_zed_pkg",
        perception / "vendor",
        *(path.relative_to(stack) for path in sorted((stack / perception).glob("*.sh"))),
    )


def add_ros_bags_to_edited_manager(target, stock):
    # Older installs may have an edited copy of the standard System Manager.
    # Add the menu item where its tool registry still follows the stock layout.
    entry = b'    "ROS Bags": "ros-bags",'
    if entry not in stock or target.is_symlink():
        return
    current = target.read_bytes()
    start = current.find(b"\nTOOLS = {")
    end = current.find(b"\n}", start)
    anchor = b'    "Map Packs": "map-packs",'
    if start < 0 or end < 0 or b'"ros-bags"' in current or current[start:end].count(anchor) != 1:
        return
    index = current.index(anchor, start, end) + len(anchor)
    newline = b"\r\n" if current[index:].startswith(b"\r\n") else b"\n"
    if not current[index:].startswith(newline):
        return
    temporary = target.with_name(target.name + ".asurt-next")
    temporary.write_bytes(current[:index] + newline + entry + current[index:])
    temporary.chmod(target.stat().st_mode & 0o777)
    temporary.replace(target)
    print(f"Added ROS Bags to the edited System Manager: {target}")


def retired_module_paths(stack):
    retirement = stack / "retired-modules.json"
    if not retirement.is_file():
        return []
    entries = read_json(str(retirement))
    if not isinstance(entries, list) or any(not isinstance(entry, str) for entry in entries):
        raise ValueError("Invalid bundled module retirement list")
    packages = [Path(entry) for entry in entries]
    for package in packages:
        if (len(package.parts) < 5 or ".." in package.parts or
                package.parts[:2] != SYSTEMS.parts or "Modules" not in package.parts[3:-1] or
                package.parts[2] not in {"Perception", "SLAM", "PathPlanning", "Control"}):
            raise ValueError(f"Invalid retired module package: {package}")
    return packages


def prune_retired_packages(stack):
    for package in retired_module_paths(stack):
        target = stack / package
        if target.is_dir() and target.resolve() == stack.resolve() / package:
            shutil.rmtree(target)


def retire_seeded_modules(workspace, stack, known, sources):
    """Remove only complete, unedited copies of retired bundled packages."""
    packages = retired_module_paths(stack)
    if not packages:
        return
    try:
        local_sources = read_json(str(sources)) if sources.is_file() else {}
    except (OSError, ValueError):
        return
    if not isinstance(local_sources, dict):
        return
    local_paths = [Path(value).expanduser().resolve() for settings in local_sources.values()
                   if isinstance(settings, dict) for name in ("binary", "setup")
                   if isinstance(value := settings.get(name), str) and value]
    for package in packages:
        if (stack / package).exists():
            continue
        target = workspace / package
        # Workspace directories can be user-managed links. Leave them alone.
        if not target.is_dir() or target.resolve() != workspace.resolve() / package:
            continue
        if any(path.is_relative_to(target.resolve()) or package.name in path.parts
               for path in local_paths):
            continue
        contents = list(target.rglob("*"))
        def unchanged(path):
            key = path.relative_to(workspace).as_posix()
            return not path.is_symlink() and (path.is_dir() or (
                path.is_file() and known.get(key) == digest(path)))

        if not all(unchanged(path) for path in contents):
            continue
        for path in sorted(contents, key=lambda item: len(item.parts), reverse=True):
            if not unchanged(path):
                continue
            if path.is_file():
                path.unlink(missing_ok=True)
            elif path.is_dir() and not any(path.iterdir()):
                path.rmdir()
        if not any(target.iterdir()):
            target.rmdir()
            print(f"Removed retired bundled module: {package.name}")


def seed_workspace(root, stack, installation):
    workspace = root / "workspace"
    workspace.mkdir(exist_ok=True)
    receipt = workspace / ".asurt-seed.json"
    known = read_json(str(receipt)) if receipt.is_file() else {}
    config = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "asurt"
    retire_seeded_modules(workspace, stack, known, config / "module-sources.json")
    current = root / "current"
    prior = current.resolve()
    previous = None
    if current.is_symlink() and not known and prior != installation:
        prefix = prior.name.split("-")[1:2]
        if prefix and (root / "stacks").is_dir():
            previous = next((path for path in (root / "stacks").iterdir()
                             if path.name.startswith(prefix[0])), None)
    if (stack / "tools").is_dir():
        bundled_tools = installation / "libexec/asurt-stock-tools"
        bundled_tools.parent.mkdir(exist_ok=True)
        activate(stack / "tools", bundled_tools)
    updated = {}
    for relative in workspace_paths(stack):
        source = stack / relative
        if not source.exists():
            continue
        files = sorted(path for path in source.rglob("*") if path.is_file()) if source.is_dir() else [source]
        for file in files:
            location = file.relative_to(stack)
            key = location.as_posix()
            target = workspace / location
            raw = file.read_bytes()
            if file.suffix in TEXT_TYPES:
                raw = raw.replace(b"@ASURT_ROOT@", os.fsencode(current))
            stock = hashlib.sha256(raw).hexdigest()
            previous_seed = known.get(key)
            existing = digest(target) if target.is_file() else None
            if existing is None and previous and (prior / location).is_file() and (previous / location).is_file():
                old_stock = (previous / location).read_bytes()
                old_actual = (prior / location).read_bytes()
                if file.suffix in TEXT_TYPES:
                    old_stock = old_stock.replace(b"@ASURT_ROOT@", os.fsencode(prior))
                if old_actual != old_stock:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(old_actual.replace(os.fsencode(prior), os.fsencode(current))
                                       if file.suffix in TEXT_TYPES else old_actual)
                    target.chmod((prior / location).stat().st_mode & 0o777)
                    existing = digest(target)
                    previous_seed = hashlib.sha256(old_stock).hexdigest()
            if existing is None or existing == previous_seed:
                target.parent.mkdir(parents=True, exist_ok=True)
                temporary = target.with_name(target.name + ".asurt-next")
                temporary.write_bytes(raw)
                temporary.chmod(file.stat().st_mode & 0o777)
                temporary.replace(target)
            elif existing != stock:
                if key == "tools/Control Center/control_center.py":
                    add_ros_bags_to_edited_manager(target, raw)
                if stock != previous_seed:
                    incoming = workspace / ".asurt-updates" / location
                    incoming.parent.mkdir(parents=True, exist_ok=True)
                    incoming.write_bytes(raw)
                    incoming.chmod(file.stat().st_mode & 0o777)
                    print(f"Preserved an edit to {location}. Incoming version: {incoming}")
            updated[key] = stock
        link = installation / relative
        destination = workspace / relative
        if link.is_symlink() and link.resolve() == destination.resolve():
            continue
        if link.is_dir() and not link.is_symlink():
            shutil.rmtree(link)
        else:
            link.unlink(missing_ok=True)
        activate(destination, link)
    temporary = receipt.with_suffix(".next")
    temporary.write_text(json.dumps(updated, indent=2) + "\n", encoding="utf-8")
    temporary.replace(receipt)
    map_packs = root / "state/maps"
    map_packs.mkdir(parents=True, exist_ok=True)
    config.mkdir(parents=True, exist_ok=True)
    for name in ("module-sources.json", "tool-paths.json"):
        try:
            with (config / name).open("x", encoding="utf-8") as file:
                file.write("{}\n")
        except FileExistsError:
            pass
    (workspace / "Assets").mkdir(exist_ok=True)
    for link, target in (
        (workspace / "Modules", workspace / SYSTEMS),
        (workspace / "Assets/Map Packs", map_packs),
        (workspace / "Settings", config),
    ):
        if not os.path.lexists(link):
            activate(target, link)


def reuse_local_cuda(root, requirements, checksum):
    """Check a CUDA runtime already selected by a previous ASURT installation."""
    active = root / 'current'
    settings = root / 'state'
    link = settings / 'inference-cuda'
    previous_requirements = active / 'bin/cuda-requirements.txt'
    if (not active.is_symlink() or not link.is_symlink() or not previous_requirements.is_file()
            or digest(previous_requirements) != checksum):
        return None
    cuda = link.resolve()
    if cuda.is_relative_to(root / 'inference') or not (cuda / 'onnxruntime').is_dir():
        return None
    version = re.search(r'^onnxruntime-gpu==([^\s]+)', requirements.read_text(), re.MULTILINE)
    if version is None:
        return None
    probe = '''import os
import runpy
import sys
from pathlib import Path
import onnxruntime

site, version, verifier = sys.argv[1:]
if (os.environ.get("ASURT_INFERENCE_BACKEND") != "cuda"
        or onnxruntime.__version__ != version
        or Path(onnxruntime.__file__).resolve().parent != (Path(site) / "onnxruntime").resolve()):
    sys.exit("The existing CUDA runtime does not have the expected ONNX Runtime.")
sys.argv[:] = [verifier, "verify"]
runpy.run_path(verifier, run_name="__main__")
'''
    print('Checking the CUDA runtime already used by this ASURT installation…', flush=True)
    try:
        subprocess.run(['bash', active / 'bin/asurt', 'exec', sys.executable, '-c', probe,
                        str(cuda), version.group(1), str(active / 'bin/inference.py')],
                       env=dict(os.environ, ASURT_ROOT=str(active), ASURT_STATE_ROOT=str(settings),
                                ASURT_RELEASE_CUDA_ROOT=str(cuda)), check=True, timeout=90)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        print('The existing CUDA runtime failed verification. Preparing the pinned runtime.', flush=True)
        return None
    print('Reusing the verified local CUDA runtime.', flush=True)
    return cuda


def prepare_inference(root, stack):
    mode = subprocess.check_output([sys.executable, stack / 'bin/inference.py', 'detect'], text=True).strip()
    if mode == 'cpu':
        print('No CUDA device is visible. ASURT will use CPU inference.', flush=True)
        return None
    if mode != 'cuda':
        raise RuntimeError(f'Unexpected inference device result: {mode}')
    requirements = stack / 'bin/cuda-requirements.txt'
    checksum = digest(requirements)
    target = root / 'inference' / checksum
    if not target.is_dir():
        existing = reuse_local_cuda(root, requirements, checksum)
        if existing is not None:
            return existing
        target.parent.mkdir(parents=True, exist_ok=True)
        wheels = root / 'cache/cuda-wheels'
        wheels.mkdir(parents=True, exist_ok=True)
        pip = [sys.executable, '-m', 'pip', '--isolated', '--disable-pip-version-check', '--no-cache-dir']
        locked_wheels = ['--only-binary=:all:', '--no-deps', '--require-hashes', '-r', requirements]
        print('Installing the pinned CUDA and cuDNN inference runtime…', flush=True)
        try:
            subprocess.run([*pip, 'download', '--index-url', 'https://pypi.org/simple',
                            '--timeout', '120', '--retries', '5',
                            '--dest', wheels, '--find-links', wheels, *locked_wheels], check=True)
        except subprocess.CalledProcessError as error:
            raise RuntimeError('CUDA wheel download failed. Rerun the installer to reuse any completed downloads.') from error
        with tempfile.TemporaryDirectory(dir=target.parent) as temp:
            stage = Path(temp) / 'site'
            subprocess.run([*pip, 'install', '--no-index', '--find-links', wheels,
                            '--target', stage, *locked_wheels], check=True)
            stage.rename(target)
    else:
        print('CUDA inference runtime is up to date.', flush=True)
    return target


def enable_dynamic_prop_sensor_trace(runtime):
    """Give movable cone meshes the same CARLA sensor response as static cones."""
    config = runtime / "CarlaUE4/Config/DefaultEngine.ini"
    data = config.read_bytes()
    header = re.search(rb"(?m)^\[/Script/Engine\.CollisionProfile\][ \t]*(\r?\n)", data)
    if header is None:
        raise ValueError(f"CARLA collision profiles are missing from {config}")
    start = header.end()
    end = data.find(b"\n[", start - 1)
    end = end + 1 if end >= 0 else len(data)
    profiles = data[start:end]
    sensor = b'(Channel="SensorTrace",Response=ECR_Block)'
    physics = re.search(rb'(?m)^\+EditProfiles=\(Name="?PhysicsActor"?,CustomResponses=\(([^\r\n]*)\)\)', profiles)
    if physics:
        responses = physics.group(1)
        updated, count = re.subn(rb'\(Channel="?SensorTrace"?(?:,[^)]*)?\)', sensor, responses)
        if not count:
            updated = responses + (b"," if responses else b"") + sensor
        profiles = profiles[:physics.start(1)] + updated + profiles[physics.end(1):]
    else:
        profiles = (b'+EditProfiles=(Name="PhysicsActor",CustomResponses=('
                    + sensor + b'))' + header.group(1) + profiles)
    updated = data[:start] + profiles + data[end:]
    if updated != data:
        with tempfile.TemporaryDirectory(dir=config.parent) as temporary:
            staged = Path(temporary) / config.name
            staged.write_bytes(updated)
            shutil.copymode(config, staged)
            staged.replace(config)


def verify_installation(candidate, settings, cuda_root):
    subprocess.run(['bash', candidate / 'bin/asurt', 'check'], check=True,
                   env=dict(os.environ, ASURT_ROOT=str(candidate), ASURT_STATE_ROOT=str(settings),
                            ASURT_EXPECT_INFERENCE_BACKEND='cuda' if cuda_root else 'cpu',
                            ASURT_RELEASE_CUDA_ROOT=str(cuda_root) if cuda_root else ''))


def launcher_matches(installation, stack, *, relocated=True):
    launch = "Simulation/CARLA/Launchers"
    bridge = "Simulation/CARLA/Bridge/Carla_ASU_Bridge/install/carla_telemetry/lib/python3.10/site-packages/carla_telemetry"
    paths = ("bin/asurt", f"{launch}/session.py", f"{launch}/world_selection.py", f"{launch}/weather_grip.py",
             f"{launch}/road_weather.py", f"{launch}/windows_interop.py", f"{launch}/windows-server.ps1",
             f"{launch}/process_ownership.py",
             f"{launch}/ros-parity-env.sh", f"{launch}/release_update.py", f"{bridge}/manual_control.py")
    try:
        for name in paths:
            source = stack / name
            if source.is_file():
                expected = source.read_bytes()
                if relocated and source.suffix in TEXT_TYPES:
                    expected = expected.replace(b"@ASURT_ROOT@", os.fsencode(installation))
                if (installation / name).read_bytes() != expected:
                    return False
        return True
    except OSError:
        return False


def make_root(args, root, manifest, runtime, stack_dir, cuda_root, wsl_runtime=None):
    settings = root / "state"
    platform = args.platform
    stack_key = stack_dir.name
    runtime_key = manifest["runtime"][platform]["sha256"]
    version = f"{platform}-{stack_key[:16]}-{runtime_key[:16]}"
    if wsl_runtime:
        version += f"-wslg-{manifest['runtime']['linux']['sha256'][:16]}"
    target = root / "installations" / version
    if target.is_dir() and launcher_matches(target, stack_dir):
        active = target / ASSET_ROOT
        linux_runtime = wsl_runtime or (runtime if platform == "linux" else None)
        try:
            valid = (target / ".asurt-platform").read_text().strip() == platform
            if linux_runtime:
                valid = valid and (active / "asurt-linux-flat").resolve() == linux_runtime
            if platform == "windows":
                expected = subprocess.check_output(["wslpath", "-w", str(runtime)], text=True).strip()
                valid = valid and (active / "asurt-win64-flat.txt").read_text().strip() == expected
            if valid:
                if not args.fixture:
                    verify_installation(target, settings, cuda_root)
                return target
        except (OSError, subprocess.CalledProcessError):
            pass
    if os.path.lexists(target):
        print("Repairing the installed launcher and packaged tools.", flush=True)
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=target.parent) as temp:
        candidate = Path(temp) / "stack"
        shutil.copytree(stack_dir, candidate)
        patch_text(candidate, target)
        active = candidate / ASSET_ROOT
        active.mkdir(parents=True, exist_ok=True)
        if platform == "windows":
            win_root = subprocess.check_output(["wslpath", "-w", str(runtime)], text=True).strip()
            (active / "asurt-win64-flat.txt").write_text(win_root + "\n")
        linux_runtime = wsl_runtime or (runtime if platform == "linux" else None)
        if linux_runtime:
            (active / "asurt-linux-flat").symlink_to(linux_runtime, target_is_directory=True)
        (candidate / ".asurt-platform").write_text(platform + "\n")
        if not args.fixture:
            verify_installation(candidate, settings, cuda_root)
        replace_directory(candidate, target)
    return target


def stack_with_patch(base, patch, *, repair=False):
    key = hashlib.sha256(f"{base.name}:{patch.name}".encode()).hexdigest()
    target = base.parent / key
    if target.is_dir() and not repair and launcher_matches(target, patch, relocated=False):
        return target
    with tempfile.TemporaryDirectory(dir=base.parent) as temporary:
        stage = Path(temporary) / "stack"
        shutil.copytree(base, stage)
        shutil.copytree(patch, stage, dirs_exist_ok=True)
        prune_retired_packages(stage)
        (stage / "Simulation/CARLA/Launchers/tools-launcher.py").unlink(missing_ok=True)
        replace_directory(stage, target)
    return target


def ask(label, policy):
    if policy == "all":
        return True
    if policy == "none" or not sys.stdin.isatty():
        print(f"Optional map available: {label}")
        return False
    return input(f"Download {label}? [y/N] ").strip().casefold() in {"y", "yes"}


def update_maps(args, root, runtime, wsl_runtime=None):
    local = args.manifest and not args.manifest.startswith("https://")
    if args.catalog:
        source = args.catalog
    elif local:
        source = str(Path(args.manifest).resolve().parent / "asurt-maps.json")
        if not Path(source).exists():
            return
    elif args.github_cli:
        branch = quote(args.branch, safe="")
        endpoint = f"repos/{args.repository}/contents/asurt-maps.json?ref={branch}"
        result = subprocess.run(["gh", "api", endpoint, "-H", "Accept: application/vnd.github.raw"],
                                capture_output=True, text=True)
        if result.returncode:
            if "HTTP 404" in result.stderr:
                return
            raise ValueError(f"Could not fetch the private map catalog: {result.stderr.strip()}")
        source = None
        catalog = json.loads(result.stdout)
    else:
        branch = quote(args.branch, safe="")
        source = f"https://raw.githubusercontent.com/{args.repository}/{branch}/asurt-maps.json"
    if source:
        try:
            catalog = read_json(source)
        except URLError as error:
            if getattr(error, "code", None) == 404 and not args.catalog and args.maps != "all":
                return
            raise
    if catalog.get("format") != 1:
        raise ValueError("Unsupported map catalog format.")
    if args.platform == "windows":
        bases = {"WindowsNoEditor": (digest(runtime / SHIP["windows"]), runtime / "WindowsNoEditor")}
        if wsl_runtime:
            bases["LinuxNoEditor"] = (digest(wsl_runtime / SHIP["linux"]), wsl_runtime)
    else:
        bases = {"LinuxNoEditor": (digest(runtime / SHIP["linux"]), runtime)}
    registry = root / "state" / "maps"
    registry.mkdir(parents=True, exist_ok=True)
    # Use the installed launcher to recognize registered packs and validate their base.
    sys.path.insert(0, str(root / "current" / "Simulation/CARLA/Launchers"))
    from map_packs import check_base, read_pack, register
    compatible_count = 0
    for item in catalog["maps"]:
        map_id = item["id"]
        if not re.fullmatch(r"[a-z][a-z0-9-]{0,47}", map_id) or map_id in {"flat-asphalt", "silverstone"}:
            raise ValueError(f"Invalid add-on map ID: {map_id!r}")
        matching = {platform: game for platform, (checksum, game) in bases.items()
                    if item["base_sha256"].get(platform) == checksum}
        if not matching:
            continue
        compatible_count += 1
        pack = registry / (map_id + ".zip")
        replaces = item.get("replaces_sha256", [])
        if not isinstance(replaces, list) or any(not isinstance(value, str) or not SHA.fullmatch(value)
                                                for value in replaces):
            raise ValueError(f"Invalid previous revision checksums: {map_id}")
        if pack.exists():
            installed = digest(pack)
            if installed == item["sha256"]:
                print(f'{item["label"]} is already installed and verified.', flush=True)
                continue
            if args.maps == "none":
                continue
            if installed not in replaces:
                try:
                    valid_local = read_pack(pack).map_id == map_id
                    with zipfile.ZipFile(pack) as local_pack:
                        valid_local = valid_local and local_pack.testzip() is None
                except (OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile):
                    valid_local = False
                if valid_local:
                    print(f'{item["label"]} has a local revision. Its installed files were preserved.')
                    continue
                print(f'Repairing the damaged {item["label"]} map pack.', flush=True)
                replaces = [*replaces, installed]
        elif not ask(item["label"], args.maps):
            continue
        archive = fetch(item, args, root / "cache", search_dirs=(Path.cwd(), Path.home() / "Downloads"))
        definition = read_pack(archive)
        if definition.map_id != map_id:
            raise ValueError(f"Map ID does not match the catalog: {map_id}")
        for platform, game in matching.items():
            check_base(definition, platform, game)
        register(archive, registry, replaces=replaces)
        print(f'Added {item["label"]}.')
    if args.maps == "all" and catalog["maps"] and compatible_count == 0:
        raise ValueError("No map pack in the catalog matches the installed CARLA runtime.")


def linux_shortcuts(root):
    applications = Path.home() / ".local/share/applications"
    desktop = Path(subprocess.check_output(["xdg-user-dir", "DESKTOP"], text=True).strip())
    for folder in (applications, desktop):
        (folder / "asurt-tools.desktop").unlink(missing_ok=True)
    entries = {
        f"asurt-{action}.desktop": (
            name, f'"{root}/current/bin/asurt" {action}', f"{root}/current/{ICON_ROOT}/{action}.png",
        )
        for name, action in ACTIONS.items()
    }
    start_icon = f"{root}/current/{ICON_ROOT}/start.png"
    open_map = f'"{root}/current/bin/asurt" open-map --map'
    entries["asurt-start.desktop"] = ("ASURT FlatAsphalt", f"{open_map} flat-asphalt", start_icon)
    sys.path.insert(0, str(root / "current" / "Simulation/CARLA/Launchers"))
    from map_packs import catalog as registered_maps
    silverstone = registered_maps(root / "state/maps").get("silverstone-section")
    if silverstone and "LinuxNoEditor" in silverstone.data["platforms"]:
        entries["asurt-silverstone.desktop"] = (
            "ASURT Silverstone Split", f"{open_map} silverstone-section", start_icon)
    entries["asurt-workspace.desktop"] = (
        "ASURT Workspace", f'xdg-open "{root}/workspace"', "folder",
    )
    for filename, (name, command, icon) in entries.items():
        data = (f"[Desktop Entry]\nType=Application\nName={name}\nExec={command}\nIcon={icon}\n"
                "Terminal=false\nCategories=Development;Education;\n")
        for folder in (applications, desktop):
            folder.mkdir(parents=True, exist_ok=True)
            shortcut = folder / filename
            shortcut.write_text(data)
            shortcut.chmod(0o755)
    for folder in (applications, desktop):
        legacy = folder / "CARLA System Manager.desktop"
        if not legacy.is_file():
            continue
        data = legacy.read_text(encoding="utf-8")
        old_entry = Path.home() / "ASURT/tools/Control Center/launch.sh"
        if any(line.startswith("Exec=") and str(old_entry) in line for line in data.splitlines()):
            legacy.write_text((folder / "asurt-manager.desktop").read_text(encoding="utf-8"),
                              encoding="utf-8")


def windows_shortcut_icons(installation, windows_root):
    destination = windows_root / "shortcut-icons"
    destination.mkdir(parents=True, exist_ok=True)
    for action in (*ACTIONS.values(), "start-wslg", "silverstone-split"):
        icon = {"start-wslg": "wslg", "start-novnc": "novnc", "novnc-viewer": "novnc"}.get(action, action)
        shutil.copyfile(installation / ICON_ROOT / f"{icon}.ico", destination / f"{action}.ico")


def record_release(state, args, manifest):
    path = state / "release-installation.json"
    production = args.repository.casefold() == PRODUCTION_REPOSITORY.casefold()
    if not args.github_cli and (not production or
                                (args.manifest and not args.manifest.startswith("https://"))):
        path.unlink(missing_ok=True)
        return
    bases = {}
    if args.platform == "windows":
        bases["WindowsNoEditor"] = manifest["runtime"]["windows"]["shipping_sha256"]
    if args.platform == "linux" or args.with_wsl_runtime:
        bases["LinuxNoEditor"] = manifest["runtime"]["linux"]["shipping_sha256"]
    catalog = args.catalog
    if catalog and not catalog.startswith("https://"):
        catalog = str(Path(catalog).resolve())
    data = dict(tag=manifest["tag"], platform=args.platform, repository=args.repository,
                branch=args.branch, base_sha256=bases, catalog=catalog)
    temporary = path.with_suffix(".next")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)
    (state / "update-channel.json").unlink(missing_ok=True)


def select_release_repository(args, manifest):
    # The transition release serves the bootstrap from the former repository.
    # Its manifest directs subsequent downloads and the install receipt to production.
    if (args.github_cli and args.repository.casefold() == LEGACY_REPOSITORY.casefold()
            and manifest.get("repository") == PRODUCTION_REPOSITORY):
        args.repository = PRODUCTION_REPOSITORY
        args.branch = "main"
    if args.repository.casefold() == PRODUCTION_REPOSITORY.casefold():
        args.github_cli = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", default="")
    parser.add_argument("--manifest")
    parser.add_argument("--metadata", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--github-cli", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--with-wsl-runtime", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--catalog")
    parser.add_argument("--branch", default="main")
    parser.add_argument("--root", type=Path, default=Path.home() / ".local/share/asurt")
    parser.add_argument("--windows-root", type=Path)
    parser.add_argument("--platform", choices=SHIP, default="linux")
    parser.add_argument("--maps", choices=("ask", "all", "none"), default="ask")
    parser.add_argument("--no-shortcuts", action="store_true")
    parser.add_argument("--fixture", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not (args.manifest and not args.manifest.startswith("https://")) and not REPO.fullmatch(args.repository):
        parser.error("Supply the GitHub owner/repository.")
    if args.platform == "windows" and args.windows_root is None:
        parser.error("A Windows installation directory is required.")
    if args.github_cli and (args.manifest or not args.metadata):
        parser.error("Authenticated downloads require the downloaded release metadata.")
    if args.with_wsl_runtime and args.platform != "windows":
        parser.error("The additional WSL runtime applies to the Windows installer.")
    root = args.root.expanduser().resolve()
    log = root / "state/log/carla"
    log.mkdir(parents=True, exist_ok=True)
    source = args.manifest or f"https://github.com/{args.repository}/releases/latest/download/asurt-release.json"
    manifest = read_json(str(args.metadata)) if args.metadata else read_json(source)
    if manifest.get("format") != 1:
        raise ValueError("Unsupported ASURT release format.")
    select_release_repository(args, manifest)
    args.tag = manifest["tag"]
    if args.repository.casefold() == PRODUCTION_REPOSITORY.casefold():
        channel = manifest.get("channel", "main")
        tags = {"main": r"v?\d+(?:\.\d+)+(?:-flat)?", "testing": r"testing-\d+"}
        if channel != args.branch or channel not in tags or not re.fullmatch(tags[channel], args.tag):
            raise ValueError("The installer channel and ASURT release manifest do not match.")
    with (log / "session.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Stop the ASURT simulator before installing an update.") from None
        runtime_asset = manifest["runtime"][args.platform]
        stack_asset = manifest["stack"]
        runtime_root = args.windows_root.resolve() if args.windows_root else root
        runtime = runtime_root / "runtimes" / runtime_asset["sha256"]
        stack = root / "stacks" / stack_asset["sha256"]
        cache = root / "cache"
        ensure_artifact(runtime_asset, args, cache, runtime, "Simulator", shipping=SHIP[args.platform])
        stack_repaired = ensure_artifact(stack_asset, args, cache, stack, "Modules and tools")
        patch_asset = manifest.get("stack_patch")
        if patch_asset:
            patch = root / "stack-patches" / patch_asset["sha256"]
            patch_repaired = ensure_artifact(patch_asset, args, cache, patch, "Editable workspace update")
        wsl_runtime = None
        if args.with_wsl_runtime:
            wsl_asset = manifest["runtime"]["linux"]
            wsl_runtime = root / "runtimes" / wsl_asset["sha256"]
            ensure_artifact(wsl_asset, args, cache, wsl_runtime, "WSLg simulator", shipping=SHIP["linux"])
        if patch_asset:
            stack = stack_with_patch(stack, patch, repair=stack_repaired or patch_repaired)
        cuda_root = None if args.fixture else prepare_inference(root, stack)
        installation = make_root(args, root, manifest, runtime, stack, cuda_root, wsl_runtime)
        enable_dynamic_prop_sensor_trace(runtime / "WindowsNoEditor" if args.platform == "windows" else runtime)
        if wsl_runtime:
            enable_dynamic_prop_sensor_trace(wsl_runtime)
        if args.platform == "windows":
            windows_shortcut_icons(installation, runtime_root)
        cuda_link = root / 'state/inference-cuda'
        if cuda_root:
            activate(cuda_root, cuda_link)
        else:
            cuda_link.unlink(missing_ok=True)
        seed_workspace(root, stack, installation)
        current = root / "current"
        previous = current.resolve() if current.is_symlink() else None
        activate(installation, current)
        record_release(root / "state", args, manifest)
        if previous and previous != installation:
            close_previous_manager(current, previous)
        try:
            update_maps(args, root, runtime, wsl_runtime)
        except (OSError, ValueError, KeyError, subprocess.CalledProcessError, zipfile.BadZipFile) as error:
            if args.maps == "all":
                raise RuntimeError(f"Base and tools installed. The requested map packs could not be installed: {error}") from error
            print(f"Base and tools installed. Silverstone or another optional map could not be installed: {error}\n"
                  "Rerun the installer with --maps all (Windows: -Maps all) to retry.", file=sys.stderr)
        if args.platform == "linux" and not args.no_shortcuts:
            linux_shortcuts(root)
    print(f"ASURT is ready at {root}.")
    print(f"Open ASURT Workspace: {root / 'workspace'}")
    print("  Modules/   ROS packages, parameters, and launch scripts")
    print("  tools/     System Manager, RViz configurations, and other desktop tools")
    print("  Assets/    Track CSVs and JSON, road descriptions, props, and installed map packs")
    print("  Settings/  Module settings and tool path overrides")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.CalledProcessError, tarfile.TarError, zipfile.BadZipFile) as error:
        sys.exit(f"ASURT installer: {error}")
