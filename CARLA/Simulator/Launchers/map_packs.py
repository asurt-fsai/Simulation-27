"""Build, register, and install independently cooked ASURT maps."""

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
import zipfile


PLATFORMS = {
    "WindowsNoEditor": Path("CarlaUE4/Binaries/Win64/CarlaUE4-Win64-Shipping.exe"),
    "LinuxNoEditor": Path("CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"),
}
CONTENT = Path("CarlaUE4/Content")
EXTENSIONS = {".umap", ".uasset", ".uexp", ".ubulk", ".uptnl", ".xodr"}
RESERVED_IDS = {"flat-asphalt", "silverstone"}
MAP_PACK_DIR = Path.home() / "ASURT/Simulation/CARLA/Maps/Installed"
ID_PATTERN = re.compile(r"[a-z][a-z0-9-]{0,47}")
PACKAGE_PART = re.compile(r"[A-Za-z0-9_]+(?:-[A-Za-z0-9_]+)*")
SHA256 = re.compile(r"[0-9a-f]{64}")


def digest(file):
    result = hashlib.sha256()
    for block in iter(lambda: file.read(1024 * 1024), b""):
        result.update(block)
    return result.hexdigest()


def file_digest(path):
    with path.open("rb") as file:
        return digest(file)


def namespace(map_id):
    if not isinstance(map_id, str) or not ID_PATTERN.fullmatch(map_id) or map_id in RESERVED_IDS:
        raise ValueError(f"Invalid or reserved map ID: {map_id!r}")
    return PurePosixPath("ASURTMaps", map_id.replace("-", "_"))


def pose(values):
    if not isinstance(values, dict) or set(values) != {"x", "y", "z", "yaw"}:
        raise ValueError("Map poses must contain x, y, z, and yaw.")
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values.values()):
        raise ValueError("Map coordinates must be finite numbers.")
    return values


def valid_content_name(name):
    path = PurePosixPath(name)
    return (path.as_posix() == name and not path.is_absolute() and path.suffix in EXTENSIONS
            and all(PACKAGE_PART.fullmatch(part) for part in (*path.parts[:-1], path.stem)))


@dataclass(frozen=True)
class MapPack:
    archive: Path
    data: dict

    @property
    def map_id(self):
        return self.data["id"]

    @property
    def map_path(self):
        return self.data["map"].removeprefix("/Game/")

    @property
    def label(self):
        return self.data["label"]

    @property
    def placement(self):
        return {name: self.data[name] for name in ("spawn", "course_origin")}


def validate(data):
    if data.get("format") != 1:
        raise ValueError("Unsupported ASURT map pack format.")
    area = namespace(data.get("id"))
    label = data.get("label")
    if not isinstance(label, str) or not label.strip() or len(label) > 80 or not label.isprintable():
        raise ValueError("Invalid map label.")
    map_path = data.get("map", "")
    package = PurePosixPath(map_path.removeprefix("/Game/"))
    if (not map_path.startswith("/Game/") or map_path != "/Game/" + package.as_posix()
            or package.parent != area or not PACKAGE_PART.fullmatch(package.name)):
        raise ValueError(f"Map must be an Unreal package under /Game/{area}/.")
    pose(data.get("spawn"))
    pose(data.get("course_origin"))
    platforms = data.get("platforms", {})
    if not isinstance(platforms, dict) or not platforms or not platforms.keys() <= PLATFORMS.keys():
        raise ValueError("The map pack must contain a supported platform.")
    for platform, payload in platforms.items():
        if not SHA256.fullmatch(payload.get("base_sha256", "")):
            raise ValueError(f"Missing shipping executable fingerprint for {platform}.")
        files = payload.get("files", {})
        if not isinstance(files, dict) or not files:
            raise ValueError(f"No cooked files for {platform}.")
        if len({name.casefold() for name in files}) != len(files):
            raise ValueError(f"Duplicate package paths for {platform}.")
        shared = payload.get("shared", {})
        if not isinstance(shared, dict) or any(not valid_content_name(name)
                                                or PurePosixPath(name).is_relative_to(area)
                                                or not SHA256.fullmatch(checksum)
                                                for name, checksum in shared.items()):
            raise ValueError(f"Invalid shared dependencies for {platform}.")
        if {name for name in files if name.endswith(".umap")} != {str(package.with_suffix(".umap"))}:
            raise ValueError(f"{platform} must contain exactly the selected map.")
        road = area / "OpenDrive" / (package.name + ".xodr")
        if {name for name in files if name.endswith(".xodr")} != {str(road)}:
            raise ValueError(f"{platform} requires the selected map's OpenDRIVE file.")
        for name, checksum in files.items():
            relative = PurePosixPath(name)
            if (not valid_content_name(name) or not relative.is_relative_to(area)
                    or not SHA256.fullmatch(checksum)):
                raise ValueError(f"Invalid map pack member: {name!r}")


def read_pack(path):
    with zipfile.ZipFile(path) as archive:
        data = json.loads(archive.read("manifest.json"))
        validate(data)
        expected = {"manifest.json"}
        for platform, payload in data["platforms"].items():
            expected.update(f"{platform}/{name}" for name in payload["files"])
        names = archive.namelist()
        if len(names) != len(expected) or set(names) != expected:
            raise ValueError("The archive contents do not match the map manifest.")
    return MapPack(path, data)


def catalog(directory=MAP_PACK_DIR):
    result = {}
    for archive in sorted(directory.glob("*.zip")):
        try:
            pack = read_pack(archive)
        except (OSError, ValueError, KeyError, TypeError, AttributeError, zipfile.BadZipFile):
            continue
        if archive.stem == pack.map_id:
            result[pack.map_id] = pack
    return result


def register(archive, directory=MAP_PACK_DIR, *, replaces=()):
    pack = read_pack(archive)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / (pack.map_id + ".zip")
    previous = file_digest(target) if target.exists() else None
    if previous is not None:
        if file_digest(archive) == previous:
            return read_pack(target)
        if previous not in replaces:
            raise ValueError(f"{pack.map_id} already exists and is not the expected previous revision.")
    with tempfile.TemporaryDirectory(dir=directory) as temporary:
        staged = Path(temporary) / target.name
        shutil.copyfile(archive, staged)
        read_pack(staged)
        if previous is None:
            try:
                os.link(staged, target)
            except FileExistsError:
                if file_digest(staged) != file_digest(target):
                    raise
        else:
            if file_digest(target) != previous:
                raise ValueError(f"{pack.map_id} changed during its update.")
            history = directory / ".history"
            history.mkdir(exist_ok=True)
            backup = history / f"{previous}.zip"
            if not backup.exists():
                os.link(target, backup)
            if file_digest(backup) != previous:
                raise ValueError(f"The previous revision backup differs: {backup}")
            staged.replace(target)
    return read_pack(target)


def check_base(pack, platform, runtime):
    payload = pack.data["platforms"].get(platform)
    if payload is None:
        raise ValueError(f"{pack.label} has no {platform} cook.")
    if file_digest(runtime / PLATFORMS[platform]) != payload["base_sha256"]:
        raise ValueError(f"{pack.label} was cooked for a different {platform} CARLA build.")
    content = (runtime / CONTENT).resolve(strict=True)
    for name, checksum in payload.get("shared", {}).items():
        dependency = content / name
        if not dependency.is_file() or file_digest(dependency) != checksum:
            raise ValueError(f"{pack.label} needs a different base asset: {name}")
    return content, payload


def matches_files(content, files):
    return all((content / name).resolve() == content / name
               and (content / name).is_file() and file_digest(content / name) == checksum
               for name, checksum in files.items())


def can_replace_installed(pack, platform, content, target):
    entries = list(target.rglob("*"))
    if any(path.is_symlink() for path in entries):
        return False
    existing = {file.relative_to(content).as_posix() for file in entries if file.is_file()}
    for archive in (pack.archive.parent / ".history").glob("*.zip"):
        if not SHA256.fullmatch(archive.stem) or file_digest(archive) != archive.stem:
            continue
        old = read_pack(archive)
        payload = old.data["platforms"].get(platform)
        if old.map_id == pack.map_id and payload and existing == set(payload["files"]):
            if matches_files(content, payload["files"]):
                return True
    return False


def install(pack, platform, runtime):
    content, payload = check_base(pack, platform, runtime)
    area = namespace(pack.map_id)
    target = content / area
    target.parent.mkdir(exist_ok=True)
    if target.parent.resolve() != content / "ASURTMaps" or target.is_symlink():
        raise ValueError("Map pack destination redirects outside the CARLA content directory.")
    replacing = target.exists()
    if replacing and matches_files(content, payload["files"]):
        return
    if replacing and not can_replace_installed(pack, platform, content, target):
        raise ValueError(f"Installed map content differs from a known revision: {target}")
    with tempfile.TemporaryDirectory(dir=runtime) as temporary:
        stage = Path(temporary) / target.name
        stage.mkdir()
        with zipfile.ZipFile(pack.archive) as archive:
            for name, checksum in payload["files"].items():
                destination = stage / PurePosixPath(name).relative_to(area)
                destination.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(f"{platform}/{name}") as src, destination.open("xb") as dst:
                    shutil.copyfileobj(src, dst)
                if file_digest(destination) != checksum:
                    raise ValueError(f"Map pack member failed its SHA-256 check: {name}")
        old = Path(temporary) / "previous"
        if replacing:
            target.rename(old)
        try:
            stage.rename(target)
        except OSError:
            if replacing:
                old.rename(target)
            raise


def build(args):
    area = namespace(args.id)
    map_path = PurePosixPath(args.map.removeprefix("/Game/"))
    if map_path.parent != area:
        raise ValueError(f"Move the map and its unique assets under /Game/{area}/ before cooking.")
    placement = dict(zip(("x", "y", "z", "yaw"), args.spawn))
    data = {"format": 1, "id": args.id, "label": args.label, "map": args.map, "spawn": placement,
            "course_origin": dict(zip(("x", "y", "z", "yaw"), args.origin or args.spawn)), "platforms": {}}
    dependencies = json.loads(args.dependencies.read_text(encoding="utf-8-sig"))
    if (not isinstance(dependencies, list) or args.map not in dependencies
            or any(not isinstance(name, str) or not name.startswith("/Game/")
                   or not valid_content_name(name.removeprefix("/Game/") + ".uasset")
                   for name in dependencies)):
        raise ValueError("The Unreal dependency report must contain the selected map.")
    required = {name.removeprefix("/Game/").casefold(): PurePosixPath(name.removeprefix("/Game/"))
                for name in dependencies}
    members = {}
    for platform, cook, base in args.platform:
        if platform not in PLATFORMS or platform in data["platforms"]:
            raise ValueError(f"Unsupported or duplicate platform: {platform}")
        cooked = Path(cook) / CONTENT
        runtime = Path(base)
        files = {}
        shared = {}
        seen = set()
        for path in sorted(cooked.rglob("*")):
            if not path.is_file() or path.suffix not in EXTENSIONS - {".xodr"}:
                continue
            relative = path.relative_to(cooked).as_posix()
            package_name = PurePosixPath(relative).with_suffix("").as_posix().casefold()
            if package_name not in required:
                continue
            if path.suffix in {".umap", ".uasset"}:
                seen.add(package_name)
            if not PurePosixPath(relative).is_relative_to(area):
                source = runtime / CONTENT / relative
                if not source.is_file():
                    raise ValueError(f"Dependency is absent from the base. Move it into this map's folder and recook: {relative}")
                shared[relative] = file_digest(source)
                continue
            files[relative] = path
        for name in required.keys() - seen:
            relative = required[name]
            if relative.is_relative_to(area):
                raise ValueError(f"Cooked map dependency missing for {platform}: {relative}")
            package = runtime / CONTENT / relative
            reused = [path for path in package.parent.glob(package.name + ".*") if path.suffix in EXTENSIONS]
            if not any(path.is_file() and path.suffix in {".umap", ".uasset"} for path in reused):
                raise ValueError(f"Shared map dependency missing for {platform}: {relative}")
            for path in reused:
                shared[path.relative_to(runtime / CONTENT).as_posix()] = file_digest(path)
        files[(area / "OpenDrive" / (map_path.name + ".xodr")).as_posix()] = args.xodr
        data["platforms"][platform] = {"base_sha256": file_digest(runtime / PLATFORMS[platform]),
                                      "files": {name: file_digest(src) for name, src in files.items()},
                                      "shared": shared}
        members.update({f"{platform}/{name}": src for name, src in files.items()})
    validate(data)
    with zipfile.ZipFile(args.output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("manifest.json", json.dumps(data, indent=2) + "\n")
        for name, source in members.items():
            archive.write(source, name)
    return read_pack(args.output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    create = actions.add_parser("build")
    create.add_argument("output", type=Path)
    create.add_argument("--id", required=True)
    create.add_argument("--label", required=True)
    create.add_argument("--map", required=True)
    create.add_argument("--xodr", type=Path, required=True)
    create.add_argument("--dependencies", type=Path, required=True)
    create.add_argument("--spawn", nargs=4, type=float, required=True, metavar=("X", "Y", "Z", "YAW"))
    create.add_argument("--origin", nargs=4, type=float, metavar=("X", "Y", "Z", "YAW"))
    create.add_argument("--platform", nargs=3, action="append", required=True,
                        metavar=("PLATFORM", "COOK_ROOT", "BASE_RUNTIME"))
    add = actions.add_parser("add")
    add.add_argument("archive", type=Path)
    actions.add_parser("list")
    args = parser.parse_args()
    if args.action == "list":
        for pack in catalog().values():
            print(f"{pack.map_id}\t{pack.label}\t{pack.map_path}")
    else:
        pack = build(args) if args.action == "build" else register(args.archive)
        print(f"{pack.label}: {pack.archive}")


if __name__ == "__main__":
    main()
