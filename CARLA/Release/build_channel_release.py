"""Package a channel update using the previous released CARLA and ROS binaries."""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import tarfile
import tempfile

import asurt_update
import build_release as builder


def startup_checks(archive, names):
    with tarfile.open(archive, "r:gz") as package:
        members = {entry.name.removeprefix("./"): entry for entry in package.getmembers()}
        hashes = {}
        for name in names:
            with package.extractfile(members[name]) as file:
                data = hashlib.sha256()
                for chunk in iter(lambda: file.read(1024 * 1024), b""):
                    data.update(chunk)
                hashes[name] = data.hexdigest()
        return hashes


def copy_changed_files(reference, overlay):
    changes = subprocess.check_output(
        ["git", "diff", "--no-renames", "--name-status", "-z", reference, "HEAD"],
        cwd=builder.REPOSITORY, text=True).split("\0")
    for status, name in zip(changes[::2], changes[1::2]):
        path = PurePosixPath(name)
        if path.as_posix() == "CARLA/Modules/FS-System-26.md":
            continue
        if path.parts == builder.DRIVING_SOURCE.parts and status in ("A", "M"):
            builder.copy_driving_window(overlay)
            continue
        if path.parts[:2] == ("CARLA", "Modules") and (
                (len(path.parts) > 5 and path.parts[3] == "Modules" and path.parts[4] == "LegacyFS26"
                 and (path.parts[2], path.parts[5]) in {(g, p) for g, p, _, _ in builder.FS26_PACKAGES})
                or (len(path.parts) == 5 and path.parts[3] == "Launch" and
                    path.name in (*builder.FS26_LAUNCHERS.get(path.parts[2], ()),
                                  "fs26_control_pair.py", "fs26_simple_bootstrap.py"))):
            continue
        if (len(path.parts) == 9 and path.parts[:5] == ("CARLA", "Modules", "Perception", "Adapters", "CARLA")
                and builder.PYTHON_ADAPTERS.get(path.parts[6]) == path.parts[5]
                and status in ("A", "M")):
            package = path.parts[6]
            supported = path.name in ("node.py", "weather.py") or (
                package == "asurt_velodyne_sim" and path.name == "ultra_puck.py")
            if path.parts[7] == package and supported:
                builder.copy_python_adapter_file(package, path.name, overlay)
                continue
            if path.parts[7] == "tests" and path.name.startswith("test_") and path.suffix == ".py":
                builder.copy(builder.REPOSITORY / path, overlay / builder.PERCEPTION / path.relative_to("CARLA/Modules/Perception"))
                continue
        if path.parts[:2] == ("CARLA", "Modules") or path.parts[:3] == ("CARLA", "Simulator", "Bridge"):
            raise ValueError(f"{name} requires updated compiled ROS overlays from a full release build.")
        if path.parts[:2] in (("CARLA", "Tools"), ("CARLA", "Assets")):
            if status.startswith("D"):
                raise ValueError(f"Removing {name} from an installed stack requires a full stack build.")
            target = ("tools" if path.parts[1] == "Tools" else "Assets", *path.parts[2:])
            builder.copy(builder.REPOSITORY / path, overlay.joinpath(*target))
            if path.parts[:3] == ("CARLA", "Tools", "MCP"):
                builder.copy(builder.REPOSITORY / path, overlay.joinpath("libexec/asurt-mcp", *path.parts[3:]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-manifest", type=Path, required=True)
    parser.add_argument("--base-patch", type=Path, required=True)
    parser.add_argument("--branch", choices=("main", "testing"), required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--asset-directory", type=Path,
                        help="Add verification for startup files from the verified base archives.")
    args = parser.parse_args()
    testing = args.branch == "testing"
    if not re.fullmatch(r"testing-\d+" if testing else r"v\d+\.\d+\.\d+-flat", args.tag):
        parser.error("Use testing-N for Testing and vX.Y.Z-flat for Release.")
    manifest = json.loads(args.base_manifest.read_text())
    reference = manifest["tag"]
    if set(manifest.get("runtime", {})) != {"linux", "windows"}:
        raise ValueError("Channel releases require both the Ubuntu and Windows runtimes.")
    if any(manifest["stack_patch"].get(key) != value for key, value in builder.item(args.base_patch).items()):
        raise ValueError("The baseline stack patch failed verification.")
    args.output.mkdir(parents=True, exist_ok=True)
    builder.write_installers(args.output, branch=args.branch)
    builder.copy(builder.HERE / "asurt_update.py", args.output / "asurt_update.py")
    builder.copy(builder.REPOSITORY / "asurt-maps.json", args.output / "asurt-maps.json")
    for asset in (manifest["stack"], *manifest["runtime"].values(), manifest.get("wsl_graphics", {})):
        if asset:
            asset.setdefault("tag", reference)
    if args.asset_directory:
        stack = manifest["stack"]
        archive = args.asset_directory / stack["file"]
        if builder.digest(archive) != stack["sha256"]:
            raise ValueError("The base stack archive failed verification.")
        launch = "Simulation/CARLA/Launchers"
        stack["startup_files"] = startup_checks(archive, ("bin/asurt", "bin/inference.py", f"{launch}/session.py"))
        for platform, asset in manifest["runtime"].items():
            archive = args.asset_directory / asset["file"]
            if builder.digest(archive) != asset["sha256"]:
                raise ValueError(f"The {platform} runtime archive failed verification.")
            prefix = "WindowsNoEditor/" if platform == "windows" else ""
            maps = prefix + "CarlaUE4/Content/Carla/Maps/"
            asset["startup_files"] = startup_checks(archive, (
                maps + "FlatAsphalt.umap", maps + "FlatAsphalt.uexp", maps + "OpenDrive/FlatAsphalt.xodr"))
    manifest.update(tag=args.tag, channel=args.branch,
                    updater=builder.item(args.output / "asurt_update.py"),
                    linux_installer=builder.item(args.output / "Install-ASURT.sh"),
                    ubuntu_packages=(builder.HERE / "ubuntu-packages.txt").read_text().splitlines())
    patch = args.output / manifest["stack_patch"]["file"]
    with tempfile.TemporaryDirectory() as temporary:
        overlay = Path(temporary) / "overlay"
        asurt_update.extract(args.base_patch, overlay)
        copy_changed_files(reference, overlay)
        launchers = builder.REPOSITORY / "CARLA/Simulator/Launchers"
        builder.adapt_launchers(launchers, overlay)
        builder.stage_fs26(overlay)
        for name in ("Install-ASURT.sh", "Install-ASURT.ps1"):
            builder.copy(args.output / name, overlay / "bin" / name)
        builder.copy(builder.REPOSITORY / "ROS_TOPICS.md", overlay / "tools/RViz/ROS_TOPIC_OVERVIEW.md")
        builder.relocate(overlay, builder.REPOSITORY)
        builder.pack_directory(overlay, patch)
    manifest["stack_patch"] = builder.item(patch)
    launch = "Simulation/CARLA/Launchers"
    manifest["stack_patch"]["startup_files"] = startup_checks(
        patch, ("bin/asurt", f"{launch}/session.py", f"{launch}/world_selection.py",
                f"{launch}/map_packs.py", f"{launch}/ros-parity-env.sh", f"{launch}/release_update.py",
                f"{launch}/weather_grip.py", f"{launch}/road_weather.py", f"{launch}/windows_interop.py",
                f"{launch}/windows-server.ps1", f"{launch}/process_ownership.py",
                "tools/RViz/modules.conf", "retired-modules.json", "libexec/asurt-fs26/control_pair",
                "libexec/asurt-fs26/classical_goal"))
    (args.output / "asurt-release.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
