"""Package a channel update using the previous released CARLA and ROS binaries."""

import argparse
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import tempfile

import asurt_update
import build_release as builder


def copy_changed_files(reference, overlay):
    changes = subprocess.check_output(
        ["git", "diff", "--no-renames", "--name-status", "-z", reference, "HEAD"],
        cwd=builder.REPOSITORY, text=True).split("\0")
    for status, name in zip(changes[::2], changes[1::2]):
        path = PurePosixPath(name)
        if path.parts[:2] == ("CARLA", "Modules") or path.parts[:3] == ("CARLA", "Simulator", "Bridge"):
            raise ValueError(f"{name} requires updated compiled ROS overlays from a full release build.")
        if path.parts[:2] in (("CARLA", "Tools"), ("CARLA", "Assets")):
            if status.startswith("D"):
                raise ValueError(f"Removing {name} from an installed stack requires a full stack build.")
            target = ("tools" if path.parts[1] == "Tools" else "Assets", *path.parts[2:])
            builder.copy(builder.REPOSITORY / path, overlay.joinpath(*target))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-manifest", type=Path, required=True)
    parser.add_argument("--base-patch", type=Path, required=True)
    parser.add_argument("--branch", choices=("main", "testing"), required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    testing = args.branch == "testing"
    if not re.fullmatch(r"testing-\d+" if testing else r"v\d+\.\d+\.\d+-flat", args.tag):
        parser.error("Use testing-N for Testing and vX.Y.Z-flat for Release.")
    manifest = json.loads(args.base_manifest.read_text())
    reference = manifest["tag"]
    if builder.item(args.base_patch) != manifest["stack_patch"]:
        raise ValueError("The baseline stack patch failed verification.")
    args.output.mkdir(parents=True, exist_ok=True)
    builder.write_installers(args.output, branch=args.branch)
    builder.copy(builder.HERE / "asurt_update.py", args.output / "asurt_update.py")
    builder.copy(builder.REPOSITORY / "asurt-maps.json", args.output / "asurt-maps.json")
    for asset in (manifest["stack"], *manifest["runtime"].values(), manifest.get("wsl_graphics", {})):
        if asset:
            asset.setdefault("tag", reference)
    manifest.update(tag=args.tag, channel=args.branch,
                    updater=builder.item(args.output / "asurt_update.py"),
                    linux_installer=builder.item(args.output / "Install-ASURT.sh"))
    patch = args.output / manifest["stack_patch"]["file"]
    with tempfile.TemporaryDirectory() as temporary:
        overlay = Path(temporary) / "overlay"
        asurt_update.extract(args.base_patch, overlay)
        copy_changed_files(reference, overlay)
        launchers = builder.REPOSITORY / "CARLA/Simulator/Launchers"
        builder.adapt_launchers(launchers, overlay)
        for name in ("Install-ASURT.sh", "Install-ASURT.ps1"):
            builder.copy(args.output / name, overlay / "bin" / name)
        builder.copy(builder.REPOSITORY / "ROS_TOPICS.md", overlay / "tools/RViz/ROS_TOPIC_OVERVIEW.md")
        builder.relocate(overlay, builder.REPOSITORY)
        builder.pack_directory(overlay, patch)
    manifest["stack_patch"] = builder.item(patch)
    (args.output / "asurt-release.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
