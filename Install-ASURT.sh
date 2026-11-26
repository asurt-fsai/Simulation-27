#!/usr/bin/env bash
set -euo pipefail
repository="${ASURT_REPOSITORY:-asurt-fsai/Simulation-27}"
manifest=
platform=linux
windows_root=
root="$HOME/.local/share/asurt"
maps=ask
skip_dependencies=false
extra=()
while (($#)); do
    case "$1" in
        --repository) repository=${2:?}; shift 2 ;;
        --manifest) manifest=${2:?}; shift 2 ;;
        --platform) platform=${2:?}; shift 2 ;;
        --windows-root) windows_root=${2:?}; shift 2 ;;
        --root) root=${2:?}; shift 2 ;;
        --maps) maps=${2:?}; shift 2 ;;
        --catalog|--branch) extra+=("$1" "${2:?}"); shift 2 ;;
        --no-shortcuts|--fixture) extra+=("$1"); shift ;;
        --skip-dependencies) skip_dependencies=true; shift ;;
        --help) echo "Usage: bash Install-ASURT.sh --repository OWNER/REPO [--maps ask|all|none]"; exit ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done

if [[ "$manifest" == https://* || -z "$manifest" ]]; then
    [[ "$repository" =~ ^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$ ]] ||
        { echo "Supply the GitHub repository with --repository OWNER/REPO." >&2; exit 2; }
fi
if [[ "$skip_dependencies" == false ]]; then
    source /etc/os-release
    if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 || "$(uname -m)" != x86_64 ]]; then
        echo "This ASURT release requires Ubuntu 22.04 x86-64." >&2
        exit 2
    fi
    if [[ "$(id -u)" == 0 ]]; then
        echo "Run as your desktop user. The installer will request sudo for system packages." >&2
        exit 2
    fi
    missing=()
    bootstrap_packages=(python3 curl ca-certificates)
    if [[ -z "$manifest" ]]; then bootstrap_packages+=(gh); fi
    for name in "${bootstrap_packages[@]}"; do
        dpkg-query -W -f='${db:Status-Status}' "$name" 2>/dev/null | grep -qx installed || missing+=("$name")
    done
    if ((${#missing[@]})); then
        sudo apt-get update
        sudo apt-get install -y "${missing[@]}"
    fi
fi

temp=$(mktemp -d)
trap 'rm -rf -- "$temp"' EXIT
manifest_source=$manifest
if [[ -z "$manifest_source" ]]; then
    if ! gh api --hostname github.com user >/dev/null 2>&1; then
        printf 'Sign into GitHub with access to %s.\n' "$repository"
        gh auth login --hostname github.com --web
    fi
    gh release download --repo "$repository" --pattern asurt-release.json --dir "$temp"
    mv -- "$temp/asurt-release.json" "$temp/manifest.json"
    extra+=(--github-cli)
elif [[ "$manifest_source" == https://* ]]; then
    curl -fsSL --retry 2 "$manifest_source" -o "$temp/manifest.json"
else
    cp -- "$manifest_source" "$temp/manifest.json"
fi
python3 - "$temp/manifest.json" "$manifest_source" "$repository" "$temp/asurt_update.py" <<'PY'
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
from urllib.request import urlopen

meta, source, repo, output = sys.argv[1:]
data = json.loads(Path(meta).read_text())
tag = data["tag"]
asset = data["updater"]
name, checksum = asset["file"], asset["sha256"]
if name != "asurt_update.py" or not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9._-]*", tag):
    sys.exit("Invalid ASURT updater asset or release tag.")
target = Path(output)
if not source:
    subprocess.run(["gh", "release", "download", tag, "--repo", repo,
                    "--pattern", name, "--dir", str(target.parent)], check=True)
elif source.startswith("https://"):
    with urlopen(f"https://github.com/{repo}/releases/download/{tag}/{name}", timeout=90) as response:
        target.write_bytes(response.read())
else:
    shutil.copyfile(Path(source).resolve().parent / name, target)
if not re.fullmatch(r"[a-f0-9]{64}", checksum) or hashlib.sha256(target.read_bytes()).hexdigest() != checksum:
    sys.exit("ASURT updater checksum verification failed.")
PY

if [[ "$skip_dependencies" == false ]]; then
    if ! apt-cache show ros-humble-ros-base >/dev/null 2>&1 && [[ ! -r /opt/ros/humble/setup.bash ]]; then
        echo "Installing the ROS 2 Ubuntu repository configurationâ€¦"
        url=$(python3 - <<'PY'
import json
from urllib.request import urlopen
with urlopen("https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest", timeout=60) as r:
    assets = json.load(r)["assets"]
for asset in assets:
    if asset["name"].startswith("ros2-apt-source_") and asset["name"].endswith(".jammy_all.deb"):
        print(asset["browser_download_url"])
        break
else:
    raise SystemExit("The ROS 2 repository did not provide an Ubuntu 22.04 package.")
PY
        )
        [[ "$url" == https://github.com/ros-infrastructure/ros-apt-source/releases/download/* ]]
        curl -fsSL --retry 2 "$url" -o "$temp/ros2-apt-source.deb"
        sudo apt-get install -y "$temp/ros2-apt-source.deb"
        sudo apt-get update
    fi
    python3 - "$temp/manifest.json" >"$temp/ubuntu-packages.txt" <<'PY'
import json
from pathlib import Path
import re
import sys
packages = json.loads(Path(sys.argv[1]).read_text())["ubuntu_packages"]
if not isinstance(packages, list) or not all(isinstance(p, str) and re.fullmatch(r"[a-z0-9][a-z0-9+.-]*", p) for p in packages):
    raise SystemExit("Invalid Ubuntu dependency list in release manifest.")
print("\n".join(packages))
PY
    mapfile -t packages <"$temp/ubuntu-packages.txt"
    if [[ "$platform" == windows ]]; then
        packages+=(vulkan-tools libdrm2 libxcb1 libx11-xcb1 libxcb-dri3-0 libxcb-present0
            libxcb-xfixes0 libxcb-sync1 libxcb-randr0 libxcb-shm0 libxshmfence1 libexpat1 zlib1g libzstd1)
    fi
    missing=()
    for name in "${packages[@]}"; do
        dpkg-query -W -f='${db:Status-Status}' "$name" 2>/dev/null | grep -qx installed || missing+=("$name")
    done
    if ((${#missing[@]})); then
        sudo apt-get update
        sudo apt-get install -y "${missing[@]}"
    fi
fi
if [[ "$platform" == windows && "$skip_dependencies" == false ]]; then
    [[ $(uname -r) == *microsoft-standard* || $(uname -r) == *WSL2* ]] || {
        echo 'The Windows installation must run inside Ubuntu on WSL2.' >&2
        exit 2
    }
    kit="$HOME/.local/opt/asurt-dozen-20261024"
    dozen_archive=asurt-mesa-dozen-wsl-team-kit-20261024.tar.gz
    dozen_sha=f729e89f8e069b4e717a3b38d5ce89311789a38b81e450d4b6014e08b3204200
    if [[ ! -e "$kit" && ! -L "$kit" ]]; then
        echo 'Installing the pinned Mesa Dozen userspace driver inside WSL.'
        dozen_release=$(python3 - "$temp/manifest.json" "$dozen_archive" "$dozen_sha" <<'PY'
import json
from pathlib import Path
import re
import sys

manifest = json.loads(Path(sys.argv[1]).read_text())
asset = manifest.get("wsl_graphics")
if asset is not None:
    tag = asset.get("tag", manifest["tag"])
    if (asset["file"], asset["sha256"]) != tuple(sys.argv[2:]) or not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9._-]*", tag):
        sys.exit("Invalid Mesa Dozen asset in the release manifest.")
    print(tag)
PY
        )
        if [[ -z "$dozen_release" ]]; then
            curl -fsSL --retry 2 "https://github.com/hassanradwannn/Mesa-Dozen-Custom-WSL-driver-for-Carla/releases/download/v0.1.0/$dozen_archive" -o "$temp/$dozen_archive"
        elif [[ -z "$manifest_source" ]]; then
            gh release download "$dozen_release" --repo "$repository" --pattern "$dozen_archive" --dir "$temp"
        elif [[ "$manifest_source" == https://* ]]; then
            curl -fsSL --retry 2 "https://github.com/$repository/releases/download/$dozen_release/$dozen_archive" -o "$temp/$dozen_archive"
        else
            cp -- "$(dirname -- "$manifest_source")/$dozen_archive" "$temp/$dozen_archive"
        fi
        printf '%s  %s\n' "$dozen_sha" "$temp/$dozen_archive" | sha256sum --check
        mkdir "$temp/dozen"
        tar -xzf "$temp/$dozen_archive" -C "$temp/dozen"
        ASURT_DOZEN_PREFIX="$kit" bash "$temp/dozen/scripts/install.sh"
    fi
    expected_driver=cce0e8ac41b33b179c58e1a315c94d153fe1a7f031e57d09585a7d20616ccf78
    printf '%s  %s\n' "$expected_driver" "$kit/variants/descriptor-reset-candidate/lib/x86_64-linux-gnu/libvulkan_dzn.so" | sha256sum --check
    bash "$kit/scripts/verify.sh" --files-only
    if "$kit/scripts/run-dozen.sh" vulkaninfo --summary >"$temp/dozen-vulkan.log" 2>&1 &&
       grep -Eiq 'deviceName[[:space:]]*=[[:space:]]*Microsoft Direct3D12' "$temp/dozen-vulkan.log" &&
       grep -Eiq 'driverName[[:space:]]*=[[:space:]]*Dozen' "$temp/dozen-vulkan.log"; then
        echo 'WSL2 sees the Windows GPU through Mesa Dozen.'
    else
        cat "$temp/dozen-vulkan.log" >&2
        echo 'The Dozen files are installed. WSLg CARLA needs a working Windows GPU driver with WSL GPU support.' >&2
    fi
    extra+=(--with-wsl-runtime)
fi
[[ -z "$windows_root" ]] || extra+=(--windows-root "$windows_root")
[[ -z "$manifest" ]] || extra+=(--manifest "$manifest")
python3 "$temp/asurt_update.py" --metadata "$temp/manifest.json" --repository "$repository" --root "$root" --platform "$platform" --maps "$maps" "${extra[@]}"
