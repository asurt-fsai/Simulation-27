"""Check the installed ASURT release and hand off updates to an interactive terminal."""

import base64
import hashlib
import json
import os
from pathlib import Path
from queue import Empty, Queue
import re
import shutil
import subprocess
import sys
import tempfile
from threading import Thread
from urllib.parse import quote
from urllib.request import urlopen


RECEIPT = "release-installation.json"
PRODUCTION_REPOSITORY = "asurt-fsai/Simulation-27"
LEGACY_REPOSITORY = "hassanradwannn/ASURTSIM"
REPOSITORY = re.compile(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+")
MAP_ID = re.compile(r"[a-z][a-z0-9-]{0,47}")
VERSION = re.compile(r"v?(\d+(?:\.\d+)+)(?:-flat)?")


def receipt(state):
    try:
        installed = json.loads((state / RECEIPT).read_text())
    except FileNotFoundError:
        return None
    if not REPOSITORY.fullmatch(installed["repository"]):
        raise ValueError("Invalid ASURT release repository.")
    if installed["repository"].casefold() == LEGACY_REPOSITORY.casefold():
        installed.update(repository=PRODUCTION_REPOSITORY, branch="main")
    return installed


def newer_revision(installed, latest):
    current, published = VERSION.fullmatch(installed), VERSION.fullmatch(latest)
    if not current or not published:
        return False
    return tuple(map(int, published[1].split("."))) > tuple(map(int, current[1].split(".")))


def available_maps(maps, registry, bases):
    result = []
    for item in maps:
        ident = item["id"]
        if not MAP_ID.fullmatch(ident) or ident in {"flat-asphalt", "silverstone"}:
            continue
        if not any(item["base_sha256"].get(platform) == digest for platform, digest in bases.items()):
            continue
        archive = registry / f"{ident}.zip"
        if archive.is_file():
            value = hashlib.sha256()
            with archive.open("rb") as source:
                for data in iter(lambda: source.read(1024 * 1024), b""):
                    value.update(data)
            checksum = value.hexdigest()
            if checksum == item["sha256"] or checksum not in item.get("replaces_sha256", []):
                continue
        result.append(item["label"])
    return result


def check_updates(state):
    installed = receipt(state)
    if not installed:
        return None
    repo = installed["repository"]
    production = repo.casefold() == PRODUCTION_REPOSITORY.casefold()
    environment = dict(os.environ, GH_PROMPT_DISABLED="1", GIT_TERMINAL_PROMPT="0")
    latest = None
    bases = installed["base_sha256"]
    # Check them separately so a map catalog outage does not suppress a release.
    try:
        if production:
            url = f"https://github.com/{repo}/releases/latest/download/asurt-release.json"
            with urlopen(url, timeout=15) as stream:
                manifest = json.load(stream)
        else:
            with tempfile.TemporaryDirectory() as directory:
                subprocess.run(["gh", "release", "download", "--repo", repo, "--pattern",
                                "asurt-release.json", "--dir", directory], env=environment,
                               stdin=subprocess.DEVNULL, capture_output=True, timeout=15, check=True)
                manifest = json.loads((Path(directory) / "asurt-release.json").read_text())
        if manifest.get("format") == 1 and newer_revision(installed["tag"], manifest["tag"]):
            updated = {platform: manifest["runtime"][key]["shipping_sha256"]
                       for platform, key in (("WindowsNoEditor", "windows"), ("LinuxNoEditor", "linux"))
                       if platform in bases}
            latest, bases = manifest["tag"], updated
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError):
        pass

    maps = []
    try:
        source = installed.get("catalog")
        if source and source.startswith("https://"):
            with urlopen(source, timeout=12) as stream:
                catalog = json.load(stream)
        elif source:
            catalog = json.loads(Path(source).read_text())
        else:
            branch = quote(installed.get("branch", "main"), safe="")
            if production:
                with urlopen(f"https://raw.githubusercontent.com/{repo}/{branch}/asurt-maps.json",
                             timeout=12) as stream:
                    catalog = json.load(stream)
            else:
                catalog = json.loads(subprocess.check_output(
                    ["gh", "api", f"repos/{repo}/contents/asurt-maps.json?ref={branch}",
                     "-H", "Accept: application/vnd.github.raw"], env=environment,
                    stdin=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=12))
        if catalog.get("format") == 1:
            maps = available_maps(catalog["maps"], state / "maps", bases)
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError):
        pass
    return (latest, maps) if latest or maps else None


def terminal_command(root, platform):
    command = ["/bin/bash", str(root / "bin/asurt"), "exec", "/usr/bin/python3", str(Path(__file__).resolve())]
    if platform == "windows":
        if not (root / "bin/Install-ASURT.ps1").is_file():
            raise RuntimeError("The bundled Windows installer is missing.")
        distro = os.environ.get("WSL_DISTRO_NAME", "Ubuntu-22.04")
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", distro):
            raise ValueError("Invalid WSL distribution name.")
        arguments = subprocess.list2cmdline(["-d", distro, "--", *command])
        quoted = "'" + arguments.replace("'", "''") + "'"
        script = ("$ErrorActionPreference='Stop'; Start-Process -FilePath wsl.exe "
                  f"-ArgumentList {quoted} -WindowStyle Normal")
        encoded = base64.b64encode(script.encode("utf-16-le")).decode()
        return ["powershell.exe", "-NoProfile", "-NonInteractive", "-EncodedCommand", encoded]
    if not (root / "bin/Install-ASURT.sh").is_file():
        raise RuntimeError("The bundled Ubuntu installer is missing.")
    terminal = shutil.which("gnome-terminal")
    if terminal:
        return [terminal, "--", *command]
    terminal = shutil.which("x-terminal-emulator")
    if terminal:
        return [terminal, "-e", *command]
    raise RuntimeError("No desktop terminal is available to run the ASURT installer.")


def launch_terminal(root, platform):
    command = terminal_command(root, platform)
    if platform == "windows" or Path(command[0]).name == "gnome-terminal":
        # These launchers return after handing off to the interactive terminal.
        subprocess.run(command, stdin=subprocess.DEVNULL, capture_output=True, check=True, timeout=10)
    else:
        subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)


def prompt(release, maps):
    import tkinter as tk
    from tkinter import messagebox

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    details = [f"New ASURT release {release}"] if release else []
    if maps:
        details.append("New or updated maps\n" + "\n".join(f"• {name}" for name in maps[:8]))
        if len(maps) > 8:
            details.append(f"Plus {len(maps)-8} more maps")
    details.append("Download now? ASURT will close its simulator and tools and open the installer in a terminal.")
    try:
        if not messagebox.askyesno("ASURT updates available", "\n\n".join(details), parent=root):
            return False
        try:
            launch_terminal(Path(os.environ["ASURT_ROOT"]), os.environ["ASURT_PROFILE"])
        except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
            messagebox.showerror("Could not open the installer", str(error), parent=root)
            return False
        return True
    finally:
        root.destroy()


def run_manager(command):
    """Check once alongside the manager, including managers with team-local edits."""
    result = Queue()

    def check():
        try:
            updates = check_updates(Path(os.environ["ASURT_STATE_ROOT"]))
        except (OSError, ValueError, KeyError, TypeError):
            updates = None
        result.put(updates)

    with subprocess.Popen(command) as manager:
        Thread(target=check, daemon=True).start()
        while manager.poll() is None:
            try:
                updates = result.get(timeout=.2)
            except Empty:
                continue
            if updates and manager.poll() is None and prompt(*updates):
                manager.terminate()
                try:
                    manager.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    manager.kill()
                    manager.wait()
                return 0
            break
        return manager.wait()


def installer_command(root, state, installed):
    repo, branch = installed["repository"], installed.get("branch", "main")
    catalog = installed.get("catalog")
    if installed["platform"] == "windows":
        def native_path(path):
            return subprocess.check_output(["wslpath", "-w", str(path)], text=True, timeout=8).strip()
        command = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
                   native_path(root / "bin/Install-ASURT.ps1"), "-Distribution",
                   os.environ.get("WSL_DISTRO_NAME", "Ubuntu-22.04"), "-Repository", repo,
                   "-Branch", branch, "-Maps", "all"]
        if catalog:
            command.extend(("-Catalog", catalog if catalog.startswith("https://") else native_path(catalog)))
        return command
    command = ["/bin/bash", str(root / "bin/Install-ASURT.sh"), "--root", str(state.parent),
               "--repository", repo, "--branch", branch, "--maps", "all"]
    if catalog:
        command.extend(("--catalog", catalog))
    return command


def main():
    from session import stop_for_update

    state = Path(os.environ["ASURT_STATE_ROOT"])
    root = Path(os.environ["ASURT_ROOT"])
    try:
        installed = receipt(state)
        if not installed:
            raise RuntimeError("ASURT release information is missing. Run your installer once.")
        command = installer_command(root, state, installed)
        print("Closing the ASURT simulator and tools before updating…", flush=True)
        stop_for_update()
        result = subprocess.call(command)
        if result:
            print(f"The installer exited with code {result}.", file=sys.stderr)
        return result
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"ASURT update: {error}", file=sys.stderr)
        return 1
    finally:
        if sys.stdin.isatty():
            try:
                input("Press Enter to close this terminal.")
            except EOFError:
                pass


if __name__ == "__main__":
    sys.exit(run_manager(sys.argv[2:]) if sys.argv[1:2] == ["--manager"] else main())
