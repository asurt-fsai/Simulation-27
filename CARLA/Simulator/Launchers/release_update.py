"""Check the installed ASURT release and hand off updates to an interactive terminal."""

import argparse
import base64
import hashlib
from itertools import count
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
from time import monotonic, sleep
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import urlopen

from windows_interop import powershell_exe


RECEIPT = "release-installation.json"
CHANNEL_SELECTION = "update-channel.json"
CHANNELS = {"main": "Release", "testing": "Testing"}
PRODUCTION_REPOSITORY = "asurt-fsai/Simulation-27"
LEGACY_REPOSITORY = "hassanradwannn/ASURTSIM"
REPOSITORY = re.compile(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+")
MAP_ID = re.compile(r"[a-z][a-z0-9-]{0,47}")
VERSION = re.compile(r"v?(\d+(?:\.\d+)+)(?:-flat)?")
TEST_VERSION = re.compile(r"testing-(\d+)")


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


def selected_branch(state, installed):
    branch = installed.get("branch")
    if branch not in ("main", "testing"):
        branch = "testing" if TEST_VERSION.fullmatch(installed["tag"]) else "main"
    if installed["repository"].casefold() == PRODUCTION_REPOSITORY.casefold():
        try:
            saved = json.loads((state / CHANNEL_SELECTION).read_text())["branch"]
        except (OSError, ValueError, KeyError, TypeError):
            pass
        else:
            if saved in ("main", "testing"):
                branch = saved
    return branch


def select_branch(state, branch):
    if branch not in CHANNELS:
        raise ValueError("Unknown ASURT update channel.")
    with tempfile.NamedTemporaryFile(mode="w", dir=state, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            json.dump({"branch": branch}, stream)
            stream.close()
            temporary.replace(state / CHANNEL_SELECTION)
        finally:
            temporary.unlink(missing_ok=True)


def read_public_json(url, *, timeout):
    for attempt in range(3):
        try:
            with urlopen(url, timeout=timeout) as stream:
                return json.load(stream)
        except HTTPError as error:
            if attempt == 2 or error.code not in (408, 429, 500, 502, 503, 504):
                raise
        except (URLError, TimeoutError, ConnectionError):
            if attempt == 2:
                raise
        sleep(attempt + 1)


def public_manifest(repo, branch):
    if branch != "testing":
        url = f"https://github.com/{repo}/releases/latest/download/asurt-release.json"
    else:
        latest = None
        for page in count(1):
            index = f"https://api.github.com/repos/{repo}/releases?per_page=100&page={page}"
            releases = read_public_json(index, timeout=15)
            builds = (match for release in releases if release["prerelease"] and not release["draft"]
                      and any(asset["name"] == "asurt-release.json" for asset in release["assets"])
                      if (match := TEST_VERSION.fullmatch(release["tag_name"])))
            for match in builds:
                if latest is None or int(match[1]) > int(latest[1]):
                    latest = match
            if len(releases) < 100:
                break
        if latest is None:
            return None
        url = f"https://github.com/{repo}/releases/download/{latest[0]}/asurt-release.json"
    manifest = read_public_json(url, timeout=15)
    if branch == "testing" and (manifest.get("tag") != latest[0] or manifest.get("channel") != "testing"):
        raise ValueError("The testing release metadata does not match the selected build.")
    if branch == "main" and (manifest.get("channel", "main") != "main"
                             or not VERSION.fullmatch(manifest["tag"])):
        raise ValueError("The Release feed did not return a stable ASURT build.")
    return manifest


def is_update(installed, manifest, branch):
    if manifest.get("format") != 1:
        raise ValueError("Unsupported release manifest.")
    tag = manifest["tag"]
    current = installed["tag"]
    if branch == "testing":
        published = TEST_VERSION.fullmatch(tag)
        if not published:
            raise ValueError("Invalid testing release tag.")
        previous = TEST_VERSION.fullmatch(current)
        return previous is None or int(published[1]) > int(previous[1])
    return newer_revision(current, tag) or bool(TEST_VERSION.fullmatch(current) and VERSION.fullmatch(tag))


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


def check_updates(state, *, branch=None, strict=False):
    installed = receipt(state)
    if not installed:
        return None
    repo = installed["repository"]
    production = repo.casefold() == PRODUCTION_REPOSITORY.casefold()
    branch = branch or selected_branch(state, installed)
    environment = dict(os.environ, GH_PROMPT_DISABLED="1", GIT_TERMINAL_PROMPT="0")
    latest = None
    failure = None
    bases = installed["base_sha256"]
    # Check them separately so a map catalog outage does not suppress a release.
    try:
        if production:
            manifest = public_manifest(repo, branch)
        else:
            with tempfile.TemporaryDirectory() as directory:
                subprocess.run(["gh", "release", "download", "--repo", repo, "--pattern",
                                "asurt-release.json", "--dir", directory], env=environment,
                               stdin=subprocess.DEVNULL, capture_output=True, timeout=15, check=True)
                manifest = json.loads((Path(directory) / "asurt-release.json").read_text())
        if manifest is not None and is_update(installed, manifest, branch if production else "main"):
            updated = {platform: manifest["runtime"][key]["shipping_sha256"]
                       for platform, key in (("WindowsNoEditor", "windows"), ("LinuxNoEditor", "linux"))
                       if platform in bases}
            latest, bases = manifest["tag"], updated
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        failure = error

    maps = []
    try:
        source = installed.get("catalog")
        if source and source.startswith("https://"):
            with urlopen(source, timeout=12) as stream:
                catalog = json.load(stream)
        elif source:
            catalog = json.loads(Path(source).read_text())
        else:
            ref = quote(branch, safe="")
            if production:
                catalog = read_public_json(
                    f"https://raw.githubusercontent.com/{repo}/{ref}/asurt-maps.json", timeout=12)
            else:
                catalog = json.loads(subprocess.check_output(
                    ["gh", "api", f"repos/{repo}/contents/asurt-maps.json?ref={ref}",
                     "-H", "Accept: application/vnd.github.raw"], env=environment,
                    stdin=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=12))
        if catalog.get("format") == 1:
            maps = available_maps(catalog["maps"], state / "maps", bases)
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        failure = failure or error
    if strict and failure and not (latest or maps):
        raise RuntimeError("The release or map feed could not be checked.") from failure
    return (latest, maps) if latest or maps else None


def updater_command(root, branch, *, add_wslg):
    command = ["/bin/bash", str(root / "bin/asurt"), "exec", "/usr/bin/python3", str(Path(__file__).resolve())]
    command.append("--reopen-manager")
    if branch:
        command.extend(("--channel", branch))
    if add_wslg:
        command.append("--add-wslg")
    return command


def windows_terminal(command):
    distro = os.environ.get("WSL_DISTRO_NAME", "Ubuntu-22.04")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", distro):
        raise ValueError("Invalid WSL distribution name.")
    arguments = subprocess.list2cmdline(["-d", distro, "--", *command])
    quoted = "'" + arguments.replace("'", "''") + "'"
    script = ("$ErrorActionPreference='Stop'; $wsl=Join-Path ([Environment]::SystemDirectory) 'wsl.exe'; "
              "Start-Process -FilePath $wsl "
              f"-ArgumentList {quoted} -WindowStyle Normal")
    encoded = base64.b64encode(script.encode("utf-16-le")).decode()
    return [powershell_exe(), "-NoProfile", "-NonInteractive", "-EncodedCommand", encoded]


def linux_terminal(command, state):
    terminals = []
    for name, options in (("gnome-terminal", ("--window", "--title=ASURT Update", "--")),
                          ("x-terminal-emulator", ("-e",))):
        if binary := shutil.which(name):
            terminals.append([binary, *options])
    if not terminals:
        raise RuntimeError("No desktop terminal is available to run the ASURT installer.")

    env = dict(os.environ)
    for variable in ("LD_LIBRARY_PATH", "GI_TYPELIB_PATH", "PYTHONHOME", "PYTHONPATH"):
        env.pop(variable, None)
    log = state / "log/carla/tools/installer-terminal.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    for terminal in terminals:
        with tempfile.TemporaryDirectory(prefix="asurt-installer-") as directory, log.open("ab") as output:
            pending = Path(directory) / "pending"
            started = pending.with_name("started")
            pending.touch()
            output.write(f"Opening the installer with {terminal[0]}\n".encode())
            output.flush()
            try:
                process = subprocess.Popen([*terminal, *command, "--terminal-token", pending], env=env,
                                           stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                deadline = monotonic() + 12
                while monotonic() < deadline and process.poll() in (None, 0):
                    if started.is_file():
                        return
                    sleep(.1)
            except OSError as error:
                output.write(f"{error}\n".encode())
            # The worker claims this token before stopping CARLA. If the launch
            # timed out, removing it prevents a delayed window starting twice.
            try:
                pending.unlink()
            except FileNotFoundError:
                if started.is_file():
                    return
    raise RuntimeError(f"The installer did not open an interactive terminal. See {log}.")


def launch_terminal(root, platform, branch=None, *, add_wslg=False):
    installer = "Install-ASURT.ps1" if platform == "windows" else "Install-ASURT.sh"
    if not (root / "bin" / installer).is_file():
        raise RuntimeError(f"The bundled {installer} is missing.")
    command = updater_command(root, branch, add_wslg=add_wslg)
    if platform == "windows":
        subprocess.run(windows_terminal(command), stdin=subprocess.DEVNULL, capture_output=True,
                       check=True, timeout=10)
    else:
        linux_terminal(command, Path(os.environ["ASURT_STATE_ROOT"]))


def open_channel_settings(parent, *, check_now=False):
    import tkinter as tk
    from tkinter import messagebox, ttk

    state = Path(os.environ["ASURT_STATE_ROOT"])
    try:
        installed = receipt(state)
        if not installed or installed["repository"].casefold() != PRODUCTION_REPOSITORY.casefold():
            raise ValueError("Install Simulation-27 to use its Release and Testing channels.")
    except (OSError, ValueError, KeyError, TypeError) as error:
        messagebox.showerror("Update channel", str(error), parent=parent)
        return
    current = selected_branch(state, installed)
    window = tk.Toplevel(parent)
    window.title("ASURT Update Channel")
    window.transient(parent)
    window.grab_set()

    def close():
        window.destroy()
        parent.grab_set()

    window.protocol("WM_DELETE_WINDOW", close)
    panel = ttk.Frame(window, padding=14)
    panel.pack(fill="both", expand=True)
    ttk.Label(panel, text=f"Installed: {installed['tag']}").grid(row=0, columnspan=2, sticky="w")
    choice = ttk.Combobox(panel, values=list(CHANNELS.values()), state="readonly", width=14)
    choice.set(CHANNELS.get(current, "Release"))
    choice.grid(row=1, column=0, sticky="w", pady=8)
    status = tk.StringVar(window)
    ttk.Label(panel, textvariable=status, wraplength=300).grid(row=2, columnspan=2, sticky="w")

    def check(_event=None):
        branch = next(key for key, label in CHANNELS.items() if label == choice.get())
        label = CHANNELS[branch]
        if _event is not None:
            try:
                select_branch(state, branch)
            except OSError as error:
                status.set(f"Could not save the channel: {error}")
                return
        status.set(f"Checking {label}…")
        choice.configure(state="disabled")
        button.configure(state="disabled")
        result = Queue()

        def fetch():
            try:
                result.put((check_updates(state, branch=branch, strict=True), None))
            except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
                result.put((None, error))

        def finish():
            try:
                updates, error = result.get_nowait()
            except Empty:
                window.after(100, finish)
                return
            choice.configure(state="readonly")
            button.configure(state="normal")
            if error:
                status.set(f"Could not check {label}. {error}")
            elif not updates:
                status.set(f"No new changes on {label}.")
            else:
                if _event is None and not messagebox.askyesno(
                        "ASURT updates available", update_details(*updates), parent=window):
                    status.set(f"Updates available on {label}.")
                    return
                status.set("Opening the installer terminal…")
                choice.configure(state="disabled")
                button.configure(state="disabled")

                def open_terminal():
                    try:
                        launch_terminal(Path(os.environ["ASURT_ROOT"]), installed["platform"], branch)
                    except (OSError, KeyError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
                        result.put(error)
                    else:
                        result.put(None)

                def report_terminal():
                    try:
                        error = result.get_nowait()
                    except Empty:
                        window.after(100, report_terminal)
                        return
                    choice.configure(state="readonly")
                    button.configure(state="normal")
                    if error:
                        status.set(f"Could not start the installer: {error}")
                    else:
                        status.set(f"Installing the {label} update in the terminal.")

                Thread(target=open_terminal, daemon=True).start()
                window.after(100, report_terminal)

        Thread(target=fetch, daemon=True).start()
        window.after(100, finish)

    button = ttk.Button(panel, text="Check for updates", command=check)
    button.grid(row=1, column=1, padx=(8, 0), sticky="w")
    choice.bind("<<ComboboxSelected>>", check)
    if check_now:
        window.after_idle(check)


def update_details(release, maps):
    details = [f"New ASURT release {release}"] if release else []
    if maps:
        details.append("New or updated maps\n" + "\n".join(f"• {name}" for name in maps[:8]))
        if len(maps) > 8:
            details.append(f"Plus {len(maps)-8} more maps")
    details.append("Download now? ASURT will close its simulator and tools and open the installer in a terminal.")
    return "\n\n".join(details)


def prompt(release, maps):
    import tkinter as tk
    from tkinter import messagebox

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    try:
        if not messagebox.askyesno("ASURT updates available", update_details(release, maps), parent=root):
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


def installer_command(root, state, installed, branch=None, *, add_wslg=False):
    repo = installed["repository"]
    branch = branch or selected_branch(state, installed)
    catalog = installed.get("catalog")
    if installed["platform"] == "windows":
        def native_path(path):
            return subprocess.check_output(["wslpath", "-w", str(path)], text=True, timeout=8).strip()
        command = [powershell_exe(), "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
                   native_path(root / "bin/Install-ASURT.ps1"), "-Distribution",
                   os.environ.get("WSL_DISTRO_NAME", "Ubuntu-22.04"), "-Repository", repo,
                   "-Branch", branch, "-Maps", "all"]
        wslg = root / "Simulation/CARLA/Runtime/Active/asurt-linux-flat"
        command.append("-WithWslg" if add_wslg or wslg.is_dir() else "-NativeOnly")
        if catalog:
            command.extend(("-Catalog", catalog if catalog.startswith("https://") else native_path(catalog)))
        return command
    command = ["/bin/bash", str(root / "bin/Install-ASURT.sh"), "--root", str(state.parent),
               "--repository", repo, "--branch", branch, "--maps", "all"]
    if catalog:
        command.extend(("--catalog", catalog))
    return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", choices=CHANNELS)
    parser.add_argument("--add-wslg", action="store_true")
    parser.add_argument("--reopen-manager", action="store_true")
    parser.add_argument("--terminal-token", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    state = Path(os.environ["ASURT_STATE_ROOT"])
    root = Path(os.environ["ASURT_ROOT"])
    terminal_claimed = not args.terminal_token
    try:
        if args.terminal_token:
            if not (sys.stdin.isatty() and sys.stdout.isatty()):
                raise RuntimeError("The installer requires an interactive terminal.")
            args.terminal_token.rename(args.terminal_token.with_name("started"))
            terminal_claimed = True
        from session import stop_for_update

        installed = receipt(state)
        if not installed:
            raise RuntimeError("ASURT release information is missing. Run your installer once.")
        if args.add_wslg and installed["platform"] != "windows":
            raise ValueError("WSLg is available on Windows.")
        command = installer_command(root, state, installed, args.channel, add_wslg=args.add_wslg)
        print("Closing the ASURT simulator and tools before updating…", flush=True)
        stop_for_update()
        print("Starting the ASURT installer…", flush=True)
        result = subprocess.call(command)
        if result:
            print(f"The installer exited with code {result}.", file=sys.stderr)
        elif args.reopen_manager:
            launcher = state.parent / "current/bin/asurt"
            env = dict(os.environ)
            env.pop("ASURT_ROOT", None)
            try:
                subprocess.Popen(["/bin/bash", launcher, "manager"], env=env,
                                 stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)
            except OSError as error:
                print(f"The update installed. Could not reopen System Manager: {error}", file=sys.stderr)
        return result
    except (OSError, ImportError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"ASURT update: {error}", file=sys.stderr)
        return 1
    finally:
        if terminal_claimed and sys.stdin.isatty():
            try:
                input("Press Enter to close this terminal.")
            except EOFError:
                pass


if __name__ == "__main__":
    sys.exit(run_manager(sys.argv[2:]) if sys.argv[1:2] == ["--manager"] else main())
