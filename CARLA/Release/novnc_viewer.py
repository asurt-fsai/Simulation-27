"""Start or reuse a loopback-only noVNC desktop for packaged WSL."""

import fcntl
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time
from urllib.request import urlopen

DISPLAY = ":1"
PORT = 6080
VNC_PORT = 5901
VIEWER = f"http://localhost:{PORT}/vnc.html?autoconnect=1&resize=scale&shared=1"


def port_ready(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=.3):
            return True
    except OSError:
        return False


def desktop_ready(env):
    try:
        return subprocess.run(["xdpyinfo"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              timeout=2).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def viewer_ready():
    try:
        with urlopen(f"http://127.0.0.1:{PORT}/vnc.html", timeout=1) as response:
            return b"noVNC" in response.read(4096)
    except OSError:
        return False


def desktop(log_root):
    log_root.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, DISPLAY=DISPLAY, QT_QPA_PLATFORM="xcb")
    env.pop("WAYLAND_DISPLAY", None)
    # A WSLg cookie is scoped to its own X server.
    env.pop("XAUTHORITY", None)
    with (log_root / "novnc.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if desktop_ready(env) and port_ready(VNC_PORT) and viewer_ready():
            return {"DISPLAY": DISPLAY, "QT_QPA_PLATFORM": "xcb"}
        if not all(shutil.which(name) for name in ("Xtigervnc", "websockify", "xdpyinfo", "openbox")):
            raise RuntimeError("noVNC desktop support is missing. Run the Windows installer with -Branch testing.")
        if not Path("/usr/share/novnc/vnc.html").is_file():
            raise RuntimeError("The noVNC viewer files are missing.")
        children = []

        def start(args):
            with (log_root / "novnc.log").open("ab") as log:
                proc = subprocess.Popen(args, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                        start_new_session=True)
                children.append(proc)

        def wait(ready):
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline and not any(proc.poll() is not None for proc in children):
                if ready():
                    return
                time.sleep(.2)
            raise RuntimeError(f"Could not start the noVNC desktop. See {log_root / 'novnc.log'}.")

        try:
            if not desktop_ready(env):
                if port_ready(VNC_PORT) or Path("/tmp/.X11-unix/X1").exists():
                    raise RuntimeError("Display :1 or VNC port 5901 is already owned by another desktop.")
                start(["Xtigervnc", DISPLAY, "-localhost", "yes", "-SecurityTypes", "None",
                       "-rfbport", str(VNC_PORT), "-nolisten", "tcp", "-geometry", "1600x900", "-depth", "24"])
                wait(lambda: desktop_ready(env) and port_ready(VNC_PORT))
                start(["openbox"])
            elif not port_ready(VNC_PORT):
                raise RuntimeError("Display :1 is running without a VNC endpoint.")
            if not viewer_ready():
                if port_ready(PORT):
                    raise RuntimeError("Port 6080 is already owned by another service.")
                start(["websockify", "--web", "/usr/share/novnc", f"127.0.0.1:{PORT}", f"127.0.0.1:{VNC_PORT}"])
                wait(viewer_ready)
        except (OSError, RuntimeError):
            for proc in reversed(children):
                if proc.poll() is None:
                    proc.terminate()
            raise
    return {"DISPLAY": DISPLAY, "QT_QPA_PLATFORM": "xcb"}


def open_viewer(log_root):
    desktop(log_root)
    subprocess.run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", f"Start-Process '{VIEWER}'"],
                   check=True, timeout=12)
