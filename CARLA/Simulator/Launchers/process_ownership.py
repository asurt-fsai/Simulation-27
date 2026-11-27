"""Find and stop children left by a past session of this ASURT installation."""

import fcntl
import os
from pathlib import Path
import re
import signal
import subprocess
import time


PROC = Path("/proc")
SESSION_DIRECTORY = re.compile(r"\d{8}-\d{6}-(?:windows|linux|wslg|novnc)-[a-z0-9-]+-\d+")


def stop_processes(processes):
    """Signal each owned process group, using one grace period per signal."""
    pending = [process for process in processes if process.poll() is None]
    for sig, grace in ((signal.SIGINT, 10), (signal.SIGTERM, 8), (signal.SIGKILL, 5)):
        if not pending:
            return
        for process in pending:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + grace
        for process in pending:
            try:
                process.wait(timeout=max(0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        pending = [process for process in pending if process.poll() is None]


def wait_for_session_lock(log_root, active, running_session, *, timeout=240):
    """Wait for the requested session's supervisor to finish releasing its resources."""
    log_root.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + timeout
    next_notice = time.monotonic() + 3
    with (log_root / "session.lock").open("a") as lock:
        while True:
            current = running_session()
            if current and (not active or current["state"] != active["state"]):
                raise RuntimeError("A different simulator session started during shutdown.")
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                pass
            else:
                if not current:
                    return
                fcntl.flock(lock, fcntl.LOCK_UN)
            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError(f"The ASURT session lock is still held after {timeout:g} seconds.")
            if now >= next_notice:
                print("Waiting for the ASURT simulator to finish shutting down…", flush=True)
                next_notice = now + 15
            time.sleep(.25)


def process_start(pid, proc_root=PROC):
    try:
        state, *fields = (proc_root / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()
        return fields[18] if state != "Z" else None
    except (OSError, IndexError):
        return None


def owned_processes(log_root, *, run_dir=None, proc_root=PROC):
    logs = log_root.resolve()
    expected = run_dir.resolve() if run_dir else None
    for entry in proc_root.glob("[0-9]*"):
        if not entry.name.isdecimal() or int(entry.name) <= 1:
            continue
        try:
            pid = int(entry.name)
            if pid == os.getpid():
                continue
            started = process_start(pid, proc_root)
            if not started or entry.stat().st_uid != os.getuid():
                continue
            environment = (entry / "environ").read_bytes().split(b"\0")
            location = next((value[len(b"ASURT_SESSION_LOG="):] for value in environment
                             if value.startswith(b"ASURT_SESSION_LOG=")), b"")
            if not location:
                continue
            recorded = Path(os.fsdecode(location)).resolve()
            if (recorded.parent == logs and SESSION_DIRECTORY.fullmatch(recorded.name)
                    and (not expected or recorded == expected)
                    and ((recorded / "session.json").is_file()
                         or (recorded / "carla_interface_config.yaml").is_file())):
                yield pid, started
        except (OSError, RuntimeError, ValueError):
            continue


def recover_orphaned_processes(log_root, *, run_dir=None):
    owned = tuple(owned_processes(log_root, run_dir=run_dir))
    remaining = owned
    for sig, grace in ((signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        for pid, started in remaining:
            if process_start(pid) == started:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + grace
        while True:
            remaining = tuple((pid, started) for pid, started in remaining if process_start(pid) == started)
            if not remaining:
                return len(owned)
            if time.monotonic() >= deadline:
                break
            time.sleep(0.1)
    raise RuntimeError("An ASURT process from a previous session is still shutting down.")
