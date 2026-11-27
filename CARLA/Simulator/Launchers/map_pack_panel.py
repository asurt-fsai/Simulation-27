"""Select a map or register a cooked map pack for an ASURT simulator."""

import os
from pathlib import Path
import subprocess
import sys
import tkinter as tk
from tkinter import filedialog, ttk
import zipfile

if os.environ.get("ASURT_ROOT"):
    sys.path.insert(0, str(Path(os.environ["ASURT_ROOT"]) / "Simulation/CARLA/Launchers"))
from map_packs import catalog, register
from session import LAUNCH, LOG_ROOT, PROFILES, PYTHON, last_session_failure, launch_error, running_session


def main():
    root = tk.Tk()
    root.title("ASURT Map Switcher")
    panel = ttk.Frame(root, padding=16)
    panel.pack(fill="both", expand=True)
    active = running_session()
    profile = tk.StringVar(value=PROFILES[active["profile"]] if active else
                           PROFILES[os.environ.get("ASURT_PROFILE", "wslg")])
    map_name = tk.StringVar()
    status = tk.StringVar(value="")
    map_entries = {}
    process = None

    ttk.Label(panel, text="Simulator").grid(row=0, column=0, sticky="w")
    profile_control = (ttk.Label(panel, textvariable=profile) if len(PROFILES) == 1 else
                       ttk.Combobox(panel, textvariable=profile, values=list(PROFILES.values()),
                                    state="readonly", width=34))
    profile_control.grid(row=0, column=1, sticky="ew", pady=4)
    ttk.Label(panel, text="Map").grid(row=1, column=0, sticky="w")
    choices = ttk.Combobox(panel, textvariable=map_name, state="readonly", width=34)
    choices.grid(row=1, column=1, sticky="ew", pady=4)

    def refresh(selected=None):
        map_entries.clear()
        map_entries["FlatAsphalt"] = "flat-asphalt"
        for pack in catalog().values():
            map_entries[f"{pack.label} ({pack.map_id})"] = pack.map_id
        choices.configure(values=list(map_entries))
        default = selected or "flat-asphalt"
        map_name.set(next((label for label, key in map_entries.items() if key == default), "FlatAsphalt"))

    def add():
        file = filedialog.askopenfilename(parent=root, title="Add Cooked Map Pack",
                                          filetypes=[("ASURT map pack", "*.zip")])
        if not file:
            return
        try:
            pack = register(Path(file))
            refresh(pack.map_id)
            status.set(f"Added {pack.label}.")
        except (OSError, ValueError, KeyError, AttributeError, TypeError, zipfile.BadZipFile) as exc:
            status.set(str(exc))

    def check_launch(key, target, previous_error, log, offset):
        state = running_session()
        failure = last_session_failure()
        if state and state["map_key"] == key and state["profile"] == target and state["phase"] == "ready":
            status.set(f"{map_name.get()} is ready.")
        elif failure and failure != previous_error and (failure["profile"], failure["map_key"]) == (target, key):
            if process.poll() is None:
                status.set(f"{failure['message']}\nClosing the failed session. Logs: {failure['log']}")
                root.after(500, check_launch, key, target, previous_error, log, offset)
                return
            status.set(f"{failure['message']}\nLogs: {failure['log']}")
        elif process.poll() is not None:
            status.set(launch_error(log, offset, process.returncode))
        else:
            root.after(500, check_launch, key, target, previous_error, log, offset)
            return
        load_button.state(["!disabled"])

    def load_map():
        nonlocal process
        current = running_session()
        target = next(key for key, label in PROFILES.items() if label == profile.get())
        key = map_entries[map_name.get()]
        if current and current["profile"] != target:
            status.set(f"Select {PROFILES[current['profile']]} to switch the running simulator.")
            return
        if current and current["phase"] != "ready":
            status.set("Wait for the current simulator to finish starting or shutting down.")
            return
        if current and current["map_key"] == key:
            status.set(f"{map_name.get()} is already selected.")
            return
        pack = catalog().get(key)
        if pack and ("WindowsNoEditor" if target == "windows" else "LinuxNoEditor") not in pack.data["platforms"]:
            status.set(f"{pack.label} has no cook for {PROFILES[target]}.")
            return
        arguments = [target, "switch-map", "--map", key]
        if current:
            arguments += ["--session-state", current["state"]]
        log = LOG_ROOT / "tools/map-switch.log"
        previous_error = last_session_failure()
        try:
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open("ab") as stream:
                offset = stream.tell()
                process = subprocess.Popen(["bash", LAUNCH / "ros-parity-env.sh", PYTHON,
                                            LAUNCH / "session.py", *arguments],
                                           stdout=stream, stderr=subprocess.STDOUT,
                                           start_new_session=True)
        except OSError as exc:
            status.set(str(exc))
            return
        status.set(f"Loading {map_name.get()} in a fresh session.")
        load_button.state(["disabled"])
        root.after(500, check_launch, key, target, previous_error, log, offset)

    ttk.Button(panel, text="Add Map Pack…", command=add).grid(row=2, column=0, sticky="ew", padx=(0, 8), pady=8)
    load_button = ttk.Button(panel, text="Load Map", command=load_map)
    load_button.grid(row=2, column=1, sticky="ew", pady=8)
    ttk.Label(panel, textvariable=status, wraplength=405).grid(row=3, column=0, columnspan=2, sticky="w")
    refresh(active["map_key"] if active else None)
    root.mainloop()


if __name__ == "__main__":
    main()
