"""Open ASURT tools for the current simulator."""

import os
from pathlib import Path
import subprocess
import sys
import tkinter as tk
from tkinter import ttk

if os.environ.get("ASURT_ROOT"):
    sys.path.insert(0, str(Path(os.environ["ASURT_ROOT"]) / "Simulation/CARLA/Launchers"))
from session import LAUNCH, LIVE_TOOLS, MAP_LABELS, PROFILES, PYTHON, RVIZ_PRESETS, running_session


def main():
    root = tk.Tk()
    root.title("ASURT CARLA Tools")
    root.resizable(False, False)
    panel = ttk.Frame(root, padding=20)
    panel.grid(sticky="nsew")
    status = tk.StringVar()
    feedback = tk.StringVar()
    buttons = {}

    def report_exit(process):
        if process.poll() is None:
            root.after(300, report_exit, process)
            return
        output, _ = process.communicate()
        if output.strip():
            feedback.set(output.strip().splitlines()[-1])

    def launch(action):
        data = running_session()
        feedback.set("")
        if action in LIVE_TOOLS and (not data or data["phase"] != "ready"):
            refresh_buttons()
            return
        command = [PYTHON, LAUNCH / "session.py", "auto", action]
        if action in LIVE_TOOLS:
            command += ["--session-state", data["state"]]
        if action in RVIZ_PRESETS:
            command = ["bash", LAUNCH / "ros-parity-env.sh", *command]
        try:
            process = subprocess.Popen(command, text=True, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        except OSError as exc:
            feedback.set(str(exc))
            return
        root.after(300, report_exit, process)

    def viewer():
        data = running_session()
        if not data or data["profile"] != "novnc":
            return
        try:
            entry = subprocess.check_output(
                ["wslpath", "-w", LAUNCH / "open-parity.ps1"], text=True).strip()
            subprocess.Popen(["powershell.exe", "-NoProfile", "-File", entry,
                              "-Profile", "novnc", "-Action", "viewer"])
        except (OSError, subprocess.CalledProcessError) as exc:
            feedback.set(str(exc))

    def refresh_buttons():
        data = running_session()
        if data:
            label = f'{MAP_LABELS[data.get("map_key", "silverstone")]} · {PROFILES[data["profile"]]}'
            state = f'{label}   •   {data["host"]}:{data["port"]}' if data["phase"] == "ready" else f"{label} is starting…"
        else:
            state = "Start an ASURT simulator to use the live tools."
        status.set(state)
        for action, button in buttons.items():
            if action in LIVE_TOOLS:
                button.state(["!disabled"] if data and data["phase"] == "ready" else ["disabled"])
        viewer_button.state(["!disabled"] if data and data["profile"] == "novnc" else ["disabled"])

    def refresh():
        refresh_buttons()
        root.after(750, refresh)

    ttk.Label(panel, textvariable=status, width=47, font=("TkDefaultFont", 11, "bold")).grid(
        row=0, column=0, sticky="w", pady=(0, 14))
    labels = {"track": "Cone Track Importer", "weather": "Weather Editor",
              "rviz": "Perception", "trackdraw": "Trackdraw",
              "manager": "CARLA System Manager", "map-packs": "Map Packs"}
    icons = {
        action: tk.PhotoImage(file=str(LAUNCH / "Icons" / f"{action}.png")).subsample(8)
        for action in (*labels, "tools", "novnc")
    }
    root.iconphoto(True, icons["tools"])
    for row, (action, label) in enumerate(labels.items(), start=1):
        button = ttk.Button(panel, text=label, image=icons[action], compound="left",
                            command=lambda selected=action: launch(selected))
        button.grid(row=row, column=0, sticky="ew", pady=3, ipady=4)
        buttons[action] = button
    viewer_button = ttk.Button(panel, text="Open noVNC Viewer", image=icons["novnc"],
                               compound="left", command=viewer)
    viewer_button.grid(row=len(labels) + 1, column=0, sticky="ew", pady=(10, 3))
    ttk.Label(panel, textvariable=feedback, wraplength=370).grid(row=len(labels) + 2, column=0, sticky="w")
    refresh()
    root.mainloop()


if __name__ == "__main__":
    main()
