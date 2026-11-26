#!/usr/bin/env python3
"""ASURT simulation and expandable module control panel."""
from pathlib import Path
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import tkinter as tk
from tkinter import messagebox

ROOT = Path(os.environ.get("ASURT_ROOT", Path(__file__).resolve().parents[2]))
LAUNCH = ROOT / "Simulation/CARLA/Launchers" if os.environ.get("ASURT_PORTABLE") else Path("/mnt/e/ASURT/Simulation/CARLA/Launchers")
sys.path[:0] = [str(ROOT / "tools/RViz"), str(LAUNCH)]
import module_control
from module_settings import ModuleSettings
from session import (LOG_ROOT, MAP_LABELS, PROFILES, RVIZ_PRESETS, SESSION_REQUIRED_TOOLS,
                     last_session_failure, launch_error, running_session)

MANAGER = ROOT / "tools/RViz/manage.py"
VIEW_STATE = module_control.CONFIG / "control-center-view.json"
TOOLS = {
    "Perception": "rviz",
    "ZED-only RViz": "zed-rviz",
    "SLAM RViz": "slam-rviz",
    "Advanced SLAM Diagnostics": "slam-diagnostics",
    "Planning RViz": "planning-rviz",
    "Control RViz": "control-rviz",
    "Track Control RViz": "track-control-rviz",
    "Cancel Goal": "cancel-goal",
    "Map Packs": "map-packs",
    "ROS Bags": "ros-bags",
    "Track Importer": "track",
    "Weather Editor": "weather",
    "Trackdraw": "trackdraw",
}
PLANNING_VIEWS = {
    "Overview": "planning-rviz",
    "Classical": "planning-classical-rviz",
    "Acceleration": "planning-acceleration-rviz",
    "Skidpad": "planning-skidpad-rviz",
    "Deep Learning": "planning-deep-learning-rviz",
    "Goal Pose": "planning-goal-pose-rviz",
}
BG, PANEL, CARD = "#14202e", "#223043", "#1a2737"
FG, DIM, TEAL, GREEN, GOLD = "#f3f6fb", "#b0bed1", "#75dae8", "#65d2a7", "#f5b875"


class ControlCenter:
    def __init__(self, root):
        self.root = root
        root.title("CARLA System Manager")
        root.geometry("950x740")
        root.minsize(760, 540)
        root.configure(bg=BG)
        self.busy = False
        self.messages = queue.Queue()
        self.tool_buttons = {}
        self.tool_processes = {}
        self.cards = {}
        self.group_buttons = []
        self.group_action_buttons = {}
        self.group_headers = {}
        self.group_bodies = {}
        self.group_summaries = {}
        self.module_headers = {}
        self.module_details = {}
        saved = self.load_view_state()
        self.group_expanded = saved.get("groups", {})
        self.module_expanded = saved.get("modules", {})
        self.signature = ()
        self.snapshot = {}
        self.info = tk.StringVar(value="Module switches persist across restarts.")
        self.planning_view = tk.StringVar(value="Overview")
        self.connection = tk.StringVar(value="Checking simulator...")
        self.layout()
        self.refresh()
        root.after(250, self.poll)

    def label(self, parent, value, size=10, color=FG, bold=False):
        return tk.Label(parent, text=value, bg=parent.cget("bg"), fg=color,
                        font=("DejaVu Sans", size, "bold" if bold else "normal"),
                        anchor="w", justify="left")

    def button(self, parent, title, action):
        return tk.Button(parent, text=title, command=action, bg="#405169", fg=FG,
                         activebackground=TEAL, activeforeground=BG, relief="flat",
                         borderwidth=0, cursor="hand2", padx=11, pady=8,
                         font=("DejaVu Sans", 10, "bold"))

    @staticmethod
    def load_view_state():
        """Only UI visibility is saved here, never module enablement."""
        try:
            value = json.loads(VIEW_STATE.read_text(encoding="utf-8"))
            if not isinstance(value, dict):
                return {}
            return {
                key: {name: visible for name, visible in
                      (value.get(key) if isinstance(value.get(key), dict) else {}).items()
                      if isinstance(name, str) and type(visible) is bool}
                for key in ("groups", "modules")
            }
        except (OSError, ValueError, TypeError):
            return {}

    def save_view_state(self):
        """Atomic, independent preference file; no calls into module_control."""
        try:
            VIEW_STATE.parent.mkdir(parents=True, exist_ok=True)
            descriptor, path = tempfile.mkstemp(prefix=".control-center-view-",
                                               dir=VIEW_STATE.parent)
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as out:
                    json.dump({"groups": self.group_expanded,
                               "modules": self.module_expanded}, out)
                    out.write("\n")
                os.replace(path, VIEW_STATE)
            finally:
                if os.path.exists(path):
                    os.unlink(path)
        except OSError as exc:
            self.info.set(f"Could not save panel layout: {exc}")

    def section_button(self, parent, title, command, color=TEAL):
        """Borderless, keyboard-accessible disclosure control, not a switch."""
        return tk.Button(parent, text=title, command=command, anchor="w",
                         bg=parent.cget("bg"), fg=color,
                         activebackground=parent.cget("bg"),
                         activeforeground=FG, relief="flat", borderwidth=0,
                         highlightthickness=0, overrelief="flat", takefocus=True,
                         cursor="hand2", padx=5, pady=5,
                         font=("DejaVu Sans", 11, "bold"))

    def toggle_section(self, group):
        expanded = not self.group_expanded.get(group, True)
        self.group_expanded[group] = expanded
        self.show_section(group)
        self.save_view_state()

    def set_all_sections(self, expanded):
        """Fold/unfold only the group bodies; leave module switches untouched."""
        for group in self.group_headers:
            self.group_expanded[group] = bool(expanded)
            self.show_section(group)
        for ident in self.module_headers:
            self.module_expanded[ident] = bool(expanded)
            self.show_details(ident)
        self.save_view_state()

    def show_section(self, group):
        expanded = self.group_expanded.get(group, True)
        header = self.group_headers[group]
        header.configure(text=f"{'▾' if expanded else '▸'} {group}",
                         # Express the visibility state to screen readers.
                         takefocus=True)
        body = self.group_bodies[group]
        if expanded:
            body.pack(fill="x", pady=(3, 0))
        else:
            body.pack_forget()

    def toggle_details(self, ident):
        expanded = not self.module_expanded.get(ident, True)
        self.module_expanded[ident] = expanded
        self.show_details(ident)
        self.save_view_state()

    def show_details(self, ident):
        expanded = self.module_expanded.get(ident, True)
        button, label = self.module_headers[ident]
        button.configure(text=f"{'▾' if expanded else '▸'} {label}")
        body = self.module_details[ident]
        if expanded:
            body.pack(fill="x", pady=(4, 0))
        else:
            body.pack_forget()

    def layout(self):
        header = tk.Frame(self.root, bg=BG, padx=22, pady=17)
        header.pack(fill="x")
        self.label(header, "ASURT", 12, TEAL, True).pack(anchor="w")
        title_row = tk.Frame(header, bg=BG)
        title_row.pack(fill="x")
        self.button(title_row, "Module Settings", self.open_module_settings).pack(side="right")
        self.weather_status = self.button(title_row, "Weather Status", lambda: self.launch_tool("Weather Status"))
        self.weather_status.pack(side="right", padx=(0, 8))
        self.wslg_download = self.button(header, "Download WSLg", self.download_wslg)
        self.label(title_row, "CARLA System Manager", 23, FG, True).pack(side="left")
        self.online = self.label(header, "", 10, DIM)
        self.online.configure(textvariable=self.connection)
        self.online.pack(anchor="w", pady=(3, 0))
        main = tk.Frame(self.root, bg=BG, padx=18)
        main.pack(expand=True, fill="both")
        main.columnconfigure(0, weight=5)
        main.columnconfigure(1, weight=3)
        main.rowconfigure(0, weight=1)
        left = tk.Frame(main, bg=PANEL, padx=13, pady=12)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 7))
        heading = tk.Frame(left, bg=PANEL)
        heading.pack(fill="x")
        self.label(heading, "Modules", 15, FG, True).pack(side="left")
        for title, expanded in (("Expand All", True), ("Collapse All", False)):
            control = self.button(heading, title,
                                  lambda visible=expanded: self.set_all_sections(visible))
            control.configure(font=("DejaVu Sans", 9), padx=7, pady=5)
            control.pack(side="right", padx=(5, 0))
        viewer = tk.Frame(left, bg=PANEL)
        viewer.pack(fill="both", expand=True, pady=(10, 0))
        canvas = tk.Canvas(viewer, bg=PANEL, highlightthickness=0)
        scrollbar = tk.Scrollbar(viewer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        self.items = tk.Frame(canvas, bg=PANEL)
        window = canvas.create_window((0, 0), window=self.items, anchor="nw")
        self.items.bind("<Configure>", lambda _: canvas.configure(
            scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(
            window, width=event.width))
        def bind_wheel(_event):
            canvas.bind_all("<MouseWheel>", lambda event: canvas.yview_scroll(int(-event.delta / 120), "units"))
            canvas.bind_all("<Button-4>", lambda event: canvas.yview_scroll(-3, "units"))
            canvas.bind_all("<Button-5>", lambda event: canvas.yview_scroll(3, "units"))

        def unbind_wheel(_event):
            for sequence in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                canvas.unbind_all(sequence)

        canvas.bind("<Enter>", bind_wheel)
        canvas.bind("<Leave>", unbind_wheel)
        right = tk.Frame(main, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(7, 0))
        for section, names in (
            ("Simulator", ("Quit CARLA",)),
            ("Tools", tuple(TOOLS)),
        ):
            frame = tk.Frame(right, bg=PANEL, padx=14, pady=14)
            frame.pack(fill="x", pady=(0, 12))
            self.label(frame, section, 14, FG, True).pack(anchor="w", pady=(0, 10))
            if section == "Tools":
                self.label(frame, "Planning RViz view", 9, DIM).pack(anchor="w")
                choice = tk.OptionMenu(frame, self.planning_view, *PLANNING_VIEWS)
                choice.configure(bg="#405169", fg=FG, relief="flat", highlightthickness=0)
                choice.pack(fill="x", pady=(2, 6))
            buttons = tk.Frame(frame, bg=PANEL)
            buttons.pack(fill="x")
            buttons.columnconfigure((0, 1), weight=1, uniform="tools")
            for index, name in enumerate(names):
                action = self.confirm_quit if name == "Quit CARLA" else (
                    lambda n=name: self.launch_tool(n))
                button = self.button(buttons, name, action)
                self.tool_buttons[name] = button
                button.configure(padx=5, pady=5, wraplength=130,
                                 font=("DejaVu Sans", 9, "bold"))
                button.grid(row=index // 2, column=index % 2, sticky="ew", padx=2, pady=2)
        footer = tk.Frame(self.root, bg=BG, padx=22, pady=12)
        footer.pack(fill="x")
        self.label(footer, "", 9, DIM).pack(anchor="w")
        footer.winfo_children()[-1].configure(textvariable=self.info, wraplength=870)

    def populate(self, modules):
        signature = tuple((m["id"], m["label"], m["group"], m.get("source")) for m in modules)
        if signature == self.signature:
            return
        self.signature = signature
        self.cards.clear()
        self.group_buttons.clear()
        self.group_action_buttons.clear()
        self.group_headers.clear()
        self.group_bodies.clear()
        self.group_summaries.clear()
        self.module_headers.clear()
        self.module_details.clear()
        for widget in self.items.winfo_children():
            widget.destroy()
        groups = {}
        for item in modules:
            group = item["group"]
            if group not in groups:
                parent = tk.Frame(self.items, bg=PANEL)
                parent.pack(fill="x", pady=(0, 5))
                row = tk.Frame(parent, bg=PANEL)
                row.pack(fill="x")
                # Keep bulk controls outside the collapsible body so they
                # remain available when only the module cards are hidden.
                actions = tk.Frame(row, bg=PANEL)
                actions.pack(side="right", padx=(0, 4))
                buttons = {}
                for title, value in (("All Off", False), ("All On", True)):
                    control = self.button(
                        actions, title,
                        lambda g=group, v=value: self.toggle_group(g, v))
                    control.configure(font=("DejaVu Sans", 9, "bold"),
                                      width=7, padx=3, pady=5, highlightthickness=0)
                    control.pack(side="right", padx=(3, 0))
                    self.group_buttons.append(control)
                    buttons[title] = control
                self.group_action_buttons[group] = buttons
                summary = self.label(row, "", 9, DIM)
                summary.pack(side="right", padx=(4, 6))
                toggle = self.section_button(
                    row, "", lambda g=group: self.toggle_section(g))
                toggle.pack(side="left", expand=True, fill="x")
                body = tk.Frame(parent, bg=PANEL)
                groups[group] = body
                self.group_headers[group] = toggle
                self.group_bodies[group] = body
                self.group_summaries[group] = summary
                self.show_section(group)
            card = tk.Frame(groups[group], bg=CARD, padx=10, pady=8)
            card.pack(fill="x", pady=3)
            top = tk.Frame(card, bg=CARD)
            top.pack(fill="x")
            label = item["label"]
            detail_toggle = self.section_button(
                top, "", lambda ident=item["id"]: self.toggle_details(ident), FG)
            detail_toggle.configure(font=("DejaVu Sans", 10, "bold"))
            detail_toggle.pack(side="left", expand=True, fill="x")
            control = self.button(top, "Turn Off", lambda ident=item["id"]: self.toggle(ident))
            control.pack(side="right")
            status = self.label(card, "Checking...", 9, DIM)
            status.pack(fill="x", pady=(4, 0))
            details = tk.Frame(card, bg=CARD)
            source = item.get("source", "Bundled")
            self.label(details, f"{source} · {item['topic']}", 8, DIM).pack(fill="x")
            self.cards[item["id"]] = (control, status)
            self.module_headers[item["id"]] = (detail_toggle, label)
            self.module_details[item["id"]] = details
            self.show_details(item["id"])

    def refresh(self, *, schedule=True):
        try:
            active = running_session()
            ready = bool(active and active["phase"] == "ready")
            self.weather_status.configure(state="normal" if ready else "disabled")
            self.tool_buttons["Quit CARLA"].configure(state="normal" if active else "disabled")
            for name, action in TOOLS.items():
                if action in SESSION_REQUIRED_TOOLS:
                    self.tool_buttons[name].configure(state="normal" if ready else "disabled")
            self.snapshot = module_control.live_states(active)
            on = self.snapshot["carla_listening"]
            session = self.snapshot.get("session")
            if session and session["phase"] != "failed":
                state = session["phase"]
                profile = PROFILES.get(session["profile"], session["profile"])
                track = MAP_LABELS.get(session["map_key"], session["map_key"])
                connection = f"{profile} · {track} · {session['host']}:{session['port']} · {state}"
            else:
                failure = last_session_failure()
                connection = (f"CARLA stopped · {failure['message']}" if failure
                              else "No ASURT simulator session is active")
            self.connection.set(connection)
            self.online.configure(fg=GREEN if on else GOLD)
            if (os.environ.get("ASURT_PROFILE") == "windows"
                    and not (LAUNCH.parent / "Runtime/Active/asurt-linux-flat").is_dir()):
                if not self.wslg_download.winfo_manager():
                    self.wslg_download.pack(anchor="w", pady=(6, 0))
            elif self.wslg_download.winfo_manager():
                self.wslg_download.pack_forget()
            self.populate(self.snapshot["modules"])
            group_counts = {}
            for item in self.snapshot["modules"]:
                counts = group_counts.setdefault(item["group"], [0, 0])
                counts[0] += 1
                counts[1] += bool(item["pids"])
                control, status = self.cards[item["id"]]
                enabled = item["enabled"]
                message = item["status"]
                status.configure(text=message[:1].upper() + message[1:] +
                                 (f" · {item['memory_mib']:.0f} MiB RAM" if item["pids"] else ""),
                                 wraplength=max(220, status.master.winfo_width() - 20),
                                 fg=GREEN if enabled and item["pids"] else DIM)
                control.configure(text="Turn Off" if enabled else "Turn On",
                                  bg=GOLD if enabled else GREEN, fg=BG,
                                  state="disabled" if self.busy else "normal")
            for group, (total, running) in group_counts.items():
                self.group_summaries[group].configure(
                    text=f"{running}/{total} running")
            for control in self.group_buttons:
                control.configure(state="disabled" if self.busy else "normal")
        except Exception as exc:
            self.info.set(f"Could not refresh module status: {exc}")
        if schedule:
            self.root.after(3000, self.refresh)

    def task(self, action, target):
        if self.busy:
            return
        self.busy = True
        self.info.set(f"Applying {action} to {target}...")
        def run():
            try:
                result = subprocess.run([sys.executable, str(MANAGER), action, target],
                                        text=True, capture_output=True, timeout=90)
                output = (result.stdout + result.stderr).strip()
                self.messages.put((result.returncode == 0, output or "Complete"))
            except Exception as exc:
                self.messages.put((False, str(exc)))
        threading.Thread(target=run, daemon=True).start()

    def toggle(self, ident):
        item = next((m for m in self.snapshot.get("modules", []) if m["id"] == ident), None)
        if item:
            self.task("--disable" if item["enabled"] else "--enable", ident)

    def open_module_settings(self):
        ModuleSettings(self.root, self.refresh)

    def download_wslg(self):
        from release_update import launch_terminal

        if not messagebox.askyesno("Install WSLg", "Download the optional WSLg renderer? "
                                  "The installer will close the simulator and open a terminal.", parent=self.root):
            return
        try:
            launch_terminal(ROOT, "windows", add_wslg=True)
            self.info.set("WSLg installer opened. Follow its progress in the terminal.")
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            self.info.set(f"Could not start the WSLg installer: {exc}")

    def toggle_group(self, group, enabled):
        self.task("--enable-group" if enabled else "--disable-group", group)

    def launch_tool(self, name):
        active = running_session()
        running = self.tool_processes.get(name)
        if running and running[0].poll() is None:
            self.info.set(f"{name} is already open.")
            return
        if name == "Quit CARLA":
            if not active:
                return
            args = [active["profile"], "stop", "--session-state", active["state"]]
        elif name == "Weather Status":
            if not active or active["phase"] != "ready":
                self.info.set("Start a simulator to view live weather.")
                return
            args = ["auto", "weather-status", "--session-state", active["state"]]
        else:
            action = (PLANNING_VIEWS[self.planning_view.get()]
                      if name == "Planning RViz" else TOOLS[name])
            if action in SESSION_REQUIRED_TOOLS and (not active or active["phase"] != "ready"):
                self.info.set(f"Start a simulator before opening {name}.")
                return
            args = ["auto", action]
            if active and action not in ("trackdraw", "ros-bags"):
                args.extend(["--session-state", active["state"]])
        command = [sys.executable, LAUNCH / "session.py", *args]
        if args[1] in (*RVIZ_PRESETS, "cancel-goal", "ros-bags"):
            command = ["bash", LAUNCH / "ros-parity-env.sh", *command]
        log = LOG_ROOT / "tools" / f"system-manager-{args[1]}.log"
        try:
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open("ab") as stream:
                offset = stream.tell()
                process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                                           stderr=subprocess.STDOUT, start_new_session=True)
            self.tool_processes[name] = process, log, offset
        except OSError as exc:
            self.info.set(f"Could not launch {name}: {exc}")

    def confirm_quit(self):
        if messagebox.askyesno("Quit CARLA", "Stop the current CARLA simulator?",
                               parent=self.root):
            self.launch_tool("Quit CARLA")

    def poll(self):
        for name, (process, log, offset) in tuple(self.tool_processes.items()):
            if (code := process.poll()) is not None:
                del self.tool_processes[name]
                if code:
                    self.info.set(f"Could not open {name}. {launch_error(log, offset, code)}")
        while not self.messages.empty():
            ok, output = self.messages.get_nowait()
            self.busy = False
            self.info.set(output if ok else f"Action failed: {output}")
            self.refresh(schedule=False)
        self.root.after(250, self.poll)


if __name__ == "__main__":
    app = tk.Tk()
    ControlCenter(app)
    app.mainloop()
