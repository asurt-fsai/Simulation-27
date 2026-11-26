#!/usr/bin/env python3
"""ASURT System Manager's ROS bag recorder and isolated RViz playback."""

from concurrent.futures import CancelledError
import datetime as dt
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, ttk
from tkinter.scrolledtext import ScrolledText
from tempfile import NamedTemporaryFile

ROOT = Path(os.environ.get("ASURT_ROOT", Path(__file__).resolve().parents[2]))
LAUNCH = Path(os.environ.get("ASURT_LAUNCHERS", ROOT / "Simulation/CARLA/Launchers"))
sys.path.insert(0, str(LAUNCH))
from session import LOG_ROOT, running_session

from bag_backend import (DISPLAYS, MIB, REPLAY_SUPPORT, Job, available_space, discover, inspect_bag,
                         live_domain, play_job, record_job)

RECORDINGS = Path.home() / "ASURT/Recordings"
PREFERENCES = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "asurt/bag-topics.json"
LEFT, RIGHT = (f"/zed/zed_node/{side}/image_rect_color" for side in ("left", "right"))
RAW = ("/velodyne_points", LEFT, RIGHT)
SYNC = ("/clock", "/tf", "/tf_static", "/odom", "/zed/zed_node/left/camera_info",
        "/zed/zed_node/right/camera_info", "/sim/feedback/imu", "/sim/feedback/gps")
PRESETS = {
    "Custom": (),
    "Raw LiDAR": RAW[:1],
    "ZED stereo": RAW[1:],
    "LiDAR + stereo": RAW,
    "SLAM fusion": RAW + SYNC,
    "Perception debug": ("/filtered_points", "/lidar/cones", "/perception_landmarks",
                         "/zed/zed_node/cone_markers", "/map/global_cones_markers"),
    "Autonomous run": ("/clock", "/tf", "/tf_static", "/odom", "/drive", "/sim/feedback/autonomous_mode",
                       "/sim/feedback/speed", "/sim/feedback/steering_angle", "/sim/feedback/imu",
                       "/perception_landmarks", "/planning/deep_learning/raw_path",
                       "/lidar/cones", "/map/global_cones", "/map/global_cones_markers", "/slam/cone_map_status",
                       *(f"/planning/{planner}/{stream}" for planner in ("classical", "acceleration", "skidpad", "deep_learning")
                         for stream in ("path", "status")),
                       *(f"/control/{controller}/{stream}" for controller in ("pure_pursuit_auto", "stanley_auto")
                         for stream in ("status", "command_preview"))),
}


def bag_name():
    return dt.datetime.now().strftime("asurt-%Y%m%d-%H%M%S")


def previous_topics():
    try:
        topics = json.loads(PREFERENCES.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return set()
    return {topic for topic in topics if isinstance(topic, str)} if isinstance(topics, list) else set()


def remember_topics(topics):
    PREFERENCES.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(mode="w", dir=PREFERENCES.parent, encoding="utf-8", delete=False) as stream:
        path = Path(stream.name)
        try:
            json.dump(topics, stream)
            stream.close()
            path.replace(PREFERENCES)
        finally:
            path.unlink(missing_ok=True)


class Terminal:
    def __init__(self, parent, title):
        self.window = tk.Toplevel(parent)
        self.window.title(title)
        self.window.geometry(f"780x260+{max(0, parent.winfo_screenwidth()-805)}"
                             f"+{max(0, parent.winfo_screenheight()-315)}")
        self.window.protocol("WM_DELETE_WINDOW", self.window.withdraw)
        self.text = ScrolledText(self.window, bg="#171d26", fg="#e8eef7",
                                 insertbackground="white", font=("DejaVu Sans Mono", 10), wrap="word")
        self.text.pack(fill="both", expand=True)
        self.text.configure(state="disabled")
        self.show()

    def show(self):
        self.window.deiconify()
        self.window.lift()
        self.window.attributes("-topmost", True)
        self.window.after(1200, self.lower_priority)

    def lower_priority(self):
        self.window.attributes("-topmost", False)

    def write(self, message):
        text = self.text
        text.configure(state="normal")
        text.insert("end", message + "\n")
        lines = int(text.index("end-1c").split(".")[0])
        if lines > 3000:
            text.delete("1.0", f"{lines-2500}.0")
        text.see("end")
        text.configure(state="disabled")


class BagWindow:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.jobs = {}
        self.terminals = {}
        self.busy_inspect = self.busy_discover = self.closing = False
        self.graph = {}
        self.selected = previous_topics()
        self.playback_topics = set()
        self.record_session = self.details = None
        self.player_paused = None
        self.shutdown_signal = False
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        root.title("ASURT ROS Bags")
        root.geometry("900x660")
        root.minsize(770, 570)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tabs = ttk.Notebook(root)
        self.tabs.pack(fill="both", expand=True, padx=10, pady=10)
        self.build_record()
        self.build_play()
        self.sync()
        root.after(80, self.poll)
        root.after(150, self.refresh_topics)

    def build_record(self):
        panel = ttk.Frame(self.tabs, padding=12)
        self.tabs.add(panel, text="Record")
        settings = ttk.Frame(panel)
        settings.pack(fill="x")
        self.preset = tk.StringVar(value="Custom")
        select = ttk.Combobox(settings, textvariable=self.preset, values=list(PRESETS),
                              state="readonly", width=17)
        select.pack(side="left", padx=(0, 8))
        select.bind("<<ComboboxSelected>>", self.apply_preset)
        self.search = tk.StringVar()
        self.search.trace_add("write", lambda *_: self.show_topics())
        ttk.Label(settings, text="Filter").pack(side="left")
        ttk.Entry(settings, textvariable=self.search, width=23).pack(side="left", padx=6)
        self.refresh = ttk.Button(settings, text="Refresh live topics", command=self.refresh_topics)
        self.refresh.pack(side="right")
        self.topic_view = self.table(panel, ("Topic", "Message type", "Publishers"), (330, 330, 85), checks=True)
        self.topic_view.bind("<ButtonRelease-1>", self.toggle_topic)
        self.topic_view.bind("<space>", self.toggle_topic)
        actions = ttk.Frame(panel)
        actions.pack(fill="x")
        for title, handler in (("Select visible", self.select_visible), ("Clear selection", self.clear_selection),
                               ("Add sync topics", self.add_sync)):
            ttk.Button(actions, text=title, command=handler).pack(side="left", padx=(0, 5))
        self.count = tk.StringVar(value="0 selected")
        ttk.Label(actions, textvariable=self.count).pack(side="right")
        out = ttk.Frame(panel)
        out.pack(fill="x", pady=6)
        self.output = tk.StringVar(value=str(RECORDINGS / "Manual"))
        self.name = tk.StringVar(value=bag_name())
        ttk.Label(out, text="Destination").pack(side="left")
        ttk.Entry(out, textvariable=self.output).pack(side="left", fill="x", expand=True, padx=6)
        ttk.Button(out, text="Browse", command=self.choose_output).pack(side="left")
        names = ttk.Frame(panel)
        names.pack(fill="x")
        ttk.Label(names, text="Bag name").pack(side="left")
        ttk.Entry(names, textvariable=self.name, width=26).pack(side="left", padx=6)
        self.sim_time = tk.BooleanVar(value=True)
        ttk.Checkbutton(names, text="Use simulation time", variable=self.sim_time).pack(side="left", padx=4)
        ttk.Label(names, text="SQLite3 · Zstd file compression").pack(side="right")
        ttk.Label(panel, text="Select /clock for clock replay. Sync topics can be added or removed individually.",
                  wraplength=795).pack(anchor="w", pady=5)
        buttons = ttk.Frame(panel)
        buttons.pack(fill="x")
        self.record_start = ttk.Button(buttons, text="Start recording", command=self.start_record)
        self.record_start.pack(side="left")
        self.record_stop = ttk.Button(buttons, text="Stop & finalize", command=lambda: self.stop("record"))
        self.record_stop.pack(side="left", padx=6)
        ttk.Button(buttons, text="Open folder", command=self.open_output).pack(side="left")
        ttk.Button(buttons, text="Terminal", command=lambda: self.show_terminal("record")).pack(side="right")
        self.record_status = tk.StringVar(value="Waiting to discover live topics")
        ttk.Label(panel, textvariable=self.record_status, wraplength=805).pack(anchor="w", pady=5)

    def build_play(self):
        panel = ttk.Frame(self.tabs, padding=12)
        self.tabs.add(panel, text="Play")
        paths = ttk.Frame(panel)
        paths.pack(fill="x")
        self.path = tk.StringVar()
        self.path.trace_add("write", self.invalidate_bag)
        ttk.Entry(paths, textvariable=self.path).pack(side="left", expand=True, fill="x")
        ttk.Button(paths, text="Browse recordings", command=self.choose_bag).pack(side="left", padx=4)
        ttk.Button(paths, text="Open metadata", command=lambda: self.choose_bag(metadata=True)).pack(side="left")
        ttk.Button(paths, text="Inspect", command=self.inspect).pack(side="left", padx=4)
        self.summary = tk.StringVar(value="Choose a bag folder or metadata.yaml.")
        ttk.Label(panel, textvariable=self.summary, wraplength=830).pack(anchor="w", pady=7)
        self.bag_view = self.table(panel, ("Recorded topic", "Message type", "Messages", "RViz"),
                                   (270, 262, 98, 76), checks=True)
        self.bag_view.bind("<ButtonRelease-1>", self.toggle_playback_topic)
        self.bag_view.bind("<space>", self.toggle_playback_topic)
        options = ttk.Frame(panel)
        options.pack(fill="x", pady=7)
        self.storage = tk.StringVar(value="Disk")
        ttk.Combobox(options, values=("Disk", "RAM"), textvariable=self.storage,
                     state="readonly", width=7).pack(side="left")
        ttk.Label(options, text="Speed ×").pack(side="left", padx=(8, 4))
        self.rate = tk.StringVar(value="0.25")
        ttk.Spinbox(options, from_=0.05, to=16., increment=.25,
                    textvariable=self.rate, width=6).pack(side="left")
        self.paused = tk.BooleanVar()
        ttk.Checkbutton(options, text="Start paused", variable=self.paused).pack(side="left", padx=8)
        self.play_all = tk.BooleanVar()
        ttk.Checkbutton(options, text="All recorded topics", variable=self.play_all,
                        command=self.show_playback_topics).pack(side="left")
        self.frame = tk.StringVar()
        ttk.Label(options, text="RViz frame").pack(side="left", padx=(12, 4))
        ttk.Entry(options, textvariable=self.frame, width=15).pack(side="left")
        ttk.Label(panel, text="Leave RViz frame blank to detect it from the bag. RAM uses temporary memory.",
                  wraplength=800).pack(anchor="w")
        controls = ttk.Frame(panel)
        controls.pack(fill="x", pady=7)
        self.play_start = ttk.Button(controls, text="Play & open RViz", command=self.start_play)
        self.play_start.pack(side="left", padx=(0, 5))
        self.controls = {}
        for title, action in (("Pause", "pause"), ("Resume", "resume"), ("Apply speed", "rate")):
            button = ttk.Button(controls, text=title, command=lambda a=action: self.player_command(a))
            button.pack(side="left", padx=(0, 5))
            self.controls[action] = button
        self.play_stop = ttk.Button(controls, text="Stop", command=lambda: self.stop("play"))
        self.play_stop.pack(side="left")
        ttk.Button(controls, text="Terminal", command=lambda: self.show_terminal("play")).pack(side="right")
        self.play_status = tk.StringVar(value="Idle")
        ttk.Label(panel, textvariable=self.play_status, wraplength=810).pack(anchor="w")

    def table(self, panel, columns, widths, *, checks=False):
        container = ttk.Frame(panel)
        container.pack(fill="both", expand=True, pady=6)
        tree = ttk.Treeview(container, columns=columns, show="tree headings" if checks else "headings",
                            selectmode="browse", height=10)
        tree.column("#0", width=40, minwidth=38, stretch=False, anchor="center")
        for column, width in zip(columns, widths):
            tree.heading(column, text=column)
            tree.column(column, width=width, minwidth=74, stretch=column in columns[:2])
        bar = ttk.Scrollbar(container, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        tree.pack(side="left", fill="both", expand=True)
        return tree

    def show_topics(self):
        self.topic_view.delete(*self.topic_view.get_children())
        query = self.search.get().casefold().strip()
        for name, (kind, count) in self.graph.items():
            if query in (name + " " + kind).casefold():
                self.topic_view.insert("", "end", iid=name, values=(name, kind, count),
                                       text="☑" if name in self.selected else "☐")
        self.count.set(f"{len(self.selected)} selected")
        self.sync()

    def toggle_topic(self, event):
        name = (self.topic_view.identify_row(event.y) if event.type == tk.EventType.ButtonRelease
                else self.topic_view.focus())
        if name and self.graph[name][1]:
            self.selected.symmetric_difference_update({name})
            self.preset.set("Custom")
            self.topic_view.item(name, text="☑" if name in self.selected else "☐")
            self.count.set(f"{len(self.selected)} selected")
            self.sync()

    def apply_preset(self, *_):
        if names := PRESETS[self.preset.get()]:
            self.selected = {name for name in names if self.graph.get(name, ("", 0))[1]}
            unavailable = set(names) - self.selected
            if unavailable:
                self.record_status.set("Unavailable preset topics: " + ", ".join(sorted(unavailable)))
            self.show_topics()

    def select_visible(self):
        self.selected.update(name for name in self.topic_view.get_children() if self.graph[name][1])
        self.preset.set("Custom")
        self.show_topics()

    def clear_selection(self):
        self.selected.clear()
        self.preset.set("Custom")
        self.show_topics()

    def add_sync(self):
        self.selected.update(name for name in SYNC if self.graph.get(name, ("", 0))[1])
        self.show_topics()

    def refresh_topics(self):
        if self.busy_discover or "record" in self.jobs:
            return
        try:
            session = running_session()
            domain = live_domain(session)
        except ValueError as error:
            self.record_status.set(str(error))
            self.record_session = None
            self.graph.clear()
            self.show_topics()
            return
        self.record_status.set(f"Discovering live publishers · ROS domain {domain}")
        self.busy_discover = True
        self.sync()
        self.background("topics", lambda: (session, domain, discover(domain)))

    def choose_output(self):
        if folder := filedialog.askdirectory(parent=self.root, initialdir=self.output.get()):
            self.output.set(folder)

    def open_output(self):
        path = Path(self.output.get()).expanduser()
        if path.is_dir():
            try:
                subprocess.Popen(["xdg-open", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError as error:
                self.record_status.set(str(error))
        else:
            self.record_status.set("The recordings folder will be created when recording starts.")

    def choose_bag(self, metadata=False):
        initial = str(RECORDINGS if RECORDINGS.is_dir() else Path.home())
        path = (filedialog.askopenfilename(parent=self.root, initialdir=initial,
                                          filetypes=[("ROS bag metadata", "metadata.yaml")])
                if metadata else filedialog.askdirectory(parent=self.root, initialdir=initial))
        if path:
            self.path.set(path)
            self.inspect()

    def invalidate_bag(self, *_):
        self.details = None
        self.playback_topics.clear()
        if hasattr(self, "play_start"):
            self.bag_view.delete(*self.bag_view.get_children())
            self.summary.set("Inspect the selected bag to see its topics.")
            self.sync()

    def show_playback_topics(self):
        self.bag_view.delete(*self.bag_view.get_children())
        if self.details:
            for name, (kind, count) in self.details[-1].items():
                included = count and (self.play_all.get() or name in self.playback_topics)
                self.bag_view.insert("", "end", iid=name, text="☑" if included else "☐",
                                     values=(name, kind, f"{count:,}", "Display" if kind in DISPLAYS else "Data"))
        self.sync()

    def toggle_playback_topic(self, event):
        if not self.details or self.play_all.get() or "play" in self.jobs:
            return
        name = (self.bag_view.identify_row(event.y) if event.type == tk.EventType.ButtonRelease
                else self.bag_view.focus())
        if name and self.details[-1][name][1]:
            self.playback_topics.symmetric_difference_update({name})
            self.bag_view.item(name, text="☑" if name in self.playback_topics else "☐")
            self.sync()

    def inspect(self):
        if self.busy_inspect or not self.path.get().strip() or "play" in self.jobs:
            return
        self.busy_inspect = True
        path = self.path.get().strip()
        self.play_status.set("Inspecting")
        self.sync()
        self.background("inspection", lambda: (path, inspect_bag(path)))

    def start_record(self):
        if "record" not in self.jobs and self.record_session and self.selected:
            topics = sorted(self.selected)
            self.start_job("record", record_job, LOG_ROOT, self.record_session, running_session,
                           self.output.get(), self.name.get(), topics, self.sim_time.get())
            try:
                remember_topics(topics)
            except OSError as error:
                self.jobs["record"].log(f"Could not remember topic selections: {error}")

    def chosen_rate(self):
        rate = float(self.rate.get())
        if not 0.05 <= rate <= 16:
            raise ValueError("Playback speed must be between 0.05× and 16×.")
        return rate

    def start_play(self):
        if "play" in self.jobs or not self.details:
            return
        try:
            rate = self.chosen_rate()
        except ValueError as error:
            self.play_status.set(str(error))
            return
        topics = None if self.play_all.get() else self.playback_topics.copy()
        self.start_job("play", play_job, LOG_ROOT, running_session, self.details[0],
                       self.storage.get() == "RAM", rate, self.paused.get(), topics, self.frame.get().strip())

    def player_command(self, action):
        if "play" in self.jobs and self.player_paused is not None:
            try:
                value = self.chosen_rate() if action == "rate" else None
                self.jobs["play"].commands.put((action, value))
            except ValueError as error:
                self.play_status.set(str(error))

    def start_job(self, kind, function, *args):
        if kind in self.terminals:
            self.terminals[kind].window.destroy()
        self.terminals[kind] = Terminal(self.root, "ASURT Recorder Terminal" if kind == "record"
                                        else "ASURT Playback & RViz Terminal")
        job = Job(lambda event, value: self.events.put((kind, event, value)))
        self.jobs[kind] = job
        self.sync()
        def run():
            try:
                return function(job, *args)
            finally:
                job.flush_output()

        self.background(kind, run)

    def show_terminal(self, kind):
        if terminal := self.terminals.get(kind):
            terminal.show()

    def stop(self, kind):
        if job := self.jobs.get(kind):
            job.stop.set()
            self.sync()

    def background(self, kind, function):
        def work():
            try:
                self.events.put((kind, "complete", function()))
            except CancelledError:
                self.events.put((kind, "cancelled", None))
            except Exception as error:
                self.events.put((kind, "error", getattr(error, "stderr", None) or str(error)))
        threading.Thread(target=work, daemon=True).start()

    def sync(self):
        def enable(button, yes):
            button.configure(state="normal" if yes else "disabled")
        if not hasattr(self, "play_start"):
            return
        recording, playing = self.jobs.get("record"), self.jobs.get("play")
        enable(self.refresh, not recording and not self.busy_discover and not self.closing)
        enable(self.record_start, not self.closing and bool(self.record_session and self.selected)
               and not recording and not self.busy_discover)
        enable(self.record_stop, bool(recording and not recording.stop.is_set()))
        enable(self.play_start, bool(self.details and (self.playback_topics or self.play_all.get())
                                     and not playing and not self.closing and not self.busy_inspect))
        enable(self.play_stop, bool(playing and not playing.stop.is_set()))
        player_ready = bool(playing and self.player_paused is not None and not playing.stop.is_set())
        enable(self.controls["pause"], player_ready and not self.player_paused)
        enable(self.controls["resume"], player_ready and self.player_paused)
        enable(self.controls["rate"], player_ready)

    def poll(self):
        if self.shutdown_signal and not self.closing:
            self.close()
        for _ in range(250):
            try:
                kind, event, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "topics":
                self.busy_discover = False
                if event == "complete":
                    self.record_session, domain, self.graph = value
                    unavailable = self.selected - {name for name, (_, publishers) in self.graph.items() if publishers}
                    self.selected.intersection_update(set(self.graph) - unavailable)
                    self.show_topics()
                    suffix = " · unavailable selections removed: " + ", ".join(sorted(unavailable)) if unavailable else ""
                    self.record_status.set(f"{len(self.graph)} topics · live ROS domain {domain}{suffix}")
                else:
                    self.record_status.set(str(value))
            elif kind == "inspection":
                self.busy_inspect = False
                if event == "complete" and self.path.get().strip() == value[0]:
                    self.details = value[1]
                    path, _, info, files, topics = self.details
                    self.playback_topics = {name for name, (kind, count) in topics.items()
                                            if count and (kind in DISPLAYS or name in REPLAY_SUPPORT)}
                    self.show_playback_topics()
                    size = sum(file.stat().st_size for file in files)
                    duration = info.get("duration", {}).get("nanoseconds", 0) / 1e9
                    compression = info.get("compression_format") or "uncompressed"
                    self.summary.set(f"{path.name} · {duration:.1f}s · {info['message_count']:,} messages · "
                                     f"{size/MIB:.0f} MiB · {len(files)} segments · {compression}\n"
                                     f"RAM available for staging: {available_space(Path('/dev/shm'), True)/MIB:.0f} MiB")
                    self.play_status.set("Ready")
                elif event == "error":
                    self.play_status.set(str(value))
            else:
                status = self.record_status if kind == "record" else self.play_status
                if event == "log":
                    self.terminals[kind].write(value)
                elif event == "state":
                    status.set(value)
                elif event == "paused":
                    self.player_paused = value
                elif event == "saved":
                    self.name.set(bag_name())
                else:
                    self.jobs.pop(kind, None)
                    if kind == "play":
                        self.player_paused = None
                    if event == "error":
                        self.terminals[kind].write(str(value))
                        status.set(str(value))
                    elif event == "cancelled":
                        status.set("Cancelled")
            self.sync()
        if self.closing and not self.jobs:
            self.root.destroy()
        else:
            self.root.after(70, self.poll)

    def close(self):
        self.closing = True
        for job in self.jobs.values():
            job.stop.set()
        self.sync()

    def request_shutdown(self, *_):
        self.shutdown_signal = True


if __name__ == "__main__":
    window = tk.Tk()
    app = BagWindow(window)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, app.request_shutdown)
    window.mainloop()
