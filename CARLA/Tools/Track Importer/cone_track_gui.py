#!/usr/bin/env python3
"""Desktop track importer and FSAI car placement control panel."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

import yaml

from import_cone_track import load_cones, suggested_invert_y, track_start


HERE = Path(__file__).resolve().parent
# This application is owned by tools/Track Importer.  The bridge remains the
# authority for the CARLA config and cone ownership state, even after moving.
ASURT_ROOT = Path(os.environ.get("ASURT_ROOT", HERE.parents[1]))
REPO = ASURT_ROOT / "Simulation/CARLA/Bridge/Carla_ASU_Bridge"
if not REPO.is_dir():
    REPO = ASURT_ROOT / "Carla_ASU_Bridge"  # legacy migration fallback
CONFIG = REPO / "config" / "carla_interface_config.yaml"
IMPORTER = HERE / "import_cone_track.py"
COMPETITION_TRACKS = {
    "2027 Acceleration": ASURT_ROOT / "Assets/Maps/Competition2027/acceleration/acceleration_2027_cones.csv",
    "2027 Skidpad": ASURT_ROOT / "Assets/Maps/Competition2027/skidpad/skidpad_2027_cones.csv",
}


def short_failure(mode: str, output: str) -> str:
    """Keep Tk warning dialogs bounded; the complete error stays in the log."""
    blocked = re.search(r"^BLOCKED: (.+)$", output, re.MULTILINE)
    if blocked:
        return (blocked.group(1) +
                "\n\nNo cones were spawned. Run Preview or Find ground-supported placement. "
                "Individual locations are listed in the log.")
    errors = re.findall(r"^ERROR: (.+)$", output, re.MULTILINE)
    if errors:
        return errors[-1][:350] + "\n\nDetails are in the importer log."
    return (f"{mode} failed.\n" + (output.splitlines()[-1][:250] if output else
            "No diagnostic output.")) + "\n\nDetails are in the importer log."


class ConeTrackWindow:
    def __init__(self, root: tk.Tk, initial_file: str = "", state: Path | None = None) -> None:
        self.root = root
        self.state = state
        root.title("ASURT | CARLA Cone Track Importer")
        root.geometry("920x760")
        root.minsize(780, 650)
        self.cones = []
        self.invalid_indices = set()
        self.buttons = []
        self.busy = False

        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        origin = json.loads(os.environ.get("ASURT_COURSE_ORIGIN", "null")) or config["world"]["spawn_point_coords"]
        self.path = tk.StringVar(value="")
        self.host = tk.StringVar(value=os.environ.get("ASURT_CARLA_HOST", str(config["carla"].get("host", "127.0.0.1"))))
        self.port = tk.StringVar(value=os.environ.get("ASURT_CARLA_PORT", str(config["carla"].get("port", 2000))))
        self.ox = tk.StringVar(value=str(origin["x"]))
        self.oy = tk.StringVar(value=str(origin["y"]))
        self.oz = tk.StringVar(value=str(origin["z"]))
        self.yaw = tk.StringVar(value=str(origin["yaw"]))
        self.forward = tk.StringVar(value="12")
        self.start_forward = tk.StringVar(value="0")
        self.invert_y = tk.BooleanVar(value=True)
        self.ground = tk.StringVar(value="project")
        self.summary = tk.StringVar(value="Choose a CSV or JSON file to begin.")
        self.feedback = tk.StringVar(value="Ready — no CARLA operation has run.")

        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Heading.TLabel", font=("DejaVu Sans", 17, "bold"))
        style.configure("Caption.TLabel", font=("DejaVu Sans", 9))
        style.configure("Action.TButton", padding=(10, 8))
        style.configure("Launch.TButton", padding=(12, 12), font=("DejaVu Sans", 11, "bold"))

        scroller = tk.Canvas(root, highlightthickness=0, borderwidth=0)
        scroller.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(root, orient=tk.VERTICAL, command=scroller.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        scroller.configure(yscrollcommand=scrollbar.set)
        body = ttk.Frame(scroller, padding=12)
        frame_window = scroller.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", lambda _: scroller.configure(scrollregion=scroller.bbox("all")))
        scroller.bind("<Configure>", lambda e: scroller.itemconfigure(frame_window, width=e.width))
        ttk.Label(body, text="ASURT  /  Cone Track Importer", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(body, text="Load a Formula Student CSV/JSON course, preview it in CARLA, place cones and position the FSAI car.",
                  style="Caption.TLabel").pack(anchor="w", pady=(2, 12))

        file_box = ttk.LabelFrame(body, text="Track file", padding=10)
        file_box.pack(fill=tk.X)
        file_row = ttk.Frame(file_box)
        file_row.pack(fill=tk.X)
        file_entry = ttk.Entry(file_row, textvariable=self.path)
        file_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(file_row, text="Choose CSV / JSON", command=self.choose_file).pack(side=tk.LEFT, padx=(8, 0))
        presets = ttk.Frame(file_box)
        presets.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(presets, text="Formula Student UK 2027:").pack(side=tk.LEFT, padx=(0, 8))
        for title in COMPETITION_TRACKS:
            ttk.Button(presets, text=title,
                       command=lambda name=title: self.load_competition_track(name)).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(file_box, textvariable=self.summary).pack(anchor="w", pady=(7, 0))

        actions = ttk.LabelFrame(body, text="CARLA actions", padding=8)
        actions.pack(fill=tk.X, pady=(8, 0))
        row = ttk.Frame(actions)
        row.pack(fill=tk.X)
        self.action(row, "Preview cones", "preview")
        self.action(row, "Spawn physical cones", "spawn")
        self.action(row, "Remove my cones", "clear")
        self.action(row, "Check status", "status")
        row2 = ttk.Frame(actions)
        row2.pack(fill=tk.X, pady=(6, 0))
        self.action(row2, "Find ground-supported placement", "find-placement")
        self.action(row2, "Preview car start", "car-preview")
        row3 = ttk.Frame(actions)
        row3.pack(fill=tk.X, pady=(6, 0))
        self.action(row3, "POSITION EXISTING FSAI CAR AT TRACK START", "place-car", emphasis=True)

        self.canvas = tk.Canvas(body, height=170, background="#f8fafc", highlightthickness=1,
                                highlightbackground="#cbd5e1")
        self.canvas.pack(fill=tk.X, pady=(10, 10))
        self.canvas.bind("<Configure>", lambda _: self.draw_map())

        cfg = ttk.LabelFrame(body, text="World placement (metres) — fixed course origin", padding=10)
        cfg.pack(fill=tk.X)
        first = ttk.Frame(cfg)
        first.pack(fill=tk.X)
        for label, variable, width in (
            ("Host", self.host, 13), ("Port", self.port, 6),
            ("X", self.ox, 12), ("Y", self.oy, 12),
            ("Z", self.oz, 9), ("Yaw °", self.yaw, 9)
        ):
            cell = ttk.Frame(first)
            cell.pack(side=tk.LEFT, padx=(0, 9))
            ttk.Label(cell, text=label).pack(anchor="w")
            ttk.Entry(cell, textvariable=variable, width=width).pack()
        ttk.Button(cfg, text="Use current FSAI position as origin", command=self.capture_hero).pack(anchor="w", pady=(9, 0))

        second = ttk.Frame(cfg)
        second.pack(fill=tk.X, pady=(8, 0))
        ttk.Checkbutton(second, text="Mirror Y (correct Trackdraw image coordinates)",
                        variable=self.invert_y, command=self.draw_map).pack(side=tk.LEFT)
        ttk.Label(second, text="Forward shift (m):").pack(side=tk.LEFT, padx=(20, 5))
        ttk.Entry(second, textvariable=self.forward, width=8).pack(side=tk.LEFT)
        ttk.Label(second, text="Car start offset (m):").pack(side=tk.LEFT, padx=(15, 5))
        start_entry = ttk.Entry(second, textvariable=self.start_forward, width=7)
        start_entry.pack(side=tk.LEFT)
        start_entry.bind("<KeyRelease>", lambda _: self.draw_map())
        third = ttk.Frame(cfg)
        third.pack(fill=tk.X, pady=(5, 0))
        ttk.Label(third, text="Height mode:").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Radiobutton(third, text="Project onto terrain", variable=self.ground,
                        value="project").pack(side=tk.LEFT)
        ttk.Radiobutton(third, text="Fixed Z (flat surface only)", variable=self.ground,
                        value="fixed").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(cfg, text="Forward shift moves the WHOLE course; Car start offset moves ONLY the car along the first gate direction (0 = gate centre).",
                  style="Caption.TLabel", wraplength=825).pack(anchor="w", pady=(5, 0))
        ttk.Label(cfg, text="Find ground-supported placement tests nearby X/Y positions and yaw on the running map. "
                  "It changes the GUI fields only after all cone positions pass ground checks; "
                  "always preview the suggestion before spawning.",
                  style="Caption.TLabel", wraplength=825).pack(anchor="w")

        ttk.Label(body, textvariable=self.feedback, font=("DejaVu Sans", 9, "bold")).pack(anchor="w", pady=(11, 5))
        self.log = scrolledtext.ScrolledText(body, height=5, font=("DejaVu Sans Mono", 9),
                                            state=tk.DISABLED, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)

        if initial_file:
            self.path.set(initial_file)
            self.load_file()
        else:
            self.draw_map()
        self.write_log("Use 'Choose CSV / JSON'. A preview in CARLA makes no changes to the car or physical cones.")

    def action(self, parent, label: str, mode: str, emphasis: bool = False) -> None:
        button = ttk.Button(parent, text=label, command=lambda: self.perform(mode),
                            style="Launch.TButton" if emphasis else "Action.TButton")
        button.pack(side=tk.LEFT, padx=(0, 7), fill=tk.X, expand=emphasis)
        self.buttons.append(button)

    def write_log(self, message: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, message.rstrip() + "\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def choose_file(self) -> None:
        location = filedialog.askopenfilename(
            title="Choose an FS Trackdraw track",
            initialdir=str(Path.home()),
            filetypes=[("Cone track CSV or JSON", "*.csv *.json"), ("All files", "*.*")]
        )
        if location:
            self.path.set(location)
            self.load_file()

    def load_competition_track(self, name: str) -> None:
        """Load a built-in cone list; no CARLA mutation or automatic spawn."""
        if self.busy:
            return
        path = COMPETITION_TRACKS[name]
        if not path.is_file():
            messagebox.showerror("Missing competition track",
                                 f"Generate the 2027 track files first:\n{path}")
            return
        self.path.set(str(path))
        self.load_file()
        self.write_log(f"Selected {name}. Check origin and terrain with Preview; "
                       "the course has not been physically spawned.")

    def load_file(self) -> None:
        try:
            self.cones = load_cones(self.path.get())
            self.invalid_indices.clear()
            try:
                self.invert_y.set(suggested_invert_y(self.cones))
                start = track_start(self.cones, self.invert_y.get())
                start_label = f" | First gate {start[3]:.2f} m"
            except ValueError as exc:
                start_label = f" | Car start needs ordered sides: {exc}"
            blue = sum(cone.color == "blue" for cone in self.cones)
            yellow = sum(cone.color == "yellow" for cone in self.cones)
            others = [(color, sum(cone.color == color for cone in self.cones))
                      for color in ("orange_big", "orange", "unknown")]
            extras = "".join(f"  •  {count} {color}" for color, count in others if count)
            self.summary.set(f"{len(self.cones)} cones  •  {blue} blue  •  {yellow} yellow{extras}{start_label}")
            self.write_log(f"Loaded {Path(self.path.get()).name}. Auto Y-mirror: {self.invert_y.get()}.")
            self.draw_map()
        except (ValueError, OSError) as exc:
            self.cones = []
            self.summary.set(str(exc))
            self.draw_map()
            messagebox.showerror("Unable to load track", str(exc))

    def draw_map(self) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 720)
        height = max(self.canvas.winfo_height(), 170)
        if not self.cones:
            self.canvas.create_text(width / 2, height / 2, text="Blue/yellow cone layout appears here when you choose a track",
                                    fill="#64748b", font=("DejaVu Sans", 11))
            return
        pts = [(cone.x, -cone.y if self.invert_y.get() else cone.y, cone.color) for cone in self.cones]
        min_x, max_x = min(p[0] for p in pts), max(p[0] for p in pts)
        min_y, max_y = min(p[1] for p in pts), max(p[1] for p in pts)
        spread_x, spread_y = max(max_x - min_x, 2), max(max_y - min_y, 2)
        scale = min((width - 68) / spread_x, (height - 60) / spread_y)
        offset_x = (width - spread_x * scale) / 2
        offset_y = (height - spread_y * scale) / 2

        def screen(x, y):
            return offset_x + (x - min_x) * scale, offset_y + (y - min_y) * scale

        for color, line_color in (("blue", "#2563eb"), ("yellow", "#eab308")):
            line = [screen(x, y) for x, y, c in pts if c == color]
            if len(line) > 1:
                coords = [n for point in line for n in point]
                self.canvas.create_line(*coords, fill=line_color, width=1.3)
            for x, y in line:
                self.canvas.create_oval(x - 2.3, y - 2.3, x + 2.3, y + 2.3,
                                        fill=line_color, outline="")
        for color, shade in (("orange", "#f97316"), ("orange_big", "#ea580c"), ("unknown", "#64748b")):
            for x, y, c in pts:
                if c == color:
                    sx, sy = screen(x, y)
                    radius = 4.5 if color == "orange_big" else 2.8
                    self.canvas.create_oval(sx-radius, sy-radius, sx+radius, sy+radius,
                                            fill=shade, outline="")
        for index in self.invalid_indices:
            if index >= len(pts):
                continue
            x, y, _ = pts[index]
            sx, sy = screen(x, y)
            self.canvas.create_line(sx-6, sy-6, sx+6, sy+6, fill="#dc2626", width=2)
            self.canvas.create_line(sx-6, sy+6, sx+6, sy-6, fill="#dc2626", width=2)
        try:
            x, y, yaw, lane_width = track_start(self.cones, self.invert_y.get())
            offset = float(self.start_forward.get())
            if math.isfinite(offset):
                x += offset * math.cos(math.radians(yaw))
                y += offset * math.sin(math.radians(yaw))
            sx, sy = screen(x, y)
            theta = math.radians(yaw)
            self.canvas.create_oval(sx - 5, sy - 5, sx + 5, sy + 5,
                                    fill="#10b981", outline="#14532d", width=1)
            self.canvas.create_line(sx, sy, sx + 25 * math.cos(theta),
                                    sy + 25 * math.sin(theta), fill="#15803d", width=3, arrow=tk.LAST)
            self.canvas.create_text(sx + 10, sy - 14, text="START", fill="#166534",
                                    font=("DejaVu Sans", 9, "bold"))
        except ValueError:
            pass
        self.canvas.create_text(14, 14, anchor="w",
                                text="Blue left • Yellow right • Orange start/finish • Grey unclassified • Red X no ground",
                                fill="#475569", font=("DejaVu Sans", 9))

    def capture_hero(self) -> None:
        if self.busy:
            return
        self.set_busy(True, "Reading current hero...")
        try:
            host, port = self.host.get().strip(), int(self.port.get())
        except ValueError:
            self.set_busy(False, "Invalid CARLA port")
            return

        def worker():
            try:
                import carla
                from import_cone_track import find_hero
                client = carla.Client(host, port)
                client.set_timeout(8.0)
                world = client.get_world()
                hero = find_hero(world)
                if hero is None:
                    raise RuntimeError("No FSAI hero exists yet; start CARLA/ROS or use the configured origin")
                t = hero.get_transform()
                values = (t.location.x, t.location.y, t.location.z, t.rotation.yaw)
                self.root.after(0, lambda: self._set_anchor(values))
            except Exception as exc:
                self.root.after(0, lambda msg=str(exc): self.write_log("ERROR: " + msg))
            finally:
                self.root.after(0, lambda: self.set_busy(False, "Anchor read completed"))
        threading.Thread(target=worker, daemon=True).start()

    def _set_anchor(self, values) -> None:
        for variable, value in zip((self.ox, self.oy, self.oz, self.yaw), values):
            variable.set(f"{value:.4f}")
        self.write_log("Captured FSAI position as the fixed course origin. Future car moves will not move the cones.")

    def command(self, mode: str) -> list[str]:
        parts = [sys.executable, str(IMPORTER)]
        if mode not in ("status", "clear", "blueprints"):
            path = Path(self.path.get()).expanduser()
            if not self.cones or not path.is_file():
                raise ValueError("Choose a valid CSV or JSON track file first")
            parts.append(str(path))
        host = self.host.get().strip()
        port = int(self.port.get())
        if not host or not 1 <= port <= 65535:
            raise ValueError("Invalid server address or port")
        parts += ["--mode", mode, "--host", host, "--port", str(port)]
        if self.state is not None:
            parts += ["--state", str(self.state)]
        if mode not in ("status", "clear", "blueprints"):
            settings = (("--origin-x", self.ox), ("--origin-y", self.oy),
                        ("--origin-z", self.oz), ("--yaw", self.yaw),
                        ("--forward-offset", self.forward))
            for option, variable in settings:
                value = float(variable.get())
                if not math.isfinite(value):
                    raise ValueError(f"{option} must be finite")
                parts += [option, str(value)]
            parts += ["--anchor", "coords"]
            parts += ["--ground", "project" if mode == "find-placement" else self.ground.get()]
            if self.invert_y.get():
                parts.append("--invert-y")
            if mode in ("car-preview", "place-car"):
                offset = float(self.start_forward.get())
                if not math.isfinite(offset):
                    raise ValueError("Car start offset must be finite")
                parts += ["--start-forward", str(offset)]
            # Physical spawning prefers the four custom ASURT cone models.
            # Do not silently fall back to oversized generic road props.
        return parts

    def perform(self, mode: str) -> None:
        if self.busy:
            return
        try:
            command = self.command(mode)
            if mode in ("car-preview", "place-car"):
                track_start(self.cones, self.invert_y.get())
        except (ValueError, OSError) as exc:
            messagebox.showerror("Invalid configuration", str(exc))
            return

        if mode == "place-car" and not messagebox.askyesno(
            "Position FSAI car", "Place the FSAI hero at the first blue/yellow gate?\n"
            "This will move the existing car and stop its current motion."):
            return
        if mode == "spawn" and self.ground.get() == "fixed" and not messagebox.askyesno(
            "Unverified ground elevation", "Fixed Z ignores ground projection. "
            "Cones may float or pass through the terrain. Spawn anyway?"):
            return
        if mode == "preview":
            self.invalid_indices.clear()
            self.draw_map()
        self.set_busy(
            True,
            "Checking nearby ground placements..." if mode == "find-placement"
            else (f"Checking FSAI start against {len(self.cones)} cones; "
                  "positioning existing car only after clearance..."
                  if mode == "place-car" else f"Running {mode}...")
        )
        self.write_log(f">>> {mode}: {Path(self.path.get()).name if self.path.get() else 'managed track'}")

        def worker():
            try:
                if mode == "find-placement":
                    code, output = self.run_ground_search(command)
                else:
                    result = subprocess.run(command, capture_output=True, text=True, timeout=180)
                    code = result.returncode
                    output = "\n".join(part.strip() for part in (result.stdout, result.stderr) if part.strip())
                self.root.after(0, lambda: self._complete(mode, code, output))
            except Exception as exc:
                self.root.after(0, lambda msg=str(exc): self._complete(mode, 1, msg))
        threading.Thread(target=worker, daemon=True).start()

    def run_ground_search(self, command):
        lines = []
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True) as process:
            timed_out = threading.Event()

            def expire():
                if process.poll() is None:
                    timed_out.set()
                    process.kill()

            timer = threading.Timer(240, expire)
            timer.daemon = True
            timer.start()
            try:
                for line in process.stdout:
                    if line.startswith("PLACEMENT_PROGRESS: "):
                        message = line.split(": ", 1)[1].strip()
                        self.root.after(0, lambda message=message: self.feedback.set(message))
                    else:
                        lines.append(line)
                code = process.wait()
            finally:
                timer.cancel()
        if timed_out.is_set():
            lines.append("Ground placement search timed out while waiting for CARLA.\n")
        return code, "".join(lines).strip()

    def set_busy(self, busy: bool, status: str) -> None:
        self.busy = busy
        self.feedback.set(status)
        for button in self.buttons:
            button.configure(state=tk.DISABLED if busy else tk.NORMAL)

    def _complete(self, mode: str, code: int, output: str) -> None:
        if mode in ("preview", "spawn"):
            match = re.search(r"INVALID_INDICES: ([0-9,]+)", output)
            self.invalid_indices = ({int(value)-1 for value in match.group(1).split(",")}
                                    if match else set())
            self.draw_map()
        self.write_log(output or f"{mode}: no output")
        if mode == "find-placement" and not code:
            match = re.search(r"^PLACEMENT_RESULT: (.+)$", output, re.MULTILINE)
            try:
                result = json.loads(match.group(1)) if match else None
            except (ValueError, TypeError):
                result = None
            if not isinstance(result, dict):
                self.set_busy(False, "Failed: missing ground-search result")
                messagebox.showerror("Ground search failed",
                                     "The importer did not return a valid placement. See the log.")
                return
            if result.get("found") and result.get("invalid") == 0:
                self.ox.set(f"{result['origin_x']:.3f}")
                self.oy.set(f"{result['origin_y']:.3f}")
                self.yaw.set(f"{result['yaw']:.3f}")
                self.ground.set("project")
                self.invalid_indices.clear()
                self.draw_map()
                self.set_busy(False, "Ground-supported candidate found — PREVIEW before spawning")
                messagebox.showinfo("Candidate found; preview required",
                                    f"All {result['total']} cones passed ground checks.\n"
                                    f"Suggested origin: ({result['origin_x']:.2f}, "
                                    f"{result['origin_y']:.2f}) m, yaw {result['yaw']:.1f}°.\n\n"
                                    "Fields updated. Preview the track in CARLA and visually "
                                    "confirm it is on drivable asphalt before spawning.")
            else:
                self.set_busy(False, "No fully grounded candidate found; original fields retained")
                messagebox.showwarning("No complete placement found",
                                       f"Checked {result['candidates_sampled']} nearby positions "
                                       f"and fully verified {result['candidates_verified']}.\n"
                                       f"Best still has {result['invalid']} unsupported "
                                       f"of {result['total']} cones.\n\n"
                                       "Your X/Y/Yaw settings were NOT changed. "
                                       "Try a different course region or orientation.")
            return
        summary = (f"Preview: {len(self.invalid_indices)} cones have no verified ground"
                   if mode == "preview" and self.invalid_indices
                   else f"Spawn blocked: {len(self.invalid_indices)} cones have no ground"
                   if mode == "spawn" and self.invalid_indices
                   else f"{'Completed' if code == 0 else 'Failed'}: {mode}")
        self.set_busy(False, summary)
        if code:
            messagebox.showerror("CARLA operation failed", short_failure(mode, output))
        elif mode == "preview" and self.invalid_indices:
            messagebox.showwarning("Track placement needs adjustment",
                                   f"{len(self.invalid_indices)} cone positions have no verified ground. "
                                   "Red X marks them on the map. Negative coordinates are valid; "
                                   "the course crosses locations without suitable ground collision. "
                                   "Adjust X/Y, Yaw or Forward shift and preview again.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("track", nargs="?", default="", help="Optional CSV/JSON file to load at startup")
    parser.add_argument("--state", type=Path, help="Use a separate managed cone ownership file")
    parser.add_argument("--smoke-test", action="store_true", help="Open and close UI automatically (no CARLA actions)")
    args = parser.parse_args()
    root = tk.Tk()
    app = ConeTrackWindow(root, initial_file=args.track, state=args.state)
    if args.smoke_test:
        root.after(1000, root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
