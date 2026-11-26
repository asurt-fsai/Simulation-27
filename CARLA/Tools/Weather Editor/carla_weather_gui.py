#!/usr/bin/env python3
"""Small native Tk GUI for live CARLA weather and sun control (observer + weather setter only).

No world ticks, actor spawning, vehicle control, ROS publishers, or sensor restarts.
The running native CARLA world is the source of truth. Slider edits are debounced,
merged and applied to freshly fetched WeatherParameters to preserve other fields.
"""
from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk

CARLA_HOST = os.environ.get("ASURT_CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("ASURT_CARLA_PORT", "2000"))

# name, display label, min, max, resolution, units
FIELDS = (
    ("sun_azimuth_angle", "Sun azimuth", 0, 360, 1, "°"),
    ("sun_altitude_angle", "Sun altitude", -90, 90, 1, "°"),
    ("cloudiness", "Cloud cover", 0, 100, 1, "%"),
    ("precipitation", "Rainfall", 0, 100, 1, "%"),
    ("precipitation_deposits", "Standing water", 0, 100, 1, "%"),
    ("wetness", "Road wetness", 0, 100, 1, "%"),
    ("wind_intensity", "Wind", 0, 100, 1, "%"),
    ("fog_density", "Fog density", 0, 100, 1, "%"),
    ("fog_distance", "Fog start distance", 0, 200, 0.5, "m"),
    ("fog_falloff", "Fog height falloff", 0, 10, 0.05, ""),
    ("dust_storm", "Dust storm", 0, 100, 1, "%"),
)
BY_NAME = {item[0]: item for item in FIELDS}
LIGHT_FIELDS = ("sun_azimuth_angle", "sun_altitude_angle", "cloudiness")
ATMOS_FIELDS = tuple(x[0] for x in FIELDS if x[0] not in LIGHT_FIELDS)

BACKGROUND = "#101826"
CARD = "#192536"
SURFACE = "#243449"
TEXT = "#eef4fa"
MUTED = "#a9b9cb"
ACCENT = "#50b9f1"
SUCCESS = "#75d7a2"
WARNING = "#ffce78"


def clean_value(name: str, raw: float) -> float:
    """Validate and quantize one allowed CARLA WeatherParameters control."""
    if name not in BY_NAME:
        raise ValueError("Unknown weather parameter: " + name)
    _, _, lower, upper, resolution, _ = BY_NAME[name]
    value = float(raw)
    if not math.isfinite(value):
        raise ValueError("Weather values must be finite")
    value = min(upper, max(lower, value))
    steps = round((value - lower) / resolution)
    return round(lower + steps * resolution, 5)


def weather_values(weather) -> dict[str, float]:
    return {name: float(getattr(weather, name))
            for name in BY_NAME if hasattr(weather, name)}


def set_weather_fields(world, updates: dict[str, float]) -> dict[str, float]:
    """Preserve unedited CARLA weather settings; only set explicitly touched fields."""
    checked = {key: clean_value(key, value) for key, value in updates.items()}
    current = world.get_weather()
    for key, value in checked.items():
        setattr(current, key, value)
    if checked:
        world.set_weather(current)
    return weather_values(world.get_weather())


CAMERA_PRESETS = ("sun_low", "sun_high", "sun_behind")


def scene_preset(name: str, heading: float | None) -> dict[str, float]:
    """Return changed fields only. Heading comes from the live ZED left camera."""
    sunny = dict(cloudiness=0, precipitation=0, precipitation_deposits=0,
                 fog_density=0, dust_storm=0)
    if name in CAMERA_PRESETS:
        if heading is None:
            raise ValueError("Camera heading unavailable. Check that the ZED stereo camera is streaming.")
        sunny["sun_azimuth_angle"] = (heading + (180 if name == "sun_behind" else 0)) % 360
        sunny["sun_altitude_angle"] = 10 if name != "sun_high" else 35
        return sunny
    presets = {
        "clear_noon": dict(sunny, sun_altitude_angle=70),
        "overcast": dict(cloudiness=90, precipitation=0, precipitation_deposits=0,
                         fog_density=6, dust_storm=0),
        "heavy_rain": dict(cloudiness=95, precipitation=85, precipitation_deposits=70,
                           wetness=90, wind_intensity=65, fog_density=16, dust_storm=0),
        "dense_fog": dict(cloudiness=70, precipitation=0, precipitation_deposits=0,
                          fog_density=85, fog_distance=0, fog_falloff=2.0,
                          dust_storm=0),
        "dust": dict(cloudiness=20, precipitation=0, precipitation_deposits=0,
                     fog_density=12, dust_storm=85, wind_intensity=70),
    }
    if name not in presets:
        raise ValueError("Unknown preset: " + name)
    return presets[name]


class CarlaWorker(threading.Thread):
    """All potentially blocking CARLA RPC calls run in one background thread."""

    def __init__(self, requests: queue.Queue, results: queue.Queue):
        super().__init__(name="carla-weather-rpc", daemon=True)
        self.requests = requests
        self.results = results
        self.client = None
        self.world = None
        self.camera_heading = None
        self.last_observed = {}

    def send(self, kind: str, **kwargs):
        self.results.put((kind, kwargs))

    def connect(self, initial: bool):
        try:
            import carla
            self.client = carla.Client(CARLA_HOST, CARLA_PORT)
            self.client.set_timeout(2.5)
            self.world = self.client.get_world()
            observed = weather_values(self.world.get_weather())
            self.last_observed = observed
            self.update_heading()
            self.send("connected", weather=observed,
                      heading=self.camera_heading,
                      map=self.world.get_map().name,
                      initial=initial)
        except Exception as exc:
            self.client = self.world = None
            self.send("error", message=f"CARLA connection failed: {exc}")

    def update_heading(self):
        if self.world is None:
            self.camera_heading = None
            return
        try:
            if self.world.get_snapshot().frame == 0:
                self.world.wait_for_tick(2.5)
            cameras = [actor for actor in self.world.get_actors().filter("sensor.camera.rgb")
                       if actor.attributes.get("role_name") == "asurt_zed2i_sim_left"]
            self.camera_heading = (float(cameras[0].get_transform().rotation.yaw)
                                   if len(cameras) == 1 else None)
        except Exception:
            self.camera_heading = None

    def run(self):
        self.connect(initial=True)
        while True:
            kind, payload = self.requests.get()
            if kind == "exit":
                return
            try:
                if kind == "connect":
                    self.connect(initial=False)
                elif kind == "refresh":
                    if self.world is None:
                        self.connect(initial=False)
                    else:
                        self.update_heading()
                        values = weather_values(self.world.get_weather())
                        self.last_observed = values
                        self.send("observed", weather=values,
                                  heading=self.camera_heading)
                elif kind in ("apply", "camera-preset"):
                    if self.world is None:
                        raise RuntimeError("Not connected to CARLA; reconnect before applying")
                    revision = payload["revision"]
                    if kind == "camera-preset":
                        self.update_heading()
                        updates = scene_preset(payload["name"], self.camera_heading)
                    else:
                        updates = dict(payload["updates"])
                        # Coalesce queued edits while preserving the order of other operations.
                        while True:
                            try:
                                following, more = self.requests.queue[0]
                            except IndexError:
                                break
                            if following != "apply":
                                break
                            self.requests.get_nowait()
                            updates.update(more["updates"])
                            revision = more["revision"]
                    values = set_weather_fields(self.world, updates)
                    self.last_observed = values
                    self.send("applied", weather=values, revision=revision, updates=updates,
                              heading=self.camera_heading, camera_preset=kind == "camera-preset")
            except Exception as exc:
                self.send("error", message=f"{kind}: {exc}")


class WeatherPanel:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ASURT | CARLA Weather Control")
        self.root.configure(bg=BACKGROUND)
        self.root.geometry("980x735")
        self.root.minsize(850, 630)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.requests = queue.Queue()
        self.results = queue.Queue()
        self.worker = CarlaWorker(self.requests, self.results)
        self.original = None
        self.heading = None
        self.connected = False
        self.last_edit = 0.0
        self.revision = 0
        self._programmatic = False
        self.pending = {}
        self.debounce_id = None
        self.controls = {}
        self.status_var = tk.StringVar(value=f"Connecting to CARLA {CARLA_HOST}:{CARLA_PORT}…")
        self.heading_var = tk.StringVar(value="Camera heading: not available")
        self.scene_var = tk.StringVar(value="Source: waiting for CARLA…")
        self.feedback_var = tk.StringVar(value="No simulation changes until you move a control or choose a preset.")
        self._style()
        self._build()
        self.worker.start()
        self.root.after(90, self._drain)
        self.root.after(3000, self._refresh_cycle)

    def _style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=BACKGROUND)
        style.configure("Card.TFrame", background=CARD)
        style.configure("TLabel", background=BACKGROUND, foreground=TEXT, font=("DejaVu Sans", 10))
        style.configure("Card.TLabel", background=CARD, foreground=TEXT, font=("DejaVu Sans", 10))
        style.configure("Muted.TLabel", background=BACKGROUND, foreground=MUTED, font=("DejaVu Sans", 10))
        style.configure("Hint.TLabel", background=CARD, foreground=MUTED, font=("DejaVu Sans", 9))
        style.configure("Title.TLabel", background=BACKGROUND, foreground=TEXT,
                        font=("DejaVu Sans", 20, "bold"))
        style.configure("Header.TLabel", background=CARD, foreground=ACCENT,
                        font=("DejaVu Sans", 12, "bold"))
        style.configure("Value.TLabel", background=CARD, foreground=SUCCESS,
                        font=("DejaVu Sans", 10, "bold"))
        style.configure("TButton", background=SURFACE, foreground=TEXT,
                        borderwidth=0, padding=(12, 8), font=("DejaVu Sans", 10))
        style.map("TButton", background=[("active", "#33506b"), ("disabled", "#202b39")],
                  foreground=[("disabled", "#73869c")])
        style.configure("Accent.TButton", background="#176c93", foreground="white", padding=(14, 9))
        style.map("Accent.TButton", background=[("active", "#2187b7")])
        style.configure("TScale", background=CARD, troughcolor=SURFACE)

    def _build(self):
        shell = ttk.Frame(self.root, padding=(20, 18))
        shell.pack(fill="both", expand=True)
        ttk.Label(shell, text="CARLA Weather Studio", style="Title.TLabel").pack(anchor="w")
        ttk.Label(shell, text="Native Silverstone  •  live weather + camera-facing sun tests  •  no vehicle commands",
                  style="Muted.TLabel").pack(anchor="w", pady=(3, 14))
        toolbar = ttk.Frame(shell)
        toolbar.pack(fill="x", pady=(0, 12))
        self.connection_label = ttk.Label(toolbar, textvariable=self.status_var)
        self.connection_label.pack(side="left")
        ttk.Button(toolbar, text="Reconnect", command=lambda: self.requests.put(("connect", {}))).pack(side="right")
        ttk.Button(toolbar, text="Read CARLA", command=self.refresh).pack(side="right", padx=(0, 8))
        ttk.Label(shell, textvariable=self.scene_var, style="Muted.TLabel").pack(anchor="w")
        ttk.Label(shell, textvariable=self.heading_var, style="Muted.TLabel").pack(anchor="w", pady=(1, 12))
        columns = ttk.Frame(shell)
        columns.pack(fill="both", expand=True)
        left = ttk.Frame(columns, style="Card.TFrame", padding=(16, 13))
        right = ttk.Frame(columns, style="Card.TFrame", padding=(16, 13))
        left.pack(side="left", expand=True, fill="both", padx=(0, 8))
        right.pack(side="left", expand=True, fill="both", padx=(8, 0))
        ttk.Label(left, text="☀  SUN & SKY", style="Header.TLabel").pack(anchor="w", pady=(0, 12))
        for name in LIGHT_FIELDS:
            self._add_slider(left, name)
        ttk.Label(left, text="Align sun with the camera for strong frontal glare.",
                  style="Hint.TLabel", wraplength=375).pack(anchor="w", pady=(12, 7))
        for title, preset in [("Sun into camera · low 10°", "sun_low"),
                              ("Sun into camera · high 35°", "sun_high"),
                              ("Sun behind vehicle", "sun_behind"),
                              ("Clear noon", "clear_noon")]:
            ttk.Button(left, text=title, command=lambda p=preset: self.preset(p)).pack(
                fill="x", pady=(0, 6))
        ttk.Label(right, text="☁  WEATHER & ATMOSPHERE", style="Header.TLabel").pack(
            anchor="w", pady=(0, 9))
        for name in ATMOS_FIELDS:
            self._add_slider(right, name)
        presets = ttk.Frame(shell)
        presets.pack(fill="x", pady=(13, 8))
        ttk.Label(presets, text="QUICK CONDITIONS", style="Muted.TLabel").pack(
            side="left", padx=(0, 8))
        for label, name in [("Overcast", "overcast"), ("Heavy rain", "heavy_rain"),
                            ("Dense fog", "dense_fog"), ("Dust", "dust")]:
            ttk.Button(presets, text=label, command=lambda p=name: self.preset(p)).pack(
                side="left", padx=3)
        ttk.Button(presets, text="Restore launch weather", command=self.restore_start).pack(
            side="right")
        ttk.Label(shell, textvariable=self.feedback_var, style="Muted.TLabel",
                  wraplength=920).pack(anchor="w", pady=(5, 3))
        ttk.Label(shell,
                  text="Sliders publish after 180 ms idle. Closing the panel keeps the last applied weather.",
                  style="Muted.TLabel").pack(anchor="w")

    def _add_slider(self, container, name):
        _, label, minimum, maximum, resolution, unit = BY_NAME[name]
        row = ttk.Frame(container, style="Card.TFrame")
        row.pack(fill="x", pady=(0, 6))
        heading = ttk.Frame(row, style="Card.TFrame")
        heading.pack(fill="x")
        ttk.Label(heading, text=label, style="Card.TLabel").pack(side="left")
        value_label = ttk.Label(heading, text="—", style="Value.TLabel")
        value_label.pack(side="right")
        variable = tk.DoubleVar(value=minimum)
        slider = ttk.Scale(row, from_=minimum, to=maximum, variable=variable,
                           command=lambda value, n=name: self.on_slider(n, value))
        slider.pack(fill="x", pady=(1, 0))
        slider.bind("<ButtonRelease-1>", lambda _e: self._submit())
        self.controls[name] = (variable, value_label, unit, resolution, slider)

    def _format(self, name, value):
        _, _, _, _, resolution, unit = BY_NAME[name]
        digits = 2 if resolution < 0.1 else (1 if resolution < 1 else 0)
        return f"{value:.{digits}f}{unit}"

    def _fill(self, values):
        self._programmatic = True
        try:
            for name, (variable, label, unit, resolution, slider) in self.controls.items():
                if name in values:
                    variable.set(values[name])
                    label.configure(text=self._format(name, values[name]))
        finally:
            self._programmatic = False

    def on_slider(self, name, raw):
        if self._programmatic or not self.connected:
            return
        value = clean_value(name, raw)
        self.controls[name][1].configure(text=self._format(name, value))
        self.pending[name] = value
        self.revision += 1
        self.last_edit = time.monotonic()
        self.feedback_var.set("Editing… applying live after drag settles")
        if self.debounce_id is not None:
            self.root.after_cancel(self.debounce_id)
        self.debounce_id = self.root.after(180, self._submit)

    def _submit(self):
        self.debounce_id = None
        if not self.pending or not self.connected:
            return
        updates = self.pending.copy()
        self.pending.clear()
        self.requests.put(("apply", {"revision": self.revision, "updates": updates}))

    def preset(self, name):
        if not self.connected:
            self.feedback_var.set("Connect to CARLA before applying a preset.")
            return
        if name in CAMERA_PRESETS:
            if self.debounce_id is not None:
                self.root.after_cancel(self.debounce_id)
            self._submit()
            self.revision += 1
            self.last_edit = time.monotonic()
            self.requests.put(("camera-preset", {"name": name, "revision": self.revision}))
            return
        try:
            updates = {k: clean_value(k, v) for k, v in scene_preset(name, None).items()}
        except Exception as exc:
            self.feedback_var.set(str(exc))
            return
        # A preset overrides incomplete slider drags without losing unrelated fields.
        if self.debounce_id is not None:
            self.root.after_cancel(self.debounce_id)
            self.debounce_id = None
        self.pending.update(updates)
        self.revision += 1
        self.last_edit = time.monotonic()
        self._fill({**{k: var.get() for k, (var, _, _, _, _) in self.controls.items()},
                    **updates})
        self._submit()

    def restore_start(self):
        if self.original is None:
            self.feedback_var.set("No launch-time weather snapshot available.")
            return
        if self.debounce_id is not None:
            self.root.after_cancel(self.debounce_id)
            self.debounce_id = None
        self.pending = self.original.copy()
        self.revision += 1
        self.last_edit = time.monotonic()
        self._fill(self.original)
        self._submit()

    def refresh(self):
        self.requests.put(("refresh", {}))

    def _refresh_cycle(self):
        if time.monotonic() - self.last_edit > 1.5 and self.requests.qsize() < 2:
            self.refresh()
        self.root.after(3000, self._refresh_cycle)

    def _drain(self):
        while True:
            try:
                kind, data = self.results.get_nowait()
            except queue.Empty:
                break
            if kind == "error":
                self.status_var.set("CARLA unavailable / command failed")
                self.feedback_var.set(data["message"])
                if data["message"].startswith("CARLA connection"):
                    self.connected = False
                continue
            if kind == "connected":
                self.connected = True
                if self.original is None:
                    self.original = data["weather"].copy()
                self.scene_var.set("Map: " + data["map"] + f"  |  CARLA {CARLA_HOST}:{CARLA_PORT}")
            if kind in ("connected", "observed", "applied"):
                if "heading" in data:
                    self.heading = data["heading"]
                    self.heading_var.set(
                        "ZED left-eye heading: " +
                        (f"{self.heading:.1f}°  •  sun-to-camera presets use this bearing"
                         if self.heading is not None else "unavailable"))
                recent = (time.monotonic() - self.last_edit < 1.0)
                stale = kind == "applied" and data["revision"] < self.revision
                if not self.pending and not stale and (not recent or data.get("camera_preset")):
                    self._fill(data["weather"])
                if kind == "connected":
                    self._fill(data["weather"])
                self.status_var.set("● LIVE · CARLA weather available")
                if kind == "applied":
                    self.feedback_var.set(
                        f"Applied + read back {len(data['updates'])} control(s) · " +
                        datetime.now().strftime("%H:%M:%S"))
                elif kind == "connected":
                    self.feedback_var.set("Connected; displaying current CARLA weather. No values changed.")
        self.root.after(100, self._drain)

    def close(self):
        if self.debounce_id is not None and self.pending:
            self._submit()
        self.requests.put(("exit", {}))
        self.root.destroy()


def main():
    root = tk.Tk()
    WeatherPanel(root)
    root.mainloop()


if __name__ == "__main__":
    main()
