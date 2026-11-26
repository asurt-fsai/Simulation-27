#!/usr/bin/env python3
"""Small native Tk GUI for live CARLA weather and sun control (observer + weather setter only).

No world ticks, actor spawning, vehicle control, ROS publishers, or sensor restarts.
The running native CARLA world is the source of truth. Slider edits are debounced,
merged and applied to freshly fetched WeatherParameters to preserve other fields.
"""
from __future__ import annotations

import argparse
import os
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from weather_control import (ATMOS_FIELDS, BY_NAME, CAMERA_PRESETS, LIGHT_FIELDS,
                             camera_heading, clean_value, read_status, scene_preset,
                             set_weather_fields, thermal_status, weather_values)

CARLA_HOST = os.environ.get("ASURT_CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("ASURT_CARLA_PORT", "2000"))
ROAD_WEATHER_DIR = os.environ.get("ASURT_ROAD_WEATHER_DIR")

BACKGROUND = "#101826"
CARD = "#192536"
SURFACE = "#243449"
TEXT = "#eef4fa"
MUTED = "#a9b9cb"
ACCENT = "#50b9f1"
SUCCESS = "#75d7a2"
WARNING = "#ffce78"


class CarlaWorker(threading.Thread):
    """All potentially blocking CARLA RPC calls run in one background thread."""

    def __init__(self, requests: queue.Queue, results: queue.Queue, *, status_only=False):
        super().__init__(name="carla-weather-rpc", daemon=True)
        self.requests = requests
        self.results = results
        self.client = None
        self.world = None
        self.camera_heading = None
        self.last_observed = {}
        self.status_only = status_only
        self.map_name = None

    def send(self, kind: str, **kwargs):
        self.results.put((kind, kwargs))

    def road(self):
        return thermal_status(ROAD_WEATHER_DIR, self.world.get_snapshot().timestamp.elapsed_seconds)

    def status(self):
        self.send("status", **read_status(self.world, self.map_name, ROAD_WEATHER_DIR))

    def connect(self, initial: bool):
        try:
            import carla
            self.client = carla.Client(CARLA_HOST, CARLA_PORT)
            self.client.set_timeout(2.5)
            self.world = self.client.get_world()
            if self.world.get_snapshot().frame == 0:
                self.world.wait_for_tick(2.5)
            self.map_name = self.world.get_map().name
            if self.status_only:
                self.status()
                return
            observed = weather_values(self.world.get_weather())
            self.last_observed = observed
            self.update_heading()
            self.send("connected", weather=observed,
                      heading=self.camera_heading,
                      map=self.map_name,
                      road=self.road(),
                      initial=initial)
        except Exception as exc:
            self.client = self.world = None
            self.send("error", message=f"CARLA connection failed: {exc}")

    def update_heading(self):
        if self.world is None:
            self.camera_heading = None
            return
        try:
            self.camera_heading = camera_heading(self.world)
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
                    elif self.status_only:
                        self.status()
                    else:
                        self.update_heading()
                        values = weather_values(self.world.get_weather())
                        self.last_observed = values
                        self.send("observed", weather=values,
                                  heading=self.camera_heading, road=self.road())
                elif kind == "temperature":
                    if self.world is None:
                        raise RuntimeError("Connect to CARLA before setting temperature.")
                    set_weather_fields(self.world, {"air_temperature_c": payload["value"]}, ROAD_WEATHER_DIR)
                    self.status()
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
                    values = set_weather_fields(self.world, updates, ROAD_WEATHER_DIR)
                    self.last_observed = values
                    self.send("applied", weather=values, revision=revision, updates=updates,
                              heading=self.camera_heading, camera_preset=kind == "camera-preset", road=self.road())
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
        self.road_var = tk.StringVar()
        self.rain_seconds = tk.StringVar(value="60")
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
        ttk.Label(shell, text="Live weather for the active CARLA map",
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
        if ROAD_WEATHER_DIR:
            self._add_slider(left, "air_temperature_c")
            row = ttk.Frame(left, style="Card.TFrame")
            row.pack(fill="x", pady=(8, 0))
            ttk.Button(row, text="Start timed rain", command=self.start_timed_rain).pack(side="left")
            ttk.Spinbox(row, from_=1, to=3600, textvariable=self.rain_seconds, width=5).pack(side="left", padx=6)
            ttk.Label(row, text="seconds", style="Card.TLabel").pack(side="left")
            ttk.Label(left, textvariable=self.road_var, style="Hint.TLabel", wraplength=360).pack(anchor="w", pady=(6, 0))
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
                            ("Dense fog", "dense_fog"), ("Dust", "dust"), ("Wet road", "wet_road")]:
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

    def start_timed_rain(self):
        try:
            duration = clean_value("rain_duration_s", self.rain_seconds.get())
            if duration <= 0:
                raise ValueError("Choose a positive duration for timed rain.")
        except ValueError as exc:
            self.feedback_var.set(str(exc))
            return
        if not self.connected:
            self.feedback_var.set("Connect to CARLA before starting timed rain.")
            return
        self.pending.update(rain_duration_s=duration,
                            precipitation=self.controls["precipitation"][0].get() or 75.0)
        self.revision += 1
        self.last_edit = time.monotonic()
        self._submit()

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
                    if "air_temperature_c" in data["road"]:
                        self.original["air_temperature_c"] = data["road"]["air_temperature_c"]
                self.scene_var.set("Map: " + data["map"] + f"  |  CARLA {CARLA_HOST}:{CARLA_PORT}")
            if kind in ("connected", "observed", "applied"):
                road = data["road"]
                if "asphalt_temperature_c" in road:
                    remaining = road.get("rain_remaining_s")
                    self.road_var.set(f"Asphalt {road['asphalt_temperature_c']:.1f} °C   ·   "
                                      f"Tyres {road['tire_temperature_c']:.1f} °C" +
                                      (f"   ·   Timed rain {remaining}s left" if remaining is not None else ""))
                readings = {**data["weather"], **road}
                if "heading" in data:
                    self.heading = data["heading"]
                    self.heading_var.set(
                        "ZED left-eye heading: " +
                        (f"{self.heading:.1f}°  •  sun-to-camera presets use this bearing"
                         if self.heading is not None else "unavailable"))
                recent = (time.monotonic() - self.last_edit < 1.0)
                stale = kind == "applied" and data["revision"] < self.revision
                if not self.pending and not stale and (not recent or data.get("camera_preset")):
                    self._fill(readings)
                if kind == "connected":
                    self._fill(readings)
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


class WeatherStatusPanel:
    def __init__(self, root):
        self.root = root
        root.title("ASURT | Weather Status")
        root.configure(bg=BACKGROUND, padx=18, pady=16)
        self.values = tk.StringVar(value="Reading live weather…")
        tk.Label(root, textvariable=self.values, bg=BACKGROUND, fg=TEXT, justify="left",
                 anchor="w", font=("DejaVu Sans", 11)).pack(fill="both", expand=True)
        self.requests = queue.Queue()
        self.results = queue.Queue()
        if ROAD_WEATHER_DIR:
            self.temperature = tk.StringVar()
            controls = ttk.Frame(root)
            controls.pack(fill="x", pady=(9, 0))
            ttk.Spinbox(controls, from_=-20, to=65, textvariable=self.temperature, width=6).pack(side="left")
            ttk.Button(controls, text="Set air temperature (°C)", command=self.set_temperature).pack(side="left", padx=6)
        CarlaWorker(self.requests, self.results, status_only=True).start()
        root.after(100, self.drain)
        root.after(1500, self.refresh)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def refresh(self):
        if self.requests.empty():
            self.requests.put(("refresh", {}))
        self.root.after(1500, self.refresh)

    def drain(self):
        while not self.results.empty():
            kind, data = self.results.get_nowait()
            if kind == "error":
                self.values.set(data["message"])
                continue
            wet = data["weather"]
            road = data["road"]
            grip = data["road_grip"]
            entries = [data["map"].rsplit("/", 1)[-1], ""]
            if "asphalt_temperature_c" in road:
                entries.extend((f"Air temperature  {road['air_temperature_c']:.1f} °C",
                                f"Asphalt temperature  {road['asphalt_temperature_c']:.1f} °C",
                                f"Estimated tyre temperature  {road['tire_temperature_c']:.1f} °C"))
                if not self.temperature.get():
                    self.temperature.set(str(road["air_temperature_c"]))
                if road["rain_remaining_s"] is not None:
                    entries.append(f"Timed rain remaining  {road['rain_remaining_s']} s")
                elif wet["wetness"]:
                    entries.append(f"Drying rate  {road['drying_percent_per_minute']:.1f}% per minute")
            for key, label in (("wetness", "Road wetness"), ("precipitation", "Rainfall"),
                               ("precipitation_deposits", "Standing water"), ("fog_density", "Fog density"),
                               ("dust_storm", "Dust"), ("cloudiness", "Cloud cover"), ("wind_intensity", "Wind")):
                entries.append(f"{label}  {wet[key]:.0f}%")
            grip_text = "Manual or map override" if grip["manual_or_map_override"] else f"{grip['weather_multiplier']:.0%} of dry grip"
            entries.extend(("", f"Weather grip setting  {grip_text}", "Tyres  FL / FR / RL / RR"))
            for key, label in (("configured", "Configured grip"), ("effective_while_rolling", "Effective grip")):
                values = grip[key]
                entries.append(f"{label}  " + (" / ".join(f"{val:.3f}" for val in values)
                                                if values else "Unavailable"))
            self.values.set("\n".join(entries))
        self.root.after(150, self.drain)

    def set_temperature(self):
        self.requests.put(("temperature", {"value": self.temperature.get()}))

    def close(self):
        self.requests.put(("exit", {}))
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--status", action="store_true", help="Show live weather and grip")
    args = parser.parse_args()
    root = tk.Tk()
    (WeatherStatusPanel if args.status else WeatherPanel)(root)
    root.mainloop()


if __name__ == "__main__":
    main()
