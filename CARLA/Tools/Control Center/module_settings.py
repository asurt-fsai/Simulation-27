"""Edit the local module registry without modifying the packaged catalog."""

from pathlib import Path
import os
import shlex
import subprocess
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import xml.etree.ElementTree as ET

import module_control
import slam_settings


class ModuleSettings:
    fields = {
        "id": "Module ID",
        "label": "Name",
        "group": "Group",
        "topic": "ROS topic",
        "dependencies": "Dependencies (comma-separated IDs)",
        "binary": "Local executable",
        "setup": "ROS workspace setup script (optional)",
        "arguments": "Command arguments (optional)",
    }

    def __init__(self, parent, changed):
        self.changed = changed
        self.build_job = None
        self.shipped = {module.name: module for module in module_control.shipped_modules()}
        self.window = tk.Toplevel(parent)
        self.window.title("Module Settings")
        self.window.transient(parent)
        self.window.grab_set()
        self.window.minsize(770, 460)
        panel = ttk.Frame(self.window, padding=12)
        panel.pack(fill="both", expand=True)
        panel.columnconfigure(1, weight=1)
        panel.rowconfigure(0, weight=1)
        catalog = ttk.Frame(panel)
        catalog.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        self.list = tk.Listbox(catalog, exportselection=False, width=30)
        scroll = ttk.Scrollbar(catalog, command=self.list.yview)
        self.list.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.list.pack(side="left", fill="both", expand=True)
        self.list.bind("<<ListboxSelect>>", self.select)
        form = ttk.Frame(panel)
        form.grid(row=0, column=1, sticky="nsew")
        form.columnconfigure(0, weight=1)
        self.variables, self.entries = {}, {}
        for row, (key, label) in enumerate(self.fields.items()):
            ttk.Label(form, text=label).grid(row=row * 2, column=0, sticky="w")
            variable = self.variables[key] = tk.StringVar(self.window)
            field = self.entries[key] = ttk.Entry(form, textvariable=variable)
            field.grid(row=row * 2 + 1, column=0, sticky="ew", pady=(0, 5))
            if key in {"binary", "setup"}:
                ttk.Button(form, text="Browse…", command=lambda k=key: self.browse(k)).grid(
                    row=row * 2 + 1, column=1, padx=(6, 0), sticky="n")
        footer = ttk.Frame(panel)
        footer.grid(row=1, columnspan=2, sticky="ew", pady=(8, 0))
        for title, action in (
            ("Add module", self.add), ("Save local build", self.save),
            ("Use bundled build", self.restore), ("Remove module", self.remove),
            ("Open workspace", self.open_workspace), ("Close", self.window.destroy),
        ):
            ttk.Button(footer, text=title, command=action).pack(side="left", padx=(0, 5))
        self.build_status = tk.StringVar(self.window)
        row = ttk.Frame(panel)
        row.grid(row=2, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(row, text="Open selected source", command=self.open_source).pack(side="left", padx=(0, 5))
        ttk.Button(row, text="SLAM Settings", command=self.open_slam_settings).pack(side="left", padx=(0, 5))
        ttk.Button(row, text="Autonomous Planner", command=self.open_planner_settings).pack(side="left", padx=(0, 5))
        if os.environ.get("ASURT_PORTABLE"):
            from release_update import open_channel_settings
            ttk.Button(row, text="Update Channel", command=lambda: open_channel_settings(self.window)).pack(
                side="left", padx=(0, 5))
        if (module_control.ROOT / "bin/dev-build").is_file():
            ttk.Button(row, text="Build and use workspace source", command=self.build).pack(side="left")
            ttk.Label(row, textvariable=self.build_status).pack(side="left", padx=10)
        self.refresh()

    def open_planner_settings(self):
        try:
            planners = module_control.autonomous_planners()
        except (OSError, ValueError, KeyError) as error:
            messagebox.showerror("Autonomous Planner", str(error), parent=self.window)
            return
        try:
            selected = module_control.autonomous_planner()
        except (OSError, ValueError) as error:
            messagebox.showwarning("Autonomous Planner", str(error), parent=self.window)
            selected = ""
        choices = {"Automatic (one live planner)": ""}
        choices.update({f"{label} — {topic}": topic for topic, label in planners.items()})
        if selected not in choices.values():
            choices[f"Unavailable — {selected}"] = selected
        window = tk.Toplevel(self.window)
        window.title("Autonomous Planner")
        window.transient(self.window)
        panel = ttk.Frame(window, padding=12)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="Path source for both autonomous controllers").pack(anchor="w")
        choice = ttk.Combobox(panel, values=list(choices), state="readonly", width=68)
        choice.pack(fill="x", pady=(6, 10))
        choice.set(next(text for text, topic in choices.items() if topic == selected))

        def close():
            window.destroy()
            self.window.grab_set()

        def save():
            try:
                module_control.save_autonomous_planner(choices[choice.get()])
            except (OSError, ValueError) as error:
                messagebox.showerror("Autonomous Planner", str(error), parent=window)
                return
            close()

        buttons = ttk.Frame(panel)
        buttons.pack(anchor="e")
        ttk.Button(buttons, text="Cancel", command=close).pack(side="left", padx=(0, 5))
        ttk.Button(buttons, text="Save", command=save).pack(side="left")
        window.protocol("WM_DELETE_WINDOW", close)
        window.grab_set()

    def open_slam_settings(self):
        window = tk.Toplevel(self.window)
        window.title("SLAM Settings")
        window.transient(self.window)
        window.columnconfigure(1, weight=1)
        ttk.Label(window, text="Position noise in metres (standard deviation per axis).\n"
                              "Use 0 for exact positions.").grid(
            row=0, columnspan=2, sticky="w", padx=12, pady=10)
        try:
            current = slam_settings.read_settings()
        except (OSError, ValueError) as error:
            messagebox.showwarning("SLAM Settings", str(error), parent=window)
            current = dict.fromkeys(slam_settings.MODES, 0.0)
        values = {}
        for row, (mode, label) in enumerate(zip(slam_settings.MODES, ("FakeMAP", "FakeSLAM")), start=1):
            ttk.Label(window, text=label).grid(row=row, column=0, sticky="w", padx=12, pady=4)
            values[mode] = tk.StringVar(window, value=str(current[mode]))
            ttk.Entry(window, textvariable=values[mode], width=12).grid(
                row=row, column=1, sticky="ew", padx=(0, 12), pady=4)

        def close():
            window.destroy()
            self.window.grab_set()

        def save():
            try:
                slam_settings.save_settings({key: float(value.get()) for key, value in values.items()})
            except (OSError, ValueError) as error:
                messagebox.showerror("SLAM Settings", str(error), parent=window)
                return
            close()

        buttons = ttk.Frame(window)
        buttons.grid(row=3, columnspan=2, sticky="e", padx=12, pady=12)
        ttk.Button(buttons, text="Cancel", command=close).pack(side="left", padx=(0, 6))
        ttk.Button(buttons, text="Save", command=save).pack(side="left")
        window.protocol("WM_DELETE_WINDOW", close)
        window.grab_set()

    def refresh(self, selection=None):
        self.sources = module_control.module_sources()
        self.names = sorted(set(self.shipped) | self.sources.keys())
        self.list.delete(0, "end")
        for name in self.names:
            entry = self.sources.get(name, {})
            status = "removed" if entry.get("hidden") else "local" if entry else "bundled"
            self.list.insert("end", f"{name}  ·  {status}")
        if self.names:
            index = self.names.index(selection) if selection in self.names else 0
            self.list.selection_set(index)
            self.list.see(index)
            self.select()
        else:
            self.add()

    def select(self, _event=None):
        indices = self.list.curselection()
        if not indices:
            return
        name = self.names[indices[0]]
        values = dict(self.sources.get(name, {}), id=name)
        module = self.shipped.get(name)
        if module:
            values.update(label=module.label, group=module.group, topic=module.topic,
                          dependencies=", ".join(module.dependencies))
        values["arguments"] = shlex.join(values.get("arguments", []))
        self.fill(values, bundled=module is not None, existing=True)

    def fill(self, values, *, bundled, existing):
        self.selected = values.get("id") if existing else None
        for key, variable in self.variables.items():
            variable.set(values.get(key, ""))
            locked = key == "id" and existing or bundled and key not in ("setup", "binary")
            self.entries[key].configure(state="readonly" if locked else "normal")

    def add(self):
        self.list.selection_clear(0, "end")
        self.fill({"group": "Custom"}, bundled=False, existing=False)
        self.entries["id"].focus_set()

    def browse(self, key):
        workspace = os.environ.get("ASURT_WORKSPACE", str(Path.home()))
        path = filedialog.askopenfilename(parent=self.window, initialdir=workspace)
        if path:
            self.variables[key].set(path)
            if key == "binary" and not self.variables["setup"].get():
                for directory in Path(path).parents:
                    setup = directory / "local_setup.bash"
                    if setup.is_file():
                        self.variables["setup"].set(str(setup))
                        break

    def save(self):
        values = {key: field.get().strip() for key, field in self.variables.items()}
        name = values.pop("id")
        if self.selected is None and name in self.names:
            messagebox.showerror("Module Settings", "That module ID already exists.", parent=self.window)
            return
        try:
            values["arguments"] = shlex.split(values["arguments"])
            values["revision"] = time.time_ns()
            self.commit(dict(self.sources, **{name: values}), name)
        except ValueError as error:
            messagebox.showerror("Module Settings", str(error), parent=self.window)

    def remove(self):
        name = self.selected
        if not name:
            return
        if not messagebox.askyesno("Remove module", f"Remove {name} from System Manager?", parent=self.window):
            return
        updated = self.sources.copy()
        updated.pop(name, None)
        if name in self.shipped:
            updated[name] = {"hidden": True}
        self.commit(updated, name)

    def restore(self):
        if self.selected in self.shipped:
            self.commit({key: value for key, value in self.sources.items() if key != self.selected},
                        self.selected)

    def commit(self, sources, selection):
        try:
            module_control.save_module_sources(sources, expected=self.sources)
        except (OSError, TypeError, ValueError, KeyError) as error:
            messagebox.showerror("Module Settings", str(error), parent=self.window)
            return
        self.refresh(selection)
        self.changed(schedule=False)

    def open_workspace(self):
        directory = os.environ.get("ASURT_WORKSPACE", str(module_control.ROOT))
        subprocess.Popen(["xdg-open", directory])

    def open_source(self):
        module = self.shipped.get(self.selected)
        if not module:
            return
        package = Path(module.binary).parts[-2]
        workspace = Path(os.environ.get("ASURT_WORKSPACE", module_control.ROOT))
        systems = workspace / "Simulation/Systems"
        roots = ("Perception/Adapters", "Perception/Modules",
                 "Perception/Workspace/autonomy_integration/src/perception_zed_pkg",
                 "SLAM/Modules", "PathPlanning/Modules", "Control/Modules")
        for folder in roots:
            for manifest in (systems / folder).rglob("package.xml"):
                try:
                    name = ET.parse(manifest).findtext("name")
                except (OSError, ET.ParseError):
                    continue
                if name == package:
                    subprocess.Popen(["xdg-open", manifest.parent])
                    return
        self.build_status.set(f"Source package {package} was not found in the workspace.")

    def build(self):
        module = self.shipped.get(self.selected)
        if not module or self.build_job:
            return
        package = Path(module.binary).parts[-2]
        root = module_control.ROOT
        workspace = Path(os.environ.get("ASURT_WORKSPACE", root))
        log = workspace / "build" / package / "build-output.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        if os.environ.get("ASURT_PORTABLE"):
            command = ["bash", str(root / "bin/asurt"), "dev-build"]
        else:
            env_script = "/mnt/e/ASURT/Simulation/CARLA/Launchers/ros-parity-env.sh"
            command = ["bash", env_script, "bash", str(root / "bin/dev-build")]
        try:
            with log.open("w", encoding="utf-8") as output:
                self.build_job = subprocess.Popen(
                    [*command, package, module.name],
                    env=dict(os.environ, ASURT_ROOT=str(root), ASURT_WORKSPACE=str(workspace)),
                    stdout=output, stderr=subprocess.STDOUT,
                )
        except OSError as error:
            messagebox.showerror("Module Build", str(error), parent=self.window)
            return
        self.build_status.set(f"Building {package}…")
        self.wait_for_build(module.name, log)

    def wait_for_build(self, name, log):
        result = self.build_job.poll()
        if result is None:
            self.window.after(300, self.wait_for_build, name, log)
            return
        self.build_job = None
        if result:
            self.build_status.set(f"Build failed. See {log}")
            output = log.read_text(encoding="utf-8", errors="replace")
            messagebox.showerror("Module Build", "\n".join(output.splitlines()[-12:]), parent=self.window)
            return
        self.build_status.set(f"Using local build for {name}.")
        self.refresh(name)
        self.changed(schedule=False)
