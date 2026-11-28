"""Manage bundled and local modules and simulator preferences."""

from pathlib import Path
from queue import Empty, Queue
from threading import Thread
import os
import shlex
import shutil
import subprocess
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk
import xml.etree.ElementTree as ET

import github_modules
import module_control
import slam_settings


class ModuleSettings:
    fields = {
        "id": "Module ID",
        "label": "Name",
        "group": "Group",
        "topic": "ROS topic",
        "drive_topic": "Controller command topic (AckermannDriveStamped, optional)",
        "dependencies": "Dependencies (comma-separated IDs)",
        "binary": "Local executable",
        "setup": "ROS workspace setup script (optional)",
        "arguments": "Command arguments (optional)",
    }

    def __init__(self, parent, changed):
        self.changed = changed
        self.busy = False
        self.workspace = Path(os.environ.get("ASURT_WORKSPACE", module_control.ROOT))
        self.catalog = {module.name: module for module in module_control.shipped_modules(include_retired=True)}
        self.window = tk.Toplevel(parent)
        self.window.title("Module Settings")
        self.window.transient(parent)
        self.window.grab_set()
        self.window.minsize(800, 545)
        self.window.protocol("WM_DELETE_WINDOW", self.close)

        notebook = ttk.Notebook(self.window)
        notebook.pack(fill="both", expand=True, padx=10, pady=10)
        panel = ttk.Frame(notebook, padding=10)
        notebook.add(panel, text="Modules")
        sim = ttk.Frame(notebook, padding=10)
        notebook.add(sim, text="Sim Settings")
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
        self.planner_choices = {}
        for row, (key, label) in enumerate(self.fields.items()):
            ttk.Label(form, text=label).grid(row=row * 2, column=0, sticky="w")
            variable = self.variables[key] = tk.StringVar(self.window)
            field = self.entries[key] = ttk.Entry(form, textvariable=variable)
            field.grid(row=row * 2 + 1, column=0, sticky="ew", pady=(0, 4))
            if key in ("binary", "setup"):
                ttk.Button(form, text="Browse…", command=lambda k=key: self.browse(k)).grid(
                    row=row * 2 + 1, column=1, padx=(6, 0), sticky="n")

        footer = ttk.Frame(panel)
        footer.grid(row=1, columnspan=2, sticky="ew", pady=(8, 0))
        self.actions = {}
        for title, action in (
            ("Add module", self.add), ("Update module", self.update), ("Save", self.save),
            ("Use bundled build", self.restore), ("Remove module", self.remove), ("Close", self.close),
        ):
            button = self.actions[title] = ttk.Button(footer, text=title, command=action)
            button.pack(side="left", padx=(0, 5))
        row = ttk.Frame(panel)
        row.grid(row=2, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(row, text="Open selected source", command=self.open_source).pack(side="left", padx=(0, 5))
        self.actions["Build"] = ttk.Button(row, text="Build", command=self.build)
        self.actions["Build"].pack(side="left")
        self.status = tk.StringVar(self.window)
        ttk.Label(panel, textvariable=self.status, wraplength=740).grid(row=3, columnspan=2, sticky="w", pady=3)
        if os.environ.get("ASURT_PORTABLE"):
            from release_update import open_channel_settings
            updates = ttk.Frame(panel)
            updates.grid(row=4, columnspan=2, sticky="ew", pady=(6, 0))
            ttk.Button(updates, text="Update Channel", command=lambda: open_channel_settings(self.window)).pack(
                side="left", padx=(0, 5))
            ttk.Button(updates, text="Check for updates", command=lambda: open_channel_settings(
                self.window, check_now=True)).pack(side="left")
        self.sim_settings(sim)
        self.refresh()

    def close(self):
        if self.busy:
            self.status.set("Wait for the current build to finish.")
        else:
            self.window.destroy()

    def sim_settings(self, panel):
        noise = ttk.LabelFrame(panel, text="SLAM position noise", padding=10)
        noise.pack(fill="x")
        ttk.Label(noise, text="Horizontal standard deviation per axis in metres. Use 0 for exact positions.").pack(anchor="w")
        try:
            current = slam_settings.read_settings()
        except (OSError, ValueError):
            current = dict.fromkeys(slam_settings.MODES, 0.0)
        values = {}
        for key, label in zip(slam_settings.MODES, ("FakeMAP", "FakeSLAM")):
            row = ttk.Frame(noise)
            row.pack(fill="x", pady=(5, 0))
            ttk.Label(row, text=label, width=12).pack(side="left")
            values[key] = tk.StringVar(self.window, value=str(current[key]))
            ttk.Entry(row, textvariable=values[key], width=12).pack(side="left")

        planner = ttk.LabelFrame(panel, text="Autonomous planner", padding=10)
        planner.pack(fill="x", pady=(10, 0))
        ttk.Label(planner, text="Path source for both bundled autonomous controllers").pack(anchor="w")
        self.planner_choice = ttk.Combobox(planner, state="readonly", width=64)
        self.planner_choice.pack(fill="x", pady=(5, 0))
        status = tk.StringVar(self.window)

        def save():
            try:
                configuration = {key: slam_settings.validate_noise(float(value.get())) for key, value in values.items()}
                topic = self.planner_choices[self.planner_choice.get()]
                if topic != module_control.autonomous_planner():
                    module_control.save_autonomous_planner(topic)
                slam_settings.save_settings(configuration)
            except (OSError, KeyError, ValueError) as error:
                messagebox.showerror("Sim Settings", str(error), parent=self.window)
            else:
                status.set("Sim settings saved.")

        ttk.Button(panel, text="Save sim settings", command=save).pack(anchor="w", pady=(10, 0))
        ttk.Label(panel, textvariable=status).pack(anchor="w")

    def refresh_planner(self):
        try:
            selected = module_control.autonomous_planner()
            planners = module_control.autonomous_planners(include_explicit=True)
        except (OSError, ValueError, KeyError, TypeError):
            return
        choices = {"Automatic (one live planner)": ""}
        choices.update({f"{label} — {topic}": topic for topic, label in planners.items()})
        if selected not in choices.values():
            choices[f"Unavailable — {selected}"] = selected
        self.planner_choices = choices
        self.planner_choice.configure(values=list(choices))
        self.planner_choice.set(next(label for label, topic in choices.items() if topic == selected))

    def refresh(self, selection=None):
        self.sources = module_control.module_sources()
        aliased = {settings["bundled_id"] for settings in self.sources.values() if settings.get("bundled_id")}
        self.names = sorted((set(self.catalog) | self.sources.keys()) - aliased)
        self.names = [name for name in self.names if name in self.sources or not self.catalog[name].retired]
        self.list.delete(0, "end")
        for name in self.names:
            entry = self.sources.get(name, {})
            status = "removed" if entry.get("hidden") else "local" if entry.get("binary") else "bundled"
            self.list.insert("end", f"{name}  ·  {status}")
        if self.names:
            index = self.names.index(selection) if selection in self.names else 0
            self.list.selection_set(index)
            self.list.see(index)
            self.select()
        else:
            self.fill({}, None, editable=False)
        self.refresh_planner()

    def stock(self, name):
        return self.catalog.get(self.sources.get(name, {}).get("bundled_id", name))

    def form_values(self, name):
        entry = dict(self.sources.get(name, {}), id=name)
        stock = self.stock(name)
        if stock and not entry.get("edited"):
            entry.update(label=stock.label, group=stock.group, topic=stock.topic,
                         dependencies=", ".join(stock.dependencies))
        entry["arguments"] = shlex.join(entry.get("arguments", []))
        return entry

    def select(self, _event=None):
        if self.busy or not self.list.curselection():
            return
        name = self.names[self.list.curselection()[0]]
        self.fill(self.form_values(name), name, editable=False)
        self.status.set("")

    def fill(self, values, selected, *, editable):
        self.selected = selected
        self.editable = editable
        self.pending_github = values.get("github")
        for key, variable in self.variables.items():
            variable.set(values.get(key, ""))
            self.entries[key].configure(state="normal" if editable else "readonly")
        self.update_actions()

    def update_actions(self):
        stock = self.stock(self.selected)
        buildable = bool(self.pending_github or (stock and (module_control.ROOT / "bin/dev-build").is_file()))
        enabled = {
            "Add module": True, "Update module": bool(self.selected), "Save": self.editable,
            "Use bundled build": bool(stock), "Remove module": bool(self.selected), "Build": buildable,
            "Close": True,
        }
        for name, button in self.actions.items():
            button.configure(state="normal" if enabled[name] and not self.busy else "disabled")
        self.list.configure(state="disabled" if self.busy else "normal")

    def choose_workflow(self, title):
        dialog = tk.Toplevel(self.window)
        dialog.title(title)
        dialog.transient(self.window)
        panel = ttk.Frame(dialog, padding=12)
        panel.pack(fill="both", expand=True)
        choice = None

        def select(value):
            nonlocal choice
            choice = value
            dialog.destroy()

        for label in ("Manually", "GitHub link", "Cancel"):
            ttk.Button(panel, text=label, command=lambda value=label: select(value)).pack(side="left", padx=4)
        dialog.grab_set()
        self.window.wait_window(dialog)
        self.window.grab_set()
        return choice

    def add(self):
        method = self.choose_workflow("Add module")
        if method == "Manually":
            self.list.selection_clear(0, "end")
            self.fill({"group": "Custom"}, None, editable=True)
            self.entries["id"].focus_set()
        elif method == "GitHub link":
            self.github(None)

    def update(self):
        name = self.selected
        method = self.choose_workflow("Update module")
        if method == "Manually":
            self.fill(self.form_values(name), name, editable=True)
            self.entries["id"].focus_set()
        elif method == "GitHub link":
            self.github(name)

    def browse(self, key):
        if not self.editable or self.busy:
            return
        path = filedialog.askopenfilename(parent=self.window, initialdir=self.workspace)
        if path:
            self.variables[key].set(path)
            if key == "binary" and not self.variables["setup"].get():
                for directory in Path(path).parents:
                    setup = directory / "local_setup.bash"
                    if setup.is_file():
                        self.variables["setup"].set(str(setup))
                        break

    @staticmethod
    def moved_dependencies(sources, old, new):
        updated = {}
        for key, entry in sources.items():
            depends = [value.strip() for value in entry.get("dependencies", "").split(",")]
            if old in depends:
                entry = dict(entry, dependencies=", ".join(new if value == old else value for value in depends))
            updated[key] = entry
        return updated

    def save(self):
        values = {key: field.get().strip() for key, field in self.variables.items()}
        name = values.pop("id")
        previous = self.selected
        stock = self.stock(previous)
        if name != previous and (name in self.names or (name in self.catalog and (not stock or stock.name != name))):
            messagebox.showerror("Module Settings", "That module ID already exists.", parent=self.window)
            return
        try:
            values["arguments"] = shlex.split(values["arguments"])
        except ValueError as error:
            messagebox.showerror("Module Settings", str(error), parent=self.window)
            return
        values.update(revision=time.time_ns(), edited=True)
        if stock and stock.name != name:
            values["bundled_id"] = stock.name
        if self.pending_github and values["binary"]:
            values["github"] = self.pending_github
        sources = self.sources.copy()
        rename = (previous, name) if previous and previous != name else None
        if rename:
            sources.pop(previous, None)
        sources[name] = values
        if rename:
            sources = self.moved_dependencies(sources, *rename)
        self.commit(sources, name, rename=rename)

    def remove(self):
        name = self.selected
        if not messagebox.askyesno("Remove module", f"Remove {name} from System Manager?", parent=self.window):
            return
        sources = self.sources.copy()
        sources.pop(name, None)
        stock = self.stock(name)
        if stock and not stock.retired:
            sources[stock.name] = {"hidden": True}
        self.commit(sources, stock.name if stock else None)

    def restore(self):
        stock = self.stock(self.selected)
        if stock:
            sources = self.sources.copy()
            sources.pop(self.selected, None)
            rename = (self.selected, stock.name) if self.selected != stock.name else None
            if rename:
                sources = self.moved_dependencies(sources, *rename)
            self.commit(sources, stock.name, rename=rename)

    def commit(self, sources, selection, *, rename=None):
        try:
            module_control.save_module_sources(sources, expected=self.sources, rename=rename)
        except (OSError, TypeError, ValueError, KeyError) as error:
            messagebox.showerror("Module Settings", str(error), parent=self.window)
            return
        self.refresh(selection)
        self.changed(schedule=False)
        self.status.set("Module settings saved.")

    def source_folder(self):
        name = self.selected
        settings = self.sources.get(name, {})
        git = self.pending_github or settings.get("github")
        if git:
            repo = Path(git["source"])
            package = Path(git.get("package_source", repo))
            return package if package.is_dir() else repo
        stock = self.stock(name)
        executable = Path(settings.get("binary") or (stock.binary if stock else ""))
        if executable.is_absolute():
            resolved = executable.resolve()
            for folder in resolved.parents:
                if (folder / "package.xml").is_file() or (folder / ".git").exists():
                    return folder
            parts = executable.parts
            if len(parts) > 3 and parts[-3] == "lib":
                package = parts[-2]
                systems = self.workspace / "Simulation/Systems"
                roots = [systems / area for area in ("Perception", "SLAM", "PathPlanning", "Control")]
                for parent in executable.parents:
                    if parent.name == "install":
                        roots.insert(0, parent.parent / "src")
                        break
                for root in roots:
                    for manifest in root.rglob("package.xml"):
                        try:
                            if ET.parse(manifest).findtext("name") == package:
                                return manifest.parent
                        except (OSError, ET.ParseError):
                            continue
        if stock:
            try:
                folder = (self.workspace / stock.script.relative_to(module_control.ROOT)).parent
                if folder.is_dir():
                    return folder
            except ValueError:
                pass
        if executable.is_absolute() and executable.parent.is_dir():
            return executable.parent
        raise ValueError("No source folder found for the selected module.")

    def open_source(self):
        try:
            directory = self.source_folder()
            if (os.environ.get("WSL_DISTRO_NAME") or os.environ.get("WSL_INTEROP")
                    or os.environ.get("ASURT_PROFILE") == "windows"):
                path = subprocess.check_output(["wslpath", "-w", str(directory)], text=True, timeout=5).strip()
                explorer = shutil.which("explorer.exe")
                if not explorer:
                    root = os.environ.get("SystemRoot", r"C:\Windows")
                    explorer = subprocess.check_output(["wslpath", "-u", root + r"\explorer.exe"],
                                                       text=True, timeout=5).strip()
                subprocess.Popen([explorer, path])
            else:
                subprocess.Popen(["xdg-open", str(directory)])
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            messagebox.showerror("Open selected source", str(error), parent=self.window)

    def github(self, name):
        previous = self.sources.get(name, {}).get("github")
        value = simpledialog.askstring("GitHub module", "GitHub repository link",
                                       initialvalue=previous["link"] if previous else "", parent=self.window)
        if value:
            self.build_github(name, value, previous)

    def build_github(self, name, value, previous, *, rebuild=False):
        log_dir = self.workspace / "build" / "github"
        log_dir.mkdir(parents=True, exist_ok=True)
        log = log_dir / f"build-{time.time_ns()}.log"
        self.background(
            "Building GitHub module…", log,
            lambda: github_modules.import_module(module_control.ROOT, self.workspace, value, log, previous,
                                                 rebuild=rebuild),
            lambda result: self.review_github(name, result, log, rebuild=rebuild),
        )

    def review_github(self, name, result, log, *, rebuild):
        executables = result["executables"]
        existing = self.form_values(name) if name else {}
        old_binary = Path(existing.get("binary", ""))
        preferred = "/".join((old_binary.parent.name, old_binary.name))
        if name and not existing.get("binary") and self.stock(name):
            old_binary = Path(self.stock(name).binary)
            preferred = "/".join((old_binary.parent.name, old_binary.name))
        key = preferred if preferred in executables else None
        if not key and len(executables) == 1:
            key = next(iter(executables))
        if not key:
            key = self.choose_executable(executables)
        if not key:
            self.status.set(f"The build is available. Executable selection cancelled. See {log}")
            return
        binary, folder = executables[key]
        fields = {field: existing.get(field, "") for field in self.fields} if name else (
            github_modules.inferred_fields(key.split("/", 1)[0], folder))
        if not name and fields["id"] in self.names:
            original = fields["id"]
            index = 2
            while f"{original}_{index}" in self.names:
                index += 1
            fields["id"] = f"{original}_{index}"
        if not name and fields.get("group", "").casefold() == "control":
            fields["drive_topic"] = fields.get("drive_topic") or "/drive"
            if not fields.get("arguments"):
                fields["arguments"] = "--ros-args -p use_sim_time:=true"
        fields.update(binary=str(binary), setup=str(result["setup"]),
                      github=dict(result["github"], package_source=str(folder)))
        if name is None:
            self.list.selection_clear(0, "end")
        self.fill(fields, name, editable=True)
        self.status.set(f"Build complete. Review the module fields and save. Log at {log}")
        if result["changes"]:
            messagebox.showinfo("GitHub changes", result["changes"][-2800:], parent=self.window)
        if rebuild and name:
            self.save()

    def choose_executable(self, executables):
        dialog = tk.Toplevel(self.window)
        dialog.title("Choose ROS executable")
        dialog.transient(self.window)
        panel = ttk.Frame(dialog, padding=12)
        panel.pack(fill="both", expand=True)
        choice = ttk.Combobox(panel, values=list(executables), state="readonly", width=55)
        choice.current(0)
        choice.pack(fill="x")
        selection = None

        def use_selection():
            nonlocal selection
            selection = choice.get()
            dialog.destroy()

        ttk.Button(panel, text="Use executable", command=use_selection).pack(anchor="e", pady=(8, 0))
        dialog.grab_set()
        self.window.wait_window(dialog)
        self.window.grab_set()
        return selection

    def build(self):
        stock = self.stock(self.selected)
        if self.pending_github:
            git = self.pending_github.copy()
            self.build_github(self.selected, git["link"], git, rebuild=True)
            return
        if not stock:
            return
        executable = Path(stock.binary)
        if len(executable.parts) < 4 or executable.parts[-3] != "lib":
            self.status.set("This bundled entry runs directly from its source file.")
            return
        package = executable.parent.name
        root = module_control.ROOT
        log = self.workspace / "build" / package / "build-output.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        if os.environ.get("ASURT_PORTABLE"):
            command = ["bash", str(root / "bin/asurt"), "dev-build"]
        else:
            command = ["bash", str(root / "Simulation/CARLA/Launchers/ros-parity-env.sh"),
                       "bash", str(root / "bin/dev-build")]
        name = self.selected

        def work():
            with log.open("w", encoding="utf-8") as output:
                github_modules.run_command([*command, package, name], output,
                    env=dict(os.environ, ASURT_ROOT=str(root), ASURT_WORKSPACE=str(self.workspace)))

        def complete(_result):
            self.refresh(name)
            self.changed(schedule=False)
            self.status.set(f"Using the local build for {name}.")

        self.background(f"Building {package}…", log, work, complete)

    def background(self, status, log, work, complete):
        if self.busy:
            return
        self.busy = True
        self.update_actions()
        self.status.set(status)
        results = Queue()

        def run():
            try:
                results.put((work(), None))
            except Exception as error:
                results.put((None, error))

        def finish():
            try:
                result, error = results.get_nowait()
            except Empty:
                self.window.after(200, finish)
                return
            self.busy = False
            self.update_actions()
            try:
                if error:
                    raise error
                complete(result)
            except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as failure:
                self.status.set(f"Build failed. See {log}")
                detail = str(failure)
                if log.is_file():
                    tail = log.read_text(encoding="utf-8", errors="replace").splitlines()[-10:]
                    if tail:
                        detail += "\n\n" + "\n".join(tail)
                messagebox.showerror("Module build", detail, parent=self.window)

        Thread(target=run, daemon=True).start()
        self.window.after(200, finish)
