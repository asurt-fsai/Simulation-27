"""Reconcile module switches within the single PC simulator supervisor."""

import time


class ManagedModules:
    def __init__(self, entries, processes, start, stop):
        self.entries = tuple(entries)
        self.processes = processes
        self.start = start
        self.stop = stop
        self.errors = {}
        self.retry_at = {}

    def reconfigure(self, entries):
        entries = tuple(entries)
        if entries == self.entries:
            return
        current = {module.name: module for module in entries}
        changed = set()
        for module in self.entries:
            if current.get(module.name) != module or changed.intersection(module.dependencies):
                changed.add(module.name)
        for module in reversed(self.entries):
            name = module.name
            if name in changed:
                process = self.processes.pop(name, None)
                if process is not None:
                    self.stop(process)
                self.retry_at.pop(name, None)
                self.errors.pop(name, None)
        self.entries = entries

    def reconcile(self, enabled):
        now = time.monotonic()
        healthy = set()
        for module in self.entries:
            name = module.name
            proc = self.processes.get(name)
            if not enabled[name]:
                self.errors.pop(name, None)
                self.retry_at.pop(name, None)
            elif proc is not None:
                code = proc.poll()
                if code is not None:
                    self.failed(name, f"Exited {code}; see {name}.log", now)
                elif all(dep in healthy for dep in module.dependencies):
                    healthy.add(name)

        # Shut down processors before their input sources.
        for module in reversed(self.entries):
            name = module.name
            if name in self.processes and name not in healthy:
                self.stop(self.processes.pop(name))

        for module in self.entries:
            name = module.name
            if not enabled[name] or name in healthy or now < self.retry_at.get(name, 0):
                continue
            if not all(dep in healthy for dep in module.dependencies):
                continue
            try:
                self.start(name, module.command())
            except OSError as exc:
                self.failed(name, f"Launch failed: {exc}", now)
                continue
            healthy.add(name)
            self.errors.pop(name, None)

    def failed(self, name, error, now):
        self.errors[name] = error
        self.retry_at[name] = now + 30

    def snapshot(self):
        return {
            module.name: {
                "group": self.processes[module.name].pid if module.name in self.processes else None,
                "error": self.errors.get(module.name),
            }
            for module in self.entries
        }
