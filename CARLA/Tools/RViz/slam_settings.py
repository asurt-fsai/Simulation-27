"""Persist simulation-only cone position noise for both map providers."""

import json
import math
import os
from pathlib import Path
import tempfile


MODES = ("fake_map", "fake_slam")


def settings_path():
    return Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "asurt/slam-settings.json"


def validate_noise(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError("Enter a finite, non-negative standard deviation in metres.")
    return float(value)


def read_settings():
    try:
        values = json.loads(settings_path().read_text(encoding="utf-8"))
    except FileNotFoundError:
        values = {}
    if not isinstance(values, dict):
        raise ValueError("SLAM settings must be a JSON object.")
    return {mode: validate_noise(values.get(mode, 0.0)) for mode in MODES}


def save_settings(values):
    document = {mode: validate_noise(values[mode]) for mode in MODES}
    path = settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         delete=False) as target:
            temp = Path(target.name)
            json.dump(document, target, indent=2)
            target.write("\n")
        temp.replace(path)
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)
