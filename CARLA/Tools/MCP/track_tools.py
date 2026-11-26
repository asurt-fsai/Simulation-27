"""Headless course generation and rulebook resources for ASURT MCP."""

from dataclasses import asdict
from hashlib import sha256
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
from uuid import uuid4

import yaml


DEFAULT_RULES = "asurt://rules/trackdraw"
DOCUMENT_TYPES = {".pdf", ".md", ".txt", ".yaml", ".yml", ".json"}


def json_write(path, data):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def read_document(directory, name, page_start=1, page_end=None):
    if name != Path(name).name or Path(name).suffix.lower() not in DOCUMENT_TYPES:
        raise ValueError("Choose a file listed by list_rulebooks.")
    path = directory / name
    if path.is_symlink() or not path.is_file():
        raise ValueError("Rulebook is missing from the workspace Rulebooks folder.")
    if path.suffix.lower() == ".pdf":
        command = shutil.which("pdftotext")
        if command is None:
            raise RuntimeError("PDF text extraction requires poppler-utils in Ubuntu or WSL.")
        page_end = page_end or page_start + 4
        if not 1 <= page_start <= page_end or page_end - page_start > 9:
            raise ValueError("Select up to ten consecutive PDF pages.")
        result = subprocess.run([command, "-f", str(page_start), "-l", str(page_end), "-layout", path, "-"],
                                check=True, capture_output=True, text=True, timeout=15)
        body = result.stdout
    else:
        with path.open(encoding="utf-8") as stream:
            body = stream.read(50001)
    return {"file": name, "pages": [page_start, page_end] if path.suffix.lower() == ".pdf" else None,
            "text": body[:50000], "truncated": len(body) > 50000}


class TrackTools:
    def __init__(self, trackdraw, rulebooks, outputs):
        self.trackdraw, self.rulebooks, self.outputs = trackdraw, rulebooks, outputs
        self.defaults = trackdraw / "config/validation_rules.yaml"

    def list_rulebooks(self):
        documents = (sorted(path.name for path in self.rulebooks.iterdir()
                            if path.is_file() and not path.is_symlink() and path.suffix.lower() in DOCUMENT_TYPES)
                     if self.rulebooks.is_dir() else [])
        return {"trackdraw_defaults": DEFAULT_RULES, "folder": str(self.rulebooks), "files": documents}

    def read_rulebook(self, name, page_start=1, page_end=None):
        return read_document(self.rulebooks, name, page_start, page_end)

    def generate_track(self, seed=7, area_m=100, width_m=3.15, spacing_m=3.5, expectation="pass",
                       width_profile=None, spacing_profile=None, control_points_m=None,
                       rulebook=None, thresholds=None):
        source_path = str(self.trackdraw / "src")
        if source_path not in sys.path:
            sys.path.insert(0, source_path)
        from parameter_function import ParameterFunction
        from trackdraw.configuration import load_rule_settings
        from trackdraw.exporter import export_track_csv, export_track_json, transform_points_to_export_frame
        from trackdraw.generator import generate_control_points
        from trackdraw.geometry import build_track_geometry
        from trackdraw.models import BackgroundSpec, GeneratorSettings, ProjectState
        from trackdraw.project_io import save_project
        from trackdraw.validation import build_default_rules, validate_track

        rules = build_default_rules(load_rule_settings(str(self.defaults)))
        source = {"name": "Trackdraw built-in checks", "sha256": sha256(self.defaults.read_bytes()).hexdigest()}
        if rulebook:
            if rulebook not in self.list_rulebooks()["files"]:
                raise ValueError("Choose a rulebook listed in the workspace Rulebooks folder.")
            with (self.rulebooks / rulebook).open("rb") as stream:
                source = {"name": rulebook, "sha256": sha256_stream(stream)}
        if thresholds and not rulebook:
            raise ValueError("Name the rulebook that supplies your threshold overrides.")
        for rule, details in (thresholds or {}).items():
            if rule not in rules or rules[rule].threshold is None:
                raise ValueError(f"Trackdraw has no numeric threshold for {rule!r}.")
            rules[rule].threshold = details["value_m"]

        scale = 10.0
        size = area_m * scale
        area = [(0., 0.), (size, 0.), (size, size), (0., size)]
        width = ParameterFunction(width_m)
        spacing = ParameterFunction(spacing_m)
        for function, profile in ((width, width_profile), (spacing, spacing_profile)):
            if profile:
                if sorted(set(row[0] for row in profile)) != [row[0] for row in profile]:
                    raise ValueError("Profile positions must be unique and increasing.")
                function.set_control_points(profile)
        settings = GeneratorSettings(seed=seed, min_clearance_m=rules["min_clearance"].threshold,
                                     attempts=24, timeout_s=8)
        if control_points_m:
            points = [(x * scale, y * scale) for x, y in control_points_m]
        else:
            proposal = generate_control_points(area, scale, width, spacing, 180, rules, settings)
            points = proposal.control_points
            if not points:
                raise ValueError(proposal.message)

        geometry = build_track_geometry(points, scale, 180, width, spacing)
        if not geometry.has_track or not len(geometry.left_cones) or not len(geometry.right_cones):
            raise ValueError("The points did not produce a track with two cone boundaries.")
        checked = validate_track(area, geometry, scale, rules)
        findings = [asdict(item) for item in checked.issues]
        if ((expectation == "pass" and not checked.is_valid) or
                (expectation == "fail" and checked.is_valid)):
            return {"matches_expectation": False, "trackdraw_pass": checked.is_valid, "findings": findings}

        ident = uuid4().hex
        output = self.outputs / ident
        output.mkdir(parents=True)
        args = (geometry.centerline, geometry.left_cones, geometry.right_cones, scale)
        export_track_csv(str(output / "cones.csv"), *args)
        export_track_json(str(output / "cones.json"), *args)
        save_project(str(output / "trackdraw.yaml"), ProjectState(
            background=BackgroundSpec(kind="grid", px_per_m=scale, grid_width_px=int(size), grid_height_px=int(size)),
            allowed_area_points=area, control_points=points, generator_settings=settings, rules=rules,
            width_control_points=width.get_control_points(), cone_spacing_control_points=spacing.get_control_points(),
        ))
        centerline = transform_points_to_export_frame(geometry.centerline, geometry.centerline, scale)
        json_write(output / "centerline.json", centerline)
        metadata = {"track_id": ident, "matches_expectation": True, "trackdraw_pass": checked.is_valid,
                    "findings": findings, "seed": seed, "rule_source": source, "threshold_overrides": thresholds or {},
                    "metrics": {key: finite_metric(value) for key, value in checked.metrics.items()},
                    "artifacts": {name: str(output / filename) for name, filename in (
                        ("csv", "cones.csv"), ("json", "cones.json"), ("trackdraw", "trackdraw.yaml"),
                        ("centerline", "centerline.json"))}}
        json_write(output / "track.json", metadata)
        return metadata


def sha256_stream(stream):
    digest = sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


def finite_metric(value):
    if isinstance(value, (int, float)):
        return value if math.isfinite(value) else None
    return value
