#!/usr/bin/env python3
"""Read-only cone-file inventory across ASURT maps and Trackdraw samples.

The source train/ and noisy_inputs/ files are regression inputs, not surveyed
competition courses. This audit intentionally NEVER adds guessed cones or
mutates them. Its generated JSON is a reproducible coverage report.
"""

from collections import Counter
import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in (HERE, *HERE.parents)
            if (p / "tools/Track Importer/import_cone_track.py").is_file())
MAPS = ROOT / "Assets/Maps"
TRACKDRAW = ROOT / "tools/Trackdraw/fs-trackdraw/trackfiles_csv"
COLORS = ("blue", "yellow", "orange", "orange_big", "unknown")


def track_record(path):
    result = {
        "path": str(path.relative_to(ROOT)),
        "category": "",
        "format": "",
        "counts": {},
        "cone_count": 0,
        "status": "",
    }
    parts = path.relative_to(MAPS).parts if path.is_relative_to(MAPS) else ()
    if parts[:1] == ("Competition2027",):
        result["category"] = "competition_2027"
    elif parts[:2] == ("JSON", "noisy_inputs"):
        result["category"] = "training_intentionally_noisy"
    elif parts[:3] == ("JSON", "JSON", "train"):
        result["category"] = "training_reference"
    elif parts[:1] == ("CSV",):
        result["category"] = "legacy_test_track"
    else:
        result["category"] = "trackdraw_example_or_metadata"
    if path.suffix.lower() == ".csv":
        with path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            fields = {s.strip().lower() for s in (reader.fieldnames or [])}
            if not {"tag", "x", "y"}.issubset(fields):
                result["status"] = "not_tagged_cone_csv"
                return result
            rows = list(reader)
        labels = Counter(str(row.get("tag", "")).strip().lower() for row in rows)
        result["format"] = "tag_xy_csv"
    else:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (ValueError, UnicodeDecodeError, OSError):
            result["status"] = "invalid_json"
            return result
        if isinstance(data, dict) and all(k in data for k in ("x", "y", "color")):
            lengths = [len(data[k]) for k in ("x", "y", "color")
                       if isinstance(data[k], list)]
            if len(lengths) != 3 or len(set(lengths)) != 1:
                result["status"] = "bad_parallel_arrays"
                return result
            labels = Counter(str(c).strip().lower() for c in data["color"])
            result["format"] = "parallel_arrays_json"
        elif isinstance(data, dict) and isinstance(data.get("cones"), list):
            labels = Counter(str(row.get("tag", row.get("color", ""))).strip().lower()
                             for row in data["cones"] if isinstance(row, dict))
            result["format"] = "cones_array_json"
        else:
            result["status"] = "metadata_or_unrecognized_track"
            return result
    result["counts"] = dict(sorted(labels.items()))
    result["cone_count"] = sum(labels.values())
    result["status"] = (
        "updated_handbook_cone_layout" if
        result["category"] == "competition_2027"
        else "preserve_training_source" if result["category"].startswith("training")
        else "no_event_entry_exit_or_timekeeping_geometry"
    )
    return result


def main():
    candidates = [
        p for folder in (MAPS, TRACKDRAW) if folder.exists()
        for p in folder.rglob("*")
        if p.is_file() and p.suffix.lower() in (".csv", ".json")
        and p.name not in ("cone_map_inventory.json", "manifest.json")
    ]
    records = [track_record(p) for p in sorted(candidates)]
    cones = [row for row in records if row["cone_count"]]
    labels = Counter()
    classes = Counter()
    statuses = Counter()
    for row in records:
        labels.update(row["counts"])
        classes[row["category"]] += 1
        statuses[row["status"]] += 1
    report = {
        "source_handbook": "FS-AI Dynamic Events Setup and Cones Specification, DE6.3.1",
        "note": "Preserves raw/noisy regression input and non-event reference "
                "tracks. Only generated event courses have reliable entry, "
                "exit, start and timekeeping coordinates for handbook markers.",
        "summary": {
            "files_examined": len(records),
            "recognized_cone_files": len(cones),
            "color_totals": dict(sorted(labels.items())),
            "categories": dict(sorted(classes.items())),
            "statuses": dict(sorted(statuses.items())),
        },
        "files": records,
    }
    dest = HERE / "cone_map_inventory.json"
    dest.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("Audit", dest, json.dumps(report["summary"], sort_keys=True))


if __name__ == "__main__":
    main()
