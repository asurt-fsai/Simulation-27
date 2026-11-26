"""Add a cooked map release asset to the installer-visible Git map catalog."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import tempfile
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pack", type=Path)
    parser.add_argument("--tag", required=True, help="GitHub release tag that holds the pack")
    parser.add_argument("--catalog", type=Path, default=Path("asurt-maps.json"))
    parser.add_argument("--replace", action="store_true", help="Update a published map under the same ID")
    args = parser.parse_args()
    valid = re.compile(r"[a-zA-Z0-9][a-zA-Z0-9._-]*")
    if not valid.fullmatch(args.tag) or not valid.fullmatch(args.pack.name):
        parser.error("Invalid release tag or asset name.")
    with zipfile.ZipFile(args.pack) as archive:
        pack = json.loads(archive.read("manifest.json"))
    if pack["format"] != 1 or pack["id"] in {"flat-asphalt", "silverstone"}:
        parser.error("Use an independently cooked map pack.")
    catalog = json.loads(args.catalog.read_text()) if args.catalog.exists() else {"format": 1, "maps": []}
    sha = hashlib.sha256()
    with args.pack.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            sha.update(chunk)
    entry = {"id": pack["id"], "label": pack["label"], "file": args.pack.name,
             "sha256": sha.hexdigest(), "bytes": args.pack.stat().st_size, "tag": args.tag,
             "base_sha256": {key: value["base_sha256"] for key, value in pack["platforms"].items()}}
    previous = next((value for value in catalog["maps"] if value["id"] == entry["id"]), None)
    if previous and all(previous.get(key) == value for key, value in entry.items() if key != "bytes"):
        if previous.get("bytes") == entry["bytes"]:
            return
        previous["bytes"] = entry["bytes"]
    elif previous:
        if not args.replace:
            parser.error("Use --replace for a compatible revision, or publish a new map ID.")
        if previous["base_sha256"] != entry["base_sha256"]:
            parser.error("A replacement must support the same base builds. Publish a new map ID.")
        entry["replaces_sha256"] = previous.get("replaces_sha256", []) + [previous["sha256"]]
        catalog["maps"][catalog["maps"].index(previous)] = entry
    else:
        catalog["maps"].append(entry)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="\n",
                                     dir=args.catalog.parent, delete=False) as file:
        json.dump(catalog, file, indent=2)
        file.write("\n")
    Path(file.name).replace(args.catalog)
    print(f'Added {entry["label"]} to {args.catalog}.')


if __name__ == "__main__":
    main()
