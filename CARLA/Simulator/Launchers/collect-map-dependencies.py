"""Record an Unreal map's project dependencies for an additive cook."""

import json
import os

import unreal


root = os.environ["ASURT_COOK_MAP"]
registry = unreal.AssetRegistryHelpers.get_asset_registry()
registry.search_all_assets(True)
options = unreal.AssetRegistryDependencyOptions()
options.set_editor_property("include_hard_package_references", True)
options.set_editor_property("include_soft_package_references", True)
options.set_editor_property("include_searchable_names", False)
options.set_editor_property("include_hard_management_references", False)
options.set_editor_property("include_soft_management_references", False)

pending = [root, *json.loads(os.environ.get("ASURT_MAP_EXTRA_ROOTS", "[]"))]
seen = set()
while pending:
    package = pending.pop()
    if package in seen or not package.startswith("/Game/"):
        continue
    seen.add(package)
    pending.extend(str(child) for child in registry.get_dependencies(package, options) or [])

with open(os.environ["ASURT_MAP_DEPENDENCIES"], "w") as output:
    json.dump(sorted(seen), output, indent=2)
    output.write("\n")
