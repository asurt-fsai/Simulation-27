#!/usr/bin/env python3
"""Control switches for the existing PC supervisor without owning ROS nodes."""

import argparse
import json
import sys

import module_control


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--state", action="store_true")
    for name in ("enable", "disable", "enable-group", "disable-group"):
        action.add_argument(f"--{name}", metavar="NAME")
    args = parser.parse_args()
    entries = module_control.definitions()
    if args.state:
        sys.path.insert(0, "/mnt/e/ASURT/Simulation/CARLA/Launchers")
        from session import running_session
        print(json.dumps(module_control.live_states(running_session()), indent=2))
        return
    group = args.enable_group or args.disable_group
    name = args.enable or args.disable
    targets = [m.name for m in entries if (m.group == group if group else m.name == name)]
    if args.enable_group:
        if any(target in module_control.DRIVE_CONTROLLERS for target in targets):
            parser.error("Select one Control module to enable.")
        targets = [target for target in targets if target not in module_control.FAKE_PROVIDERS]
    if not targets:
        parser.error(f"Unknown module or group: {group or name}")
    enabled = bool(args.enable_group or args.enable)
    module_control.save_switches(dict.fromkeys(targets, enabled))
    print(f"{'Enabled' if enabled else 'Disabled'} {' / '.join(targets)}")


if __name__ == "__main__":
    main()
