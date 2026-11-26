#!/usr/bin/env python3
"""Derive RViz planning-mode presets with independent display checkboxes.

Only writes RViz visualization files. The combined/SLAM views and ROS
publishers are untouched. The existing planning_classical.rviz is the
overview template; this builder also accepts its own grouped output and is
idempotent, preserving actual display settings inside each group.
"""
from copy import deepcopy
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / "config"
OVERVIEW = CONFIG / "planning_classical.rviz"
MODES = ("classical", "acceleration", "skidpad", "deep_learning")
DISPLAY_PATH = {
    "classical": "/planning/classical/path",
    "acceleration": "/planning/acceleration/path",
    "skidpad": "/planning/skidpad/path",
    "deep_learning": "/planning/deep_learning/path",
}


def flatten(displays):
    for item in displays:
        if item.get("Class") == "rviz_common/Group":
            yield from flatten(item.get("Displays", []))
        else:
            yield deepcopy(item)


def group(name, displays, enabled=True):
    return {
        "Class": "rviz_common/Group", "Name": name,
        "Enabled": enabled, "Displays": displays,
    }


def configure(source, active):
    if active not in ("overview", *MODES):
        raise ValueError("Unknown planning profile " + active)
    data = deepcopy(source)
    manager = data["Visualization Manager"]
    displays = list(flatten(manager["Displays"]))
    tracks, cones, diagnostics = [], [], []
    modes = {mode: [] for mode in MODES}
    for item in displays:
        topic = item.get("Topic", {}).get("Value", "")
        if item["Class"] in ("rviz_default_plugins/Grid", "rviz_default_plugins/RobotModel"):
            tracks.append(item)
        elif topic == "/map/global_cones_markers":
            cones.append(item)
        elif topic.startswith("/planning/"):
            parts = topic.split("/")
            mode = parts[2]
            if mode not in modes:
                raise ValueError("Unexpected planning display " + topic)
            # MarkerArray is the same centreline drawn a second time:
            # enable only by deliberate action when diagnosing publication.
            if topic.endswith("/markers"):
                item["Enabled"] = False
            modes[mode].append(item)
        elif item["Class"] in ("rviz_default_plugins/Odometry", "rviz_default_plugins/TF"):
            diagnostics.append(item)
        elif topic == "/zed/zed_node/cone_markers":
            # Not a planning input and would conceal duplicate map entries.
            continue
        else:
            raise ValueError("Unclassified planning RViz display: " + str(item))

    # Every planner has its own status-independent line marker topic for
    # optional inspection; markers duplicate Path geometry when both are
    # enabled, so keep marker displays unchecked until deliberately selected.
    original_markers = next(
        (item for item in modes["classical"]
         if item.get("Topic", {}).get("Value") == "/planning/classical/markers"),
        None,
    )
    if original_markers is None:
        raise ValueError("Classical path diagnostic marker template missing")
    # The original LSTM output remains a distinct, full 15-point Path.
    # The separate experimental simulator controller may subscribe to it,
    # while the long-lived RViz preview does not imply command freshness.
    if not modes["deep_learning"]:
        original_path = next(
            item for item in modes["classical"]
            if item.get("Topic", {}).get("Value") == "/planning/classical/path")
        model_path = deepcopy(original_path)
        model_path["Name"] = "Deep Learning | SAFETY-GATED path (optional)"
        model_path["Topic"]["Value"] = DISPLAY_PATH["deep_learning"]
        model_path["Color"] = "245; 110; 150"
        model_path["Enabled"] = False
        modes["deep_learning"].append(model_path)
    # Idempotent when input already contains raw and gated displays.
    gated_path = next((item for item in modes["deep_learning"]
                       if item.get("Topic", {}).get("Value") == DISPLAY_PATH["deep_learning"]), None)
    if gated_path is None:
        raise ValueError("Missing safety-gated Deep Learning path")
    gated_path["Enabled"] = False
    gated_path["Name"] = "Deep Learning | SAFETY-GATED path (optional)"
    raw_display = next((item for item in modes["deep_learning"]
                        if item.get("Topic", {}).get("Value") ==
                        "/planning/deep_learning/raw_path"), None)
    if raw_display is None:
        raw = deepcopy(gated_path)
        raw["Topic"]["Value"] = "/planning/deep_learning/raw_path"
        raw["Color"] = "65; 220; 240"
        raw["Line Width"] = 0.15
        raw["Enabled"] = True
        modes["deep_learning"].insert(0, raw)
        raw_display = raw
    raw_display["Name"] = "Deep Learning | RAW LSTM 15-point prediction (simulator Control optional)"
    for mode in ("acceleration", "skidpad"):
        topic = "/planning/" + mode + "/markers"
        if not any(item.get("Topic", {}).get("Value") == topic for item in modes[mode]):
            additional = deepcopy(original_markers)
            additional["Name"] = mode.title() + " | optional path markers"
            additional["Topic"]["Value"] = topic
            additional["Enabled"] = False
            modes[mode].append(additional)

    if len(tracks) != 2 or len(cones) != 1 or not diagnostics:
        raise ValueError("Incomplete vehicle/map/diagnostic displays")
    for mode in MODES:
        if not any(item.get("Topic", {}).get("Value") == DISPLAY_PATH[mode]
                   for item in modes[mode]):
            raise ValueError("Missing proposed path for " + mode)
    if not any(item.get("Topic", {}).get("Value") == "/planning/deep_learning/raw_path"
               for item in modes["deep_learning"]):
        raise ValueError("Missing raw LSTM prediction display")

    # Deep Learning uses /perception_landmarks in base_link, NOT the SLAM map.
    # Give its dedicated mode a local-frame vehicle, grid and real ZED camera
    # cylinders. Leave the global cone map as an explicitly disabled comparison
    # only. Other planning modes and overview retain their original map view.
    if active == "deep_learning":
        perception = yaml.safe_load(
            (CONFIG / "perception_combined.rviz").read_text(encoding="utf-8"))
        camera = next(
            (deepcopy(item) for item in perception["Visualization Manager"]["Displays"]
             if item.get("Topic", {}).get("Value") == "/zed/zed_node/cone_markers"),
            None,
        )
        if camera is None or camera["Class"] != "rviz_default_plugins/MarkerArray":
            raise ValueError("Missing ZED live cylinder marker display")
        camera["Enabled"] = True
        camera["Name"] = "ZED Perception | LIVE blue/yellow/orange cone cylinders"
        local_tracks = deepcopy(tracks)
        for item in local_tracks:
            if item["Class"] == "rviz_default_plugins/Grid":
                item["Reference Frame"] = "base_link"
                item["Name"] = "Live vehicle-relative ground grid"
                item["Offset"]["Z"] = 0
            elif item["Class"] == "rviz_default_plugins/RobotModel":
                item["Name"] = "FSAI | vehicle-local blue outline"
                # Resolve from the same authoritative local preview used by
                # Combined Perception, not a map-ground shifted URDF.
                model = next(
                    (x for x in perception["Visualization Manager"]["Displays"]
                     if x["Class"] == "rviz_default_plugins/RobotModel"), None)
                if model is None:
                    raise ValueError("Missing local Perception vehicle model")
                item["Description File"] = model["Description File"]
        data["Visualization Manager"]["Global Options"]["Fixed Frame"] = "base_link"
        ground_group = group("Ground and vehicle | LIVE base_link", local_tracks)
        cone_groups = [
            group("Perception | LIVE ZED cone cylinders (LSTM input source)",
                  [camera], True),
            group("SLAM | accumulated map cones (COMPARISON ONLY)", cones, False),
        ]
        controller_target = {
            "Class": "rviz_default_plugins/Marker",
            "Enabled": True,
            "Name": "Control | Pure Pursuit lookahead target (small green point)",
            "Topic": {
                "Value": "/control/pure_pursuit/target",
                "History Policy": "Keep Last",
                "Depth": 10,
                "Reliability Policy": "Reliable",
                "Durability Policy": "Volatile",
            },
        }
    else:
        ground_group = group("Ground and vehicle | FlatAsphalt map z=0", tracks)
        cone_groups = [
            group("SLAM | accumulated cones (map planning input)", cones),
        ]

    manager["Displays"] = [
        ground_group,
        *cone_groups,
        *(group(mode.replace("_", " ").title() + " | proposed path",
                modes[mode], active in ("overview", mode)) for mode in MODES),
        *([group("Control | Pure Pursuit lookahead target",
                 [controller_target], True)] if active == "deep_learning" else []),
        group("Diagnostics | odometry and TF (optional)", diagnostics, False),
    ]
    data["Visualization Manager"]["Views"]["Current"]["Distance"] = 19
    data["Visualization Manager"]["Views"]["Current"]["Focal Point"]["Z"] = 0.35
    return data


def configure_goal_pose(source):
    """A separate goal-clicking view; existing four cone/LSTM modes unchanged."""
    view = configure(source, "classical")
    manager = view["Visualization Manager"]
    groups = manager["Displays"]
    for entry in groups:
        if entry["Name"].startswith(("Classical |", "SLAM |")):
            entry["Enabled"] = False
    source_path = next(x for x in flatten(groups)
                       if x.get("Topic", {}).get("Value") == "/planning/classical/path")
    path = deepcopy(source_path)
    path["Topic"]["Value"] = "/planning/goal_pose/path"
    path["Name"] = "RViz Goal Pose | forward Dubins route"
    path["Enabled"] = True
    path["Color"] = "245; 180; 70"
    path["Line Width"] = .16
    target = {
        "Class": "rviz_default_plugins/Marker",
        "Enabled": True,
        "Name": "RViz goal | fixed map destination (orange arrow)",
        "Topic": {"Value": "/planning/goal_pose/goal_marker",
                  "History Policy": "Keep Last", "Depth": 10,
                  "Reliability Policy": "Reliable",
                  "Durability Policy": "Volatile"},
    }
    lookahead = deepcopy(target)
    lookahead["Name"] = "Pure Pursuit | steering lookahead (green dot)"
    lookahead["Topic"]["Value"] = "/control/pure_pursuit/target"
    groups.insert(-1, group("Goal Pose | clicked route and controller", [
        path, target, lookahead], True))
    if not any(tool.get("Class") == "rviz_default_plugins/SetGoal"
               for tool in manager["Tools"]):
        manager["Tools"].append({"Class": "rviz_default_plugins/SetGoal",
                                 "Topic": "/goal_pose"})
    return view


def configure_control(source):
    """Dedicated RViz test bench: clicked goal vs planned and driven paths."""
    view = configure_goal_pose(source)
    manager = view["Visualization Manager"]
    displays = manager["Displays"]
    ground = next(g for g in displays if g["Name"].startswith("Ground and vehicle"))
    ground["Enabled"] = True
    map_grid = next(item for item in ground["Displays"]
                    if item["Class"] == "rviz_default_plugins/Grid")
    map_grid["Name"] = "Map grid | fixed world (1 m)"
    map_grid["Enabled"] = True
    map_grid["Alpha"] = .7
    map_grid["Color"] = "120; 145; 170"
    map_grid["Cell Size"] = 1
    map_grid["Plane Cell Count"] = 120
    local_grid = deepcopy(map_grid)
    local_grid["Name"] = "Car-centred grid | 1 m (requires map-to-base_link TF)"
    local_grid["Reference Frame"] = "base_link"
    local_grid["Plane Cell Count"] = 36
    local_grid["Alpha"] = .65
    local_grid["Color"] = "75; 175; 195"
    local_grid["Offset"]["Z"] = .01
    ground["Displays"].append(local_grid)

    # Keep cone layers optional and separate: they are useful for checking
    # goal clicks against the track, not inputs to this free-space planner.
    slam = next(g for g in displays if g["Name"].startswith("SLAM |"))
    slam["Name"] = "SLAM cones | global map (optional)"
    slam["Enabled"] = False
    perception = yaml.safe_load(
        (CONFIG / "perception_combined.rviz").read_text(encoding="utf-8"))
    cylinders = next((deepcopy(item)
                      for item in perception["Visualization Manager"]["Displays"]
                      if item.get("Topic", {}).get("Value") ==
                      "/zed/zed_node/cone_markers"), None)
    if cylinders is None or cylinders["Class"] != "rviz_default_plugins/MarkerArray":
        raise ValueError("Missing ZED live cone-cylinder display")
    cylinders["Enabled"] = True
    cylinders["Name"] = "Perception | ZED blue/yellow/orange cone cylinders"
    displays.insert(displays.index(slam)+1,
                    group("Perception cones | ZED cylinders (optional)",
                          [cylinders], False))
    control = next(group for group in displays
                   if group["Name"].startswith("Goal Pose | clicked route"))
    planned = control["Displays"][0]
    driven = deepcopy(planned)
    driven["Name"] = "Control | ACTUAL driven vehicle trajectory (blue)"
    driven["Topic"]["Value"] = "/control/pure_pursuit/driven_path"
    driven["Color"] = "65; 160; 255"
    driven["Line Width"] = .16
    status = deepcopy(control["Displays"][1])
    status["Name"] = "Control | speed, steering, ROS/manual and tracking status"
    status["Topic"]["Value"] = "/control/pure_pursuit/diagnostic_marker"
    control["Name"] = "Control | goal, planned path, actual path, diagnostics"
    control["Displays"].extend([driven, status])
    diagnostics = next(group for group in displays
                       if group["Name"].startswith("Diagnostics |"))
    diagnostics["Enabled"] = True
    for item in diagnostics["Displays"]:
        if item["Class"] == "rviz_default_plugins/Odometry":
            item["Enabled"] = True
    return view


def configure_track_control(source):
    """High-speed Stanley + SLAM cone-map goal test, independent of Pure Pursuit."""
    view = configure_control(source)
    manager = view["Visualization Manager"]
    displays = manager["Displays"]
    slam = next(g for g in displays if g["Name"].startswith("SLAM cones |"))
    slam["Enabled"] = True
    # ZED cones remain a separately selectable comparison layer; enabling
    # them is not required for the SLAM gate-graph path display.
    live = next(g for g in displays if g["Name"].startswith("Perception cones |"))
    live["Enabled"] = False
    control = next(g for g in displays if g["Name"].startswith("Control | goal"))
    control["Name"] = "Track Control | SLAM goal, obstacle-aware path and Stanley"
    replacements = {
        "/planning/goal_pose/path": "/planning/track_goal/path",
        "/planning/goal_pose/goal_marker": "/planning/track_goal/goal_marker",
        "/control/pure_pursuit/target": "/control/stanley/target",
        "/control/pure_pursuit/driven_path": "/control/stanley/driven_path",
        "/control/pure_pursuit/diagnostic_marker": "/control/stanley/diagnostic_marker",
    }
    for display in control["Displays"]:
        topic = display.get("Topic", {})
        source_topic = topic.get("Value")
        if source_topic in replacements:
            topic["Value"] = replacements[source_topic]
        display["Name"] = display["Name"].replace("Pure Pursuit", "Stanley")
    control["Displays"][0]["Name"] = "SLAM track corridor | cone-aware planned route"
    control["Displays"][0]["Color"] = "255; 160; 70"
    control["Displays"][3]["Name"] = "Stanley | ACTUAL driven trajectory (blue)"
    return view


def main():
    with OVERVIEW.open(encoding="utf-8") as stream:
        original = yaml.safe_load(stream)
    # Save overview plus four independent view modes. Each group is a
    # checkbox in the RViz Displays tree; choosing a mode does NOT enable a
    # planner or modify any System Manager saved switch.
    for mode, destination in [
        ("overview", OVERVIEW),
        *[(m, CONFIG / ("planning_" + m + "_mode.rviz")) for m in MODES],
        ("goal_pose", CONFIG / "planning_goal_pose_mode.rviz"),
        ("control", CONFIG / "control_goal_pose.rviz"),
        ("track_control", CONFIG / "control_track_mode.rviz"),
    ]:
        output = (configure_goal_pose(original) if mode == "goal_pose"
                  else configure_control(original) if mode == "control"
                  else configure_track_control(original) if mode == "track_control"
                  else configure(original, mode))
        destination.write_text(yaml.safe_dump(output, sort_keys=False),
                               encoding="utf-8")
        print(destination)


if __name__ == "__main__":
    main()
