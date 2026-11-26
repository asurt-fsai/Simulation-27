#!/usr/bin/env python3
"""Deterministic FSUK 2027 D4/D5 CARLA Cone Track Importer course fixtures.

Sources: supplied Dynamic_Missions_Details.pdf (2027 FSUK, pp. 116-117, 119).
Generated local Cartesian METRES: +x along the entry/driving direction;
CARLA +y right. The importer's first blue/yellow pairs define car heading,
so blue lies at negative y and yellow at positive y on entrance straights.
CARLA->ROS y reflection is handled by the existing simulation bridge.
No CARLA imports, actors, driving commands, or map edits.
"""

import csv
from collections import Counter
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROUND = 6
INNER_RADIUS = 15.25 / 2
OUTER_RADIUS = 21.25 / 2
CIRCLE_SEPARATION = 18.25
SKIDPAD_CENTERLINE_RADIUS = (INNER_RADIUS + OUTER_RADIUS) / 2
CONE_LINE_OFFSET_M = 0.15  # Assumed cone centre offset outside painted lines
INNER_CONE_RADIUS = INNER_RADIUS - CONE_LINE_OFFSET_M
OUTER_CONE_RADIUS = OUTER_RADIUS + CONE_LINE_OFFSET_M
GATE_CONE_OFFSET_M = 0.15
BIG_ORANGE_GAP_BEFORE_AFTER_LINE_M = 0.75
BIG_ORANGE_GATE_LATERAL_M = 3.5


def add_orange_lane_pairs(cones, stations, half_width, role):
    """Handbook DE6.3.1 small orange cones bound entry and exit lanes."""
    for x in stations:
        for y in (-half_width, half_width):
            add(cones, "orange", x, y, role=role)


def add_big_orange_line_pair(cones, line_x, role):
    """Two large orange cones on EACH side, before/after the line."""
    for x in (line_x - BIG_ORANGE_GAP_BEFORE_AFTER_LINE_M,
              line_x + BIG_ORANGE_GAP_BEFORE_AFTER_LINE_M):
        for y in (-BIG_ORANGE_GATE_LATERAL_M,
                   BIG_ORANGE_GATE_LATERAL_M):
            add(cones, "orange_big", x, y, role=role)


def add_stop_barrier(cones, x, half_width):
    """Figure 1/3 orange stop-area row, 1 m maximum lateral cone spacing."""
    add_orange_lane_pairs(cones, (x,), half_width, "stop_lane_end_side")
    for y in (-1.0, 0.0, 1.0):
        add(cones, "orange", x, y, role="stop_lane_terminal_barrier")


def n(x):
    return round(float(x), ROUND) + 0.0


def add(cones, tag, x, y, *, role, ring=None, side=None, ordinal=None):
    cone = dict(tag=tag, x=n(x), y=n(y), z=0.0, role=role)
    if ring:
        cone.update(ring=ring, circle=side, index=ordinal)
    cones.append(cone)


def save_csv(path, records):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("tag", "x", "y", "z"))
        for c in records:
            writer.writerow((c["tag"], f'{c["x"]:.6f}',
                             f'{c["y"]:.6f}', f'{c["z"]:.6f}'))


def save_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8")


def save_path(path, points):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("stage", "x", "y", "z"))
        for stage, x, y in points:
            writer.writerow((stage, f'{n(x):.6f}', f'{n(y):.6f}', "0.000000"))


def acceleration():
    """75m timed straight with 3.5m width (>=3), extra exit/staging lanes."""
    timed = 75.0
    track_width = 3.5
    cone_y = track_width / 2 + GATE_CONE_OFFSET_M
    cones = []
    # Blue/yellow appear ONLY on the timed track. Preserve first two ordered
    # gate pairs at x=0,5 for legacy importer heading and existing planners.
    for x in range(0, 76, 5):
        role = "timed_boundary"
        add(cones, "blue", x, -cone_y, role=role)
        add(cones, "yellow", x, cone_y, role=role)
    add_orange_lane_pairs(cones, (-5.0, -2.5), cone_y, "entry_lane")
    # Stopping lane ends at x=125, matching the 2027 D5 driverless maximum
    # of 50m. The older 2021/22 FS-AI handbook Figure 1 depicts 100m;
    # the discrepancy is documented in the layout metadata.
    add_orange_lane_pairs(cones, tuple(range(80, 125, 5)),
                          cone_y, "stopping_lane")
    add_stop_barrier(cones, 125.0, cone_y)
    add_big_orange_line_pair(cones, 0, "start_timekeeping_before_after")
    add_big_orange_line_pair(cones, 75, "finish_timekeeping_before_after")
    path = [("staging" if i < 0 else "timed" if i <= timed else "stop_lane",
             float(i), 0.0) for i in range(-5, 121)]
    layout = {
        "event": "acceleration", "rules": ["2027 FSUK D5.1.1", "D5.3.2", "D5.3.4"],
        "source_pdf_pages": [4],
        "cone_handbook": {
            "source": "FS-AI Dynamic Events Setup and Cones Specification, DE6.3.1 and Figure 1, pp.1,3",
            "entry_and_exit": "orange (small)",
            "before_and_after_timing": "orange_big",
            "2021_handbook_illustrated_stop_area_m": 100.0,
            "2027_rule_driverless_stop_limit_m": 50.0,
            "note": "The older handbook image shows a 100m stop area, while "
                    "the supplied 2027 D5 driverless text requires the car "
                    "to stop within 50m. This 2027 fixture ends the cone "
                    "lane at x=125 and provides a stop barrier; do not "
                    "confuse an illustration with a 2027 timing requirement.",
        },
        "geometry": {
            "type": "straight", "start_line_x_m": 0.0, "finish_line_x_m": timed,
            "timed_distance_m": timed, "minimum_rule_track_width_m": 3.0,
            "selected_driving_lane_width_m": track_width,
            "boundary_cone_centre_separation_m": 2 * cone_y,
            "nominal_cone_spacing_m": 5.0,
            "stage_approach_m": 5.0,
            "stop_lane_end_x_m": 125.0, "stop_distance_after_finish_m": 50.0,
            "drive_centerline": {"from": [-5, 0], "to": [120, 0]},
            "reference_stop_x_m": 120.0,
        },
        "staging": {
            "frontmost_vehicle_point_target_m": [-0.30, 0.0],
            "heading_degrees_importer_local": 0.0,
            "note": "D5.3.2 uses the FOREMOST vehicle part, not base_link. "
                    "Measure the actual FSAI bounding box and position its "
                    "nose at x=-0.30 before timing. The first ordered "
                    "blue/yellow gate now lies at x=0; the small orange "
                    "approach lane starts at x=-5.",
        },
        "cone_assignment": "Simulation choice: blue left/y<0, yellow right/y>0 in CARLA local XY.",
        "selected_parameters_not_given_in_rules": {
            "lane_width_m": track_width,
            "cone_offset_from_boundary_line_m": GATE_CONE_OFFSET_M,
            "entry_orange_stations_x_m": [-5.0, -2.5],
            "exit_lane_cone_spacing_m": 5.0,
            "stop_area_orange_stations_x_m": list(range(80, 126, 5)),
            "stop_barrier_small_orange": [[125, -1], [125, 0], [125, 1]],
            "big_orange_offset_along_track_m": BIG_ORANGE_GAP_BEFORE_AFTER_LINE_M,
            "orange_big_indicators": {
                "start_line_before_after": [
                    [-0.75, -3.5], [-0.75, 3.5], [0.75, -3.5], [0.75, 3.5]
                ],
                "finish_line_before_after": [
                    [74.25, -3.5], [74.25, 3.5],
                    [75.75, -3.5], [75.75, 3.5]
                ],
                "note": "DE6.3.1 prescribes big orange before/after each "
                        "line; exact longitudinal/lateral offsets are "
                        "simulation choices based on Figure 1.",
            },
        },
    }
    return cones, path, layout


def skidpad():
    """Two concentric pairs, right-first two laps then left-first two laps."""
    cones = []
    lane_line_halfwidth = 1.5
    lane_cone_halfwidth = lane_line_halfwidth + GATE_CONE_OFFSET_M
    # Start station x=0 is 15m BEFORE the centre-to-centre timing line x=15.
    # List the first two blue and yellow stations in approach order so the
    # unmodified importer computes +x heading, zero mirror, 3.3m cone gate.
    for x in (0, 5):
        add(cones, "blue", x, -lane_cone_halfwidth, role="entry_lane")
        add(cones, "yellow", x, lane_cone_halfwidth, role="entry_lane")

    # +y is CARLA right. The LEFT/RIGHT circles become the opposite y signs
    # in REP-103 ROS after the native CARLA bridge Y conversion.
    for side, cy in (("right", +CIRCLE_SEPARATION/2),
                     ("left", -CIRCLE_SEPARATION/2)):
        # For right-first traversal: blue outside right ring, yellow inside;
        # left traversal: blue inside left, yellow outside.
        colors = {
            "right": {"inner": "yellow", "outer": "blue"},
            "left": {"inner": "blue", "outer": "yellow"},
        }[side]
        for ring, count, radius in (
            ("inner", 16, INNER_CONE_RADIUS),
            ("outer", 13, OUTER_CONE_RADIUS),
        ):
            for i in range(count):
                if ring == "inner":
                    a = 2 * math.pi * i / count
                else:
                    # D4.1 Figure 8 has NO outer cones across the central
                    # crossover. Uniform full-circle spacing would place
                    # cones directly on the *other* loop's driving centre.
                    # Keep 13 cones on the remaining outside-facing 260° arc
                    # and the central 100° crossover clear.
                    start = 320.0 if side == "right" else 140.0
                    a = math.radians(start + 260.0*i/(count-1))
                add(cones, colors[ring],
                    15 + radius * math.cos(a), cy + radius * math.sin(a),
                    role="circle_boundary", ring=ring, side=side, ordinal=i)

    # Figure 3: orange entry and exit lane, unlike blue/yellow ring cones.
    # Preserve two blue/yellow approach pairs at x0/5 for the importer
    # start-heading heuristic, but delimit the extra approach in orange.
    # The loop path crosses the inbound x=10 station: placing a cone there
    # would block the competition driving line, so terminate the orange
    # entrance pair before the staged first blue/yellow gate.
    add_orange_lane_pairs(cones, (-5., -2.5),
                          lane_cone_halfwidth, "entry_lane")
    add_orange_lane_pairs(cones, (25., 30., 35.),
                          lane_cone_halfwidth, "exit_stopping_lane")
    add_stop_barrier(cones, 40.0, lane_cone_halfwidth)
    # A timing gate before and after x=15 with a left/right pair on each
    # side; the large cones are never placed on the centreline.
    add_big_orange_line_pair(cones, 15, "timekeeping_before_after")

    path = [("entry", float(i)/2, 0.) for i in range(0, 31)]
    # In CARLA handedness, a physical RIGHT circle at y>0 is traversed
    # counterclockwise with +x tangent heading. The LEFT circle is clockwise.
    for side, cy, clockwise in (("right", +9.125, False),
                                ("left", -9.125, True)):
        start_angle = -math.pi/2 if side == "right" else math.pi/2
        for i in range(1, 2*144+1):
            angle = start_angle + (-1 if clockwise else 1) * (4*math.pi*i/(2*144))
            path.append((side + "_lap_" + str(1 if i <= 144 else 2),
                         15 + SKIDPAD_CENTERLINE_RADIUS*math.cos(angle),
                         cy + SKIDPAD_CENTERLINE_RADIUS*math.sin(angle)))
    # Stop short of the final row of physical small orange stop markers.
    path.extend(("exit", 15+i/2, 0.) for i in range(1, 46))
    layout = {
        "event": "skidpad",
        "rules": ["2027 FSUK D4.1.1-D4.1.6", "D4.3.3", "D4.3.5", "D4.3.6"],
        "source_pdf_pages": [1, 2],
        "cone_handbook": {
            "source": "FS-AI Dynamic Events Setup and Cones Specification, DE6.3.1 and Figure 3, pp.1,4",
            "entry_and_exit": "orange (small)",
            "before_and_after_timekeeping": "orange_big",
            "note": "The handbook fixes cone roles but the figure is "
                    "schematic; exact longitudinal positions use bounded "
                    "simulation offsets in the metadata below.",
        },
        "geometry": {
            "type": "figure_eight_two_concentric_pairs",
            "circle_centres_m": {
                "right": [15.0, +9.125], "left": [15.0, -9.125],
            },
            "centre_separation_m": CIRCLE_SEPARATION,
            "inner_circle_diameter_m": 15.25,
            "outer_circle_diameter_m": 21.25,
            "painted_inner_radius_m": INNER_RADIUS,
            "painted_outer_radius_m": OUTER_RADIUS,
            "inner_cone_centre_radius_m": INNER_CONE_RADIUS,
            "outer_cone_centre_radius_m": OUTER_CONE_RADIUS,
            "centreline_radius_m": SKIDPAD_CENTERLINE_RADIUS,
            "driving_corridor_width_m": 3.0,
            "inner_cones_per_circle": 16,
            "outer_cones_per_circle": 13,
            "intersection_start_finish_x_m": 15.0,
            "entry_line_halfwidth_m": lane_line_halfwidth,
            "exit_lane_end_x_m": 40.0,
            "drive_sequence": [
                "entry_forward", "right_untimed_lap", "right_timed_lap",
                "left_untimed_lap", "left_timed_lap", "exit_forward",
            ],
        },
        "staging": {
            "frontmost_vehicle_point_target_m": [0.0, 0.0],
            "distance_before_start_finish_m": 15.0,
            "heading_degrees_importer_local": 0.0,
            "note": "D4.3.3 stages the foremost part, not the CARLA base_link. "
                    "Use the actual FSAI footprint to stage the nose at x=0.",
        },
        "stop": {
            "required_stop_distance_after_timing_line_m": 25.0,
            "exit_lane_endpoint_x_m": 40.0,
        },
        "cone_assignment": "Simulation choice: blue on track left and yellow on right in CARLA XY. "
                           "Right circle outer blue/inner yellow; left circle inner blue/outer yellow.",
        "selected_parameters_not_given_in_rules": {
            "radial_cone_centre_offset_outside_painted_lines_m": CONE_LINE_OFFSET_M,
            "entrance_exit_lane_cone_offset_m": GATE_CONE_OFFSET_M,
            "ordered_blue_yellow_entry_stations_for_importer_m": [0, 5],
            "small_orange_entry_stations_m": [-5., -2.5],
            "small_orange_exit_stations_m": [25., 30., 35., 40.],
            "stop_barrier_small_orange": [[40, -1], [40, 0], [40, 1]],
            "big_orange_offset_along_track_m": BIG_ORANGE_GAP_BEFORE_AFTER_LINE_M,
            "outer_ring_occupied_arc_degrees": {
                "right": [320, 580],
                "left": [140, 400],
            },
            "orange_big_indicators": {
                "timing_line_before_after": [
                    [14.25, -3.5], [14.25, 3.5],
                    [15.75, -3.5], [15.75, 3.5]
                ],
                "note": "Four big orange markers around timekeeping as "
                        "required by DE6.3.1; exact offsets are schematic.",
            },
            "note": "Figure 8 specifies placement counts, not exact azimuths, "
                    "cone colours or offset in metres; these are reproducible "
                    "simulation choices, not official coordinates. The 100 "
                    "degree open wedges at each outer ring prevent cones "
                    "blocking the other loop at the centre crossover.",
        },
    }
    return cones, path, layout


def svg_preview(destination, cones, path, layout):
    # SVG path/markers with a metre-labelled Cartesian grid. SVG y points
    # downward, matching CARLA local right-handed screen view where +y=right.
    from xml.sax.saxutils import escape

    xs = [c["x"] for c in cones]
    ys = [c["y"] for c in cones]
    min_x, max_x = min(xs)-5, max(xs)+5
    min_y, max_y = min(ys)-5, max(ys)+5
    m = 12.0 if layout["event"] == "skidpad" else 9.0
    width = max(750, int((max_x-min_x)*m)+120)
    height = max(450, int((max_y-min_y)*m)+110)
    x0, y0 = 65, 60
    scale = min((width-120)/(max_x-min_x), (height-100)/(max_y-min_y))

    def xy(x, y):
        return f"{x0+(x-min_x)*scale:.2f},{y0+(y-min_y)*scale:.2f}"

    def line_xy(x1, y1, x2, y2, **opts):
        color = opts.get("color", "#334155")
        dash = opts.get("dash", "")
        return f'<line x1="{xy(x1,y1).split(",")[0]}" y1="{xy(x1,y1).split(",")[1]}" x2="{xy(x2,y2).split(",")[0]}" y2="{xy(x2,y2).split(",")[1]}" stroke="{color}" stroke-width="{opts.get("width",1)}" stroke-dasharray="{dash}"/>'

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#111c2a"/>',
        f'<text x="30" y="28" fill="#e2e8f0" font-family="sans-serif" font-size="19" font-weight="bold">{escape(layout["event"].title())} • FSUK 2027 simulation layout</text>',
    ]
    for x in range(math.floor(min_x/5)*5, math.ceil(max_x/5)*5+1, 5):
        out.append(line_xy(x, min_y, x, max_y))
    for y in range(math.floor(min_y/5)*5, math.ceil(max_y/5)*5+1, 5):
        out.append(line_xy(min_x, y, max_x, y))
    if layout["event"] == "skidpad":
        for cy in (+9.125, -9.125):
            for radius in (INNER_RADIUS, OUTER_RADIUS):
                x, y = xy(15, cy).split(",")
                out.append(f'<circle cx="{x}" cy="{y}" r="{radius*scale:.2f}" fill="none" stroke="#94a3b8" stroke-width="1.2" stroke-dasharray="6 4"/>')
        out.append(line_xy(15, -12.7, 15, 12.7, color="#fbbf24", dash="3 5",width=2))
        out.append(f'<text x="{xy(15,0).split(",")[0]}" y="44" fill="#fbbf24" font-family="sans-serif" font-size="11">D4 timing line x=15 m</text>')
    else:
        out.append(line_xy(0, -3, 0, 3, color="#fde047", width=2))
        out.append(line_xy(75, -3, 75, 3, color="#fb7185", width=2))
    for color, fill in (("blue", "#2885f6"), ("yellow", "#fde047"),
                        ("orange_big", "#f97316"), ("orange", "#fb923c")):
        for c in cones:
            if c["tag"] != color:
                continue
            x, y = xy(c["x"], c["y"]).split(",")
            radius = "6" if color == "orange_big" else "3.8"
            out.append(f'<circle cx="{x}" cy="{y}" r="{radius}" fill="{fill}" stroke="#121826" stroke-width=".6"/>')
    # Use a thin path so cones remain visible for map diagnostics.
    polyline = " ".join(xy(x,y) for _,x,y in path)
    out.append(f'<polyline points="{polyline}" fill="none" stroke="#2dd4bf" stroke-width="1.3" stroke-opacity=".8"/>')
    out.append('<text x="30" y="50" fill="#93c5fd" font-family="sans-serif" font-size="11">Blue/Yellow = course • small orange = entry/exit/stop • big orange = before/after timing • cyan = reference</text>')
    out.append('</svg>')
    destination.write_text("\n".join(out)+"\n",encoding="utf-8")


def png_preview(destination, cones, path, layout):
    """Optional portable, high-resolution overview when matplotlib exists."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import pyplot as plt
        from matplotlib.patches import Circle
    except ImportError:
        return False
    event = layout["event"]
    fig, ax = plt.subplots(figsize=(14, 5.5) if event == "acceleration"
                           else (11, 8), dpi=130)
    fig.patch.set_facecolor("#f6f8fc")
    ax.set_facecolor("#ffffff")
    x_points = [p[1] for p in path]
    y_points = [p[2] for p in path]
    ax.plot(x_points, y_points, color="#0d9488", lw=1.8, alpha=.8,
            label="Intended path (reference only)", zorder=2)
    if event == "skidpad":
        for cy in (9.125, -9.125):
            for r in (INNER_RADIUS, OUTER_RADIUS):
                ax.add_patch(Circle((15, cy), r, facecolor="none",
                                    edgecolor="#9ca3af", linestyle="--",
                                    linewidth=1.0, zorder=1))
        ax.axvline(15, color="#d97706", linestyle="--", linewidth=1.2,
                   label="Start/finish at central intersection")
        ax.text(15.5, 20, "Right +Y (CARLA)", fontsize=9, color="#374151")
        ax.text(15.5, -20, "Left -Y (CARLA)", fontsize=9, color="#374151")
    else:
        ax.axvline(0, linestyle="--", color="#d97706", label="Start x=0 m")
        ax.axvline(75, linestyle="--", color="#be123c", label="Finish x=75 m")
        ax.axvspan(75, 125, facecolor="#d1fae5", alpha=.4,
                   label="Driverless stopping lane (50 m)")
    for color, face in (("blue", "#2563eb"), ("yellow", "#eab308"),
                        ("orange", "#fb923c"), ("orange_big", "#ea580c")):
        selected = [c for c in cones if c["tag"] == color]
        ax.scatter([c["x"] for c in selected],
                   [c["y"] for c in selected],
                   s=90 if color == "orange_big" else 35 if color == "orange" else 45,
                   c=face, edgecolor="#111827", linewidth=.5,
                   label=f"{color.replace('_big', ' big').title()} cones ({len(selected)})",
                   zorder=3)
    ax.set_xlabel("Local X (m), forward")
    ax.set_ylabel("Local Y (m), CARLA right positive")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(f"ASURT Formula Student UK 2027 • {event.title()}",
                 fontweight="bold", pad=12)
    ax.grid(True, linestyle=":", alpha=.4)
    ax.legend(loc="best", fontsize=8, framealpha=.95)
    ax.margins(.08)
    fig.tight_layout()
    fig.savefig(destination, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)
    return True


def generate():
    index = {"title": "ASURT 2027 FSUK D4/D5 Cone Tracks", "units": "m",
             "frame": "local CARLA XY (+x forward, +y vehicle right)", "events": {}}
    for name, builder in (("acceleration", acceleration), ("skidpad", skidpad)):
        cones, reference, layout = builder()
        folder = HERE / name
        folder.mkdir(exist_ok=True, parents=True)
        prefix = name + "_2027"
        csv_path = folder / (prefix+"_cones.csv")
        json_path = folder / (prefix+"_cones.json")
        layout_path = folder / (prefix+"_layout.json")
        route_path = folder / (prefix+"_reference_path.csv")
        preview_path = folder / (prefix+"_preview.svg")
        png_path = folder / (prefix+"_preview.png")
        save_csv(csv_path, cones)
        save_json(json_path, {
            "schema": "ASURT_CARLA_TrackImporter_2026",
            "event": name, "units": "m",
            "coordinate_system": "local_CARLA_xy_x_forward_y_right",
            "cones": cones,
        })
        save_json(layout_path, layout)
        save_path(route_path, reference)
        svg_preview(preview_path, cones, reference, layout)
        made_png = png_preview(png_path, cones, reference, layout)
        index["events"][name] = {
            "cone_count": len(cones),
            "color_counts": dict(sorted(Counter(c["tag"] for c in cones).items())),
            "file_csv": str(csv_path.relative_to(HERE)),
            "file_json": str(json_path.relative_to(HERE)),
            "layout_metadata": str(layout_path.relative_to(HERE)),
            "reference_driving_path": str(route_path.relative_to(HERE)),
            "preview": str(preview_path.relative_to(HERE)),
        }
        if made_png:
            index["events"][name]["preview_png"] = str(png_path.relative_to(HERE))
        print(name, len(cones), index["events"][name]["color_counts"],
              csv_path.relative_to(HERE))
    save_json(HERE / "manifest.json", index)


if __name__ == "__main__":
    generate()
