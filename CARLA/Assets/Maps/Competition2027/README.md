# Formula Student UK 2027 — Acceleration and Skidpad cone-track fixtures

These are deterministic, **importer-ready cone courses** for the current
ASURT CARLA FlatAsphalt simulator and its independent planning-mode tests.
They are **not Unreal .umap assets**, and opening them does not modify CARLA.
They combine geometry from `Dynamic_Missions_Details.pdf` (© 2027 IMechE,
Formula Student UK 2027 Rules, pages 116–117 and 119) with the additional
**FS-AI Dynamic Events Setup and Cones Specification** (© 2021 IMechE,
DE6.3.1, its Table 2 and Figure 1 on PDF page 3 / Figure 3 on PDF page 4).
The latter explicitly assigns **small orange** to entry/exit lanes and
**big orange** before/after timing/start/finish lines. Each layout records
which coordinate offsets are reproducible simulator choices.
The generated drawing/reference path is a simulation test design, **not**
an official organizer-supplied coordinate list or a verified autonomous path.

## Open and import

In **CARLA System Manager → Tools → Cone Track Importer**, click either
**2027 Acceleration** or **2027 Skidpad**. This only loads a local CSV and
draws its 2D preview. You can also choose either generated `*_cones.csv` or
`*_cones.json` using **Choose CSV / JSON**; both contain identical cone
positions and pass the importer's actual `load_cones` parser.

1. Choose **2027 Acceleration** or **2027 Skidpad**. Confirm the blue,
   yellow, **small orange**, and **big orange** counts and the first gate;
   for these files, **Mirror Y is OFF**.
2. Select a large, unobstructed portion of the running **FlatAsphalt** map
   using the GUI's fixed X/Y/Z/yaw fields. Local +X is forward along the
   approach, and +Y is CARLA's right. Ensure the entire cone course fits.
   The importer defaults may not fit a 130 m Acceleration course. The
   already-running CARLA course was previously loaded from an older
   58-cone file; regenerating the CSV does **not** relocate or modify
   those actors. Remove the old managed course explicitly and re-import
   the new 67-cone version after a fresh preview.
3. Keep **Project onto terrain** selected. Use **Find ground-supported
   placement** if required, then **Preview cones** and inspect all lane
   markers. These operations do not create physical cone actors.
4. When the preview is on drivable ground, use **Spawn physical cones**.
   Its ownership guards prevent overwriting an existing live managed course.
   To change event, use **Remove my cones** only after checking ownership,
   then select the other course and repeat.
5. **Preview car start** before using the separate **POSITION EXISTING FSAI
   CAR AT TRACK START** button. These importer tests use the *first blue/yellow
   gate* as the initial car reference. The competition rules instead measure
   staging from the **foremost point of the vehicle**: adjust **Car start
   offset** by the actual FSAI front-footprint distance before a timing test.
   Neither generated cone file moves the car automatically.
6. In the System Manager select the corresponding PathPlanning module and
   the same **Planning RViz view** to compare its output with the imported
   cones. The SLAM cone mapper must observe the imported physical cones
   before the map-only planners can publish a meaningful course.

Use a separate full course at a time so mapped landmarks from different
tracks are never mixed. A route reference is **not a controller command**.
Do not use the exported reference CSV as accepted Control authority.

## Files

| Event | Import cones | Geometry and timing | Ideal preview |
| --- | --- | --- | --- |
| Acceleration | [CSV](acceleration/acceleration_2027_cones.csv) / [JSON](acceleration/acceleration_2027_cones.json) | [Layout JSON](acceleration/acceleration_2027_layout.json) | [PNG](acceleration/acceleration_2027_preview.png) / [SVG](acceleration/acceleration_2027_preview.svg) |
| Skidpad | [CSV](skidpad/skidpad_2027_cones.csv) / [JSON](skidpad/skidpad_2027_cones.json) | [Layout JSON](skidpad/skidpad_2027_layout.json) | [PNG](skidpad/skidpad_2027_preview.png) / [SVG](skidpad/skidpad_2027_preview.svg) |

Each event also includes a `*_reference_path.csv` with `stage,x,y,z`
for offline geometry inspection. `manifest.json` indexes all outputs.
`generate_dynamic_courses.py` regenerates all coordinates and previews;
it makes no simulator calls. `test_dynamic_courses.py` checks dimensions,
spacing, ring cone counts, importer compatibility, clearance and planner
geometry without interacting with the running ROS or CARLA processes.

## Complete source-map audit

`audit_cone_files.py` scans the **entire** `Assets/Maps` map-data collection,
plus Trackdraw's sample CSVs. The machine-readable per-file result is
[cone_map_inventory.json](cone_map_inventory.json). The audited directory
contains **972** CSV/JSON files, of which **967** are recognized as cone
coordinate files:

- **4 event cone files:** two generated competition CSVs and their identical
  JSON partners, now containing both **small** and **big** orange markers.
- **142 original training reference JSONs** and **816 deliberately noisy
  training JSONs:** these contain existing cone class annotations, sometimes
  `orange_big` and sometimes removed/swapped/unknown labels by design. They
  do **not** define separate competition entry/exit lanes or timing-line
  orientations. Their input labels are retained exactly to avoid destroying
  training and perception regression expectations.
- **3 legacy test-track CSVs** under `Assets/Maps/CSV` and **2 Trackdraw
  sample CSVs:** closed-loop/general test geometry with no event designation
  or surveyed start/finish line. Adding gate cones at guessed locations
  would turn them into different test tracks, so they remain as original
  fixtures. Dedicated, rules-grounded dynamic event courses are provided
  above.
- Other JSONs are metadata, reference-path exports or non-cone records and
  have been classified separately in the inventory.

The audit distinguishes **lack of orange labels in a raw test file** from
**incorrect cone roles on a competition event course**. It does not convert
unknown layouts into simulated regulation events without start/finish data.
Re-run with `python3 Assets/Maps/Competition2027/audit_cone_files.py`.

## D4 Skidpad

Rules D4.1.1–D4.1.6 and Figure 8 (PDF page 1, rulebook page 116):

- Right/left circle centres: `(15,+9.125)` and `(15,-9.125)` metres
  in local **CARLA** XY. Separation: **18.25 m**.
- Painted inner diameter **15.25 m**, outer diameter **21.25 m**.
  Painted inner/outer radii **7.625 m** and **10.625 m**; middle of the
  driving lane has radius **9.125 m** and its nominal width is **3.0 m**.
- Exactly **16 inner** and **13 outer** cone markers per circle:
  58 ring cones. Two blue/yellow entry pairs at X=0,5 preserve the
  importer's starting-heading detection. Small orange cones mark
  approach stations X=-5,-2.5 and the exit at X=25,30,35,40;
  a tightly spaced small-orange terminal row closes X=40. Four
  `orange_big` cones surround the timekeeping line at X=14.25/15.75,
  Y=±3.5 m. There are **81 cones total**: 31 blue, 31 yellow,
  15 small orange and 4 big orange.
- The central timing line is X=15 m. The prescribed foremost-point staging
  is 15 m before it at X=0; the exit/stop lane extends to X=40 m, 25 m
  beyond that timing line. The reference driving path stops at X=37.5,
  before the physical small-orange terminal row. Rule D4.3.5 calls for
  **two right-circle laps,
  then two left-circle laps**, followed by an exit in the entry direction.

Figure 8 shows the exterior ring cones **absent through the crossover**.
Uniformly spacing 13 cones around the full outside circumference would block
the opposite loop. The generated outer-ring cones occupy 260 degrees on each
exterior arc, preserving 100-degree clear crossover openings. This is a
documented approximation of the printed pattern because the rules do not
provide exact outer-cone angular coordinates. The generator checked
every reference route sample against every cone; clearance is at least
**1.65 m to the centre of a cone**, and the closest two cone centres are
more than **1.83 m** apart. This is an offline 2D check, not collision
certification for a real vehicle footprint.

For this simulation the cone centres are placed 0.15 m **inside** the painted
inner circle and 0.15 m **outside** the painted outer circle. That offset,
the particular blue/yellow assignments and the gate spacing are explicit
layout choices, not specified in the PDF. The physical painted corridor
remains 3.0 m wide between the stated diameters.

The colors are chosen for the ASURT planner and CARLA importer: blue is left
and yellow right relative to motion. On the CARLA right circle this maps to
blue outer/yellow inner; on the CARLA left circle it is blue inner/yellow
outer. The CARLA bridge converts Y into ROS REP-103, so the corresponding
map-only Skidpad planner uses its original calibrated circle directions.
The **four orange_big cones** mark before/after the central timekeeping
line, and the **15 small orange** cones mark entrance and exit/stopping
lanes. These colors and roles are required by DE6.3.1. The exact x/y
offsets are simulation estimates derived from the schematic Figure 3,
which does not supply surveyed coordinates. No new cone interferes with
the intended loop path; the clearance regression tests verify this.

## D5 Acceleration

Rule D5.1.1 (PDF page 4, rulebook page 119) specifies **75 m** from starting
to finish line, **at least 3 m width**, and cones at **about 5 m** intervals.
This test course has:

- Timed straight X=0 to X=75, chosen lane width **3.5 m** (exceeds minimum).
- Boundary stations every **5 m**, inclusive at X=0/5/…/75.
- Small-orange approach lane pairs at X=-5,-2.5 and a stopping lane
  X=80/85/…/125 with a terminal row spaced no more than 1 m apart.
  The 2027 D5.3.4 driverless limit is to stop within 50 m after finish;
  the older FS-AI handbook **Figure 1 illustrates a 100 m stop area**.
  For this 2027 test fixture the terminal row remains at X=125.
- **67 cones**: 16 blue, 16 yellow, **27 small orange**, and **8 big orange**.
  Four big-orange markers surround the X=0 start line at X=±0.75,
  Y=±3.5, and four surround the X=75 finish at X=74.25/75.75,
  Y=±3.5. The offsets are simulated; the **before/after placement and
  cone size** follow DE6.3.1. The blue and yellow cone centres are 0.15 m outside
  the 3.5 m nominal line edges, resulting in **3.8 m** centre separation.

The importer uses separate small and large ASURT orange meshes with rigid-body
masses of 0.45 kg and 1.05 kg, respectively. Both types appear in the CSV, JSON,
previews and the importer's total and colour breakdown. Select the
event preset again in an importer GUI that had loaded the old CSV before the
orange markers were added.

Per D5.3.2, the **foremost point** of the FSAI must be 0.30 m behind the
X=0 start line. The imported first blue/yellow gate is at X=0,
while the entry lane begins with small orange at X=-5. Do not mistake
its auto-placement pose for certified
staging. Use the car preview, physical body footprint, and offset field.
The marked 50 m stopping lane is an inferred testing layout: the rules
mandate the distance and marked lane but do not specify exact cone coordinates.
The reference path ends at X=120, before the terminal orange barrier at 125.

## Rebuild and verify (offline)

From the ASURT repository root:

```bash
python3 Simulation/CARLA/Maps/Competition2027/generate_dynamic_courses.py
python3 -m unittest discover -s Simulation/CARLA/Maps/Competition2027 -p 'test_*.py' -v
```

The tests call the same actual `load_cones`, `track_start` and
`suggested_invert_y` functions as the importer, then feed a simulated
CARLA-to-ROS Y-reflected cone map into the **Acceleration** and **Skidpad**
planning geometries. They do not assume that a sensor has yet detected
or mapped the imported physical cones. Actual Map RViz path quality depends
on ZED detection, odometry and SLAM association, and still requires live
moving-vehicle validation. These fixtures establish correct source layouts,
not proven competition runs.

### Recovery when Remove my cones reports none but cones are visible

CARLA's newly connected actor cache may initially report frame zero and no
actors. Earlier importer builds could incorrectly archive active cone
ownership based on that incomplete snapshot. The corrected importer now
waits for a real external CARLA tick (without calling `world.tick()`) and
can recover exactly one complete archived record **only if every live tagged
cone has the matching actor ID, type, role, map and server**. It then clears
only those verified cones; an ambiguous or partial match blocks deletion.
Archived audit records remain in the native session log folder. This
recovery is triggered through the existing **Remove my cones** button.
