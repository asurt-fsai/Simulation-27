# ASURT cone importer maintenance requirements

The bridge-owned .cone_track_actors.json is an actor ownership record and
must be retained across restarts. CARLA actor IDs may be recycled. Inspect
type_id and role_name as well as all live asurt_cone_* roles before any
physical actor deletion or record replacement. Fully stale records can be
archived to timestamped state files only when no managed cones exist;
mixed or ambiguous ownership must block automatic reconciliation.

find-placement is a ground probe, never a spawn or vehicle mutation.
Preview modes do not create physical cones. Clear may destroy only verified
owned cones. Place Car separately verifies the FSAI hero and physical
clearance. Do not change the CARLA world ticker or sensor ownership.

Keep test_import_state.py passing after editing importer state handling.

The FSAI start clearance must check every owned cone. CARLA actor.bounding_box
is expensive (an on-demand server query), so cache the FSAI box and fixed
static.prop cone box by its verified blueprint type. Do not change the
underlying oriented-footprint collision calculation or approximate ground
height for speed. A 470-cone read-only benchmark improved from 94.1 s to
5.8 s; rerun the overlap and box-query-count regression test after changes.
