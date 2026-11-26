# ASURT SLAM workstream guardrails

SLAM consumes existing Perception and the CARLA bridge. Never create
another FSAI hero, ZED camera, Velodyne actor, stereo converter, world
ticker or /clock publisher. Never issue vehicle control commands.
Current ZED landmarks use base_link; existing /odom and TF use map.

Every new SLAM algorithm gets an independent process, explicit dependencies,
tests and its own System Manager registry entry. New modules default OFF
until enabled by the user. Keep tools/RViz/launch_slam.sh functional and
preserve its dedicated RViz configuration and topic interfaces.

Reset the map across CARLA restarts, time rewinds and vehicle teleports;
never mix Silverstone and FlatAsphalt landmarks. Actor/cone truth may be
used for OFFLINE validation only, not production SLAM observations.
FakeMAP and FakeSLAM are opt-in CARLA planning fixtures. They may use import
placements and verified actor truth while selected as test providers. Keep them
off by default and mutually exclusive with each other and the cone landmark mapper.
FakeSLAM's observations use the simulator's stable map frame, so it retains known
cones across hero repositioning in the same episode. Clear those observations on
world reload, time rewind, or course removal.

The old cone_mapping required pyzed and obsolete publishers. Port its
numerical algorithms, not duplicate sensors, stale pose topics or extra
static TF broadcasters. The present odometry-aided mapping is not full
loop-closing SLAM; do not claim solved localization or use it as accepted
control feedback before experiments.
