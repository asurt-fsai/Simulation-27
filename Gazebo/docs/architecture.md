# Simulator Architecture

This repository is the integration boundary between Gazebo Harmonic and an autonomous Formula Student AI stack.

| Area | Package | Primary responsibility |
|---|---|---|
| Vehicle description | fsai_description | Geometry, joints, frames, sensor mounting frames |
| Worlds and tracks | fsai_worlds | Silverstone, future tracks, cones, environment, spawn definitions |
| Vehicle simulation | fsai_gazebo | Physics, collision behavior, tires, suspension, aerodynamics, Gazebo systems |
| Runtime integration | fsai_bringup | World selection, spawning, bridge, launch, ROS-facing simulator interface |
| Developer tooling | fsai_sim_tools | Reset, spawn, diagnostics and workflow helpers |
| Validation | fsai_sim_tests | Spawn, collision, topic, sensor, physics and regression tests |

The autonomy stack is external. It communicates through ROS 2 interfaces and should not depend on Gazebo APIs.

Maps and vehicles are replaceable inputs. Silverstone and the first FSAI car are the first implementations of those contracts, not hard-coded assumptions in generic integration code.
