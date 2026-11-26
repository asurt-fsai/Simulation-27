# Vehicle Delivery Contract

The vehicle package owns what physically exists on the simulated car. The first vehicle should be usable in Gazebo before high-fidelity dynamics are attempted.

## Vehicle deliverables

Work in fsai_description/ on branch feature/vehicle-model.

Provide or update:

- fsai_description/models/fsai_car/model.config;
- fsai_description/models/fsai_car/model.sdf as the Gazebo vehicle entry point;
- visual meshes under fsai_description/meshes/;
- primitive or simplified collision geometry;
- chassis and four wheel links/joints;
- correct mass and inertia when known, clearly marked placeholders otherwise;
- wheel radius/width, wheelbase and track widths when known;
- stable link names and ROS-compatible frame orientation;
- sensor mounting links/frames when their geometry becomes known.

## Required conventions

- **1 Gazebo unit = 1 metre.**
- Vehicle frame: +X forward, +Y left, +Z up.
- Keep high-detail visual CAD separate from collision geometry.
- Do not embed Silverstone-specific coordinates or logic into the vehicle.
- Do not add perception, SLAM, planning, or control modules to this repository.

The simulation lead owns final Gazebo physics tuning, spawning, collision validation, and the ROS-facing simulator interface on integration.
