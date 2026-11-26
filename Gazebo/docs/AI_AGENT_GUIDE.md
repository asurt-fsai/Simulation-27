# AI Agent Guide — FSAI Simulation Repository

This file is for future coding agents working on this repository. Human-facing onboarding belongs in the root README.

## Project goal

Build and maintain a modular Gazebo Harmonic simulator for autonomous Formula Student AI vehicles. The simulator must remain usable by an external ROS 2 autonomy stack rather than becoming coupled to one implementation of perception, SLAM, planning or control.

## Fixed platform baseline

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic / Gazebo Sim 8
- ros_gz
- colcon / ament_cmake

Do not migrate the project to Jazzy, another Gazebo release, Gazebo Classic, or a different simulator unless the user explicitly changes the baseline.

## Repository boundaries

- fsai_description: vehicle description, geometry, links, joints, sensor frames.
- fsai_worlds: Silverstone and future worlds/maps/environment assets.
- fsai_gazebo: physics, collisions, tires, suspension, aero and Gazebo-specific vehicle behavior.
- fsai_bringup: launch, world/vehicle selection, spawning, ros_gz and ROS-facing integration.
- fsai_sim_tools: developer utilities.
- fsai_sim_tests: integration and physics validation.

Autonomy modules do not belong here.

## Branch model

- main: stable reviewed simulator state.
- feature/map-silverstone: initial map-team branch.
- feature/vehicle-model: initial vehicle-team branch.
- integration: simulation-lead staging branch for map + vehicle + physics + ROS validation.

Future branches should describe features or fixes, not people.

## Invariants

1. One Gazebo unit is one metre.
2. ROS vehicle convention is +X forward, +Y left, +Z up.
3. Do not hardcode Silverstone assumptions into generic launch or vehicle code.
4. Use named spawn points instead of scattering map coordinates through scripts.
5. Keep detailed visual geometry separate from simpler collision geometry where practical.
6. Keep tunable physical parameters in configuration or model data instead of scattering constants through code.
7. Prefer standard Gazebo Harmonic systems and ros_gz before custom plugins.
8. Keep the autonomy stack outside this repository and communicate through ROS 2.
9. Preserve unrelated work and inspect current branch/status before editing.
10. Placeholder values must be documented as placeholders and never presented as validated FSAI data.

## Roadmap

### v0.1 — integration foundation

- repository/package contracts;
- placeholder world and vehicle;
- stable launch entry point;
- simulation clock bridge;
- move / steer / stop path;
- basic odometry and IMU;
- reset capability.

### v0.2 — Silverstone integration

- Silverstone loads from fsai_worlds;
- named spawn locations;
- real map collision geometry;
- vehicle is spawned independently from the world;
- integration branch validates contact/collisions and repeatable launch.

### v0.3 — autonomy interface

- stable ROS 2 control interface;
- odometry, IMU and wheel/steering feedback;
- camera/lidar as required;
- external perception/SLAM/planning/control can run without Gazebo-specific code.

### v0.4+ — fidelity and validation

- real vehicle visuals/dimensions;
- validated mass, inertia and CG;
- tire, steering, suspension and aero refinement;
- repeatable physics validation worlds/tests;
- additional competition and test maps.

## Source of truth

For Gazebo-native simulation behavior, SDF model/world files are currently the canonical runtime representation. If URDF/Xacro is introduced later for ROS tooling, document generation or synchronization so URDF and SDF do not silently drift.
