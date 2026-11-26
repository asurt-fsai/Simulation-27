# Map / World Delivery Contract

The map package owns track and environment assets. Silverstone is the first target, and future tracks must follow the same contract.

## Silverstone deliverables

Work in fsai_worlds/ on branch feature/map-silverstone.

Provide:

- fsai_worlds/worlds/silverstone.sdf as the world entry point;
- track/environment models under fsai_worlds/models/silverstone/ or clearly named reusable model directories;
- visual geometry and separate collision geometry where practical;
- Gazebo materials/textures using relative resource paths;
- documented world origin and coordinate convention;
- one or more validated vehicle spawn poses in fsai_worlds/config/spawn_points.yaml;
- a short README beside the map assets for any required resource or import notes.

## Required conventions

- **1 Gazebo unit = 1 metre.**
- World coordinates use +Z up.
- Do not bake the vehicle into the world.
- Do not add autonomy-stack code to this package.
- Do not hardcode machine-specific absolute paths.
- Keep visual meshes and collision geometry logically separate so collision complexity can be tuned independently.

The integration branch owns spawning the vehicle into the world and validating vehicle-track collisions.
