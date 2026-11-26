# Integration Workflow

The integration branch is owned by the simulation integrator. It is the staging branch where map and vehicle work are proven together before they reach main.

## Integration responsibilities

1. Bring in the current Silverstone and vehicle work.
2. Load the selected Gazebo world through fsai_bringup.
3. Spawn the selected vehicle at a named pose from spawn_points.yaml.
4. Verify scale, orientation, gravity, ground contact and collision behavior.
5. Verify the vehicle can move and steer through the simulator control path.
6. Bridge simulation time, state and sensors through ros_gz.
7. Keep the ROS-facing interface independent of the autonomy implementation.
8. Add repeatable integration tests as each subsystem becomes stable.

The long-term consumer is an external FSAI ROS 2 stack containing perception, SLAM/localization, planning and control. Those modules remain in separate repositories.
