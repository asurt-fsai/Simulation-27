#!/usr/bin/env bash
# Publish a single cancel event to the already-running goal planner.
# Does not create a controller, robot, or additional ROS authority.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
# Both planners subscribe to independent cancels. A first click cancels the
# high-speed track goal promptly; the legacy Pure Pursuit demo also stops.
timeout 5s ros2 topic pub --once \
  /planning/track_goal/cancel std_msgs/msg/Bool "{data: true}" || true
timeout 5s ros2 topic pub --once \
  /planning/goal_pose/cancel std_msgs/msg/Bool "{data: true}" || true
