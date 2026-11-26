#!/usr/bin/env bash
# Passive planning RViz viewer. Subscriptions do not switch on ROS modules.
set -euo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
root="$(cd -- "$here/../.." && pwd)"
profile="${1:-overview}"
case "$profile" in
  overview) config="$here/config/planning_classical.rviz" ;;
  goal_pose) config="$here/config/planning_goal_pose_mode.rviz" ;;
  control) config="$here/config/control_goal_pose.rviz" ;;
  track_control) config="$here/config/control_track_mode.rviz" ;;
  classical|acceleration|skidpad|deep_learning) config="$here/config/planning_${profile}_mode.rviz" ;;
  *) echo "Unknown planning RViz profile: $profile" >&2; exit 2 ;;
esac
cache="$HOME/.cache/asurt"
mkdir -p "$cache"
if [[ ! -r "$config" ]]; then
  echo "Missing planning RViz preset: $config" >&2
  exit 1
fi
exec 9>"$cache/planning-rviz-$profile.lock"
if ! flock -n 9; then
  # Existing RViz holds the lock; focus its window if a desktop is available.
  for pid in $(pgrep -u "$(id -u)" -x rviz2 || true); do
    if tr '\0' '\n' <"/proc/$pid/cmdline" 2>/dev/null | grep -Fxq "$config"; then
      wid=$(wmctrl -lp 2>/dev/null | awk -v pid="$pid" '$3==pid{print $1; exit}')
      [[ -z "$wid" ]] || wmctrl -ia "$wid" >/dev/null 2>&1 || true
      exit 0
    fi
  done
  exit 0
fi

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ROS Humble setup not available: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
# RViz only needs its ROS environment; optional installed overlays are loaded
# for message/robot plugins without launching a planner, simulator or publisher.
set +u
source /opt/ros/humble/setup.bash
for setup in \
  "$root/Simulation/Systems/Common/ROSUnderlays/fs-system-26-test-build/install/setup.bash" \
  "$root/Simulation/Systems/Perception/Workspace/autonomy_integration/install/setup.bash" \
  "$root/Simulation/Systems/SLAM/Workspace/slam_integration/install/setup.bash" \
  "$root/Simulation/Systems/PathPlanning/Workspace/planning_integration/install/setup.bash"; do
  if [[ -r "$setup" ]]; then
    source "$setup"
  fi
done
set -u
if [[ "$profile" == "control" || "$profile" == "goal_pose" || "$profile" == "track_control" ]]; then
  # RViz SetGoal must stamp /goal_pose in the CARLA /clock domain.
  exec rviz2 -d "$config" --ros-args -p use_sim_time:=true \
    >>"$cache/planning-rviz-$profile.log" 2>&1
fi
exec rviz2 -d "$config" >>"$cache/planning-rviz-$profile.log" 2>&1
