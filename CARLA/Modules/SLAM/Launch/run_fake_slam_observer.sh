#!/usr/bin/env bash
# Publish only imported cones the simulated car could currently see ahead.
ASURT_FAKE_MAP_ENTRY=fake_slam_observer_node exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_fake_slam.sh" "$@"
