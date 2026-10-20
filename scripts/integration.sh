#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
export LIBGL_ALWAYS_SOFTWARE=1
mkdir -p artifacts
ros2 launch urc_bringup sim.launch.py >artifacts/simulation.log 2>&1 &
sim_pid=$!
trap 'kill -INT "$sim_pid" 2>/dev/null || true; wait "$sim_pid" 2>/dev/null || true' EXIT
python3 tools/ros_integration.py
