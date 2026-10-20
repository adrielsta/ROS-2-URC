#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
export LIBGL_ALWAYS_SOFTWARE=1
mkdir -p artifacts
setsid ros2 launch urc_bringup sim.launch.py >artifacts/simulation.log 2>&1 &
sim_pid=$!
cleanup() {
  result=$?
  kill -TERM -- "-$sim_pid" 2>/dev/null || true
  timeout 15s tail --pid="$sim_pid" -f /dev/null || kill -KILL -- "-$sim_pid" 2>/dev/null || true
  if [ "$result" -ne 0 ]; then tail -n 250 artifacts/simulation.log; fi
}
trap cleanup EXIT
timeout 420s python3 tools/ros_integration.py
