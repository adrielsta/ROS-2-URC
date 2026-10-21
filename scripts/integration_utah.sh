#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-28}"
export GZ_PARTITION="urc-practice-${ROS_DOMAIN_ID}"
export LIBGL_ALWAYS_SOFTWARE=1
mkdir -p artifacts
setsid ros2 launch urc_bringup practice.launch.py scenario:=utah gui:=false >artifacts/utah-simulation.log 2>&1 &
sim_pid=$!
cleanup() {
  result=$?
  kill -TERM -- "-$sim_pid" 2>/dev/null || true
  timeout 15s tail --pid="$sim_pid" -f /dev/null || kill -KILL -- "-$sim_pid" 2>/dev/null || true
  if [ "$result" -ne 0 ]; then tail -n 150 artifacts/utah-simulation.log; fi
}
trap cleanup EXIT
timeout 650s python3 tools/practice_integration.py
