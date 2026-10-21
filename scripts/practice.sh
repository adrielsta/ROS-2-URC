#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if [[ "${1:-}" == list ]]; then
  printf '%s\n' \
    'utah                 Short real-terrain drive + forced C2-heartbeat dropout' \
    'utah-route           Hill and far-side target practice (longer, harder)' \
    'astronaut-approach   Initial rendezvous; no Follow/Fetch/Give yet' \
    'delivery-navigation Pickup/drop travel checkpoints; no object manipulation' \
    'science-navigation  Two site visits and return; no sampling/bioassays' \
    'equipment-approach  Approach the mock lander; no arm/typing yet'
  exit 0
fi
scenario="${1:-utah}"
if [[ $# -gt 0 ]]; then shift; fi
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
export GZ_PARTITION="urc-practice-${ROS_DOMAIN_ID}"
ros2 launch urc_bringup practice.launch.py scenario:="$scenario" "$@"
