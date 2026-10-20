#!/usr/bin/env bash
# Install the project's native Ubuntu / WSL 2 development environment.
# Run as your normal Linux user: bash scripts/setup_ubuntu.sh
set -Eeuo pipefail
trap 'printf "\nSetup stopped at line %s. Copy the error above before retrying.\n" "$LINENO" >&2' ERR

source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  printf 'This project requires Ubuntu 24.04. Detected: %s\n' "${PRETTY_NAME:-unknown}" >&2
  exit 1
fi
if [[ "$EUID" -eq 0 ]]; then
  printf 'Run this script as your normal Linux user, without sudo. It asks for sudo when needed.\n' >&2
  exit 1
fi
case "$(dpkg --print-architecture)" in
  amd64|arm64) ;;
  *) printf 'Supported architectures: amd64 and arm64.\n' >&2; exit 1 ;;
esac

# ROS apt packages use Ubuntu's Python, not a separately installed Conda Python.
export PATH="/usr/sbin:/usr/bin:/sbin:/bin:$PATH"
cd "$(dirname "$0")/.."
printf 'Installing ROS 2 Jazzy, Gazebo Harmonic, Nav2 and terrain tools on Ubuntu 24.04.\n'
sudo -v
sudo apt-get update
sudo apt-get install -y ca-certificates curl locales software-properties-common
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
sudo add-apt-repository -y universe

# Official ROS repository configuration; pin and verify the bootstrap package.
# https://github.com/ros-infrastructure/ros-apt-source/releases/tag/1.3.0
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -qx 'install ok installed'; then
  urc_setup_tmp="$(mktemp -d)"
  trap 'rm -rf -- "$urc_setup_tmp"' EXIT
  curl --fail --location --retry 3 --connect-timeout 20 \
    --output "$urc_setup_tmp/ros2-apt-source.deb" \
    'https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb'
  printf '%s  %s\n' \
    'f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296' \
    "$urc_setup_tmp/ros2-apt-source.deb" | sha256sum --check
  sudo dpkg -i "$urc_setup_tmp/ros2-apt-source.deb"
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git \
  ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz ros-jazzy-robot-localization ros-jazzy-slam-toolbox \
  ros-jazzy-foxglove-bridge python3-colcon-common-extensions \
  python3-pytest python3-numpy python3-yaml python3-pil python3-rasterio \
  python3-pyproj python3-requests libegl1-mesa-dev libgl1-mesa-dri mesa-utils

bash scripts/build.sh
# ROS-generated setup scripts are not guaranteed compatible with nounset.
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/ros_construct.py
printf '\nSETUP COMPLETE\n'
printf 'Launch the 3D simulation: bash scripts/sim.sh gui:=true rviz:=true\n'
printf 'In another Ubuntu terminal in this repo: bash scripts/start_mission.sh\n'
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  printf 'No graphical display detected. For WSL, update WSL from Windows and reopen Ubuntu before using the GUI.\n'
fi
