# URC 2027 — Rover Brain

ROS 2 autonomy foundation for [adrielsta](https://github.com/adrielsta)'s University Rover Challenge project. **Simulation development, not field-ready rover software.**

The foundation connects simulated sensors → Nav2 path planning/control → supervised drive commands → verified arrival. A bundled real Utah patch now supports navigation practice with ideal simulator localization. Neither a successful simulation nor an overhead map proves a real rover can climb a slope.

**Verified simulations:** [37 unit tests and real Gazebo + Nav2 integration passed](https://github.com/adrielsta/ROS-2-URC/actions/runs/39524530901) on October 21, 2026 UTC. The short Utah trial reached and stopped at both targets with 0.402m / 0.407m errors and continued through a forced C2-heartbeat outage. The original flat-course checks also passed. See [readiness and exact scope](docs/READINESS.md), the [verification record](docs/evidence/utah-verification.json), and the [Utah data preview](docs/evidence/README.md).

## What is implemented

- ROS 2 Jazzy / Ubuntu 24.04 / Gazebo Harmonic workspace and cloud devcontainer.
- Differential-drive **placeholder** robot, lidar, IMU, GNSS and RGB camera in a small obstacle course.
- Nav2 Smac 2D planner, regulated pure-pursuit controller, obstacle/inflation costmaps, explicit mission start/cancel, measured stop and arrival checks.
- Disarmed startup, command arbitration, velocity limits, steady-clock watchdogs, latched software emergency stop, telemetry and LED-state topics.
- Autonomous mode does not require C2 heartbeat; teleoperation does. Sensor timeout disarms either mode.
- USGS 3DEP / NAIP / recent Sentinel-2 catalog discovery for the actual 2027 route-finding envelope; DEM clipping, metric reprojection, provenance, Gazebo heightfield and slope-prior map export.
- Unit tests and a GitHub Actions integration job that builds the ROS environment and runs actual Gazebo + Nav2.
- Six [real-terrain practice profiles](docs/PRACTICE.md), including natural-hill/far-side targets, measured stop checks, and a C2-heartbeat outage model. Other mission profiles cover approach/travel only.

## What is not implemented yet

**No field-ready autonomy, astronaut tracking/gestures, hammer pickup, arm control, final radio driver, complete base-station GUI, or validated outdoor localization.** IMU/GNSS are exposed but not fused. The flat demo uses wheel odometry and a fixed `map → odom` transform. The Utah practice pack uses ideal Gazebo pose and a known-DEM ground filter to isolate navigation tests. These are simulation-only shortcuts. See [readiness](docs/READINESS.md) before interpreting any result.

## Start on your Windows PC

Use Ubuntu 24.04 in WSL 2 for a local 3D Gazebo window. Follow the [Windows setup guide](docs/WINDOWS.md); `bash scripts/setup_ubuntu.sh` installs the native dependencies and builds the workspace. Then run `bash scripts/sim.sh gui:=true rviz:=true`, and start the mission in a second terminal. GitHub keeps the source synchronized; simulation runs on your PC.

After the first flat-course run, stop that simulator and try the real terrain:

```bash
git pull --ff-only
bash scripts/build.sh
bash scripts/practice.sh utah
```

In a second Ubuntu terminal in this repository, run `bash scripts/start_mission.sh`. The terrain patch is bundled; no extra map download is needed. See [practice missions](docs/PRACTICE.md) for the longer `utah-route` hill course and the explicit coverage of all four URC missions.

## Start in the cloud

Open **Code → Codespaces → Create codespace on main** in GitHub. The devcontainer installs ROS, Gazebo, Nav2 and the data tools. Codespaces runs on a cloud machine; its availability/usage quota belongs to your GitHub account. This repository does not create or purchase a machine automatically.

After the container finishes building:

```bash
bash scripts/build.sh
bash scripts/sim.sh
```

In a second terminal:

```bash
bash scripts/start_mission.sh
```

To see progress in that second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=27
ros2 topic echo /mission/status
```

The rover should travel around the central block to `(6,0)` and then `(6,3)`, stop, and report `complete`. The LED topic becomes `flashing_green`. First build and simulation startup can take several minutes; a failed start service means Nav2/sensors are not ready yet. The rover never starts moving just because the simulator launches.

For a Linux workstation with a display, `bash scripts/sim.sh gui:=true rviz:=true` opens Gazebo and RViz. For cloud use, see [visualization and cloud workflow](docs/CLOUD_AND_C2.md).

## Docker alternative

```bash
docker build -t urc-brain .
docker run --rm -it --shm-size=1g urc-brain bash
```

Inside the container, run `bash scripts/integration.sh` for the automated full-stack demo. The Docker image has the repository source and built workspace. To change source persistently, use the devcontainer or edit and rebuild the image.

## Stop controls

After sourcing ROS/workspace and setting `ROS_DOMAIN_ID=27`:

```bash
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
ros2 service call /safety/estop std_srvs/srv/Trigger '{}'
ros2 service call /safety/reset_estop std_srvs/srv/Trigger '{}'
```

Reset requires fresh sensors and a measured stop; it returns to HOLD and does **not** re-arm. These ROS services are software interlocks. Real hardware needs an independent physical emergency stop and motor-controller command timeout. Do not connect this simulation launch directly to motor drivers.

## Get terrain for the real route-finding area

```bash
python3 tools/terrain.py catalog --output data/catalog.json
python3 tools/terrain.py fetch-dem --catalog data/catalog.json
python3 tools/terrain.py fetch-imagery --catalog data/catalog.json
python3 tools/terrain.py fetch-satellite --catalog data/catalog.json
python3 tools/terrain.py export --dem data/raw/urc_dem.tif
```

An offline route experiment is also available after export:

```bash
python3 tools/route_preview.py --start 700 200 --goal 350 750
```

This is slope-cost planning on real elevation data, not a physics simulation or a proven driveable route.

The default bounds come from rule 1.e.xiii: west/south/east/north = `-110.786 38.411 -110.768 38.425`. This rectangle is an approximate search envelope, **not a surveyed legal operating boundary**. See [terrain sources and limitations](docs/TERRAIN.md). Large raw/derived data stays outside Git; catalogs and code are versioned. To inspect the exported terrain only:

```bash
source /opt/ros/jazzy/setup.bash
gz sim data/processed/utah/terrain.sdf
```

## Project map

| Location | Responsibility |
|---|---|
| `src/urc_brain` | Mission executive, safety gate, ROS adapters |
| `src/urc_bringup` | Simulation launch, robot model, Nav2 configuration, training course |
| `tools/terrain.py` | Live catalog queries and DEM conversion |
| `tools/ros_integration.py` | Full-stack navigation verification |
| `tests` | Safety and data-coordinate invariants |
| `docs` | Architecture, source review, URC rules, development gates |
| `.devcontainer`, `Dockerfile` | Same cloud/workstation build environment |

Read [architecture](docs/ARCHITECTURE.md), [2027 rule mapping](docs/URC_2027.md), and [source review](docs/SOURCE_REVIEW.md). Dependency packages are installed from the Jazzy apt distribution; this pins the distribution pairing, not every package version or image digest. Record versions from each accepted integration build before a competition release.
