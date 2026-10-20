# URC 2027 — Rover Brain

ROS 2 autonomy foundation for [adrielsta](https://github.com/adrielsta)'s University Rover Challenge project. **Simulation development, not field-ready rover software.**

The first milestone is a repeatable chain: simulated sensors → Nav2 path planning/control → supervised drive commands → verified arrival. Real Utah data is a separate, reproducible terrain pipeline. Neither a successful simulation nor an overhead map proves a real rover can climb a slope.

## What is implemented

- ROS 2 Jazzy / Ubuntu 24.04 / Gazebo Harmonic workspace and cloud devcontainer.
- Differential-drive **placeholder** robot, lidar, IMU, GNSS and RGB camera in a small obstacle course.
- Nav2 Smac 2D planner, regulated pure-pursuit controller, obstacle/inflation costmaps, explicit mission start/cancel, measured stop and arrival checks.
- Disarmed startup, command arbitration, velocity limits, steady-clock watchdogs, latched software emergency stop, telemetry and LED-state topics.
- Autonomous mode does not require C2 heartbeat; teleoperation does. Sensor timeout disarms either mode.
- USGS 3DEP / NAIP / recent Sentinel-2 catalog discovery for the actual 2027 route-finding envelope; DEM clipping, metric reprojection, provenance, Gazebo heightfield and slope-prior map export.
- Unit tests and a GitHub Actions integration job that builds the ROS environment and runs actual Gazebo + Nav2.

## What is not implemented yet

**No field-ready autonomy, astronaut tracking/gestures, hammer pickup, arm control, final radio driver, complete base-station GUI, or validated outdoor localization.** IMU/GNSS are exposed in the simulator but not fused into the demo navigation solution. The flat demo uses wheel odometry and a fixed `map → odom` transform. The exported Utah world contains terrain only; it is not yet an end-to-end geolocated rover trial. See [readiness](docs/READINESS.md) before interpreting any result.

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
python3 tools/terrain.py export --dem data/raw/urc_dem.tif
```

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
