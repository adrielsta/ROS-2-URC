# Readiness and acceptance gates

Version 0.1 is a runnable development foundation, not a completed competition brain.

| Capability | Implemented | Verified evidence / remaining gate |
|---|---|---|
| Pure command arbitration | Yes | Unit tests for invalid/stale inputs, mode switches, e-stop and link loss |
| Mission input validation | Yes | Unit tests reject wrong frames and invalid coordinates |
| ROS mission and command nodes | Yes | Full ROS behavior requires the integration job |
| Small Gazebo/Nav2 route | Yes | Inspect GitHub Actions `ros-simulation` result and its uploaded logs |
| Utah live catalog | Yes | Timestamped catalog; per-source failures remain visible |
| Georeferenced DEM conversion | Yes | Synthetic raster orientation/scale tests; actual source-run report when available |
| Full Utah driving trial | No | Spawn/model calibration, frame alignment, outdoor localization and collision-map validation |
| GNSS/IMU/encoder fusion | No | Sensor drivers, extrinsics, covariance and dropout trials |
| ArUco target refinement | No | Calibrated RGB/depth detector and measured target stop accuracy |
| Astronaut Follow/Stay/Fetch/Come/Give | No | Separate perception, mission and arm integration |
| Base station/radio hardware | No | Telemetry contract only; gateway and packet-loss testing next |
| Physical safety | No | Independent physical stop and MCU watchdog before motors |

## Milestones

1. **Integration course:** repeatable Nav2 completion, correct stop/LED state, cancellation races, stale sensors and teleop link-loss tests. Capture logs and package versions.
2. **Utah prior:** retrieve and review available data; verify imagery acquisition dates and DEM seams; compare elevation orientation and coordinate transforms against independent surveyed points.
3. **Outdoor estimator:** implement local/global state estimation, uncertainty limits and GNSS dropout behavior. No simulator truth in the production navigation path.
4. **Terrain-aware driving:** 3D terrain perception, steepness/roughness/step/negative-obstacle rules; use unknown-as-unsafe where appropriate. Replace the proxy chassis with measured dimensions, mass, center of gravity, steering and tires.
5. **Route task:** two configurable GNSS targets, near-target refinement, stop within 1m, correct LED, one target with complete C2 blackout, path-history-based retreat.
6. **Astronaut task:** follow, stay, fetch, come, give in official order. Start with explicit astronaut commands, then add gesture Follow for the full point opportunity. Arm work is a separate major subsystem.
7. **Field acceptance:** measure stop distance, traction, slopes, slip, localization error and radio performance on actual hardware. Simulate different friction, lighting, GNSS errors, packet loss, rocks and drops, then compare with physical measurements.

Do not label software “500-point capable” based on a demo. The autonomy mission is only 100 of the competition's 500 points, and half of its score is astronaut assistance. Other missions also need science, mechanics, manipulation, teleoperation and SAR evidence.

## Known limitations of this baseline

- Two drive wheels with passive casters are intentionally a placeholder; no suspension or terramechanics.
- Fixed `map→odom` works only for the small synthetic map; drift is not globally corrected.
- Software freshness uses reception times. It does not yet reject every replayed sensor timestamp or enforce covariance limits.
- A 2D scan is insufficient for holes, drop-offs, overhangs and tilted rover perception.
- The static slope map is a prior, not a 3D traversability estimator.
- Network gateways, RBAC, physical controls, autonomy mode audit logging and competition retreat rules are not complete.
- Apt versions/image tags can evolve; accepted release environments need recorded package versions and image digests.
