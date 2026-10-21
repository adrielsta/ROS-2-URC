# Readiness and acceptance gates

This is a runnable development foundation, not a completed competition brain. The original flat baseline and the newer [Utah practice pack](PRACTICE.md) have separate validation scopes.

**Original flat baseline, October 20, 2026:** [GitHub Actions run 39511218743](https://github.com/adrielsta/ROS-2-URC/actions/runs/39511218743) passed both jobs at commit `44dc8c500664a21fc1df5bbfab6fe385acf36332`; the suite then contained 29 tests. Integration verified disarmed startup with live simulated sensors, completion of both navigation targets, stopped arrival with the green LED state, and a latched software emergency stop that blocks re-arming. These checks apply to the small synthetic obstacle course. See [original verification record](evidence/verification.json).

**Short Utah trial verified October 21, 2026 UTC:** [run 39524530901](https://github.com/adrielsta/ROS-2-URC/actions/runs/39524530901) passed the 37-test unit job, original flat integration, and actual-heightfield integration at commit `b7358c80f8d17a94aaae3bbfbf774363f40a23c3`. The trial completed in 46.99 simulation seconds, with measured stopped-arrival errors of 0.402m and 0.407m, maximum observed body tilt of 10.29 degrees, and successful continuation after the C2 heartbeat dropped. The test also checks stationary disarmed startup with less than 0.3m spawn drift. See [complete report and limits](evidence/utah-verification.json). The full hill route and other navigation profiles have not been driven by this integration test.

| Capability | Implemented | Verified evidence / remaining gate |
|---|---|---|
| Pure command arbitration | Yes | Unit tests for invalid/stale inputs, mode switches, e-stop and link loss |
| Mission input validation | Yes | Unit tests reject wrong frames and invalid coordinates |
| ROS mission and command nodes | Yes | ROS construction and live full-stack integration passed in the linked run |
| Small Gazebo/Nav2 route | Yes | Two-target completion, stopped arrival and latched e-stop passed in the linked run |
| Utah live catalog | Yes | Timestamped catalog; per-source failures remain visible |
| Georeferenced DEM conversion | Yes | Synthetic raster orientation/scale tests and [actual source-run evidence](evidence/README.md); surveyed coordinate checks remain |
| Short real Utah driving trial | Verified in simulation | Actual DEM, two measured stopped arrivals and forced heartbeat loss passed in the linked Utah run; ideal localization and known-DEM ground filter |
| Hill/far-side route practice | Configured | Natural terrain, known safe-prior path, two 1m checks, geometric LOS heartbeat model; full course completion is not certified by the short test |
| Other mission practice | Navigation only | Astronaut rendezvous, delivery checkpoints, two science-site visits/return, lander approach; no manipulation or bioassays |
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
- The Utah practice pack instead uses ideal simulation pose; this does not resolve the missing outdoor estimator. Its known-DEM scan filter is also simulation-only.
- Software freshness uses reception times. It does not yet reject every replayed sensor timestamp or enforce covariance limits.
- A 2D scan is insufficient for holes, drop-offs, overhangs and tilted rover perception.
- The static slope map is a prior, not a 3D traversability estimator.
- Network gateways, RBAC, physical controls, autonomy mode audit logging and competition retreat rules are not complete.
- Apt versions/image tags can evolve; accepted release environments need recorded package versions and image digests.
