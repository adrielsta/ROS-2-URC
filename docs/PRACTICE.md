# URC 2027 practice missions on real Utah terrain

Rules checked against the supplied 2027 PDF and the official [Q&A](https://urc.marssociety.org/home/requirements-guidelines/qa), including the October 11 updates. The six profiles below are **navigation practice subsets**. They are not official courses, judge-certified scores, or six completed competition missions.

## Run on your Windows / Ubuntu setup

Stop the previous simulator with Ctrl+C in its launch terminal. Run only one rover simulation at a time in ROS domain 27.

```bash
cd ~/ROS-2-URC
git pull --ff-only
bash scripts/build.sh
bash scripts/practice.sh utah
```

Gazebo opens the real terrain patch. The rover stays parked until you open a second Ubuntu terminal and run:

```bash
cd ~/ROS-2-URC
bash scripts/start_mission.sh
```

The default `utah` run drives a short route near the start and deliberately removes the C2 heartbeat after the first stop. This checks that onboard navigation continues without that heartbeat. It does not disconnect every ROS topic or emulate an actual radio.

To inspect its assessment after the run:

```bash
python3 -m json.tool artifacts/utah-practice.json
```

For a different profile, stop the old simulator, then use one of these commands. Start the mission in the second terminal as above.

| Command | What runs | What is still missing |
|---|---|---|
| `bash scripts/practice.sh utah` | Short real-terrain drive; two measured stops; forced heartbeat dropout | Full route and real outdoor localization |
| `bash scripts/practice.sh utah-route` | Two targets on a natural hill and behind it; 1m arrival checks; geometric LOS heartbeat model | GNSS/IMU fusion, marker perception, legal breadcrumb retreat, calibrated chassis |
| `bash scripts/practice.sh astronaut-approach` | Initial rendezvous at an illustrative astronaut location | Follow/Stay/Fetch/Come/Give, perception, suit link, arm |
| `bash scripts/practice.sh delivery-navigation` | Three illustrative pickup/drop travel checkpoints | Official stage script/gate, identifying and manipulating objects, signs/search |
| `bash scripts/practice.sh science-navigation` | Visit two selected sample-site checkpoints and return | Site reasoning, evidence capture, sampling, bioassays, sealing/returning a real cache |
| `bash scripts/practice.sh equipment-approach` | Drive to a simple 1.5m-high lander fixture | Articulated mechanisms, arm/gripper, autonomous typing and all servicing tasks |

These independent drills each start a new clock. They do **not** yet enforce a single shared 40-minute clock across both complete Autonomy sub-missions or the 10-minute configuration change to Equipment Servicing.

Use `gui:=false` after a profile name for a headless run, or `rviz:=true` for the additional ROS viewer. If Windows graphics fails, stop the simulator and try `LIBGL_ALWAYS_SOFTWARE=1 bash scripts/practice.sh utah`; this can be slower.

## What the terrain represents

The versioned patch is 389.12m square at 1.52m height-sample spacing, cropped from the actual 1m USGS data already retrieved for the allowed route-finding envelope. Heights are not exaggerated or flattened. The aerial texture comes from the July 6, 2024 NAIP acquisition. Height and texture sources are recorded in `models/utah_site/manifest.json` and `docs/evidence`.

Bundling this small patch makes a normal Git pull enough to run the practice scenarios offline. It is not the full square-mile search envelope. `tools/bundle_utah.py` reproduces the bundle from the original retrieved datasets; `tools/generate_urc_scenarios.py` reproduces scenario maps/worlds from the versioned bundle. The selected sample sites, lander, astronaut, delivery locations and target coordinates are invented practice locations. **Only the route-finding mission's approximate geographic envelope is published in the rules; this patch is not evidence of where the other missions will occur.**

The hill target is above the start and has approaches rejected by the 18-degree slope prior. The far target is geometrically hidden by the natural terrain from a modeled 2m-high base antenna and 0.8m-high rover antenna. Those heights and the 18-degree cutoff are adjustable development assumptions, not URC requirements. The planned paths total about 308m; actual distance and completion depend on the controller and physics. Driving at 0.35m/s takes many minutes. This is intentionally more demanding than the short `utah` check.

The two route posts have three 20cm-square ArUco 4x4_50 faces, a one-cell white border, 2.5cm cells, and face centers 1.1m above the local ground. Detection is not implemented. Navigation uses selected approach waypoints offset 0.8m from the post; the referee checks actual simulator position against the post center, not merely the commanded waypoint.

The navigation cost image subdivides each 1.52m source cell into 38cm cells using nearest-neighbor copying. This gives the path controller enough intermediate points for its 40cm minimum lookahead. It preserves all slope costs and blocked cells; it does not create finer terrain measurements.

## Honest simulation boundary

- The rover is still the two-drive-wheel/caster placeholder, not your measured rover. The Utah model raises its fixed caster supports 2cm relative to the flat demo so they do not bridge small depressions and unload the driven axle. This is an explicit proxy-chassis assumption, not calibrated suspension. A hard slope may expose this limitation; a failed run is useful evidence, not a reason to secretly flatten the terrain.
- **Localization is ideal Gazebo ground truth.** Nav2 uses a level `base_footprint` frame; sensors retain the body's actual height and tilt through `base_link`. Encoder, IMU and GPS messages exist, but a real outdoor estimator is not being tested. Map axes are EPSG:26912 grid east/north; Gazebo's illustrative GPS is not used as the geodetic authority.
- The scan filter uses the known DEM and ideal pose to remove ground returns. That isolates navigation testing. It is not a deployable perception algorithm for holes, changing terrain or map errors.
- The 1.52m DEM omits small rocks, holes, loose soil and traction. Aerial imagery is visual texture; the robot does not currently understand it. The color photograph is not the collision surface.
- The communications fault removes a modeled heartbeat. It does not model RF diffraction, multipath, throughput, video quality or a complete C2 network blackout. Autonomy remains onboard.
- Arrival requires fresh pose, a measured stop, target distance and the green LED **state topic**. No physically visible animated LED has been added to the rover model yet.
- A simulation tilt over 35 degrees or departure from the patch triggers a software e-stop. The 35-degree setting is an assumed protective test limit, not a certified tip-over threshold.
- Reports set `official_score` to `null`. Navigation success never automatically awards science, manipulation, astronaut-command, or official competition points.

## Acceptance ladder

1. Short real-terrain drive and heartbeat fault, measured stop and bounded attitude.
2. Complete the natural-hill/far-side route; inspect stall locations, planned path, collision assumptions and arrival error.
3. Replace ideal pose with sensor-based estimation and inject GNSS outages, bias, wheel slip and IMU error.
4. Add sensor-derived terrain classification and negative-obstacle handling; compare against the known terrain for evaluation only.
5. Implement astronaut following and its ordered commands, plus calibrated arm/manipulation simulations shared with Equipment and Delivery.
6. Run a combined timed Autonomy mission, allowed intervention/retreat accounting, and the Equipment transition.
7. Correlate simulation with measured chassis, tire, stopping, localization and radio results before field acceptance.
