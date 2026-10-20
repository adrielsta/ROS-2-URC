# Cloud development and C2

You can develop from a laptop, desktop or phone using the same GitHub repo and Codespace. A phone is useful for editing and inspecting test results; it is not a practical replacement for a wide-screen simulator/teleoperation console.

- **GitHub:** source, reviewed configuration, issues and test history.
- **Codespace/cloud VM:** development, headless simulation and terrain processing. Gazebo sensor rendering can be slow on software graphics; use a GPU-equipped workstation/VM for larger worlds and camera-heavy tests.
- **Rover:** runtime localization, perception, route planning, mission execution, logging and safety supervision.
- **C2:** operator UI, mission upload while stopped, telemetry, teleoperation, video and explicit intervention controls.

The actual competition must not depend on a cloud connection. Download maps, models and dependencies before departure. URC's current Q&A prohibits a satellite internet link to the robot; this is separate from allowed satellite imagery and GNSS.

## Visualize a cloud simulation

In a ROS-sourced terminal with `ROS_DOMAIN_ID=27`, run:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765
```

Keep the Codespaces port **private**. Use a trusted Foxglove client through authenticated forwarding or an SSH tunnel to inspect `/map`, `/scan`, `/tf`, camera and status topics. A private forwarded URL may require GitHub authentication that a particular client cannot supply; in that case use an SSH port forward and a local client. Browser editing works independently of this visualization connection.

The repository provides ROS telemetry and a visualization bridge dependency, not a custom base-station web app. For a local graphical Gazebo view use `bash scripts/sim.sh gui:=true`. Avoid exposing writable ROS bridges to the public internet.

## Communications can start before buying radios

Implement two gateway profiles behind the same application interface:

| Channel | Payload | Design constraint |
|---|---|---|
| Low-bandwidth control/health | Sequenced bounded commands, mode, stop, battery, GNSS summary, health, acknowledgement | Rate-limited; TTL; explicit mode; retransmission/deduplication; no images/point clouds |
| High-bandwidth IP | Compressed video, detailed telemetry, map updates during allowed programming periods | Capped bitrate; telemetry priority; reconnect without changing drive mode |

Radio models/frequencies remain hardware decisions. First use an Ethernet or simulated transport to inject loss, jitter, delay, out-of-order packets and disconnects. Encode the same application messages later over the selected radios. The existing `/c2/heartbeat` and `/cmd_vel_teleop` are local prototype inputs, not a finished radio protocol.

For the 6–7 person comms group, useful ownership is: C2 UI, low-bandwidth gateway, video/IP gateway, rover hardware/MCU interface, simulation/fault injection, and integration/documentation. Coordinate interfaces with the software/autonomy lead before parallel implementation.
