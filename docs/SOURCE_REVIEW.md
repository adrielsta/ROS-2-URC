# Open-source and user-source review — 2026-10-20

## ROS choices

[ROS 2](https://github.com/ros2) supplies middleware, messages, tools and libraries; it is not a complete rover brain. [Nav2](https://github.com/ros-navigation/navigation2) supplies tested navigation components; [Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/) supplies physics/sensors and the `ros_gz` bridge.

ROS 2 Lyrical is the newer release (May 2026). This foundation deliberately uses **Jazzy / Ubuntu 24.04 / Harmonic**, a documented compatible pair with Jazzy support through May 2029. This is a compatibility choice, not a claim that Jazzy is the latest release. Re-evaluate Lyrical/Jetty after the hardware drivers are chosen and a passing integration baseline exists. Do not build a competition branch on Rolling by default.

The package configuration and mission/safety adapters are original code. ROS, Nav2 and Gazebo are installed as dependencies rather than forked wholesale. Proposed next dependencies are `robot_localization`, OpenCV's ArUco detector and a tested 3D odometry/perception stack selected for the actual sensors. No untested sensor-specific repository is silently adopted as production code.

## Missouri S&T source inspection

Reviewed [MissouriMRDT/Autonomy_Software](https://github.com/MissouriMRDT/Autonomy_Software) at `dfff910cdcc9dad68501dd1b86edb49cb54cde32`:

- `src/states/NavigatingState.cpp`: mission states, waypoint transitions, tag/object checks, stuck handling, geographic pose and controller integration.
- `src/algorithms/planners`: route-planning separation.
- `src/algorithms/controllers`: pure pursuit and predictive Stanley implementations.
- `src/AutonomyNetworking.h`: RoveComm TCP/UDP interface.
- README: custom C++20 stack with ZED/CUDA and GCC constraints; simulation path described as Webots; GPLv3 license.

Useful principles: isolate hardware drivers, mission state transitions, detection, planning, communications and recovery; track stuck conditions explicitly. It is not a drop-in ROS 2 workspace. This project does not copy that code or claim to reproduce their competition system. Their implementation and license must be reviewed before importing code; learning from its structure is distinct from incorporating a GPL implementation.

## Your research links

| Link | Assessment for this build |
|---|---|
| [ROS 2 organization](https://github.com/ros2) | Correct ecosystem; use a supported packaged distribution plus Nav2 and Gazebo |
| [Utah FORGE 2025 model](https://catalog.data.gov/dataset/utah-forge-native-state-numerical-model-2025-update) | Subsurface geothermal thermal/hydraulic/stress model, 6km×6km×4.5km; wrong data/model for surface rover navigation at MDRS |
| [Varun Shankar / Utah Digital Twin Lab](https://varunshankar.com/research.html) | Numerical methods, scientific ML and digital twins; research relevance, not a turnkey URC terrain or autonomy package |
| [NAIRR Mapping Every Drop](https://nairrpilot.org/projects/highlights/mapping-every-drop) | Direct page could not be retrieved in this review. Project-lead descriptions identify a US water-cycle digital twin, not an available URC rover simulator. Do not infer a usable terrain asset from the title |

A verified geometric/sensor/behavior baseline is the next step. Learned world models, neural operators and reinforcement learning can be evaluated later against that baseline if they solve an observed problem.
