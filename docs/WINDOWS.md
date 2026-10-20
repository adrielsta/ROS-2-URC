# Run the rover on a Windows PC

Use Ubuntu 24.04 in WSL 2. ROS 2 Jazzy and Gazebo Harmonic run inside Ubuntu; WSLg displays the simulator on the Windows desktop. The initial simulation is the synthetic obstacle course, not the exported Utah heightfield.

## Install Ubuntu

In **Windows PowerShell as administrator**:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart if prompted, open **Ubuntu 24.04** from Start, and create a Linux username and password. Use the Ubuntu terminal for every remaining command. Windows 11 or Windows 10 build 19044+ is required for WSLg; keep the Windows graphics driver current.

## Install and build the project

For a first checkout, paste this block into Ubuntu:

```bash
sudo apt-get update &&
sudo apt-get install -y git &&
git clone https://github.com/adrielsta/ROS-2-URC.git "$HOME/ROS-2-URC" &&
cd "$HOME/ROS-2-URC" &&
bash scripts/setup_ubuntu.sh
```

The password prompt uses your Linux password. Characters are invisible while typing. The installer checks Ubuntu 24.04, adds the official ROS package repository, installs dependencies, builds the workspace, runs unit checks and constructs the ROS nodes. It prints `SETUP COMPLETE` only after those steps succeed. It does not test your PC's graphics or complete a navigation mission.

Keep the checkout in your Linux home folder for build performance. If the checkout already exists, inspect any local changes, update it with `git pull --ff-only`, then run `bash scripts/setup_ubuntu.sh` from that folder. Do not delete an existing checkout to rerun installation. The setup script can be rerun after an interrupted download.

## Open the simulator

In Ubuntu:

```bash
cd ~/ROS-2-URC
bash scripts/sim.sh gui:=true rviz:=true
```

Keep that terminal open. Gazebo opens the 3D course; RViz is a separate sensor/map viewer and initially uses its default layout. Startup alone does not move the rover.

Open a second Ubuntu terminal, then run:

```bash
cd ~/ROS-2-URC
bash scripts/start_mission.sh
```

If the start request reports that sensors or Nav2 are not ready, wait for startup to finish and retry. The rover should navigate to two targets and stop. `Ctrl+C` in the simulation terminal shuts down the simulation. The README also documents software stop services.

## Graphics troubleshooting

If GUI windows do not open, check `wsl -l -v` in PowerShell; the Ubuntu distribution must use version 2. Run `wsl --update` in administrator PowerShell. Close your running Linux work before `wsl --shutdown`, then reopen Ubuntu.

If Gazebo opens with a blank viewport or reports an OpenGL rendering error, first stop the simulator and try software rendering for diagnosis:

```bash
LIBGL_ALWAYS_SOFTWARE=1 bash scripts/sim.sh gui:=true rviz:=true
```

Software rendering can be much slower. If it works, inspect `glxinfo -B` in Ubuntu and the Windows GPU driver before returning to hardware rendering. Do not add arbitrary DISPLAY overrides to `.bashrc`; WSLg normally sets the display environment itself.

## Sources

- [Microsoft WSL GUI applications](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Official ROS apt source package](https://github.com/ros-infrastructure/ros-apt-source/releases/tag/1.3.0)
- [Gazebo and ROS compatibility](https://gazebosim.org/docs/harmonic/ros_installation/)
