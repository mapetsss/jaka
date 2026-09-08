# JAKA S5 Six-Joint Controller

Standalone ROS 2 Humble packages for controlling a JAKA S5 through a Qt joint
panel and MoveIt. This repository contains only the S5 controller, its two ROS
interfaces, launch files, tests, and operating documentation. Robot description
and MoveIt configuration packages come from the official JAKA ROS 2 repository.

## Features

- Live six-joint feedback in degrees and radians
- Single-joint planning and incremental SDK JOG commands
- Confirmed six-joint motion through a configurable task-ready pose
- Initial-pose and task-ready-pose commands
- Stop, diagnostic, orderly shutdown, and optional automatic power-off handling
- Startup motion disabled by default

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 and Qt 5 development packages
- Network access to a JAKA S5 controller
- Official `JAKARobotics/jaka_ros2` source tree for `jaka_description` and
  `jaka_s5_moveit_config`

## Workspace Setup

```bash
mkdir -p ~/jaka_s5_ws/src
cd ~/jaka_s5_ws/src
git clone https://github.com/JAKARobotics/jaka_ros2.git
git clone https://github.com/mapetsss/jaka.git jaka_s5_controller_source

cd ~/jaka_s5_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to jaka_s5_controller
source install/setup.bash
```

## Real Robot

Read [the real-robot checklist](docs/JAKA_S5_REAL_ROBOT_TEST.md) before enabling
or moving the robot. Do not run another JAKA SDK control node against the same
robot at the same time.

```bash
ros2 launch jaka_s5_controller s5_joint_controller.launch.py \
  ip:=<robot_ip> \
  use_rviz:=true \
  auto_move_to_initial_on_start:=false
```

Replace `<robot_ip>` with the actual address without angle brackets. The backend
logs in, powers on, and enables the robot. Closing the window powers it off by
default. These software controls do not replace the physical emergency stop.

Automatic startup motion requires explicit opt-in:

```bash
ros2 launch jaka_s5_controller s5_joint_controller.launch.py \
  ip:=<robot_ip> auto_move_to_initial_on_start:=true
```

Use that option only after validating the configured pose at low speed.

## Ready-Pose Utility

With the unified controller stack already running, plan the ready pose without
executing it:

```bash
ros2 launch jaka_s5_controller s5_ready_pose.launch.py execute:=false
```

Passing `execute:=true` moves the real robot and requires the same physical
safety checks as the main controller.

## Tests

```bash
source /opt/ros/humble/setup.bash
source ~/jaka_s5_ws/install/setup.bash
colcon test --packages-select jaka_s5_controller
colcon test-result --verbose
```

The Qt JOG controls target the real-robot SDK backend and are not a Gazebo
controller. See [the simulation checklist](docs/JAKA_S5_SIM_TEST.md) for checks
that do not command real hardware.

## License and SDK

The source is derived from the official JAKA ROS 2 repository and is provided
under the repository's Apache-2.0 license. The bundled `libjakaAPI.so` is the
vendor SDK binary required at runtime; review
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before redistribution.
