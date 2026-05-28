# Qmini ROS 2

ROS 2 Humble control stack for the Qmini bipedal robot. Designed to run on a
Raspberry Pi 5 (Ubuntu 22.04, 4 GB RAM). Companion the training repo [Qmini Isaac Lab](https://github.com/tonylabs/python_qmini_isaaclab).

## Packages

| Package | Build | Role |
|---|---|---|
| `qmini_description` | ament_cmake | URDF + meshes; canonical joint ordering |
| `qmini_msgs` | ament_cmake | MotorCommand/State, JoystickCommand, SafetyHeartbeat, MotionGate |
| `qmini_hardware` | ament_cmake (C++) | RS-485 motor bus driver; vendors `unitree_actuator_sdk` |
| `qmini_imu` | ament_python | Wheeltec N100 wrapper + stale-message watchdog |
| `qmini_controllers` | ament_cmake (C++) | Thin PD packer (motor closes the loop in firmware) |
| `qmini_rl` | ament_cmake (C++) | ONNX policy runner (44→10 dims, fixed 1.5 Hz gait phase) |
| `qmini_joystick` | ament_python | PS4 DualShock → JoystickCommand |
| `qmini_safety` | ament_cmake (C++) | Watchdog + state machine + heartbeat owner |
| `qmini_bringup` | ament_cmake | Launch composition |
| `qmini_calibration` | ament_python | Measurement nodes + offline analyzers (M4) |

## First build

```bash
# One-time per shell
source /opt/ros/humble/setup.bash

# Or add to the bashrc
source /opt/ros/humble/setup.bash
source "$HOME/qmini_ws/install/setup.bash"


# Install ROS deps declared in package.xml
rosdep install --from-paths src --ignore-src -r -y

# Vendor the Unitree SDK (see src/qmini_hardware/third_party/README.md)
git submodule add https://github.com/unitreerobotics/unitree_actuator_sdk src/qmini_hardware/third_party/unitree_actuator_sdk
git submodule update --init --recursive

# Build (M0 only needs these three packages to succeed)
colcon build --symlink-install --packages-select qmini_msgs qmini_description qmini_bringup

# Verify the URDF in RViz
source install/setup.bash
ros2 launch qmini_bringup view_robot.launch.py
```

## Common commands

```bash
# Build everything (M1+ once the C++/Python nodes exist)
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select qmini_controllers

# Tests
colcon test --packages-select qmini_controllers
colcon test-result --verbose

# Run the full system (M1+)
ros2 launch qmini_bringup hardware.launch.py
```
