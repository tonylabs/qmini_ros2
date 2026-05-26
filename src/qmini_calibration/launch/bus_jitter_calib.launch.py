"""M4 step 3 — bus polling-rate + loop-jitter measurement.

    ros2 launch qmini_calibration bus_jitter_calib.launch.py
    ros2 launch qmini_calibration bus_jitter_calib.launch.py duration_s:=60.0

Brings up qmini_safety + qmini_hardware's motor_bus_node READ-ONLY (no torque),
records a bag, and runs the measurement node. The robot does not move. Run this
on the Pi 5 (the target host) — bus rate/jitter is host-specific.
"""

import os
from datetime import date

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    run_dir = os.path.join(
        os.getcwd(), "src", "qmini_calibration", "data",
        f"{date.today().isoformat()}_bus_jitter")
    os.makedirs(run_dir, exist_ok=True)
    bag_dir = os.path.join(run_dir, "bag")

    layout = PathJoinSubstitution(
        [FindPackageShare("qmini_hardware"), "config", "motor_layout.yaml"])

    safety = Node(
        package="qmini_safety", executable="safety_node", name="qmini_safety",
        output="screen")
    # read-only: enable_motor_torque defaults false; we never command torque.
    motor_bus = Node(
        package="qmini_hardware", executable="motor_bus_node", name="motor_bus_node",
        output="screen",
        parameters=[{"layout_file": layout, "allow_missing_channels": True}])
    calib = Node(
        package="qmini_calibration", executable="bus_jitter_calib",
        name="bus_jitter_calib", output="screen",
        parameters=[{
            "settle_s": float(LaunchConfiguration("settle_s").perform(context)),
            "duration_s": float(LaunchConfiguration("duration_s").perform(context)),
            "output_dir": run_dir,
        }])
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_dir, "/joint_states", "/calibration/event"],
        output="screen")

    return [safety, motor_bus, bag, calib]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("settle_s", default_value="3.0"),
        DeclareLaunchArgument("duration_s", default_value="30.0"),
        OpaqueFunction(function=_setup),
    ])
