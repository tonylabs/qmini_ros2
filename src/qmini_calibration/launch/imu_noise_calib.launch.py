"""M4 step 2 — IMU noise-floor + mount measurement.

    ros2 launch qmini_calibration imu_noise_calib.launch.py
    ros2 launch qmini_calibration imu_noise_calib.launch.py duration_s:=60.0

Brings up ONLY the IMU driver (no motor bus, no policy), records a bag, and runs
the measurement node. Robot must be STATIONARY on a LEVEL surface. When the node
finishes it prints a summary and writes summary.yaml; run analysis/analyze_imu_bag.py
on the recorded bag to append the canonical row to calibration_results.yaml.
"""

import os
from datetime import date

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    run_dir = os.path.join(
        os.getcwd(), "src", "qmini_calibration", "data",
        f"{date.today().isoformat()}_imu_noise")
    os.makedirs(run_dir, exist_ok=True)
    bag_dir = os.path.join(run_dir, "bag")

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("qmini_imu"), "launch", "imu.launch.py"])),
    )

    calib = Node(
        package="qmini_calibration",
        executable="imu_noise_calib",
        name="imu_noise_calib",
        output="screen",
        parameters=[{
            "settle_s": float(LaunchConfiguration("settle_s").perform(context)),
            "duration_s": float(LaunchConfiguration("duration_s").perform(context)),
            "output_dir": run_dir,
        }],
    )

    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_dir, "/imu/data", "/calibration/event"],
        output="screen",
    )

    return [imu_launch, bag, calib]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("settle_s", default_value="3.0",
                              description="Seconds to discard before collecting."),
        DeclareLaunchArgument("duration_s", default_value="30.0",
                              description="Static collection window (seconds)."),
        OpaqueFunction(function=_setup),
    ])
