"""M4 step 5 — foot-floor friction (inclined-plane test).

    ros2 launch qmini_calibration friction_calib.launch.py
    ros2 launch qmini_calibration friction_calib.launch.py n_trials:=8

Brings up ONLY the IMU driver (no motor bus, no torque, no policy) and records a
bag. Rest the N100 on a weighted foot-sole sample on a board; tilt slowly until
it slides; the node detects the slip and marks it. Run
analysis/analyze_friction_bag.py on the recorded bag to append the canonical row
to calibration_results.yaml, then diff_against_isaaclab.py.
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
        f"{date.today().isoformat()}_friction")
    os.makedirs(run_dir, exist_ok=True)
    bag_dir = os.path.join(run_dir, "bag")

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("qmini_imu"), "launch", "imu.launch.py"])),
    )

    calib = Node(
        package="qmini_calibration",
        executable="friction_calib",
        name="friction_calib",
        output="screen",
        parameters=[{
            "n_trials": int(LaunchConfiguration("n_trials").perform(context)),
            "slip_accel_thresh": float(
                LaunchConfiguration("slip_accel_thresh").perform(context)),
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
        DeclareLaunchArgument("n_trials", default_value="5",
                              description="Number of slip trials to collect."),
        DeclareLaunchArgument("slip_accel_thresh", default_value="1.5",
                              description="Accel above gravity (m/s^2) that counts as slip."),
        OpaqueFunction(function=_setup),
    ])
