"""Bring up the Wheeltec N100 IMU driver.

    ros2 launch qmini_imu imu.launch.py
    ros2 launch qmini_imu imu.launch.py serial_port:=/dev/serial/by-id/usb-...

Publishes sensor_msgs/Imu on /imu/data. Standalone — no motor bus, no policy.
Useful on its own for the M4 IMU noise-floor / mount calibration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    config = PathJoinSubstitution(
        [FindPackageShare("qmini_imu"), "config", "imu.yaml"]
    )

    # config/imu.yaml is the source of truth. A non-empty serial_port arg
    # overrides just that one value; an empty arg leaves the file's value
    # untouched (don't append an empty override — it would clobber the port).
    params = [config]
    serial_port = LaunchConfiguration("serial_port").perform(context)
    if serial_port:
        params.append({"serial_port": serial_port})

    return [
        Node(
            package="qmini_imu",
            executable="imu_driver_node",
            name="imu_driver_node",
            output="screen",
            parameters=params,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="",
            description="Override the IMU serial port (else use config/imu.yaml). "
            "Prefer a /dev/serial/by-id/... path.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
