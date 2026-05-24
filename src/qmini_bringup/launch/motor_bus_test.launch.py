"""M1 motor bus smoke test.

Brings up qmini_safety (publishes the heartbeat and a HARD_STOPPED gate)
plus qmini_hardware's motor_bus_node (read-only, torque-disabled). No
joystick, no policy.

After launch, in another terminal:

    ros2 topic echo /joint_states

then hand-rotate one joint — the corresponding position field should change.

If a channel can't open its serial port the node will warn and continue with
the rest; the missing motors show up as NaN in /joint_states.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_layout = PathJoinSubstitution(
        [FindPackageShare("qmini_hardware"), "config", "motor_layout.yaml"]
    )

    layout_arg = DeclareLaunchArgument(
        "layout_file",
        default_value=default_layout,
        description="Path to motor_layout.yaml.",
    )
    log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="rclcpp log level for both nodes.",
    )
    allow_missing = DeclareLaunchArgument(
        "allow_missing_channels",
        default_value="true",
        description=(
            "If true, the node logs a warning when a channel's serial port "
            "can't be opened and keeps running. Set false to fail-fast."
        ),
    )

    return LaunchDescription([
        layout_arg,
        log_level,
        allow_missing,
        Node(
            package="qmini_safety",
            executable="safety_node",
            name="qmini_safety",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="qmini_hardware",
            executable="motor_bus_node",
            name="motor_bus_node",
            output="screen",
            parameters=[{
                "layout_file": LaunchConfiguration("layout_file"),
                "allow_missing_channels": LaunchConfiguration("allow_missing_channels"),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ])
