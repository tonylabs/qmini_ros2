"""View the Qmini URDF in RViz2.

No hardware. No policy. Just `robot_state_publisher` + `joint_state_publisher_gui`
+ `rviz2` so you can sanity-check the URDF and meshes after a fresh build:

    colcon build --symlink-install --packages-select \\
        qmini_msgs qmini_description qmini_bringup
    source install/setup.bash
    ros2 launch qmini_bringup view_robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("qmini_description"), "urdf", "qmini.urdf"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("qmini_description"), "rviz", "view_robot.rviz"]
    )

    # Force string interpretation — the URDF is XML, not YAML, so
    # ROS 2 launch's default YAML parser would reject it.
    robot_description = ParameterValue(
        Command(["cat ", urdf_file]), value_type=str
    )

    use_joint_gui = DeclareLaunchArgument(
        "use_joint_gui",
        default_value="true",
        description="Run joint_state_publisher_gui so the URDF is animatable in RViz.",
    )

    return LaunchDescription([
        use_joint_gui,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_joint_gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
