"""Run the ONNX policy runner (M5).

    ros2 launch qmini_rl policy.launch.py
    ros2 launch qmini_rl policy.launch.py gate_required:=false   # bench test

Subscribes: /imu/data, /joint_states, /cmd_vel, /safety/motion_gate.
Publishes:  /joint_target (sensor_msgs/JointState) -> pd_packer -> motors.

policy_path and home_pose_file default to the installed package paths; the home
pose is shared with qmini_hardware so the action offset matches the controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("qmini_rl"), "config", "policy.yaml"]
    )
    policy_path = PathJoinSubstitution(
        [FindPackageShare("qmini_rl"), "policies", "policy.onnx"]
    )
    home_pose = PathJoinSubstitution(
        [FindPackageShare("qmini_hardware"), "config", "home_pose.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gate_required",
            default_value="true",
            description="Only publish targets when MotionGate=ENABLED. "
            "Set false for bench tests without qmini_safety.",
        ),
        Node(
            package="qmini_rl",
            executable="policy_runner_node",
            name="policy_runner_node",
            output="screen",
            parameters=[
                config,
                {
                    "policy_path": policy_path,
                    "home_pose_file": home_pose,
                    "gate_required": LaunchConfiguration("gate_required"),
                },
            ],
        ),
    ])
