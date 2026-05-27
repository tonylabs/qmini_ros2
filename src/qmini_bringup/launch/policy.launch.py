"""Full policy bringup (M5) — the walking stack, end to end.

Composes the hardware/teleop stack with the ONNX policy runner:

    hardware.launch.py    joy + safety + pd_packer + imu + motor_bus
    qmini_rl policy       policy_runner_node: obs[44] -> action[10]
                          -> q_des on /joint_target -> pd_packer -> motors

Data path once running:
    /imu/data + /joint_states + /cmd_vel + /safety/motion_gate
        -> policy_runner_node -> /joint_target
        -> pd_packer_node -> /motor_command -> motor_bus (4x RS-485)

Safety defaults are conservative:
  * gate_required:=true   — the policy only emits targets when MotionGate=ENABLED
                            (deadman held). Release the deadman -> targets stop.
  * enable_torque:=false  — motors stay passive until you opt in.

    # bring up the whole stack read-only first (no torque): watch /joint_target
    # and /motor_command while you move the robot / send /cmd_vel.
    ros2 launch qmini_bringup policy.launch.py

    # on the rope, deadman in hand, ready to drive:
    ros2 launch qmini_bringup policy.launch.py enable_torque:=true

    # bench test, no robot and no safety gate:
    ros2 launch qmini_bringup policy.launch.py use_hardware:=false gate_required:=false

The home pose is shared (qmini_hardware/config/home_pose.yaml) so the policy's
action offset (q_des = home + 0.5*action) matches the controller's home exactly.
RUN ON THE ROPE the first time — this is the first config that can both energize
the motors and drive them from the policy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare("qmini_bringup"), "launch", "hardware.launch.py"])
    policy_launch = PathJoinSubstitution(
        [FindPackageShare("qmini_rl"), "launch", "policy.launch.py"])

    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument(
            "use_hardware", default_value="true",
            description="Launch the IMU + motor bus (real robot). false = bench."),
        DeclareLaunchArgument(
            "enable_torque", default_value="false",
            description="Motor bus applies torque when MotionGate=ENABLED."),
        DeclareLaunchArgument(
            "gain_scale", default_value="1.0",
            description="Scale on motor kp/kd (1.0 = full Isaac Lab gains)."),
        DeclareLaunchArgument(
            "gate_required", default_value="true",
            description="Policy emits /joint_target only when MotionGate=ENABLED."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={
                "log_level": LaunchConfiguration("log_level"),
                "use_hardware": LaunchConfiguration("use_hardware"),
                "enable_torque": LaunchConfiguration("enable_torque"),
                "gain_scale": LaunchConfiguration("gain_scale"),
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(policy_launch),
            launch_arguments={
                "gate_required": LaunchConfiguration("gate_required"),
            }.items()),
    ])
