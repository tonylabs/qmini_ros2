"""Hardware bringup — the standing / teleop stack (no policy).

Composes the realtime + safety + teleop nodes that M2/M3 validated:

    joy_node              (raw /joy)
    qmini_safety          (deadman gate -> /safety/motion_gate + heartbeat)
    qmini_controllers     (pd_packer_node: /joint_target|home -> /motor_command)
    qmini_imu             (/imu/data)          [use_hardware:=true]
    qmini_hardware        (motor_bus_node, 4 RS-485 channels)  [use_hardware:=true]

This is the "hold home / teleop" configuration — nothing here runs the ONNX
policy. `qmini_bringup policy.launch.py` includes this and adds the policy runner.

Safety defaults are conservative, matching controller_test.launch.py:
  * enable_torque:=false  — motors do NOT apply torque until you opt in.
  * Holding the deadman (LB+RB) ramps q_des to the home crouch under PD.

    ros2 launch qmini_bringup hardware.launch.py                  # read-only
    ros2 launch qmini_bringup hardware.launch.py enable_torque:=true   # on the rope
    ros2 launch qmini_bringup hardware.launch.py use_hardware:=false   # bench, no robot

DS4 pairing precondition + udev port pinning checks belong in a pre-launch hook
here (future work); for now the launch assumes ports are pinned (see docs/0,1,6).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gains_file = PathJoinSubstitution(
        [FindPackageShare("qmini_controllers"), "config", "gains.yaml"])
    home_pose_file = PathJoinSubstitution(
        [FindPackageShare("qmini_hardware"), "config", "home_pose.yaml"])
    layout_file = PathJoinSubstitution(
        [FindPackageShare("qmini_hardware"), "config", "motor_layout.yaml"])
    imu_launch = PathJoinSubstitution(
        [FindPackageShare("qmini_imu"), "launch", "imu.launch.py"])

    use_hardware = LaunchConfiguration("use_hardware")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument(
            "use_hardware", default_value="true",
            description="Launch the IMU + motor bus (real robot). false = bench."),
        # *** ENERGIZING SWITCH *** default false = no torque. Set true ONLY on the
        # rope with valid homing; holding LB+RB then drives to the home crouch.
        DeclareLaunchArgument(
            "enable_torque", default_value="false",
            description="Motor bus applies torque when MotionGate=ENABLED."),
        DeclareLaunchArgument(
            "gain_scale", default_value="1.0",
            description="Scale on motor kp/kd (1.0 = full Isaac Lab gains)."),

        Node(package="joy", executable="joy_node", name="joy_node", output="screen"),

        Node(
            package="qmini_safety", executable="safety_node", name="qmini_safety",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level]),

        Node(
            package="qmini_controllers", executable="pd_packer_node",
            name="pd_packer_node", output="screen",
            parameters=[{"gains_file": gains_file, "home_pose_file": home_pose_file}],
            arguments=["--ros-args", "--log-level", log_level]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
            condition=IfCondition(use_hardware)),

        Node(
            package="qmini_hardware", executable="motor_bus_node",
            name="motor_bus_node", output="screen",
            condition=IfCondition(use_hardware),
            parameters=[{
                "layout_file": layout_file,
                "allow_missing_channels": True,
                "enable_motor_torque": ParameterValue(
                    LaunchConfiguration("enable_torque"), value_type=bool),
                "command_gain_scale": ParameterValue(
                    LaunchConfiguration("gain_scale"), value_type=float),
            }],
            arguments=["--ros-args", "--log-level", log_level]),
    ])
