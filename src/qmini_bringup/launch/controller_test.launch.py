"""M2.1 controller test — PD packer, NO TORQUE.

Brings up the joystick, qmini_safety (deadman gate), the pd_packer_node, and
(optionally) the read-only motor bus so /joint_states is real. Nothing
subscribes to /motor_command yet (that is M2.2), so this is completely safe:
the controller only *publishes* command packets you can inspect.

What to verify (in another terminal):

    ros2 topic echo /motor_command

  - With the deadman released: NO messages (controller is silent).
  - Hold BOTH LB+RB (buttons 4 and 5): MotionGate -> ENABLED and /motor_command
    starts publishing at ~50 Hz. q_des ramps smoothly from the current pose to
    the home/standing pose over ~3 s (watch it ease in), then holds home.
  - kp/kd fields are the joint-side gains from qmini_controllers/config/gains.yaml.
  - Every q_des stays within the per-joint position limits.
  - Release the deadman: publishing stops again.

Set use_hardware:=false for a bench test with no robot connected (the ramp then
starts from the home pose since there is no /joint_states).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gains_file = PathJoinSubstitution(
        [FindPackageShare("qmini_controllers"), "config", "gains.yaml"]
    )
    home_pose_file = PathJoinSubstitution(
        [FindPackageShare("qmini_hardware"), "config", "home_pose.yaml"]
    )
    layout_file = PathJoinSubstitution(
        [FindPackageShare("qmini_hardware"), "config", "motor_layout.yaml"]
    )

    log_level = DeclareLaunchArgument("log_level", default_value="info")
    use_hardware = DeclareLaunchArgument(
        "use_hardware",
        default_value="true",
        description=(
            "If true, also launch the motor bus so /joint_states is real and the "
            "ramp starts from the actual pose."
        ),
    )
    # *** ENERGIZING SWITCH (M2.3) *** Default false = read-only (M2.1/M2.2 test,
    # no torque). Set true ONLY when ready to drive the motors: holding LB+RB will
    # then ramp the robot to the home/standing crouch under full PD. Confirm
    # homing is valid and the robot is rope-suspended first (see docs).
    enable_torque = DeclareLaunchArgument(
        "enable_torque",
        default_value="false",
        description="If true, the motor bus applies torque when MotionGate=ENABLED.",
    )
    gain_scale = DeclareLaunchArgument(
        "gain_scale",
        default_value="1.0",
        description="Scale on motor kp/kd (1.0 = full Isaac Lab gains).",
    )

    return LaunchDescription([
        log_level,
        use_hardware,
        enable_torque,
        gain_scale,
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),
        Node(
            package="qmini_safety",
            executable="safety_node",
            name="qmini_safety",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="qmini_controllers",
            executable="pd_packer_node",
            name="pd_packer_node",
            output="screen",
            parameters=[{
                "gains_file": gains_file,
                "home_pose_file": home_pose_file,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
        Node(
            package="qmini_hardware",
            executable="motor_bus_node",
            name="motor_bus_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_hardware")),
            parameters=[{
                "layout_file": layout_file,
                "allow_missing_channels": True,
                "enable_motor_torque": ParameterValue(
                    LaunchConfiguration("enable_torque"), value_type=bool),
                "command_gain_scale": ParameterValue(
                    LaunchConfiguration("gain_scale"), value_type=float),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ])
