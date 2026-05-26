"""M4 step 4 — actuator latency + effective-PD measurement.

    # dry run first (no torque: protocol + markers publish, motors freewheel):
    ros2 launch qmini_calibration actuator_latency_calib.launch.py
    # then for real, on the rope, deadman held:
    ros2 launch qmini_calibration actuator_latency_calib.launch.py enable_torque:=true
    ros2 launch qmini_calibration actuator_latency_calib.launch.py enable_torque:=true \
        test_joints:="['knee_pitch_l','hip_pitch_l']" gain_scale:=0.5

THE ROBOT MOVES UNDER TORQUE. Rope on, hold the deadman (LB+RB). Releasing the
deadman drops torque (qmini_hardware) AND pauses the protocol (calib node).

Brings up joystick + qmini_safety + qmini_hardware (torque gated) + the calib node
(which publishes /motor_command directly, in place of pd_packer) + a bag.
"""

import os
from datetime import date

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    run_dir = os.path.join(
        os.getcwd(), "src", "qmini_calibration", "data",
        f"{date.today().isoformat()}_actuator_latency")
    os.makedirs(run_dir, exist_ok=True)
    bag_dir = os.path.join(run_dir, "bag")

    gains = PathJoinSubstitution([FindPackageShare("qmini_controllers"), "config", "gains.yaml"])
    home = PathJoinSubstitution([FindPackageShare("qmini_hardware"), "config", "home_pose.yaml"])
    layout = PathJoinSubstitution([FindPackageShare("qmini_hardware"), "config", "motor_layout.yaml"])

    import ast
    test_joints = ast.literal_eval(LaunchConfiguration("test_joints").perform(context))

    joy = Node(package="joy", executable="joy_node", name="joy_node", output="screen")
    safety = Node(package="qmini_safety", executable="safety_node", name="qmini_safety",
                  output="screen")
    motor_bus = Node(
        package="qmini_hardware", executable="motor_bus_node", name="motor_bus_node",
        output="screen",
        parameters=[{
            "layout_file": layout,
            "allow_missing_channels": True,
            "enable_motor_torque": ParameterValue(
                LaunchConfiguration("enable_torque"), value_type=bool),
        }])
    calib = Node(
        package="qmini_calibration", executable="actuator_latency_calib",
        name="actuator_latency_calib", output="screen",
        parameters=[{
            "home_pose_file": home,
            "gains_file": gains,
            "test_joints": test_joints,
            "step_rad": float(LaunchConfiguration("step_rad").perform(context)),
            "gain_scale": float(LaunchConfiguration("gain_scale").perform(context)),
            "output_dir": run_dir,
        }])
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_dir,
             "/motor_command", "/joint_states", "/calibration/event"],
        output="screen")

    return [joy, safety, motor_bus, calib, bag]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("enable_torque", default_value="false",
                              description="false = dry run (no torque). true = motors move."),
        DeclareLaunchArgument("test_joints", default_value="['knee_pitch_l']",
                              description="Python list of joint names to step, one at a time."),
        DeclareLaunchArgument("step_rad", default_value="0.08"),
        DeclareLaunchArgument("gain_scale", default_value="1.0",
                              description="Scale on the nominal kp/kd used for the step."),
        OpaqueFunction(function=_setup),
    ])
