"""Hardware bringup — placeholder until M1.

Will compose:
    qmini_safety (must start first, in STATE_BOOTING)
    qmini_imu    (publishes /imu/data)
    qmini_hardware (4 motor-bus executors, one per RS-485 channel)
    qmini_controllers (thin PD packer)
    qmini_joystick (parses /joy)

DS4 pairing precondition + udev port pinning checks belong in this launch
file's pre-launch hook (M1).
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    raise NotImplementedError(
        "hardware.launch.py is a placeholder. Implement in M1 once qmini_hardware exists."
    )
