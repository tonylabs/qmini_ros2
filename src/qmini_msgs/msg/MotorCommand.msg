# Per-joint command packet matching the GO-M8010-6 MotorCmd struct.
# The motor closes the PD loop in firmware using these gains; the ROS 2 side
# is a packer, not a software PD loop.
#
# All arrays are length 10 (one entry per joint, ordered as in qmini.urdf:
#   hip_yaw_l, hip_roll_l, hip_pitch_l, knee_pitch_l, ankle_pitch_l,
#   hip_yaw_r, hip_roll_r, hip_pitch_r, knee_pitch_r, ankle_pitch_r).
# This ordering MUST match the policy's action ordering.

std_msgs/Header header

# Desired joint position [rad]. NaN means "leave previous q_des".
float64[10] q_des

# Desired joint velocity feed-forward [rad/s]. NaN means 0.
float64[10] dq_des

# Feed-forward torque [N*m]. NaN means 0.
float64[10] tau_ff

# Position gain [N*m/rad]. NaN means "use config default".
float64[10] kp

# Velocity gain [N*m*s/rad]. NaN means "use config default".
float64[10] kd
