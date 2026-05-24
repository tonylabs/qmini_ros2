# Per-joint state reported back from the GO-M8010-6 bus.
# Length 10, same joint ordering as MotorCommand.

std_msgs/Header header

float64[10] q              # measured position [rad]
float64[10] dq             # measured velocity [rad/s]
float64[10] tau            # measured torque [N*m]
float32[10] temperature_c  # motor temperature [degC]
uint8[10]   error_flags    # per-motor error bitmask (Unitree SDK convention)
