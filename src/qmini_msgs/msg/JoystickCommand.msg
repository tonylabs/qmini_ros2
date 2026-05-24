# Parsed PS4 DualShock input from qmini_joystick. The safety state machine
# (qmini_safety) is the only legitimate consumer of this message.

std_msgs/Header header

# Velocity command, raw analog stick output mapped into trained range.
# Final clamp by qmini_safety's training-wheels speed cap.
float32 vx              # m/s, +forward
float32 vy              # m/s, +left
float32 wz              # rad/s, +counter-clockwise

# True while L1 is held. Required for motion to be enabled. Releasing it
# does NOT disable torque — it zeros the velocity command.
bool deadman

# One of the MODE_* constants.
uint8 mode
uint8 MODE_UNKNOWN = 0
uint8 MODE_HOLD_HOME = 1   # Triangle pressed
uint8 MODE_WALKING = 2     # X pressed

# One of the STOP_* constants. Latched at the joystick layer; the
# safety node owns the canonical latched state.
uint8 stop_event
uint8 STOP_NONE = 0
uint8 STOP_SOFT_REQUESTED = 1   # Options pressed
uint8 STOP_HARD_REQUESTED = 2   # PS + Options held >=0.5s
uint8 STOP_RELEASE_REQUESTED = 3 # L2+R2 held >=1s

# DualShock battery state. NaN if unknown (most BT stacks expose 0.0-1.0).
float32 battery_fraction
