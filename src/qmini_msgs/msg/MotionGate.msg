# Canonical safety-state latch published by qmini_safety. Consumed by
# qmini_rl (gates inference) and qmini_controllers (gates command emission).
# Published as RELIABLE + TRANSIENT_LOCAL so late subscribers immediately
# learn the current state.

std_msgs/Header header

# One of the STATE_* constants. STATE_ENABLED is the only state in which
# the policy may publish actions and qmini_controllers may emit motor commands.
uint8 state
uint8 STATE_ENABLED = 0          # motion permitted
uint8 STATE_SOFT_STOPPED = 1     # hold last position under firmware PD
uint8 STATE_HARD_STOPPED = 2     # torque off; latched until explicit release
uint8 STATE_BOOTING = 3          # initial state before first /joy seen

# Human-readable reason for the most recent transition. For logs/UI; not
# load-bearing for downstream behavior.
string reason
