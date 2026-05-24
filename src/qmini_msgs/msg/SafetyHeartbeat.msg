# Heartbeat from qmini_safety to qmini_hardware. Published at >=50 Hz on a
# RELIABLE + TRANSIENT_LOCAL topic. qmini_hardware MUST stop applying torque
# if it doesn't see one of these within ~100 ms.
#
# qmini_safety is the SOLE publisher. No other node may emit this message.

std_msgs/Header header

uint32 sequence            # monotonic counter; missed numbers indicate drops
builtin_interfaces/Time origin_stamp  # when the safety node generated this (vs header.stamp which may be later)
