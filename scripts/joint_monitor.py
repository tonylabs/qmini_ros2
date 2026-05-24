#!/usr/bin/env python3
"""Live, in-place /joint_states monitor (no console scrolling).

Subscribes once and rewrites the same block of lines at a fixed refresh rate,
so you can move a joint and watch its angle update in place — handy for the
homing sign/ratio check (see docs/2-HOMING.md).

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 scripts/joint_monitor.py                 # default 10 Hz, /joint_states
    python3 scripts/joint_monitor.py --rate 20
    python3 scripts/joint_monitor.py --topic /joint_states

Press Ctrl-C to quit.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointMonitor(Node):
    def __init__(self, topic: str, rate_hz: float):
        super().__init__("joint_monitor")
        # /joint_states is published BEST_EFFORT by the driver — match it, or the
        # subscription silently never fires.
        self.create_subscription(
            JointState, topic, self._on_msg, qos_profile_sensor_data
        )
        self._min_period = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self._last_draw = 0.0
        self._lines_drawn = 0
        self._topic = topic
        sys.stdout.write(f"Watching {topic} (Ctrl-C to quit)\n")

    def _on_msg(self, msg: JointState):
        now = time.monotonic()
        if now - self._last_draw < self._min_period:
            return  # throttle redraws for readability
        self._last_draw = now

        rows = []
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else float("nan")
            vel = msg.velocity[i] if i < len(msg.velocity) else float("nan")
            rows.append(f"  {name:<15s} {pos:+9.4f} rad   ({vel:+7.3f} rad/s)")

        # Move the cursor back up over the previously drawn block, then reprint.
        if self._lines_drawn:
            sys.stdout.write(f"\033[{self._lines_drawn}A")
        sys.stdout.write("\033[J")  # clear from cursor to end of screen
        sys.stdout.write("\n".join(rows) + "\n")
        sys.stdout.flush()
        self._lines_drawn = len(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument("--rate", type=float, default=10.0, help="redraw rate (Hz)")
    args = ap.parse_args()

    rclpy.init()
    node = JointMonitor(args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
