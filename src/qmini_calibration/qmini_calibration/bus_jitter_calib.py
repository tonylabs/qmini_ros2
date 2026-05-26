"""Bus polling-rate + loop-jitter measurement node (M4, step 3).

Measures the cadence at which the motor bus delivers state to its consumers —
the effective control-loop tick the PD packer and policy see. This caps the
achievable `decimation` on the sim side (policy rate = 1/(sim.dt*decimation)).

Brings up qmini_hardware read-only (no torque); the robot does not move. Over a
static window it collects /joint_states and reports, from the DRIVER-stamped
header times (honouring "timestamps come from the driver layer"):

  * publish rate (mean) and jitter (std of the period), min/max, p95/p99
  * dropped-tick count (period > 2x median)
  * per-channel freshness: fraction of samples where a channel's joints are NaN
    (channel down, port not open, or a stale/failed poll)

The aggregated /joint_states cannot expose true per-channel poll rates (the node
publishes one combined message), so per-channel health is inferred from NaNs.

HARD RULE: refuses to start if the policy node is running.
"""

import math
import os
from datetime import date

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

GUARD_NODES = ("policy_runner_node",)

# joint -> RS-485 channel (from motor_layout.yaml channel grouping)
CHANNEL_OF = {
    "hip_yaw_l": "ch1_hip_yaw", "hip_yaw_r": "ch1_hip_yaw",
    "hip_roll_l": "ch2_hip_roll", "hip_roll_r": "ch2_hip_roll",
    "hip_pitch_l": "ch3_left_lower", "knee_pitch_l": "ch3_left_lower",
    "ankle_pitch_l": "ch3_left_lower",
    "hip_pitch_r": "ch4_right_lower", "knee_pitch_r": "ch4_right_lower",
    "ankle_pitch_r": "ch4_right_lower",
}
CHANNELS = ["ch1_hip_yaw", "ch2_hip_roll", "ch3_left_lower", "ch4_right_lower"]


def _mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def _std(xs):
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def _pct(xs, p):
    if not xs:
        return 0.0
    s = sorted(xs)
    k = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


class BusJitterCalib(Node):
    def __init__(self):
        super().__init__("bus_jitter_calib")
        self.declare_parameter("settle_s", 3.0)
        self.declare_parameter("duration_s", 30.0)
        self.declare_parameter("output_dir", "")
        self.settle_s = self.get_parameter("settle_s").value
        self.duration_s = self.get_parameter("duration_s").value
        out_dir = self.get_parameter("output_dir").value or os.path.join(
            os.getcwd(), "src", "qmini_calibration", "data",
            f"{date.today().isoformat()}_bus_jitter")
        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)

        clash = set(self.get_node_names()).intersection(GUARD_NODES)
        if clash:
            self.get_logger().fatal(
                f"Refusing to start: policy/forbidden node(s) running: {sorted(clash)}.")
            raise SystemExit(1)

        self.event_pub = self.create_publisher(String, "calibration/event", 10)
        self.sub = self.create_subscription(
            JointState, "joint_states", self.on_js, qos_profile_sensor_data)

        self.stamps = []           # driver header stamps (s) within window
        self.ch_nan = {c: 0 for c in CHANNELS}
        self.n = 0

        self.phase = "settle"
        self.t0 = self.get_clock().now()
        self.t_collect = None
        self.timer = self.create_timer(0.1, self.tick)
        self._emit("settle_begin")
        self.get_logger().info(
            f"Bus jitter calib: robot read-only/stationary. Settling "
            f"{self.settle_s:.0f}s then collecting {self.duration_s:.0f}s of /joint_states.")

    def _emit(self, label):
        self.event_pub.publish(String(data=label))
        self.get_logger().info(f"[event] {label}")

    def on_js(self, msg: JointState):
        if self.phase != "collect":
            return
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        # per-channel NaN/stale: a channel is "bad" this sample if ANY of its
        # joints is NaN (driver marks down/stale channels NaN)
        bad = {c: False for c in CHANNELS}
        for name, pos in zip(msg.name, msg.position):
            ch = CHANNEL_OF.get(name)
            if ch is not None and (math.isnan(pos)):
                bad[ch] = True
        for c in CHANNELS:
            if bad[c]:
                self.ch_nan[c] += 1
        self.n += 1

    def tick(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if self.phase == "settle" and elapsed >= self.settle_s:
            self.phase = "collect"
            self.t_collect = self.get_clock().now()
            self._emit("static_begin")
        elif self.phase == "collect":
            if (self.get_clock().now() - self.t_collect).nanoseconds * 1e-9 >= self.duration_s:
                self.phase = "done"
                self._emit("static_end")
                self.finish()

    def finish(self):
        if self.n < 10:
            self.get_logger().error(
                f"Only {self.n} /joint_states samples — is motor_bus_node running?")
            rclpy.shutdown()
            return
        periods = [1000.0 * (self.stamps[i] - self.stamps[i - 1])
                   for i in range(1, len(self.stamps))]  # ms
        periods = [p for p in periods if p > 0]
        med = _pct(periods, 50)
        dropped = sum(1 for p in periods if p > 2 * med) if med > 0 else 0
        mean_p = _mean(periods)
        rate = 1000.0 / mean_p if mean_p > 0 else float("nan")

        summary = {
            "measurement": "bus_jitter",
            "date": date.today().isoformat(),
            "n_samples": self.n,
            "rate_hz_mean": round(rate, 2),
            "period_ms_mean": round(mean_p, 4),
            "period_ms_std": round(_std(periods), 4),
            "period_ms_min": round(min(periods), 4),
            "period_ms_max": round(max(periods), 4),
            "period_ms_p95": round(_pct(periods, 95), 4),
            "period_ms_p99": round(_pct(periods, 99), 4),
            "dropped_ticks": dropped,
            "channel_nan_fraction": {
                c: round(self.ch_nan[c] / self.n, 4) for c in CHANNELS},
        }
        path = os.path.join(self.out_dir, "summary.yaml")
        with open(path, "w") as f:
            yaml.safe_dump(summary, f, sort_keys=False)

        self.get_logger().info(
            "\n==== Bus jitter summary ====\n"
            f"  samples {self.n}\n"
            f"  rate {rate:.1f} Hz  (period mean {mean_p:.3f} ms, std {_std(periods):.3f} ms)\n"
            f"  period min/max {min(periods):.3f}/{max(periods):.3f} ms, "
            f"p95 {_pct(periods,95):.3f}, p99 {_pct(periods,99):.3f} ms\n"
            f"  dropped ticks (>2x median): {dropped}\n"
            f"  channel NaN fraction: {summary['channel_nan_fraction']}\n"
            f"  wrote {path}\n"
            "  (run diff_against_isaaclab.py: rate must exceed the policy rate "
            "1/(sim.dt*decimation) with margin)\n"
            "============================")
        rclpy.shutdown()


def main():
    rclpy.init()
    try:
        node = BusJitterCalib()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
