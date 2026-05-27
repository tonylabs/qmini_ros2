"""Foot-floor friction measurement node (M4, step 5).

Measures the static (and, best-effort, kinetic) friction coefficient between the
robot's foot sole and the floor — the quantity the Isaac Lab
`EventCfg.physics_material.static_friction_range` randomizes. This robot has no
foot force/torque sensor, so we use the classic inclined-plane test, made
objective by reusing the already-validated N100 IMU:

  * Rest the N100 on a weighted foot-SOLE sample sitting on a board (the same
    sole material + a representative normal load). The IMU rides ON the sliding
    sample, not on the board.
  * Tilt the board slowly. The IMU's projected gravity gives the tilt angle.
  * At breakaway the sample slides; the sample-mounted IMU registers a down-slope
    acceleration spike. The tilt angle at that instant is the breakaway angle:
        mu_static = tan(theta_breakaway)

No motors, no torque — the safest M4 measurement. The node detects slip from an
accel spike, emits a /calibration/event "slip_N" marker per trial, and prints a
live tilt readout. The AUTHORITATIVE mu is computed offline by
analysis/analyze_friction_bag.py from the recorded bag (one canonical writer) —
this node never writes calibration_results.yaml.

Protocol: tilt slowly; the moment it slides, STOP and return the board to level;
the node re-arms for the next trial. Repeat n_trials times.

HARD RULE: refuses to start if the policy node is running.
"""

import math
import os
from datetime import date

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

GUARD_NODES = ("policy_runner_node",)

GRAVITY = 9.80665


def quat_rotate_inverse(w, x, y, z, v):
    """Express world vector v in the body frame (matches Isaac Lab)."""
    qv = (x, y, z)
    w2 = 2.0 * w * w - 1.0
    a = [vi * w2 for vi in v]
    cross = (qv[1] * v[2] - qv[2] * v[1],
             qv[2] * v[0] - qv[0] * v[2],
             qv[0] * v[1] - qv[1] * v[0])
    dot = qv[0] * v[0] + qv[1] * v[1] + qv[2] * v[2]
    return [a[i] - 2.0 * w * cross[i] + 2.0 * dot * qv[i] for i in range(3)]


def tilt_from_quat(w, x, y, z):
    """Tilt of the IMU's +up axis from vertical, in degrees (0 = level)."""
    pg = quat_rotate_inverse(w, x, y, z, (0.0, 0.0, -1.0))
    norm = math.sqrt(sum(c * c for c in pg)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, -pg[2] / norm))))


class FrictionCalib(Node):
    def __init__(self):
        super().__init__("friction_calib")
        self.declare_parameter("n_trials", 5)
        self.declare_parameter("slip_accel_thresh", 1.5)   # m/s^2 above gravity = slip
        self.declare_parameter("min_tilt_deg", 5.0)        # ignore spikes below this tilt
        self.declare_parameter("rearm_tilt_deg", 3.0)      # board back near-level to re-arm
        self.declare_parameter("output_dir", "")
        self.n_trials = int(self.get_parameter("n_trials").value)
        self.slip_accel_thresh = float(self.get_parameter("slip_accel_thresh").value)
        self.min_tilt_deg = float(self.get_parameter("min_tilt_deg").value)
        self.rearm_tilt_deg = float(self.get_parameter("rearm_tilt_deg").value)
        out_dir = self.get_parameter("output_dir").value or os.path.join(
            os.getcwd(), "src", "qmini_calibration", "data",
            f"{date.today().isoformat()}_friction")
        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)

        clash = set(self.get_node_names()).intersection(GUARD_NODES)
        if clash:
            self.get_logger().fatal(
                f"Refusing to start: policy/forbidden node(s) running: {sorted(clash)}.")
            raise SystemExit(1)

        self.event_pub = self.create_publisher(String, "calibration/event", 10)
        self.sub = self.create_subscription(
            Imu, "imu/data", self.on_imu, qos_profile_sensor_data)

        self.state = "arming"      # arming -> ready -> (slip) -> arming
        self.trial = 0
        self.max_tilt = 0.0
        self.slip_angles = []
        self.last_log = 0.0

        self._emit("session_begin")
        self.get_logger().info(
            f"Friction calib: rest the N100 on a weighted foot-sole sample on a "
            f"board. Tilt SLOWLY until it slides, then return to level. "
            f"{self.n_trials} trials. Slip threshold {self.slip_accel_thresh} m/s^2.")

    def _emit(self, label):
        self.event_pub.publish(String(data=label))
        self.get_logger().info(f"[event] {label}")

    def on_imu(self, msg: Imu):
        o = msg.orientation
        tilt = tilt_from_quat(o.w, o.x, o.y, o.z)
        a = msg.linear_acceleration
        accel_excess = abs(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z) - GRAVITY)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log > 0.5:
            self.last_log = now
            self.get_logger().info(
                f"  [{self.state}] trial {self.trial + 1}/{self.n_trials}  "
                f"tilt {tilt:5.1f} deg  accel_excess {accel_excess:4.2f} m/s^2"
                + (f"  (max {self.max_tilt:.1f})" if self.state == "ready" else ""))

        if self.state == "arming":
            if tilt < self.rearm_tilt_deg:
                self.state = "ready"
                self.max_tilt = 0.0
        elif self.state == "ready":
            self.max_tilt = max(self.max_tilt, tilt)
            slipped = (tilt > self.min_tilt_deg
                       and accel_excess > self.slip_accel_thresh)
            if slipped:
                self.trial += 1
                self.slip_angles.append(self.max_tilt)
                mu = math.tan(math.radians(self.max_tilt))
                self._emit(f"slip_{self.trial}")
                self.get_logger().info(
                    f"  >>> SLIP at ~{self.max_tilt:.1f} deg  ->  mu_static ~= "
                    f"{mu:.3f}  (return board to level for the next trial)")
                if self.trial >= self.n_trials:
                    self.finish()
                else:
                    self.state = "arming"

    def finish(self):
        self._emit("session_end")
        mus = [math.tan(math.radians(a)) for a in self.slip_angles]
        mean_mu = sum(mus) / len(mus) if mus else float("nan")
        summary = {
            "measurement": "friction",
            "date": date.today().isoformat(),
            "n_trials": len(self.slip_angles),
            "slip_angles_deg": [round(a, 2) for a in self.slip_angles],
            "mu_static_per_trial": [round(m, 4) for m in mus],
            "mu_static_mean": round(mean_mu, 4),
        }
        path = os.path.join(self.out_dir, "summary.yaml")
        with open(path, "w") as f:
            yaml.safe_dump(summary, f, sort_keys=False)
        self.get_logger().info(
            "\n==== Friction summary ====\n"
            f"  trials {len(self.slip_angles)}\n"
            f"  breakaway angles (deg): {[round(a, 1) for a in self.slip_angles]}\n"
            f"  mu_static (= tan theta): mean {mean_mu:.3f}\n"
            f"  wrote {path}\n"
            "  (authoritative mu is from analyze_friction_bag.py on the bag;\n"
            "   diff vs EventCfg.static_friction_range with diff_against_isaaclab.py)\n"
            "==========================")
        rclpy.shutdown()


def main():
    rclpy.init()
    try:
        node = FrictionCalib()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
