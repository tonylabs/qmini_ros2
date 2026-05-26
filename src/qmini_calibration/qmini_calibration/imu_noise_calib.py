"""IMU noise-floor + mount measurement node (M4, step 2).

Robot stationary on a LEVEL surface — the IMU does not need to move. Collects a
static window of /imu/data and reports:

  * gyro noise floor (per-axis std) and bias (per-axis mean)
  * accel noise floor + gravity magnitude
  * projected-gravity noise (std of quat_rotate_inverse(q, (0,0,-1)))
  * IMU mount tilt from vertical (requires the base to actually be level)

These feed the Isaac Lab diff: gyro/proj-gravity noise vs the PolicyCfg Unoise
ranges, and the measured mount vs ImuCfg.OffsetCfg. The node emits
/calibration/event phase markers so a recorded bag can be segmented offline, and
writes a per-run summary.yaml for immediate feedback. The AUTHORITATIVE row in
calibration_results.yaml is appended by the offline analyzer from the bag
(analysis/analyze_imu_bag.py) — this node never writes that file.

HARD RULE: refuses to start if the policy node is running (policy + a
calibration sweep must never share the robot).
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

# Node names that must NOT be running during a calibration sweep.
GUARD_NODES = ("policy_runner_node",)

# Isaac Lab reference (for inline context only — the authoritative green/red diff
# is analysis/diff_against_isaaclab.py, which imports the cfg directly).
REF_ANG_VEL_NOISE = 0.35   # Unoise half-range on imu_ang_vel (scaled obs units)
REF_ANG_VEL_SCALE = 0.2    # imu_ang_vel obs scale
REF_PROJ_GRAV_NOISE = 0.1  # Unoise half-range on imu_projected_gravity


def quat_rotate_inverse(w, x, y, z, v):
    """Express world vector v in the body frame (matches Isaac Lab)."""
    qv = (x, y, z)
    w2 = 2.0 * w * w - 1.0
    a = [vi * w2 for vi in v]
    cross = (
        qv[1] * v[2] - qv[2] * v[1],
        qv[2] * v[0] - qv[0] * v[2],
        qv[0] * v[1] - qv[1] * v[0],
    )
    dot = qv[0] * v[0] + qv[1] * v[1] + qv[2] * v[2]
    return [a[i] - 2.0 * w * cross[i] + 2.0 * dot * qv[i] for i in range(3)]


def _mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def _std(xs):
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


class ImuNoiseCalib(Node):
    def __init__(self):
        super().__init__("imu_noise_calib")
        self.declare_parameter("settle_s", 3.0)
        self.declare_parameter("duration_s", 30.0)
        self.declare_parameter("output_dir", "")
        self.settle_s = self.get_parameter("settle_s").value
        self.duration_s = self.get_parameter("duration_s").value
        out_dir = self.get_parameter("output_dir").value
        if not out_dir:
            out_dir = os.path.join(
                os.getcwd(), "src", "qmini_calibration", "data",
                f"{date.today().isoformat()}_imu_noise")
        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)

        # --- HARD RULE: policy must not be running ---
        running = set(self.get_node_names())
        clash = running.intersection(GUARD_NODES)
        if clash:
            self.get_logger().fatal(
                f"Refusing to start: policy/forbidden node(s) running: {sorted(clash)}. "
                "A calibration sweep and the policy must never share the robot.")
            raise SystemExit(1)

        self.event_pub = self.create_publisher(String, "calibration/event", 10)
        self.sub = self.create_subscription(
            Imu, "imu/data", self.on_imu, qos_profile_sensor_data)

        self.gyro = [[], [], []]
        self.accel = [[], [], []]
        self.projg = [[], [], []]
        self.n = 0
        self.first_stamp = None
        self.last_stamp = None

        self.phase = "settle"
        self.t0 = self.get_clock().now()
        self.t_collect = None
        self.timer = self.create_timer(0.1, self.tick)

        self._emit("settle_begin")
        self.get_logger().info(
            f"IMU noise/mount calib starting. Place the robot STATIONARY on a "
            f"LEVEL surface, IMU powered, untouched. Settling {self.settle_s:.0f}s, "
            f"then collecting {self.duration_s:.0f}s.")

    def _emit(self, label):
        self.event_pub.publish(String(data=label))
        self.get_logger().info(f"[event] {label}")

    def on_imu(self, msg: Imu):
        if self.phase != "collect":
            return
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self.gyro[0].append(g.x); self.gyro[1].append(g.y); self.gyro[2].append(g.z)
        self.accel[0].append(a.x); self.accel[1].append(a.y); self.accel[2].append(a.z)
        o = msg.orientation
        pg = quat_rotate_inverse(o.w, o.x, o.y, o.z, (0.0, 0.0, -1.0))
        for i in range(3):
            self.projg[i].append(pg[i])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp
        self.n += 1

    def tick(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if self.phase == "settle" and elapsed >= self.settle_s:
            self.phase = "collect"
            self.t_collect = self.get_clock().now()
            self._emit("static_begin")
        elif self.phase == "collect":
            dt = (self.get_clock().now() - self.t_collect).nanoseconds * 1e-9
            if dt >= self.duration_s:
                self.phase = "done"
                self._emit("static_end")
                self.finish()

    def finish(self):
        if self.n < 10:
            self.get_logger().error(
                f"Only {self.n} IMU samples collected — is qmini_imu running and "
                "publishing /imu/data?")
            rclpy.shutdown()
            return

        gyro_mean = [_mean(self.gyro[i]) for i in range(3)]
        gyro_std = [_std(self.gyro[i]) for i in range(3)]
        accel_mean = [_mean(self.accel[i]) for i in range(3)]
        accel_std = [_std(self.accel[i]) for i in range(3)]
        projg_mean = [_mean(self.projg[i]) for i in range(3)]
        projg_std = [_std(self.projg[i]) for i in range(3)]
        g_mag = math.sqrt(sum(m * m for m in accel_mean))
        rate = (self.n - 1) / (self.last_stamp - self.first_stamp) \
            if self.last_stamp and self.last_stamp > self.first_stamp else float("nan")
        # tilt of measured gravity from vertical (base assumed level)
        tilt_deg = float("nan")
        if g_mag > 1e-6:
            cos_t = abs(accel_mean[2]) / g_mag
            tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_t))))

        summary = {
            "measurement": "imu_noise_mount",
            "date": date.today().isoformat(),
            "n_samples": self.n,
            "rate_hz": round(rate, 2),
            "gyro_bias_rad_s": [round(v, 6) for v in gyro_mean],
            "gyro_noise_std_rad_s": [round(v, 6) for v in gyro_std],
            "accel_mean_m_s2": [round(v, 4) for v in accel_mean],
            "accel_noise_std_m_s2": [round(v, 4) for v in accel_std],
            "gravity_magnitude_m_s2": round(g_mag, 4),
            "proj_gravity_mean": [round(v, 5) for v in projg_mean],
            "proj_gravity_noise_std": [round(v, 5) for v in projg_std],
            "mount_tilt_from_vertical_deg": round(tilt_deg, 3),
        }
        path = os.path.join(self.out_dir, "summary.yaml")
        with open(path, "w") as f:
            yaml.safe_dump(summary, f, sort_keys=False)

        # immediate feedback (authoritative diff is the separate script)
        worst_gyro_obs = max(gyro_std) * REF_ANG_VEL_SCALE
        worst_projg = max(projg_std)
        self.get_logger().info(
            "\n==== IMU noise/mount summary ====\n"
            f"  samples {self.n} @ ~{rate:.1f} Hz\n"
            f"  gyro bias   (rad/s): {self._fmt(gyro_mean)}\n"
            f"  gyro noise  std    : {self._fmt(gyro_std)}\n"
            f"  accel mean (m/s^2) : {self._fmt(accel_mean)}  |g|={g_mag:.3f}\n"
            f"  proj-gravity mean  : {self._fmt(projg_mean)}\n"
            f"  proj-gravity std   : {self._fmt(projg_std)}\n"
            f"  mount tilt from vertical: {tilt_deg:.2f} deg (base must be level)\n"
            f"  -- vs Isaac Lab (context; run diff script for authoritative) --\n"
            f"  ang_vel noise in obs units (std*{REF_ANG_VEL_SCALE}) = {worst_gyro_obs:.4f} "
            f"vs DR +/-{REF_ANG_VEL_NOISE}\n"
            f"  proj-gravity noise = {worst_projg:.4f} vs DR +/-{REF_PROJ_GRAV_NOISE}\n"
            f"  wrote {path}\n"
            "=================================")
        rclpy.shutdown()

    @staticmethod
    def _fmt(v):
        return "[" + ", ".join(f"{x:+.5f}" for x in v) + "]"


def main():
    rclpy.init()
    try:
        node = ImuNoiseCalib()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
