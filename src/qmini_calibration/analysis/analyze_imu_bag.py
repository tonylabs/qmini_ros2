#!/usr/bin/env python3
"""Offline analyzer for the IMU noise/mount bag (M4 step 2).

Reads a rosbag2 recorded by imu_noise_calib.launch.py, segments to the
[static_begin, static_end] window using the /calibration/event markers, computes
the authoritative noise/mount statistics, and APPENDS one row to
calibration_results.yaml (it does not rewrite existing rows).

    python3 analyze_imu_bag.py <bag_dir> [--results <calibration_results.yaml>]

Timestamps come from the bag (driver-stamped messages) — never re-stamped here.
Run in an environment with ROS 2 sourced (needs rosbag2_py + the message types).
A thin notebook (imu_noise_mount.ipynb) wraps this for plots; the logic lives
here so it stays testable.
"""

import argparse
import math
import os
from datetime import date

import yaml


def quat_rotate_inverse(w, x, y, z, v):
    qv = (x, y, z)
    w2 = 2.0 * w * w - 1.0
    a = [vi * w2 for vi in v]
    cross = (qv[1] * v[2] - qv[2] * v[1],
             qv[2] * v[0] - qv[0] * v[2],
             qv[0] * v[1] - qv[1] * v[0])
    dot = qv[0] * v[0] + qv[1] * v[1] + qv[2] * v[2]
    return [a[i] - 2.0 * w * cross[i] + 2.0 * dot * qv[i] for i in range(3)]


def _stats(xs):
    n = len(xs)
    if n == 0:
        return 0.0, 0.0
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / (n - 1) if n > 1 else 0.0
    return m, math.sqrt(var)


def read_bag(bag_dir):
    """Return (events, imu) where events=[(label, t_ns)], imu=[(t_ns, gyro, accel, quat)]."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""))

    events, imu = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/calibration/event":
            events.append((deserialize_message(data, String).data, t))
        elif topic == "/imu/data":
            m = deserialize_message(data, Imu)
            imu.append((
                t,
                (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z),
                (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z),
                (m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z),
            ))
    return events, imu


def analyze(bag_dir):
    events, imu = read_bag(bag_dir)
    begin = next((t for lbl, t in events if lbl == "static_begin"), None)
    end = next((t for lbl, t in events if lbl == "static_end"), None)
    if begin is None or end is None:
        raise RuntimeError(
            "Missing static_begin/static_end markers in bag — was the run aborted?")
    window = [s for s in imu if begin <= s[0] <= end]
    if len(window) < 10:
        raise RuntimeError(f"Only {len(window)} IMU samples in the static window.")

    gyro = list(zip(*[s[1] for s in window]))
    accel = list(zip(*[s[2] for s in window]))
    projg = list(zip(*[quat_rotate_inverse(*s[3], (0.0, 0.0, -1.0)) for s in window]))

    gyro_stats = [_stats(list(gyro[i])) for i in range(3)]
    accel_stats = [_stats(list(accel[i])) for i in range(3)]
    projg_stats = [_stats(list(projg[i])) for i in range(3)]
    accel_mean = [accel_stats[i][0] for i in range(3)]
    g_mag = math.sqrt(sum(m * m for m in accel_mean))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(accel_mean[2]) / g_mag)))) \
        if g_mag > 1e-6 else float("nan")
    dur_s = (window[-1][0] - window[0][0]) * 1e-9
    rate = (len(window) - 1) / dur_s if dur_s > 0 else float("nan")

    return {
        "measurement": "imu_noise_mount",
        "date": date.today().isoformat(),
        "bag": os.path.basename(os.path.normpath(bag_dir)),
        "n_samples": len(window),
        "rate_hz": round(rate, 2),
        "gyro_bias_rad_s": [round(gyro_stats[i][0], 6) for i in range(3)],
        "gyro_noise_std_rad_s": [round(gyro_stats[i][1], 6) for i in range(3)],
        "accel_mean_m_s2": [round(accel_mean[i], 4) for i in range(3)],
        "accel_noise_std_m_s2": [round(accel_stats[i][1], 4) for i in range(3)],
        "gravity_magnitude_m_s2": round(g_mag, 4),
        "proj_gravity_mean": [round(projg_stats[i][0], 5) for i in range(3)],
        "proj_gravity_noise_std": [round(projg_stats[i][1], 5) for i in range(3)],
        "mount_tilt_from_vertical_deg": round(tilt, 3),
    }


def append_row(results_path, row):
    data = {}
    if os.path.exists(results_path):
        with open(results_path) as f:
            data = yaml.safe_load(f) or {}
    data.setdefault("imu_noise_mount", []).append(row)
    with open(results_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"Appended row to {results_path}")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_results = os.path.join(here, "..", "calibration_results.yaml")
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir", help="rosbag2 directory recorded by the launch file")
    ap.add_argument("--results", default=os.path.normpath(default_results))
    args = ap.parse_args()

    row = analyze(args.bag_dir)
    print(yaml.safe_dump(row, sort_keys=False))
    append_row(args.results, row)


if __name__ == "__main__":
    main()
