#!/usr/bin/env python3
"""Offline analyzer for the foot-floor friction bag (M4 step 5).

Reads a rosbag2 recorded by friction_calib.launch.py. For each /calibration/event
"slip_N" marker it takes the breakaway angle = the MAX IMU tilt reached in that
trial window (operator tilts slowly and stops at slip, so the peak tilt is the
breakaway angle), and computes:

    mu_static  = tan(theta_breakaway)                       (authoritative)
    mu_kinetic = (g*sin(theta) - a_slide) / (g*cos(theta))  (best-effort)

mu_kinetic uses the peak down-slope acceleration just after slip; a short slide on
a 6-axis IMU makes it noisy, so it is reported as approximate. The Isaac Lab DR
that matters is the STATIC range, which mu_static diffs against cleanly.

Appends one `friction` row to calibration_results.yaml (one canonical writer) and
writes a per-sample tilt/accel CSV for analysis/matlab/plot_friction.m.

    python3 analyze_friction_bag.py <bag_dir> [--results <...>]

Timestamps come from the bag (driver-stamped) — never re-stamped here. Run with
ROS 2 sourced.
"""

import argparse
import glob
import math
import os
from datetime import date

import yaml

GRAVITY = 9.80665


def _detect_storage_id(bag_dir):
    if glob.glob(os.path.join(bag_dir, "*.mcap")):
        return "mcap"
    if glob.glob(os.path.join(bag_dir, "*.db3")):
        return "sqlite3"
    return "mcap"


def quat_rotate_inverse(w, x, y, z, v):
    qv = (x, y, z)
    w2 = 2.0 * w * w - 1.0
    a = [vi * w2 for vi in v]
    cross = (qv[1] * v[2] - qv[2] * v[1],
             qv[2] * v[0] - qv[0] * v[2],
             qv[0] * v[1] - qv[1] * v[0])
    dot = qv[0] * v[0] + qv[1] * v[1] + qv[2] * v[2]
    return [a[i] - 2.0 * w * cross[i] + 2.0 * dot * qv[i] for i in range(3)]


def _tilt_deg(quat):
    pg = quat_rotate_inverse(*quat, (0.0, 0.0, -1.0))
    norm = math.sqrt(sum(c * c for c in pg)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, -pg[2] / norm))))


def _mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def _std(xs):
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=_detect_storage_id(bag_dir)),
        rosbag2_py.ConverterOptions("", ""))

    events, imu = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/calibration/event":
            events.append((deserialize_message(data, String).data, t))
        elif topic == "/imu/data":
            m = deserialize_message(data, Imu)
            tilt = _tilt_deg((m.orientation.w, m.orientation.x,
                              m.orientation.y, m.orientation.z))
            amag = math.sqrt(m.linear_acceleration.x ** 2
                             + m.linear_acceleration.y ** 2
                             + m.linear_acceleration.z ** 2)
            imu.append((t, tilt, amag))
    return events, imu


def analyze(bag_dir):
    events, imu = read_bag(bag_dir)
    if len(imu) < 10:
        raise RuntimeError(f"Only {len(imu)} IMU samples in the bag.")
    slips = [(lbl, t) for lbl, t in events if lbl.startswith("slip_")]
    if not slips:
        raise RuntimeError(
            "No slip_N markers in the bag — did any trial reach breakaway? "
            "(lower slip_accel_thresh or tilt further).")

    begin = next((t for lbl, t in events if lbl == "session_begin"), imu[0][0])
    slip_angles, mu_s, mu_k = [], [], []
    trial_start = begin
    for _lbl, t_slip in slips:
        trial = [s for s in imu if trial_start <= s[0] <= t_slip]
        if not trial:
            trial_start = t_slip
            continue
        theta = max(s[1] for s in trial)                 # breakaway angle (deg)
        slip_angles.append(theta)
        th = math.radians(theta)
        mu_s.append(math.tan(th))
        # best-effort kinetic: peak excess accel in the 0.3 s slide after slip
        slide = [s for s in imu if t_slip < s[0] <= t_slip + 0.3e9]
        if slide and math.cos(th) > 1e-6:
            a_slide = max(s[2] for s in slide) - GRAVITY  # down-slope specific force
            mk = (GRAVITY * math.sin(th) - a_slide) / (GRAVITY * math.cos(th))
            if 0.0 < mk < 5.0:
                mu_k.append(mk)
        trial_start = t_slip

    _write_samples_csv(os.path.dirname(os.path.normpath(bag_dir)), imu, slips)

    row = {
        "measurement": "friction",
        "date": date.today().isoformat(),
        "bag": os.path.basename(os.path.normpath(bag_dir)),
        "n_trials": len(slip_angles),
        "slip_angles_deg": [round(a, 2) for a in slip_angles],
        "mu_static_per_trial": [round(m, 4) for m in mu_s],
        "mu_static_mean": round(_mean(mu_s), 4),
        "mu_static_std": round(_std(mu_s), 4),
        "mu_kinetic_mean": round(_mean(mu_k), 4) if mu_k else None,
        "mu_kinetic_note": "approximate (short slide, 6-axis IMU)",
    }
    return row


def _write_samples_csv(run_dir, imu, slips):
    import csv
    t0 = imu[0][0]
    slip_ts = set(round((t - t0) * 1e-9, 4) for _l, t in slips)
    path = os.path.join(run_dir, "samples.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "tilt_deg", "accel_mag"])
        for t, tilt, amag in imu:
            w.writerow([(t - t0) * 1e-9, tilt, amag])
    # separate marker file so MATLAB can draw the slip lines
    mpath = os.path.join(run_dir, "slip_markers.csv")
    with open(mpath, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"])
        for ts in sorted(slip_ts):
            w.writerow([ts])
    print(f"Wrote per-sample time series to {path} and slip markers to {mpath}")


def append_row(results_path, row):
    data = {}
    if os.path.exists(results_path):
        with open(results_path) as f:
            data = yaml.safe_load(f) or {}
    data.setdefault("friction", []).append(row)
    with open(results_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"Appended row to {results_path}")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_results = os.path.normpath(os.path.join(here, "..", "calibration_results.yaml"))
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("--results", default=default_results)
    args = ap.parse_args()
    row = analyze(args.bag_dir)
    print(yaml.safe_dump(row, sort_keys=False))
    append_row(args.results, row)


if __name__ == "__main__":
    main()
