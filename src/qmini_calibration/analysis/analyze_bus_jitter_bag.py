#!/usr/bin/env python3
"""Offline analyzer for the bus-jitter bag (M4 step 3).

Reads the rosbag2 from bus_jitter_calib.launch.py, segments to
[static_begin, static_end] via /calibration/event, computes the driver-stamped
publish-period statistics + per-channel NaN fractions, and APPENDS one row to
calibration_results.yaml.

    python3 analyze_bus_jitter_bag.py <bag_dir> [--results <calibration_results.yaml>]

Run with ROS 2 sourced (needs rosbag2_py + message types). Timestamps come from
the bag (driver-stamped); never re-stamped here.
"""

import argparse
import math
import os
from datetime import date

import yaml

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


def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""))
    events, js = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/calibration/event":
            events.append((deserialize_message(data, String).data, t))
        elif topic == "/joint_states":
            m = deserialize_message(data, JointState)
            hstamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            js.append((t, hstamp, list(m.name), list(m.position)))
    return events, js


def analyze(bag_dir):
    events, js = read_bag(bag_dir)
    begin = next((t for lbl, t in events if lbl == "static_begin"), None)
    end = next((t for lbl, t in events if lbl == "static_end"), None)
    if begin is None or end is None:
        raise RuntimeError("Missing static_begin/static_end markers in bag.")
    window = [s for s in js if begin <= s[0] <= end]
    if len(window) < 10:
        raise RuntimeError(f"Only {len(window)} /joint_states samples in window.")

    stamps = [s[1] for s in window]
    periods = [1000.0 * (stamps[i] - stamps[i - 1]) for i in range(1, len(stamps))]
    periods = [p for p in periods if p > 0]
    med = _pct(periods, 50)
    dropped = sum(1 for p in periods if p > 2 * med) if med > 0 else 0
    mean_p = _mean(periods)

    ch_nan = {c: 0 for c in CHANNELS}
    for _, _, names, pos in window:
        bad = {c: False for c in CHANNELS}
        for name, p in zip(names, pos):
            ch = CHANNEL_OF.get(name)
            if ch is not None and math.isnan(p):
                bad[ch] = True
        for c in CHANNELS:
            if bad[c]:
                ch_nan[c] += 1

    return {
        "measurement": "bus_jitter",
        "date": date.today().isoformat(),
        "bag": os.path.basename(os.path.normpath(bag_dir)),
        "n_samples": len(window),
        "rate_hz_mean": round(1000.0 / mean_p, 2) if mean_p > 0 else None,
        "period_ms_mean": round(mean_p, 4),
        "period_ms_std": round(_std(periods), 4),
        "period_ms_min": round(min(periods), 4),
        "period_ms_max": round(max(periods), 4),
        "period_ms_p95": round(_pct(periods, 95), 4),
        "period_ms_p99": round(_pct(periods, 99), 4),
        "dropped_ticks": dropped,
        "channel_nan_fraction": {c: round(ch_nan[c] / len(window), 4) for c in CHANNELS},
    }


def append_row(results_path, row):
    data = {}
    if os.path.exists(results_path):
        with open(results_path) as f:
            data = yaml.safe_load(f) or {}
    data.setdefault("bus_jitter", []).append(row)
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
