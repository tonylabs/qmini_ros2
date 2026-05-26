#!/usr/bin/env python3
"""Offline analyzer for the actuator latency + effective-PD bag (M4 step 4).

Reads /motor_command (q_des, kp — by-leg order) and /joint_states (q, dq,
effort=tau — by name), both driver-stamped. For each commanded position step on
a joint it measures:
  * latency: command edge -> first joint motion (|q-q0| > pos_thresh)
  * effective kp: steady-state mean(tau) / mean(q_des - q) at the end of the hold
    (vs the commanded kp, which is read from the bag)

Appends one actuator_latency row (per-joint stats) to calibration_results.yaml.

    python3 analyze_actuator_bag.py <bag_dir> [--results <...>] [--pos-thresh 0.02]

Run with ROS 2 sourced.
"""

import argparse
import glob
import math
import os
from datetime import date

import yaml

JOINTS = ["hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
          "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r"]


def _detect_storage_id(bag_dir):
    if glob.glob(os.path.join(bag_dir, "*.mcap")):
        return "mcap"
    if glob.glob(os.path.join(bag_dir, "*.db3")):
        return "sqlite3"
    return "mcap"


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
    from sensor_msgs.msg import JointState
    from qmini_msgs.msg import MotorCommand

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=_detect_storage_id(bag_dir)),
        rosbag2_py.ConverterOptions("", ""))

    cmds, states = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/motor_command":
            m = deserialize_message(data, MotorCommand)
            ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            cmds.append((ts, list(m.q_des), list(m.kp)))
        elif topic == "/joint_states":
            m = deserialize_message(data, JointState)
            ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            q = {n: p for n, p in zip(m.name, m.position)}
            tau = {n: e for n, e in zip(m.name, m.effort)}
            states.append((ts, q, tau))
    cmds.sort(key=lambda x: x[0])
    states.sort(key=lambda x: x[0])
    return cmds, states


def analyze(bag_dir, pos_thresh=0.02, step_min=0.03):
    cmds, states = read_bag(bag_dir)
    if len(cmds) < 2 or len(states) < 2:
        raise RuntimeError("Not enough /motor_command or /joint_states in the bag.")

    per_joint = {}
    for ji, jname in enumerate(JOINTS):
        # rising edges in q_des for this joint
        edges = []
        for k in range(1, len(cmds)):
            dq = cmds[k][1][ji] - cmds[k - 1][1][ji]
            if dq > step_min:
                edges.append((cmds[k][0], cmds[k][1][ji], cmds[k][2][ji]))  # t, q_des_hi, kp_cmd
        if not edges:
            continue

        latencies, kp_eff_list = [], []
        kp_cmd = edges[0][2]
        for ei, (t_edge, q_hi, kp_c) in enumerate(edges):
            t_next = edges[ei + 1][0] if ei + 1 < len(edges) else states[-1][0]
            # baseline q just before the edge
            pre = [s for s in states if s[0] <= t_edge]
            if not pre:
                continue
            q0 = pre[-1][1].get(jname, float("nan"))
            win = [s for s in states if t_edge < s[0] <= t_next]
            if len(win) < 3 or math.isnan(q0):
                continue
            # latency: first sample where q moved past pos_thresh
            resp = next((s for s in win if abs(s[1].get(jname, q0) - q0) > pos_thresh), None)
            if resp is not None:
                latencies.append((resp[0] - t_edge) * 1000.0)
            # effective kp: steady-state (last 30% of the window) tau / (q_des - q)
            tail = win[int(0.7 * len(win)):]
            errs = [q_hi - s[1].get(jname, q_hi) for s in tail]
            taus = [s[2].get(jname, 0.0) for s in tail]
            mean_err = _mean(errs)
            if abs(mean_err) > 1e-3:
                kp_eff_list.append(_mean(taus) / mean_err)

        per_joint[jname] = {
            "n_steps": len(edges),
            "kp_commanded": round(kp_cmd, 3),
            "kp_effective": round(_mean(kp_eff_list), 3) if kp_eff_list else None,
            "latency_ms_mean": round(_mean(latencies), 2) if latencies else None,
            "latency_ms_std": round(_std(latencies), 2) if latencies else None,
            "latency_ms_max": round(max(latencies), 2) if latencies else None,
        }

    if not per_joint:
        raise RuntimeError("No position-step edges found — was enable_torque true and "
                           "the deadman held so the joints actually moved?")
    return {
        "measurement": "actuator_latency",
        "date": date.today().isoformat(),
        "bag": os.path.basename(os.path.normpath(bag_dir)),
        "pos_thresh_rad": pos_thresh,
        "per_joint": per_joint,
    }


def append_row(results_path, row):
    data = {}
    if os.path.exists(results_path):
        with open(results_path) as f:
            data = yaml.safe_load(f) or {}
    data.setdefault("actuator_latency", []).append(row)
    with open(results_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"Appended row to {results_path}")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_results = os.path.normpath(os.path.join(here, "..", "calibration_results.yaml"))
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("--results", default=default_results)
    ap.add_argument("--pos-thresh", type=float, default=0.02,
                    help="rad of joint motion that counts as 'responded' (latency)")
    args = ap.parse_args()
    row = analyze(args.bag_dir, pos_thresh=args.pos_thresh)
    print(yaml.safe_dump(row, sort_keys=False))
    append_row(args.results, row)


if __name__ == "__main__":
    main()
