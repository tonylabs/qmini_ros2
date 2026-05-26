#!/usr/bin/env python3
"""Green/red diff: measured reality vs the Isaac Lab trained-against ranges.

Imports the Isaac Lab cfg DIRECTLY (qmini_isaaclab) so the DR ranges have exactly
one source of truth — this repo never duplicates them. Reads the latest row from
calibration_results.yaml and reports whether each measured value falls inside the
range the policy was trained to be robust to. Any RED row → widen the Isaac Lab
DR range and retrain before deployment.

    # in the isaac env, with qmini_isaaclab importable:
    python3 diff_against_isaaclab.py [--results <calibration_results.yaml>]

Covers the IMU noise/mount (M4 step 2) and bus-jitter (step 3) measurements.
Add more measurements here as their rows land.
"""

import argparse
import math
import os
import sys

import yaml

GREEN, RED, RESET = "\033[32m", "\033[31m", "\033[0m"


def load_isaaclab_imu_refs():
    """Return the IMU-related ranges, read live from the Qmini cfg."""
    try:
        from Qmini.tasks.qmini_locomotion.qmini_env_cfg import QminiEnvCfg
    except Exception as e:  # noqa: BLE001
        sys.exit(
            "Could not import qmini_isaaclab's QminiEnvCfg — run this in the isaac "
            f"env with the Qmini package on PYTHONPATH.\n  ({type(e).__name__}: {e})")

    cfg = QminiEnvCfg()
    policy = cfg.observations.policy

    refs = {}
    for attr, term in vars(policy).items():
        func = getattr(term, "func", None)
        noise = getattr(term, "noise", None)
        scale = getattr(term, "scale", 1.0)
        fname = getattr(func, "__name__", "") if func is not None else ""
        if fname == "imu_ang_vel":
            refs["ang_vel_scale"] = scale if scale is not None else 1.0
            if noise is not None:
                refs["ang_vel_noise"] = max(abs(noise.n_min), abs(noise.n_max))
        elif fname == "imu_projected_gravity":
            if noise is not None:
                refs["proj_gravity_noise"] = max(abs(noise.n_min), abs(noise.n_max))

    imu = cfg.scene.imu
    refs["mount_pos"] = tuple(imu.offset.pos)
    refs["mount_rot_wxyz"] = tuple(imu.offset.rot)
    return refs


def load_isaaclab_rate_refs():
    """Return sim.dt + decimation -> policy rate, read live from the Qmini cfg."""
    try:
        from Qmini.tasks.qmini_locomotion.qmini_env_cfg import QminiEnvCfg
    except Exception as e:  # noqa: BLE001
        sys.exit(
            "Could not import qmini_isaaclab's QminiEnvCfg — run this in the isaac env.\n"
            f"  ({type(e).__name__}: {e})")
    cfg = QminiEnvCfg()
    sim_dt = float(cfg.sim.dt)
    decimation = int(cfg.decimation)
    return {"sim_dt": sim_dt, "decimation": decimation,
            "policy_rate_hz": 1.0 / (sim_dt * decimation)}


def latest_row(results_path, key):
    with open(results_path) as f:
        data = yaml.safe_load(f) or {}
    rows = data.get(key, [])
    return rows[-1] if rows else None


def status(ok):
    return f"{GREEN}GREEN{RESET}" if ok else f"{RED}RED  {RESET}"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_results = os.path.normpath(os.path.join(here, "..", "calibration_results.yaml"))
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default=default_results)
    args = ap.parse_args()

    # ---- IMU noise/mount ----
    imu_row = latest_row(args.results, "imu_noise_mount")
    if imu_row is None:
        print("imu_noise_mount: no rows yet — skipping.\n")
    else:
        refs = load_isaaclab_imu_refs()
        print(f"IMU noise/mount diff  (measured {imu_row['date']} vs Isaac Lab)\n")
        scale = refs.get("ang_vel_scale", 0.2)
        meas = max(imu_row["gyro_noise_std_rad_s"]) * scale
        ref = refs.get("ang_vel_noise")
        if ref is not None:
            print(f"  [{status(meas <= ref)}] ang_vel noise (obs units): measured "
                  f"{meas:.4f} <= DR +/-{ref:.3f}  (raw std * scale {scale})")
        meas_pg = max(imu_row["proj_gravity_noise_std"])
        ref_pg = refs.get("proj_gravity_noise")
        if ref_pg is not None:
            print(f"  [{status(meas_pg <= ref_pg)}] proj-gravity noise: measured "
                  f"{meas_pg:.4f} <= DR +/-{ref_pg:.3f}")
        rot = refs["mount_rot_wxyz"]
        tilt = imu_row.get("mount_tilt_from_vertical_deg", float("nan"))
        identity = max(abs(rot[1]), abs(rot[2]), abs(rot[3])) < 1e-6
        print(f"\n  IMU mount (Isaac Lab): pos={refs['mount_pos']}, rot(wxyz)={rot}"
              f"{' [identity]' if identity else ''}")
        print(f"  measured mount tilt from vertical: {tilt:.2f} deg (base must be "
              "level; large tilt => fix in the on-robot transform)")
        print("  Position offset must be measured physically (tape/CAD), not from the IMU.\n")

    # ---- bus jitter ----
    bus_row = latest_row(args.results, "bus_jitter")
    if bus_row is None:
        print("bus_jitter: no rows yet — skipping.")
    else:
        rate_refs = load_isaaclab_rate_refs()
        policy_rate = rate_refs["policy_rate_hz"]
        meas_rate = bus_row.get("rate_hz_mean") or 0.0
        # the bus must sustain at least the policy rate, with margin (2x is comfy)
        ok = meas_rate >= policy_rate
        comfy = meas_rate >= 2.0 * policy_rate
        print(f"bus_jitter diff  (measured {bus_row['date']} vs Isaac Lab)\n")
        print(f"  Isaac Lab: sim.dt={rate_refs['sim_dt']}, decimation="
              f"{rate_refs['decimation']} -> policy rate {policy_rate:.1f} Hz")
        print(f"  [{status(ok)}] bus rate {meas_rate:.1f} Hz >= policy rate "
              f"{policy_rate:.1f} Hz" + ("  (2x margin OK)" if comfy else
              "  (WARNING: < 2x margin — raise decimation or speed up the bus)"))
        # implied minimum decimation at the measured rate (sim.dt fixed)
        if meas_rate > 0:
            min_dec = math.ceil((1.0 / meas_rate) / rate_refs["sim_dt"])
            print(f"  implied min decimation at this bus rate (sim.dt fixed): {min_dec} "
                  f"(trained {rate_refs['decimation']})")
        print(f"  jitter: period std {bus_row['period_ms_std']} ms, "
              f"p99 {bus_row['period_ms_p99']} ms, dropped ticks {bus_row['dropped_ticks']}")
        nanfrac = bus_row.get("channel_nan_fraction", {})
        bad = {c: f for c, f in nanfrac.items() if f > 0.01}
        print(f"  [{status(not bad)}] per-channel NaN fraction: "
              + (f"{bad} (channel(s) dropping samples)" if bad else "all channels clean"))


if __name__ == "__main__":
    main()
