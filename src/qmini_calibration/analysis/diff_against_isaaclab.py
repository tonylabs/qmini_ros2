#!/usr/bin/env python3
"""Green/red diff: measured reality vs the Isaac Lab trained-against ranges.

Imports the Isaac Lab cfg DIRECTLY (qmini_isaaclab) so the DR ranges have exactly
one source of truth — this repo never duplicates them. Reads the latest row from
calibration_results.yaml and reports whether each measured value falls inside the
range the policy was trained to be robust to. Any RED row → widen the Isaac Lab
DR range and retrain before deployment.

    # in the isaac env, with qmini_isaaclab importable:
    python3 diff_against_isaaclab.py [--results <calibration_results.yaml>]

Currently covers the IMU noise/mount measurement (M4 step 2). Add more
measurements here as their rows land.
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


def latest_row(results_path, key="imu_noise_mount"):
    with open(results_path) as f:
        data = yaml.safe_load(f) or {}
    rows = data.get(key, [])
    if not rows:
        sys.exit(f"No '{key}' rows in {results_path} — run the measurement + analyzer first.")
    return rows[-1]


def status(ok):
    return f"{GREEN}GREEN{RESET}" if ok else f"{RED}RED  {RESET}"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_results = os.path.normpath(os.path.join(here, "..", "calibration_results.yaml"))
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default=default_results)
    args = ap.parse_args()

    refs = load_isaaclab_imu_refs()
    row = latest_row(args.results)

    print(f"IMU noise/mount diff  (measured {row['date']} vs Isaac Lab)\n")

    # gyro noise: compare in OBSERVATION units (raw std * obs scale) to the DR range
    scale = refs.get("ang_vel_scale", 0.2)
    meas = max(row["gyro_noise_std_rad_s"]) * scale
    ref = refs.get("ang_vel_noise")
    if ref is not None:
        ok = meas <= ref
        print(f"  [{status(ok)}] ang_vel noise (obs units): measured {meas:.4f} "
              f"<= DR +/-{ref:.3f}  (raw std * scale {scale})")

    meas_pg = max(row["proj_gravity_noise_std"])
    ref_pg = refs.get("proj_gravity_noise")
    if ref_pg is not None:
        ok = meas_pg <= ref_pg
        print(f"  [{status(ok)}] proj-gravity noise: measured {meas_pg:.4f} "
              f"<= DR +/-{ref_pg:.3f}")

    # mount: Isaac Lab uses rot=(1,0,0,0) identity rel base_link. We can only check
    # the orientation tilt here (position offset is a physical measurement).
    rot = refs["mount_rot_wxyz"]
    tilt = row.get("mount_tilt_from_vertical_deg", float("nan"))
    identity = max(abs(rot[1]), abs(rot[2]), abs(rot[3])) < 1e-6
    print(f"\n  IMU mount (Isaac Lab): pos={refs['mount_pos']}, rot(wxyz)={rot}"
          f"{' [identity]' if identity else ''}")
    print(f"  measured mount tilt from vertical: {tilt:.2f} deg "
          "(base must be level; large tilt => driver apply_ros_transform or a real "
          "mount-frame offset to fix in the on-robot transform)")
    print("\n  Position offset must be measured physically (tape/CAD), not from the IMU.")


if __name__ == "__main__":
    main()
