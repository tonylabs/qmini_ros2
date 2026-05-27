#!/usr/bin/env python3
"""Green/red diff: measured reality vs the Isaac Lab trained-against ranges.

Imports the Isaac Lab cfg DIRECTLY (qmini_isaaclab) so the DR ranges have exactly
one source of truth — this repo never duplicates them. Reads the latest row from
calibration_results.yaml and reports whether each measured value falls inside the
range the policy was trained to be robust to. Any RED row → widen the Isaac Lab
DR range and retrain before deployment.

    # in the isaac env, with qmini_isaaclab importable:
    python3 diff_against_isaaclab.py [--results <calibration_results.yaml>]

Importing the env cfg pulls in Isaac Sim (omni/pxr), which only loads after the
Isaac Sim app is started — so this script boots a HEADLESS AppLauncher first,
exactly like scripts/rsl_rl/train.py. That makes it slow to start (the simulator
boots) but keeps the DR ranges single-sourced from the cfg. Run with the isaac
env's python.

Covers IMU noise/mount (step 2), bus-jitter (step 3), actuator latency + PD
(step 4) and foot-floor friction (step 5). Add more measurements here as their
rows land.
"""

import argparse
import math
import os
import sys

import yaml

GREEN, RED, RESET = "\033[32m", "\033[31m", "\033[0m"

_SIM_APP = None


def _ensure_isaac_app():
    """Boot a headless Isaac Sim app once, so the env cfg (omni/pxr) can import."""
    global _SIM_APP
    if _SIM_APP is not None:
        return
    try:
        from isaaclab.app import AppLauncher
    except Exception as e:  # noqa: BLE001
        sys.exit(
            "Could not import isaaclab.app.AppLauncher — run this with the isaac "
            f"env's python (the same one that runs train.py).\n  ({type(e).__name__}: {e})")
    print("Booting headless Isaac Sim to read the cfg (this takes a moment)...")
    _SIM_APP = AppLauncher(headless=True).app


def _close_isaac_app():
    global _SIM_APP
    if _SIM_APP is not None:
        _SIM_APP.close()
        _SIM_APP = None


def load_isaaclab_imu_refs():
    """Return the IMU-related ranges, read live from the Qmini cfg."""
    _ensure_isaac_app()
    try:
        from Qmini.tasks.qmini_locomotion.qmini_env_cfg import QminiEnvCfg
    except Exception as e:  # noqa: BLE001
        sys.exit(
            "Could not import qmini_isaaclab's QminiEnvCfg — is the Qmini package "
            f"on PYTHONPATH in this isaac env?\n  ({type(e).__name__}: {e})")

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
    _ensure_isaac_app()
    from Qmini.tasks.qmini_locomotion.qmini_env_cfg import QminiEnvCfg
    cfg = QminiEnvCfg()
    sim_dt = float(cfg.sim.dt)
    decimation = int(cfg.decimation)
    return {"sim_dt": sim_dt, "decimation": decimation,
            "policy_rate_hz": 1.0 / (sim_dt * decimation)}


def load_isaaclab_actuator_refs():
    """Return the actuator delay (DelayedPDActuatorCfg) + sim.dt, from the cfg."""
    _ensure_isaac_app()
    from Qmini.tasks.qmini_locomotion.qmini_env_cfg import QminiEnvCfg
    cfg = QminiEnvCfg()
    sim_dt = float(cfg.sim.dt)
    max_delay = None
    for act in cfg.scene.robot.actuators.values():
        md = getattr(act, "max_delay", None)
        if md is not None:
            max_delay = md if max_delay is None else max(max_delay, int(md))
    return {"sim_dt": sim_dt, "max_delay_steps": max_delay}


def load_isaaclab_friction_refs():
    """Return the foot/floor friction ranges from EventCfg, read live from the cfg."""
    _ensure_isaac_app()
    from Qmini.tasks.qmini_locomotion.qmini_env_cfg import QminiEnvCfg
    cfg = QminiEnvCfg()
    refs = {}
    for _attr, term in vars(cfg.events).items():
        params = getattr(term, "params", None)
        if isinstance(params, dict) and "static_friction_range" in params:
            refs["static_friction_range"] = tuple(params["static_friction_range"])
            if "dynamic_friction_range" in params:
                refs["dynamic_friction_range"] = tuple(params["dynamic_friction_range"])
            break
    return refs


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

    # ---- actuator latency + effective PD ----
    act_row = latest_row(args.results, "actuator_latency")
    if act_row is None:
        print("\nactuator_latency: no rows yet — skipping.")
    else:
        refs = load_isaaclab_actuator_refs()
        md = refs["max_delay_steps"]
        trained_ms = md * refs["sim_dt"] * 1000.0 if md is not None else None
        print(f"\nactuator_latency diff  (measured {act_row['date']} vs Isaac Lab)\n")
        if trained_ms is not None:
            print(f"  Isaac Lab DelayedPDActuatorCfg max_delay={md} steps "
                  f"-> {trained_ms:.1f} ms (sim.dt={refs['sim_dt']})")
        else:
            print("  Isaac Lab actuator has no delay configured (max_delay unset/0).")
        for j, d in act_row["per_joint"].items():
            lat = d.get("latency_ms_mean")
            if lat is not None and trained_ms is not None:
                ok = lat <= trained_ms
                print(f"  [{status(ok)}] {j}: latency {lat:.1f} ms (max {d.get('latency_ms_max')}) "
                      f"<= trained delay {trained_ms:.1f} ms"
                      + ("" if ok else "  -> WIDEN DelayedPDActuatorCfg max_delay + retrain"))
            elif lat is not None:
                print(f"  [{RED}RED  {RESET}] {j}: measured latency {lat:.1f} ms but sim delay "
                      "is 0 -> set DelayedPDActuatorCfg max_delay to cover it + retrain")
            kpc, kpe = d.get("kp_commanded"), d.get("kp_effective")
            if kpe is not None and kpc:
                ok = abs(kpe - kpc) <= 0.2 * abs(kpc)
                print(f"  [{status(ok)}] {j}: effective kp {kpe:.1f} vs commanded {kpc:.1f} "
                      f"({100*(kpe-kpc)/kpc:+.0f}%; firmware PD realizes the gain if within 20%)")

    # ---- foot-floor friction ----
    fri_row = latest_row(args.results, "friction")
    if fri_row is None:
        print("\nfriction: no rows yet — skipping.")
    else:
        refs = load_isaaclab_friction_refs()
        sr = refs.get("static_friction_range")
        mu_s = fri_row.get("mu_static_mean")
        print(f"\nfriction diff  (measured {fri_row['date']} vs Isaac Lab)\n")
        if sr is not None and mu_s is not None:
            ok = sr[0] <= mu_s <= sr[1]
            print(f"  Isaac Lab EventCfg static_friction_range = [{sr[0]}, {sr[1]}]")
            print(f"  [{status(ok)}] measured mu_static {mu_s:.3f} "
                  f"(n={fri_row['n_trials']}, breakaway "
                  f"{fri_row.get('slip_angles_deg')}) within trained range"
                  + ("" if ok else "  -> widen static_friction_range + retrain"))
        else:
            print("  no static_friction_range in EventCfg (or no measured mu) — skipping.")
        dr = refs.get("dynamic_friction_range")
        mu_k = fri_row.get("mu_kinetic_mean")
        if dr is not None and mu_k is not None:
            ok = dr[0] <= mu_k <= dr[1]
            print(f"  [{status(ok)}] measured mu_kinetic ~{mu_k:.3f} (approximate) "
                  f"vs dynamic_friction_range [{dr[0]}, {dr[1]}]")


if __name__ == "__main__":
    try:
        main()
    finally:
        _close_isaac_app()
