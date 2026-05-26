# Calibration — Measuring the Real Robot (M4)

The `qmini_calibration` package measures **what this robot actually is**, so the
Isaac Lab DR ranges can be
widened to cover reality. The ROS 2 side does not tune the robot — it measures,
then the training side is adjusted and retrained. Permanent package, not
throwaway scripts.

Each measurement: minimum-hardware node → defined protocol (open-loop or low-gain
PD only, **never the policy**) → rosbag → offline analyzer appends one row to
`calibration_results.yaml` → `diff_against_isaaclab.py` prints green/red vs the
Isaac Lab cfg (imported live — single source of truth).

**Hard rule:** calibration nodes refuse to start if the policy node is running.

## M4 order (each unblocks the next)

1. Joint zero / direction — done in homing (M2.0) + the M2.3 direction check.
2. **IMU mount & noise floor** — cheap, robot doesn't move. ← implemented
3. Bus polling rate & loop jitter — sets the achievable `decimation`.
4. Actuator latency + effective PD — the two likeliest sim-to-real gaps.
5. Friction — usually fine; last.
6. Run the diff. Any RED → widen the DR range and retrain.

## Step 2: IMU noise floor + mount

```bash
# robot STATIONARY on a LEVEL surface; brings up only qmini_imu, records a bag
ros2 launch qmini_calibration imu_noise_calib.launch.py duration_s:=30.0

# then append the canonical row from the recorded bag (ROS 2 sourced):
python3 src/qmini_calibration/analysis/analyze_imu_bag.py src/qmini_calibration/data/<date>_imu_noise/bag

# green/red vs Isaac Lab — run with the isaac env's python (same one as
# train.py). It boots a headless Isaac Sim to import the cfg, so it's slow to
# start; that keeps the DR ranges single-sourced from the cfg.
python3 src/qmini_calibration/analysis/diff_against_isaaclab.py
```

The node prints a live summary and writes a per-run `summary.yaml` for immediate
feedback; the **authoritative** row in `calibration_results.yaml` is appended by
`analyze_imu_bag.py` from the bag (one canonical writer). The node never writes
the canonical file.

What it measures, and the Isaac Lab reference it diffs against:

| measured | vs Isaac Lab |
|---|---|
| gyro / proj-gravity noise std | `Unoise(±0.35)` on `imu_ang_vel` (obs units), `Unoise(±0.1)` on `imu_projected_gravity` |
| IMU mount tilt from vertical | `ImuCfg.OffsetCfg(rot=(1,0,0,0))` identity rel. `base_link` |
| gravity magnitude / accel noise | sanity |

Mount **position** offset (`pos=(-0.04718, 0.0663, 0.11094)`) is a physical
measurement (tape/CAD), not derivable from the IMU. The tilt check needs the base
actually leveled; a persistent tilt means either the driver's `apply_ros_transform`
or a real mount-frame offset must be applied in the on-robot transform.

## Step 3: bus polling rate + loop jitter

Run **on the Pi 5** (the target host — bus rate/jitter is host-specific). Brings
up the motor bus read-only (no torque); the robot does not move.

```bash
ros2 launch qmini_calibration bus_jitter_calib.launch.py duration_s:=30.0

python3 src/qmini_calibration/analysis/analyze_bus_jitter_bag.py \
    src/qmini_calibration/data/<date>_bus_jitter/bag
python3 src/qmini_calibration/analysis/diff_against_isaaclab.py   # isaac env
```

Measures the driver-stamped `/joint_states` publish period (mean rate, jitter
std, p95/p99, dropped ticks) and per-channel NaN fraction (channel-down / failed
polls). The diff imports `sim.dt` + `decimation` live and checks the bus sustains
≥ the policy rate (`1/(sim.dt·decimation)`) with margin, and reports the implied
minimum `decimation`. The aggregated `/joint_states` can't expose true per-channel
poll rates, so per-channel health is inferred from NaNs.

## Conventions

- Bags live in `data/<YYYY-MM-DD>_<measurement>/` and are **not committed** (only
  the `calibration_results.yaml` rows are).
- Timestamps come from the driver layer (`qmini_imu`/`qmini_hardware`); analyzers
  never re-stamp.
- `diff_against_isaaclab.py` imports the Isaac Lab cfg directly — the DR ranges are
  never duplicated in this repo.
