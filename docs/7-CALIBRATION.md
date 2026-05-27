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

## Step 4: actuator latency + effective PD

**The first calibration that moves the robot under torque — run it ON THE ROPE,
deadman held.** It publishes `/motor_command` directly (in place of `pd_packer`),
so the step has a known shape and known kp/kd; `qmini_hardware` still gates torque
on `enable_motor_torque` + `MotionGate==ENABLED` + a fresh heartbeat (release the
deadman → motors freewheel and the protocol pauses).

```bash
# DRY RUN first (no torque — protocol + markers publish, motors freewheel):
ros2 launch qmini_calibration actuator_latency_calib.launch.py

# then for real, on the rope, holding the deadman; one joint at a time:
ros2 launch qmini_calibration actuator_latency_calib.launch.py enable_torque:=true
ros2 launch qmini_calibration actuator_latency_calib.launch.py enable_torque:=true \
    test_joints:="['knee_pitch_l','hip_pitch_l']" gain_scale:=0.5 step_rad:=0.08

# analyze + diff
python3 src/qmini_calibration/analysis/analyze_actuator_bag.py \
    src/qmini_calibration/data/<date>_actuator_latency/bag
python3 src/qmini_calibration/analysis/diff_against_isaaclab.py   # isaac env
```

Holds all joints at the home pose and applies a small square-wave position step
(`step_rad`) to each tested joint in turn, `n_steps` times. Measures, from the
driver-stamped `/motor_command` + `/joint_states` (which carries q, dq, and
**effort = measured torque**):

- **round-trip latency** — command edge → first joint motion (`> pos_thresh`).
  Compared against `DelayedPDActuatorCfg(max_delay)` (currently 0 → any real
  latency is RED → widen `max_delay` and retrain).
- **effective kp** — steady-state `mean(tau) / mean(q_des − q)` vs the commanded
  kp (firmware PD realizes the gain if within ~20%).

Start with `gain_scale:=0.5` and one joint; the rope is the only fall protection.

## Step 5: foot-floor friction (inclined-plane test)

The Isaac Lab DR that matters here is `EventCfg.physics_material.static_friction_range`
— the **foot↔floor contact** coefficient μ, not joint friction. This robot has no
foot force/torque sensor, so we measure μ with the classic inclined-plane test,
made objective by reusing the validated N100 IMU. **No motors, no torque — the
safest M4 measurement.**

Setup: rest the N100 on a weighted **foot-sole sample** (the real sole material +
a representative normal load) sitting on a flat board. The IMU rides **on the
sliding sample**, not on the board.

```bash
ros2 launch qmini_calibration friction_calib.launch.py n_trials:=5

python3 src/qmini_calibration/analysis/analyze_friction_bag.py \
    src/qmini_calibration/data/<date>_friction/bag
python3 src/qmini_calibration/analysis/diff_against_isaaclab.py   # isaac env
```

Protocol: tilt the board **slowly** until the sample breaks away, then **stop and
return to level** — the node re-arms for the next trial. Repeat `n_trials` times.
The IMU's projected gravity gives the tilt angle; at breakaway the sample-mounted
IMU sees a down-slope acceleration spike (`slip_accel_thresh`, default 1.5 m/s²),
which marks the trial. The breakaway angle is the **peak tilt** before each slip:

- **μ_static = tan(θ_breakaway)** — authoritative; diffed against
  `static_friction_range`. (θ from 5.7° to 63.4° spans the trained μ 0.1–2.0.)
- **μ_kinetic** — best-effort from the post-slip slide acceleration; noisy on a
  short slide with a 6-axis IMU, so reported as approximate.

If a trial never trips, lower `slip_accel_thresh` or tilt further (a very slippery
floor, μ < 0.09, breaks away below the 5° floor and is missed).

## Visualizing the results (MATLAB)

The analyzers also export a **per-sample time series CSV** into the run dir
(`samples.csv` for IMU and friction, `periods.csv` for bus jitter) alongside the
`calibration_results.yaml` row. MATLAB scripts in `analysis/matlab/` load those
and plot — base MATLAB only, no toolboxes (or MATLAB ROS Toolbox) required:

```matlab
% IMU: time series, per-axis noise histograms, gyro Allan deviation,
%      measured-noise-vs-DR bars (optional DR args overlay reference lines)
plot_imu_noise('src/qmini_calibration/data/<date>_imu_noise/samples.csv', 0.35, 0.1)

% Bus jitter: period-over-time (dropped ticks flagged) + period histogram
plot_bus_jitter('src/qmini_calibration/data/<date>_bus_jitter/periods.csv', 50)

% Friction: tilt-over-time with slip lines + specific-force magnitude
%           (optional [min max] static_friction_range draws μ reference bands)
plot_friction('src/qmini_calibration/data/<date>_friction/samples.csv', [0.1 2.0])
```

The CSVs are per-run artifacts under `data/` (gitignored, not committed). The
plots are a visual sanity check; the authoritative green/red comparison is still
`diff_against_isaaclab.py` (single source). The optional trailing args only draw
reference lines — pass the numbers the diff prints (e.g. ang_vel DR 0.35 in obs
units, proj-gravity DR 0.1; policy rate 50 Hz).

## Conventions

- Bags live in `data/<YYYY-MM-DD>_<measurement>/` and are **not committed** (only
  the `calibration_results.yaml` rows are).
- Timestamps come from the driver layer (`qmini_imu`/`qmini_hardware`); analyzers
  never re-stamp.
- `diff_against_isaaclab.py` imports the Isaac Lab cfg directly — the DR ranges are
  never duplicated in this repo.
