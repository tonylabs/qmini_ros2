# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

The workspace is **scaffolded and the M1–M2 hardware-bring-up path is implemented and validated on the real robot.** As of 2026-05-26 the realtime motor bus (`qmini_hardware`), the PD command packer (`qmini_controllers`), and the safety/deadman node (`qmini_safety`) exist and run; homing and per-joint direction are calibrated for this physical robot, and **M2 is closed** — the robot has driven to the home crouch under full-gain torque (feet splay correctly, ramp smooth, deadman + heartbeat gating verified) on the rope. The IMU driver (`qmini_imu`) is implemented in C++ (native FDILink parser) and validated on the real Wheeltek N100 (publishes `/imu/data` at ~65 Hz, gravity magnitude correct). The policy runner (`qmini_rl`) is implemented (ONNX Runtime C++, 44-dim observation assembler at 50 Hz) and validated on x86 with synthetic inputs, but not yet run on the real robot (needs joint-order verification + aarch64 onnxruntime on the Pi). The joystick parser (`qmini_joystick`) and calibration package (`qmini_calibration`) are still scaffold-only. The architecture below remains the spec — when adding code, fill in the packages as described rather than inventing a different layout. See the **Task List** below for per-milestone status.

## Target platform

- **Runtime host:** Raspberry Pi 5, Ubuntu 24.04, 4 GB RAM. Memory and CPU are tight — prefer C++ for the realtime control loop and ONNX inference path. Reserve Python for launch files, configuration, and offline tooling.
- **ROS 2 distribution:** Jazzy Jalisco (the matching distro for Ubuntu 24.04).
- **Workspace layout:** standard colcon workspace — packages live under `src/`, built into `build/`, `install/`, `log/` (all should be gitignored once they exist).

## Task List

Milestone status as of **2026-05-25**. Legend: ✅ done · 🔄 in progress · ⬜ not started.
Each stage's operator guide lives in `docs/` (numbered to match the bring-up order).

| # | Milestone | Status | What it covers / package(s) | Guide |
|---|---|---|---|---|
| **M0** | Workspace scaffolding | ✅ | colcon workspace, 10 package skeletons, URDF in `qmini_description`, custom msgs in `qmini_msgs` (`MotorCommand`, …), gitignore for build artifacts | — |
| **M1** | Motor bus driver + smoke test | ✅ | `qmini_hardware/motor_bus_node` — 4-channel RS-485, per-channel polling, publishes `/joint_states` (read-only, no torque). Gear ratios incl. hip-roll 18.99 | `docs/1-SMOKE_TEST.md` |
| **M2.0** | Homing (joint zeros) | ✅ | Capture per-joint offsets at the jig pose into `qmini_hardware/config/joint_offsets.yaml`. GO-M8010-6 zeros at power-up → pose jig first | `docs/2-HOMING.md` |
| **M2.1** | PD command packer | ✅ | `qmini_controllers/pd_packer_node` — joint-side gains/limits from Isaac Lab, ease-out ramp, deadman-gated, publishes `/motor_command` (no torque yet) | `docs/4-CONTROLLER_TEST.md` |
| **M2.2** | Hardware command path | ✅ | `qmini_hardware` consumes `/motor_command`: safety gating + offset/ratio/direction conversion + `kp/kd÷ratio²`. Torque opt-in, off by default | `docs/4-CONTROLLER_TEST.md` |
| **M2.3** | First energize → drive to home | ✅ | Drive to the home crouch under torque, deadman + heartbeat gated. Direction check done (both hip-rolls were inverted → `direction:-1`, re-homed; hip-yaws OK). Full-gain energize run on hardware completed 2026-05-26 — feet splay correctly, ramp smooth. **M2 closed.** | `docs/5-ENERGIZE.md` |
| **M3** | Teleop / joystick | 🔄 | `joy_node` + `qmini_safety/safety_node` deadman + `/joy` watchdog **work**. **Not done:** dedicated `qmini_joystick` parser node (vx/vy/wz, modes, battery, requested-stop events → `MotionGate`/`SafetyHeartbeat` inputs) | `docs/3-JOYSTICK.md` |
| **M4** | Calibration | 🔄 | `qmini_calibration`: **IMU noise/mount** (`imu_noise_calib`, ran on real N100 — noise GREEN vs DR, \|g\|=9.84) **and bus-jitter** (`bus_jitter_calib`: driver-stamped `/joint_states` rate/jitter/p99/dropped-ticks + per-channel NaN, diff vs live `sim.dt`/`decimation`) measurements done. **Actuator latency + effective-PD** (`actuator_latency_calib`: drives /motor_command steps under torque, measures command→motion latency + steady-state kp from /joint_states effort; diff vs `DelayedPDActuatorCfg.max_delay` + commanded kp) built — **run on the robot on the rope, deadman held**. Bus-jitter + actuator validated synthetically/by-build; **need real runs on hardware**. Each has launch+bag+analyzer; `diff_against_isaaclab.py` is the single-source diff (boots headless Isaac Sim). **Remaining:** friction; IMU mount-tilt check needs the base leveled | `docs/7-CALIBRATION.md` |
| **M5** | Policy runner | 🔄 | `qmini_rl/policy_runner_node` **implemented**: ONNX Runtime C++ loads the Isaac-Lab policy (`obs[44]→action[10]`), assembles the 44-dim observation on-robot (per-term scales, `gait_phase_sincos` `[sinL,sinR,cosL,cosR]` @1.5 Hz, `static_flag` @0.15), publishes `q_des=home+0.5·a` on `/joint_target` at 50 Hz, MotionGate-gated. Builds + validated on x86. **Joint order verified 2026-05-27 via `play.py`: policy uses INTERLEAVED order `[yaw_l,yaw_r,roll_l,roll_r,pitch_l,pitch_r,knee_l,knee_r,ankle_l,ankle_r]`, NOT the hardware by-leg order — `kJointOrder` fixed to match; bridged to pd_packer by JointState names.** Pending: run on real robot (Pi aarch64 onnxruntime + real IMU/joint_states). Depends on `qmini_imu` | `docs/8-POLICY_RUNNER.md` |
| **—** | IMU driver | ✅ | `qmini_imu` (C++, native FDILink parser over raw termios, no external serial dep): Wheeltek N100 → `sensor_msgs/Imu` on `/imu/data`, own process, graceful port-loss retry. **Validated on real N100 2026-05-26**: parses CRC-checked frames, gravity \|a\|≈9.83 m/s², ~65 Hz (>50 Hz policy rate). **Mount rotation resolved 2026-05-26**: `mount_rotation` param (quaternion, applied to orientation+gyro+accel) = `[0,0,1,0]` (180° about Y = official SDK's `(-1,+1,-1)`); `imu_projected_gravity` now reads (0,0,-1) upright | `docs/6-IMU_DRIVER.md` |
| **M6** | Standing / walking (unsupported) | ⬜ | Full system under teleop with the M6-only safety rules (5% velocity ramp, 3-step cutoff, two-person, battery preconditions). Gated on better fall protection than low-platform-+-human-catch | — |

**Immediate next action:** M2 closed; `qmini_imu` hardware-validated; `qmini_rl` policy runner implemented + validated on x86 with synthetic inputs; M4 IMU noise/mount measurement done (noise floors GREEN vs DR). Joint order is now **verified** (interleaved policy order, `kJointOrder` fixed) and the IMU mount/frame is resolved (GREEN diff). Before a real-robot policy run: (1) build with the **aarch64 onnxruntime** on the Pi (vendored in `qmini_official_sdk`), (2) finish M4 (run bus-jitter + actuator-latency on the robot; build friction). Bus-jitter, IMU-noise, and actuator-latency measurements are built; the next M4 build is the **friction** measurement, and the built ones need real hardware runs.

## Hardware topology

The robot is a **bipedal platform with 10 actuated joints total** (5 per leg, Unitree GO-M8010-6 motors). The actuators are not wired as one bus per leg — they are grouped **functionally** across four RS-485 channels exposed by a single USB-to-4-channel RS-485 adapter:

| RS-485 channel | Motors on the bus                          | Count |
|----------------|--------------------------------------------|-------|
| 1              | Hip-yaw series (left hip yaw + right hip yaw) | 2     |
| 2              | Hip-roll series (left hip roll + right hip roll) | 2     |
| 3              | Left leg lower 3 joints (hip-pitch, knee, ankle) | 3     |
| 4              | Right leg lower 3 joints (hip-pitch, knee, ankle) | 3     |

Implications that are easy to get wrong:
- A single "left leg command" touches **three different channels** (1, 2, 3). Do not assume one bus == one leg.
- Each of the four `/dev/ttyUSB*` (or `/dev/serial/by-id/...`) ports can be polled in parallel; that parallelism is the main lever for hitting the control loop rate on a Pi 5. Prefer per-channel threads / executors over a single serialized loop.
- Use `/dev/serial/by-id/...` symlinks (or a `udev` rules file under `config/udev/`) to pin channel→port mapping. USB enumeration order is not stable across reboots.

**IMU:** Wheeltek N100 over USB (CP2102 USB-to-serial). It appears as a separate `/dev/ttyUSB*` and must also be pinned by `by-id` / udev. Treat IMU and motor-bus serial ports as independent devices.

## Intended ROS 2 architecture

The control stack should be split into small nodes so the realtime path stays isolated from inference and telemetry. Suggested package layout under `src/`:

- `qmini_description/` — URDF/xacro of the robot, joint names, joint limits, transmission. The joint name ordering defined here is the **canonical** ordering; the ONNX policy's input/output index ↔ joint mapping must be derived from this, not hardcoded twice.
- `qmini_msgs/` — custom messages/services (e.g. `MotorCommand`, `MotorState`, `PolicyObservation`) if `sensor_msgs/JointState` + `control_msgs` are insufficient.
- `qmini_hardware/` — RS-485 driver for the GO-M8010-6 bus. Four nodes (or one node with four executors), one per channel. Publishes `JointState`, subscribes to torque/position commands. This is the realtime layer — C++, no allocations in the hot loop, SCHED_FIFO if permissions allow.
- `qmini_imu/` — Wheeltek N100 driver publishing `sensor_msgs/Imu`. Keep it separate from motor I/O so a stalled IMU port can't block motor commands.
- `qmini_controllers/` — the PD controller and any safety/limit logic. Subscribes to policy actions + joint state, publishes motor commands. PD gains live in YAML under `config/`.
- `qmini_rl/` — ONNX policy runner. Loads the Isaac-Lab-exported `.onnx`, subscribes to the observation sources (IMU, joint state, command/velocity input), publishes the action vector at the policy's training rate. Use **ONNX Runtime C++** with the CPU execution provider on the Pi 5; the XNNPACK EP is worth trying for small policies.
- `qmini_joystick/` — wraps `joy_node` and maps PS4 DualShock (over Bluetooth) into parsed commands (`vx`, `vy`, `wz`, deadman state, mode, requested-stop events). Publishes to `qmini_safety`, not directly to motors.
- `qmini_safety/` — single point of safety enforcement. Owns the deadman / E-stop state machine, runs the `/joy` watchdog, emits a heartbeat that `qmini_hardware` requires to keep torque enabled. See **Safety architecture** below.
- `qmini_bringup/` — launch files that compose the above into "sim", "hardware", and "hardware+teleop" configurations.
- `qmini_calibration/` — measurement nodes + offline analysis notebooks for characterizing the real robot (joint zeros, IMU noise/mount, actuator latency, effective PD, bus jitter, friction). Outputs feed back into the Isaac Lab DR ranges. See **Calibration as a first-class deliverable** below.

### Data flow (steady state)

```
IMU (N100) ─┐
            ├─► observation assembler ─► ONNX policy ─► action (joint targets)
JointState ─┘                                                │
                                                             ▼
                                                  PD controller (config/gains.yaml)
                                                             │
                                              ┌──────────────┼──────────────┐
                                              ▼              ▼              ▼
                                      ch1 hip-yaw    ch2 hip-roll   ch3/ch4 lower-leg
```

Key correctness concerns when wiring this up:
- **Joint ordering must match the policy's training order**, not the URDF traversal order or the bus order. Export the order from Isaac Lab alongside the ONNX file and load both together; do not re-derive it.
- **Action scaling / clipping** used at training time must be re-applied at inference. Bake these constants into a config file (`config/policy.yaml`) so they are reviewable, not buried in code.
- **Observation history / framestack**, if the policy was trained with it, must be reconstructed identically on-robot. A common silent failure is feeding the policy a single observation when it was trained on `n` stacked frames.
- The policy runs at a fixed rate (Isaac Lab default is often 50 Hz). The PD loop does **not** need to run faster than the policy on this robot: the GO-M8010-6 closes the PD loop in **motor firmware** (see *Motor PD lives on the motor* below), so `qmini_controllers` only forwards `(q_des, dq_des, tau_ff, kp, kd)` packets at policy rate or moderately faster.

### Motor PD lives on the motor

The GO-M8010-6's `MotorCmd` carries `q_des`, `dq_des`, `tau_ff`, `kp`, `kd` in a single packet. The motor's onboard firmware closes the PD loop at very high rate using those gains. This has several consequences for the ROS 2 design:

- `qmini_controllers` is **not** a 1 kHz software PD loop. It is a thin packer that loads kp/kd from `config/gains.yaml` (values copied verbatim from `DelayedPDActuatorCfg` in `qmini.py`), maps the policy's joint targets into the motor command struct, and emits commands at policy rate (or a small integer multiple).
- The Pi 5 control loop just needs to ≥ policy rate. Bus polling rate per channel (measured in M4 calibration) sets the ceiling, not CPU.
- "Actuator latency" in calibration measures **USB → RS-485 framing latency + motor command-execution lag**, NOT the inner PD loop. This is still a real sim-to-real gap and is what the Isaac Lab `DelayedPDActuatorCfg(max_delay=...)` should be widened to cover.
- The exact field names, units (rad vs. encoder counts, N·m vs. unitless), and CRC layout must be verified against the `unitree_actuator_sdk` headers during M1 — these have shifted between SDK versions.

## Build, run, test (standard ROS 2 Jazzy workflow)

Once `src/` has packages, these are the commands you'll use. Run them from the workspace root (this directory).

```bash
# One-time per shell: source the distro
source /opt/ros/jazzy/setup.bash

# Install ROS deps declared in package.xml across the workspace
rosdep install --from-paths src --ignore-src -r -y

# Build everything
colcon build --symlink-install

# Build a single package (faster iteration)
colcon build --symlink-install --packages-select qmini_controllers

# Build a package and everything that depends on it
colcon build --symlink-install --packages-up-to qmini_bringup

# Source the workspace overlay (after every fresh build into a new shell)
source install/setup.bash

# Run the full system
ros2 launch qmini_bringup hardware.launch.py

# Tests — colcon test wraps gtest (C++) and pytest (Python)
colcon test --packages-select qmini_controllers
colcon test-result --verbose            # summarize failures

# Run one C++ gtest binary directly (after build) for fast iteration
./build/qmini_controllers/test_pd_controller --gtest_filter=PDControllerTest.*
```

Lint/format conventions for ROS 2 Jazzy: `ament_cpplint`, `ament_uncrustify`, `ament_flake8`, `ament_copyright` are wired in automatically by `ament_lint_auto` when you add it to `package.xml`. Prefer enabling these from the first package rather than retrofitting.

## Permissions / device access

The user running the nodes must be in the `dialout` group to access `/dev/ttyUSB*`:

```bash
sudo usermod -aG dialout $USER   # log out / back in to take effect
```

Pinning device paths via udev (write rules into `config/udev/99-qmini.rules`, then `sudo udevadm control --reload && sudo udevadm trigger`) is preferable to hardcoding `/dev/ttyUSB0..4` in launch files.

## Reference repo: the original Isaac Gym project (`qmini_official_rl`)

A separate repo at **`~/Documents/GitHub/qmini_official_rl`** (RoboTamer4Qmini v1.0, VSISLab Shandong University, MIT) is the **previously-tested Isaac Gym training framework** for this robot. It includes a working ONNX (`experiments/q2/deploy/policy.onnx`, input [1, 129] → output [1, 12]) that has been deployed before. **It is NOT the deployment policy for this project** (decision 2026-05-23: deploy from Isaac Lab instead), but it is the **authoritative reference for features the Isaac Lab project must reach parity on** before its retrained policy is deployable.

When upgrading `qmini_isaaclab`, read these IG files for reference implementations:

| Need to implement in Isaac Lab… | Read in `qmini_official_rl/` |
|---|---|
| Gait phase signal (sin/cos per leg, gated by static_flag) | `env/utils/phase_modulator.py`, and how it's consumed in `env/tasks/birl_task.py` |
| Observation delay simulation | `env/utils/delay_torch_deque.py` |
| Standing-mode reward gating (`static_flag = norm(cmd) < 0.15`) | `env/tasks/birl_task.py` (search for `static_flag`) |
| Foot scheduling rewards (swing-clear / stance-contact gated by phase) | `env/tasks/birl_task.py` reward terms `foot_clear`, `foot_support`, `foot_height` |
| Motor strength / kp/kd / torque DR (±20%) | `config/BIRL.py` and `experiments/q2/model/cfg.yaml` `domain_rand:` section |
| Frozen training config that produced the working policy | `experiments/q2/model/cfg.yaml` |

The IG ONNX is a **"break glass" fallback** if the Isaac Lab retraining stalls. Falling back would require `qmini_rl` to be rewritten with a 129-dim observation assembler (3-frame history + phase + base_euler) and a 12-dim incremental action handler — not trivial, but possible.

## Companion repo: Isaac Lab training side

The PPO policy is trained in a sibling repo at **`~/Documents/GitHub/qmini_isaaclab`** (rsl_rl + Isaac Lab). It is the **authoritative source** for every sim-side number the on-robot stack must match — when debugging "the robot moves wrong" issues, read these files first instead of guessing on the ROS 2 side.

| If you're debugging…                          | Read this file (relative to `~/Documents/GitHub/qmini_isaaclab/`)                |
|-----------------------------------------------|-----------------------------------------------------------------------------------|
| **kp / kd mismatch** (jittery, soft, stiff, oscillating) | `source/Qmini/robots/qmini.py` — `DelayedPDActuatorCfg` `stiffness` (kp) and `damping` (kd) dicts, keyed by joint regex |
| **Effort / velocity saturation** in sim vs real | `source/Qmini/robots/qmini.py` — `effort_limit_sim`, `velocity_limit_sim` (per joint) |
| **Default / home pose** for safe startup       | `source/Qmini/robots/qmini.py` — `init_state.joint_pos` (the mirrored crouch pose, with joint name → radians) |
| **Joint name order** for ONNX I/O              | `source/Qmini/robots/qmini.py` (`init_state.joint_pos` keys) and the URDF at `source/Qmini/assets/descriptions/qmini/qmini.urdf` |
| **Step frequency / control rate** (does the policy run at 50 Hz? PD loop rate to match?) | `source/Qmini/tasks/qmini_locomotion/qmini_env_cfg.py` and its parent `LocomotionVelocityRoughEnvCfg` — check `sim.dt`, `decimation`. Policy rate = `1 / (sim.dt * decimation)`. The qmini cfg doesn't override these, so the parent defaults apply |
| **Action scaling** (is the policy output a delta? a multiplier? absolute target?) | `source/Qmini/tasks/qmini_locomotion/qmini_env_cfg.py` — `ActionsCfg.joint_pos = mdp.JointPositionActionCfg(... scale=0.5, use_default_offset=True)`. `use_default_offset=True` means the action is **`scale * policy_output + default_joint_pos`**, NOT an absolute target. The on-robot inference path must apply the same scale and the same default offset (from `init_state.joint_pos`). |
| **Policy observation vector** (what to feed the ONNX input, in what order, with what scales/noise) | `source/Qmini/tasks/qmini_locomotion/qmini_env_cfg.py` — `ObservationsCfg.PolicyCfg`. **After 2026-05-23 Tier 1 upgrade** (order preserved): `imu_ang_vel` (3, scale 0.2) → `imu_projected_gravity` (3) → `joint_pos_rel` (10) → `joint_vel_rel` (10, scale 0.05) → `last_action` (10) → `velocity_commands` (3, vx/vy/wz) → `gait_phase_sincos` (4, fixed 1.5 Hz, anti-phase) → `static_flag` (1, threshold 0.15). **Total 44 dims**. `concatenate_terms=True` ⇒ one flat vector. The ROS 2 `qmini_rl` must reconstruct `gait_phase_sincos` and `static_flag` on-robot using the same constants. |
| **IMU mounting / orientation** (why is gravity pointing the "wrong" way? why does ang_vel sign flip?) | `source/Qmini/tasks/qmini_locomotion/qmini_env_cfg.py` — `__post_init__` sets `self.scene.imu = ImuCfg(prim_path=".../base_link", offset=ImuCfg.OffsetCfg(pos=(-0.04718, 0.0663, 0.11094), rot=(1, 0, 0, 0)))`. The Wheeltek N100 mount on the real robot must reproduce this **position offset and frame rotation relative to `base_link`**, or the observation transform on-robot must compensate. `rot=(1,0,0,0)` is identity in `(w, x, y, z)`. |
| **Observation normalization**                   | `source/Qmini/tasks/qmini_locomotion/agents/rsl_rl_ppo_cfg.py` — `empirical_normalization = False` and the actor's `obs_normalization=False`. **No running-mean/std normalization is applied**, so the ROS 2 side does NOT need to track normalization stats — just feed raw observations with the same per-term scales listed in the row above. |
| **PPO hyperparameters / network shape**         | `source/Qmini/tasks/qmini_locomotion/agents/rsl_rl_ppo_cfg.py` — actor & critic MLP `hidden_dims=[512, 256, 128]`, `activation="elu"`, Gaussian policy `init_std=1.0`. Algorithm: `clip_param=0.2`, `entropy_coef=0.008`, `gamma=0.99`, `lam=0.95`, `learning_rate=1e-3` (adaptive), `desired_kl=0.01`, `num_steps_per_env=24`, `num_learning_epochs=5`, `num_mini_batches=4`, `max_iterations=10000`. |
| **Framestacking?**                              | The current `PolicyCfg` does NOT stack frames — each term is a single-step observation. No history buffer needed on the robot side. |
| **Reward shaping / what behavior was rewarded** (e.g. "why does it lift its feet so high?") | `source/Qmini/tasks/qmini_locomotion/qmini_env_cfg.py` `QminiRewards` and `source/Qmini/tasks/qmini_locomotion/mdp/rewards.py` |
| **Domain randomization / what variation the policy is robust to** | `source/Qmini/tasks/qmini_locomotion/qmini_env_cfg.py` `EventCfg` (friction, base mass, push, joint reset scales) |
| **Termination criteria** (what "failure poses" the policy was trained to avoid) | `qmini_env_cfg.py` `TerminationsCfg` — `bad_orientation` at 0.9 rad tilt, `base_too_low` at 0.15 m height |
| **Training / play entry points**                | `scripts/rsl_rl/train.py`, `scripts/rsl_rl/play.py`. The exported ONNX comes out of `play.py`. |

When numbers don't match between the two repos, **change the ROS 2 side to match the Isaac Lab side**, not the other way around — the policy was trained against those exact values and re-tuning the simulator after the fact invalidates the policy.

## Calibration as a first-class deliverable

The ROS 2 side does **not tune** the robot to make it walk — it **measures** what the robot actually is, so the Isaac Lab side can be widened to cover that reality. Calibration is a permanent package (`qmini_calibration/`), not a throwaway script, and its outputs flow back into the training-side cfgs before every retrain. Without this feedback loop, every retrain is a guess.

### Workflow

Each calibration is a node that:

1. Brings up only the minimum hardware needed (e.g. just `qmini_imu`, or `qmini_hardware` with the policy disabled).
2. Drives the robot through a defined protocol — open-loop or low-gain PD only, **never the policy**.
3. Records to `rosbag2`: `/joint_states`, `/imu/data`, `/motor_command`, plus custom `/calibration/event` markers per phase.
4. Hands the bag to an offline analyzer (`qmini_calibration/analysis/*.ipynb`) that appends one row to `calibration_results.yaml`.
5. A diff script imports the Isaac Lab cfgs directly (they are Python configclasses — `from Qmini.tasks.qmini_locomotion.qmini_env_cfg import QminiEnvCfg`) and prints a green/red report comparing each measured value to its trained-against range.

### What gets measured

| Measurement                                         | Compared against (Isaac Lab)                                                                 |
|-----------------------------------------------------|-----------------------------------------------------------------------------------------------|
| Joint zero offset & direction per motor             | URDF joint frame in `source/Qmini/assets/descriptions/qmini/qmini.urdf`                       |
| IMU noise floor (gyro / accel / proj-gravity)       | `Unoise(-0.35, 0.35)` on `imu_ang_vel`, `(-0.1, 0.1)` on `imu_projected_gravity` in `PolicyCfg` |
| IMU mount offset & orientation                      | `ImuCfg.OffsetCfg(pos=(-0.04718, 0.0663, 0.11094), rot=(1, 0, 0, 0))`                         |
| Per-channel bus polling rate & PD loop jitter (Pi 5)| Caps the achievable `decimation` choice on the sim side                                       |
| Actuator round-trip latency                         | `DelayedPDActuatorCfg(min_delay=0, max_delay=0)` — currently zero, almost certainly wrong     |
| Effective kp / kd of motor + onboard PD             | `stiffness` / `damping` dicts in `DelayedPDActuatorCfg`                                       |
| Static / kinetic friction at ankle / floor          | `physics_material.static_friction_range = (0.1, 2.0)` in `EventCfg`                           |
| Base mass                                           | `add_base_mass.mass_distribution_params = (-0.5, 0.5)` around URDF base mass                  |

### Order of operations (each step unblocks the next)

1. **Joint zero / direction** — without this, every other measurement is meaningless.
2. **IMU mount & noise floor** — cheap; the robot doesn't need to move.
3. **Bus polling rate & loop jitter** — tells you what `decimation` value the sim should target.
4. **Actuator latency + effective PD** — the two most likely sim-to-real gaps.
5. **Friction** — usually fine; do last.
6. **Run the diff script.** Any red row → widen the Isaac Lab DR range and retrain before deployment.

### Conventions

- Bags live in `qmini_calibration/data/<YYYY-MM-DD>_<measurement>/`. Do **not** commit bags; commit only the resulting YAML row.
- Analyzers are Jupyter notebooks in `qmini_calibration/analysis/`. Each notebook **appends** one row to `calibration_results.yaml` — it does not rewrite the file.
- Calibration nodes must refuse to start if the policy node is running. Policy and a calibration sweep must never share the bus simultaneously.
- Timestamps are produced at the **driver layer** (`qmini_hardware`, `qmini_imu`), never re-stamped in the analyzer. Latency numbers are only as good as the driver's timestamping discipline.
- The diff script imports Isaac Lab cfgs directly rather than duplicating the DR ranges in this repo — there must be exactly one source of truth.

## Safety architecture

This robot has **no hardware E-stop**. Stop is enforced entirely in software, via a PS4 DualShock over **Bluetooth**. This is the largest single risk in the project — a future hardware NC button in the 24 V motor rail is strongly recommended but not blocking. Until then, the design assumes the wireless link is unreliable and enforces stop via layered watchdogs.

### Components

- **Hardware bus:** PS4 DualShock → Bluetooth (Pi 5) → `joy_node` publishes `sensor_msgs/Joy`.
- **`qmini_joystick`** parses `Joy` into a `qmini_msgs/JoystickCommand` (vx, vy, wz, deadman bool, requested mode, requested stop event, battery %).
- **`qmini_safety`** subscribes to BOTH the raw `/joy` (for the watchdog) and the parsed `JoystickCommand` (for state-machine input). It owns the canonical safety state and publishes:
  - `qmini_msgs/SafetyHeartbeat` at 50 Hz (consumed by `qmini_hardware`).
  - `qmini_msgs/MotionGate` (latched) — tells `qmini_rl` and `qmini_controllers` whether motion is enabled.

### Layered watchdogs (defense in depth)

1. **`/joy` heartbeat watchdog** in `qmini_safety`: no `/joy` message in **100 ms** → automatic hard stop. Covers BT dropout, dead controller, `joy_node` crash.
2. **Deadman:** L1 must be held continuously for motion to be enabled. Release zeros vx/vy/wz but leaves torque on (robot keeps standing). Released-deadman is the default for all degraded states.
3. **`qmini_safety` → `qmini_hardware` heartbeat:** the motor driver requires the safety heartbeat at ≥ 50 Hz on a reliable topic; loss → motors zero torque autonomously. Covers a `qmini_safety` crash.
4. **Battery watchdog:** warn at < 20% DS4 battery, force soft-stop at < 5%. Dead controller mid-walk is the most common deadman-loss failure.
5. **BT-pairing precondition at bringup:** `qmini_bringup` checks the expected DS4 MAC is bonded before launching motion-enabling nodes. If not, refuses to start.

### Two-tier stop semantics

| Stop type | Trigger | Behavior | Recovery |
|---|---|---|---|
| **Soft stop** | Options button (press) | Policy disabled; robot holds last commanded joint position under firmware PD | Press X (walking mode) or Triangle (home pose) |
| **Hard E-stop** | PS + Options (hold ≥ 0.5 s) **OR** any watchdog trip | All `tau_ff = 0`, `kp = 0`, `kd = 0` in MotorCmd → robot collapses but is not driven into a fall | L2 + R2 (hold ≥ 1 s) — deliberate two-handed gesture |

Hard stop is **latched**. Once tripped, only a deliberate release recovers — an accidental button-release can't "un-stop" the robot mid-fall.

### PS4 button mapping

| Input | Function |
|---|---|
| Left stick X / Y | `vx`, `vy` |
| Right stick X | `wz` |
| **L1 (hold)** | **Deadman** — motion enabled only while held |
| **Options (press)** | Soft stop |
| **PS + Options (hold ≥ 0.5 s)** | Hard E-stop (latched) |
| **L2 + R2 (hold ≥ 1 s)** | Release latched stop |
| Triangle | Mode: hold home pose |
| X | Mode: walking |
| D-pad ↑ / ↓ | Velocity-cap ramp (training wheels: bump max-vel up or down 10% per press) |
| Square | Re-tare IMU (calibration helper) |
| Circle | (reserved) |

### DDS QoS for safety topics

All safety-relevant topics use:

- `RELIABLE` reliability
- `TRANSIENT_LOCAL` durability (`KEEP_LAST(1)`)
- This ensures a late-subscribing node (or one that briefly disconnects) immediately learns the current safety state instead of starting in an unknown one.

### Hard rules

- **Calibration nodes (`qmini_calibration`) must refuse to start if the policy node is running.** (Restated from the calibration section — it's a safety rule.)
- **`qmini_safety` is the only publisher of `MotionGate` and `SafetyHeartbeat`.** No other node may emit these. The motor driver trusts only `qmini_safety`.
- **Soft-stop holds last position; hard-stop zeros torque.** Do not invert these — holding torque on a tipping robot accelerates the fall.

### Physical fall-protection setup (as of 2026-05-23)

The user's current plan for fall protection is **low platform + human catch**, with no tether or gantry. This is fine for M0–M5 but is the limiting factor on M6. The chosen plan dictates extra M6-only rules baked into `qmini_safety`:

- **Velocity ramp starts at 5% of trained range**, not 100%. D-pad ↑ increases by 10% per press, with no auto-increase. M6's `qmini_safety` config (`config/m6_safety.yaml`) hard-caps initial speed.
- **Step counter cutoff:** `qmini_safety` counts foot-contact events from the contact-sensor / IMU heuristic and forces soft-stop after **3 steps** on the first run. Resetting the counter requires releasing and re-engaging the deadman.
- **Two-person rule:** M6 launch refuses to start unless `qmini_bringup` is invoked with `two_person_present:=true`. This is a checkbox, not a real check — but it forces the operator to think.
- **Battery preconditions at M6 launch:** DS4 ≥ 60%, Pi 5 not in low-voltage state. Hard-fail otherwise.

These rules are **only** active in M6 (`mode: walking` with non-zero velocity command). M2/M3/M5 are unchanged.

A future simple chest-strap-to-vertical-post upgrade is strongly recommended before any sustained walking development, and would let us relax some of the above (especially the step-cutoff).

## Things to confirm with the user (not in either repo)

1. **Motor protocol details** for the GO-M8010-6 on RS-485 (CRC, command IDs, units for position/velocity/torque). Unitree's SDK is the reference; mirror its framing rather than re-deriving it.
2. The **homing / calibration procedure** to go from "power on" to the `init_state.joint_pos` crouch pose before handing off to the policy.
3. The **exact path of the exported ONNX file** and whether it bundles the action scale + default offset, or expects the runtime to apply them.
