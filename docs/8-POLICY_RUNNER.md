# Policy Runner — ONNX Inference (M5)

The `qmini_rl` package: the M5 ONNX **policy runner**. Loads the
Isaac-Lab-exported policy (`obs[44] →
action[10]`), reconstructs the exact training observation on-robot, runs
inference at 50 Hz, and publishes absolute joint-position targets on
`/joint_target` (`sensor_msgs/JointState`). `qmini_controllers/pd_packer_node`
turns those into `MotorCommand`s.

```
/imu/data ─┐
/joint_states ─┼─► observation[44] ─► ONNX ─► action[10] ─► q_des = home + 0.5·a
/cmd_vel ──┘                                                   │
/safety/motion_gate (gate) ───────────────────────────────────┴─► /joint_target
```

## Run

```bash
# point at an ONNX Runtime install (see below), then:
ros2 launch qmini_rl policy.launch.py                 # gated on MotionGate=ENABLED
ros2 launch qmini_rl policy.launch.py gate_required:=false   # bench test, no safety
```

Velocity command comes in on `/cmd_vel` (`geometry_msgs/Twist`:
`linear.x→vx, linear.y→vy, angular.z→wz`), clamped to the trained ranges. In the
full system `qmini_safety` will publish the gated, speed-capped command here;
for bench tests use `teleop_twist_keyboard` or `ros2 topic pub`.

## ONNX Runtime

Not a ROS package — install a prebuilt release and point `ONNXRUNTIME_DIR` at it
before `colcon build`. CMake searches `include/{,onnx}` and `lib/{,onnx}`.

- **Pi 5 (aarch64, deployment):** the aarch64 `libonnxruntime.so` (v1.19.2) is
  vendored in `qmini_official_sdk/{include,lib}/onnx` — `export
  ONNXRUNTIME_DIR=~/Documents/GitHub/qmini_official_sdk`.
- **x86 dev box:** download the matching x64 release
  (`onnxruntime-linux-x64-1.19.2.tgz`) and `export ONNXRUNTIME_DIR=/path/to/it`.
  The SDK's lib is ARM-only and won't link on x86.

At **runtime** the `.so` must be on `LD_LIBRARY_PATH` (e.g.
`export LD_LIBRARY_PATH=$ONNXRUNTIME_DIR/lib:$LD_LIBRARY_PATH`).

## Parity with Isaac Lab — every constant is mirrored, not invented

See `qmini_rl/config/policy.yaml` and the header comment in
`qmini_rl/src/policy_runner_node.cpp`.
Observation order (no normalization / clipping / framestack / on-robot noise):

| # | term | dims | scale |
|---|------|------|-------|
| 1 | imu_ang_vel | 3 | 0.2 |
| 2 | imu_projected_gravity | 3 | 1.0 |
| 3 | joint_pos_rel = q − home | 10 | 1.0 |
| 4 | joint_vel_rel = dq | 10 | 0.05 |
| 5 | last_action (prev **raw** output) | 10 | 1.0 |
| 6 | velocity_commands [vx,vy,wz] | 3 | 1.0 |
| 7 | gait_phase_sincos `[sinL,sinR,cosL,cosR]` | 4 | 1.0 |
| 8 | static_flag (1.0 if ‖[vx,vy,wz]‖₂ < 0.15) | 1 | 1.0 |

- Action: `q_des[i] = home[i] + 0.5 · action[i]`.
- Gait phase: free-running 1.5 Hz, `φR = φL + 0.5`, `+0.03`/step, **never freezes**.
- `gait_phase_sincos` element order follows the Isaac Lab **code** (`[sinL, sinR,
  cosL, cosR]`), not its docstring.

## Joint order (verified)

`kJointOrder` is the **policy's** joint order — the order the ONNX obs/action use.
**Verified 2026-05-27** via `env.scene["robot"].joint_names` in Isaac Lab
`play.py`:

```
[hip_yaw_l, hip_yaw_r, hip_roll_l, hip_roll_r, hip_pitch_l, hip_pitch_r,
 knee_pitch_l, knee_pitch_r, ankle_pitch_l, ankle_pitch_r]
```

This is **interleaved by joint type** (l, r, l, r, …) — **different** from the
hardware/canonical **by-leg** order in `qmini_controllers`/`qmini_hardware`
(all-left-then-all-right). That's intentional and safe: `qmini_rl` reads
`/joint_states` + home pose **by name** into this order, runs the ONNX in it, and
publishes `/joint_target` as a `JointState` with explicit joint **names** —
`pd_packer` maps the target by name, so the two orderings never need to agree.
**Re-verify against `play.py` if the policy is retrained.**

## Status / before you trust it on hardware

Built and validated on x86 with synthetic inputs: 50 Hz, dims checked, bounded
deltas around home for a zero (standing) command. Joint order verified (above).
**Not yet run on the real robot.** Still to settle:

- **IMU frame alignment** — `imu_projected_gravity` assumes `/imu/data` is in
  `base_link`. Resolved 2026-05-26 via the `qmini_imu` `mount_rotation`
  (`[0,0,1,0]`); the M4 IMU diff is GREEN.
- **aarch64 onnxruntime** build on the Pi (lib vendored in `qmini_official_sdk`).

`qmini_rl/policies/policy.onnx` is the current export; replace it with the final
validated export when retraining settles.
