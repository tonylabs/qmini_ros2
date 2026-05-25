# M2.3 Energize — Drive to Home Under Torque

The first time the robot moves under power. `qmini_hardware` consumes
`/motor_command` from the controller and drives the motors to the home/standing
crouch, gated by the deadman + safety heartbeat.

> ⚠️ **The rope stays on for all of M2.3.** This robot has no hardware E-stop,
> and driving to the home crouch is **not balancing** — it holds joint angles,
> it does not hold itself upright. The rope is the only fall protection. It does
> not come off until M6 (and even then: low platform + human catch).

### Home-pose hold is NOT standing — expect it to fall

Holding the home pose under PD will **not** make the robot stand on its own; left
unsupported it tips over (typically **forward**, since the crouch's CoM sits ahead
of the feet). **This is expected, not a bug, and you must not try to fix it by
re-tuning the home pose** — those angles are copied from Isaac Lab and changing
them invalidates the trained policy.

A static joint-angle hold has no feedback loop: nothing reads the IMU and corrects.
Robust standing is a **closed-loop behavior** — the RL policy reads IMU + joint
state and adjusts joint targets every ~20 ms (50 Hz), actively keeping the CoM over
the feet, even in standing mode. That arrives with **M5 (`qmini_rl`)**, the ONNX
policy runner publishing `/joint_target`. (For reference, the `qmini_official_rl`
deployment that stood robustly did exactly this — see `rl_controller.cpp` in
`qmini_official_sdk`, running the ONNX policy in a loop.)

So M2.3's purpose is narrow: confirm torque flows, the safety gating works, joint
**directions** are correct, and the ramp is smooth — all on the rope. The home pose
here is just the **nominal pose the policy will output deltas around**
(`q_des = home + 0.5 × policy_output`), not a balance controller.

## Prerequisites

- **M2.0 homing valid** for this power session (see [2-HOMING.md](2-HOMING.md)).
  Remember the GO-M8010-6 zeros at power-up — if you power-cycled since homing,
  re-verify `/joint_states ≈ 0` at the jig pose first.
- **M2.1/M2.2 verified** (see [4-CONTROLLER_TEST.md](4-CONTROLLER_TEST.md)) — the
  controller publishes the home pose and the bus consumes it (torque off).
- **Joystick on and connected** (see [3-JOYSTICK.md](3-JOYSTICK.md)).
- **Robot rope-suspended**, with enough slack for the legs to fold into the
  crouch (hips/knees move ~1.5 rad from straight).

## The three torque gates

Current flows to the motors **only** when all three are true:

1. `enable_motor_torque:=true` — the launch arg (**default false**).
2. `MotionGate == ENABLED` — you are holding **LB + RB** (two-handed deadman).
3. The safety heartbeat is fresh (<100 ms).

Otherwise the bus sends `kp=kd=tau=0` (freewheel). Mapped to safety states:

| State | Behavior |
|---|---|
| ENABLED + fresh command | drive to target |
| SOFT_STOPPED (deadman released) | **hold** last commanded position under PD |
| HARD_STOPPED / BOOTING / stale heartbeat / no command | **zero torque** |

---

## Step 1 — Verify joint **direction** before energizing

Homing fixed each joint's *zero offset*, but not its *rotation direction*. If a
motor spins opposite the URDF convention, commanding the home pose drives that
joint the wrong way — a classic symptom is **the feet converging** (instead of a
proper stance) because hip-roll / hip-yaw move inward. `/joint_states` will NOT
reveal this: it reads back through the same encoder, so it always echoes the
commanded value regardless of physical direction.

So confirm directions first, with **no torque**.

### Launch the local URDF viewer with sliders

```bash
# build the description packages once if you haven't:
colcon build --symlink-install --packages-select qmini_msgs qmini_description qmini_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch qmini_bringup view_robot.launch.py
```

This opens RViz (the robot model) plus a small "Joint State Publisher" window
with one slider per joint. No hardware involved — it's pure kinematics.

### How to read the sign

In the slider window:

1. **Isolate `hip_roll_l`.** Drag only the `hip_roll_l` slider. Watch the left
   foot in RViz:
   - Move it toward **−0.1** (its home value). Does the foot go **outward**
     (abduct, wider stance) or **inward**?
   - That tells you what "−0.1" means geometrically in the URDF.
2. **Isolate `hip_yaw_l`.** Set everything else back, drag `hip_yaw_l` toward
   **+0.4**. See which way the leg rotates (toe-in vs toe-out).
3. **Set the full home pose** (all sliders to the `home_pose.yaml` values) → this
   is the "good" stance you already saw. Now you know the intended geometry and
   the sign of each lateral joint.

> Tip: the slider GUI has a **"Center"** button — use it to zero all joints back
> to the jig pose between tests so you isolate one joint at a time.

### Then compare to the real robot (torque OFF, suspended)

```bash
ros2 launch qmini_bringup controller_test.launch.py   # torque off
python3 scripts/joint_monitor.py
```

Hand-abduct the left leg (foot outward) and read `hip_roll_l`:
- RViz said abduction = negative, and the real robot also reads negative when you
  abduct → direction OK.
- Signs **disagree** → that joint is inverted, and that's why the feet converge
  under power.

> **Mirror trap:** left and right are mirrored (`hip_roll_l` home −0.1,
> `hip_roll_r` home +0.1), so abduction has **opposite** signs between the two
> legs. That is expected — it is NOT an inversion. A joint is inverted only when
> the **viewer and the real robot disagree for the *same* leg**, never because
> left and right differ from each other.

Do the same for `hip_yaw_l` / `hip_yaw_r` (rotate the leg about the vertical
axis — toe-in vs toe-out). The pitch joints (hip_pitch / knee / ankle) were
already validated by the 90°→1.57 rad check during homing, so they don't need
re-checking here.

### Results for this robot (record what you find)

Fill one row per lateral joint. "Inverted?" is **yes only when the two sign
columns disagree**.

| Joint | Viewer: abduct/yaw-out → sign | Real robot: same motion → sign | Inverted? → `direction` |
|---|---|---|---|
| `hip_roll_l` | + (0 → +0.300) | − (+0.8 → 0) | **yes → -1** |
| `hip_roll_r` | − (0 → −0.300) | + (−0.9 → 0) | **yes → -1** |
| `hip_yaw_l`  | − (0.7 → −0.1) | − (0.8 → 0) | no → +1 |
| `hip_yaw_r`  | + (−0.7 → 0.1) | + (−0.8 → 0) | no → +1 |

_(Verified 2026-05-25: both hip-rolls inverted — `direction: -1` set in
`motor_layout.yaml`; both hip-yaws agree with the URDF and stay `+1`. The
hip-roll inversion is why the feet converged at the home pose.)_

### Fix an inverted joint

Set `direction: -1` for that joint in
`src/qmini_hardware/config/motor_layout.yaml` (alongside `gear_ratio`). Then:

1. **Relaunch — no rebuild.** `motor_layout.yaml` is already symlink-installed
   and the node reads it at startup, so a content-only edit needs **no
   `colcon build`** — just kill and relaunch the motor bus node so it re-reads
   the file. (Rebuild is only needed when you add a *new* file.)
2. **Re-home.** The offset is captured as `offset = direction × (motor_q /
   ratio)`, so flipping `direction` flips that joint's stored offset sign — the
   existing `joint_offsets.yaml` value is now wrong for it. Return the robot to
   the jig pose and re-run the capture (see [2-HOMING.md](2-HOMING.md)). The
   joints left at `+1` recapture to the same values; the flipped ones change
   sign. (No power-cycle needed if power stayed on since the last homing; if you
   power-cycled, re-pose the jig **before** power-up as usual.)

Re-run the comparison until every joint matches the URDF.

---

## Step 2 — Energize

Only once Step 1 passes for every joint:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch qmini_bringup controller_test.launch.py enable_torque:=true
```

You'll see a `*** TORQUE ENABLED ***` warning at startup — confirmation the arm
flag took. With the deadman **released**, nothing drives yet (freewheel).

**Optional gentle first pass:** add `gain_scale:=0.3` to verify direction and
stability softly, then relaunch at full (`gain_scale` defaults to 1.0).

- **Hold LB + RB** → the robot ease-out ramps from its current pose to the home
  crouch over ~3 s under full PD. Keep your thumbs ready to release.
- **Release LB + RB** → it **holds** the crouch (soft stop, PD on). It does NOT
  go limp.
- **Fully de-energize** → `Ctrl-C` the launch, or trigger a hard stop (let `/joy`
  go stale, or the hard-stop gesture) → zero torque, legs limp.

## Abort checklist

- Anything looks wrong (wrong direction, oscillation, runaway) → **release the
  deadman** (holds position) or **`Ctrl-C`** (drops torque). The rope catches it.
- Re-run with `gain_scale` lower if motion is too aggressive.
- Never remove the rope. Never command past the position limits (the controller
  clamps, but verify limits match real mechanical travel).

## Next

M5 wires `qmini_rl` (the ONNX policy) to publish `/joint_target`, so the robot
does more than hold a static pose. Standing/walking unsupported is M6.
