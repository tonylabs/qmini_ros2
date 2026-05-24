# M2.3 Energize — Drive to Home Under Torque

The first time the robot moves under power. `qmini_hardware` consumes
`/motor_command` from the controller and drives the motors to the home/standing
crouch, gated by the deadman + safety heartbeat.

> ⚠️ **The rope stays on for all of M2.3.** This robot has no hardware E-stop,
> and driving to the home crouch is **not balancing** — it holds joint angles,
> it does not hold itself upright. The rope is the only fall protection. It does
> not come off until M6 (and even then: low platform + human catch).

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
colcon build --symlink-install \
  --packages-select qmini_msgs qmini_description qmini_bringup \
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

Do the same for `hip_yaw_l` and the right-leg mirror.

### Fix an inverted joint

Set `direction: -1` for that joint in
`src/qmini_hardware/config/motor_layout.yaml` (alongside `gear_ratio`), then:

```bash
colcon build --symlink-install --packages-select qmini_hardware \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

**Re-home** afterward — the offset sign changes with direction (see
[2-HOMING.md](2-HOMING.md)). Joints left at `+1` (default) are unaffected. Re-run
the comparison until every joint matches the URDF.

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
