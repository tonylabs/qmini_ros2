# Joint Homing (Zero-Offset Calibration)

The GO-M8010-6 has only a **rotor-side encoder**, so the joint zero is unknown
at power-on. Each joint needs a one-time (per-robot) **offset** so the motor bus
can report true joint angles and accept correct position commands. The motor bus
applies, **per joint**:

```
joint_q   = motor_q  / gear_ratio - offset
joint_dq  = motor_dq / gear_ratio
joint_tau = motor_tau * gear_ratio
```

`gear_ratio` is **6.33** for all joints **except hip-roll** (`hip_roll_l`,
`hip_roll_r`), which carry a second gear stage → **18.99**. These live in
`src/qmini_hardware/config/motor_layout.yaml`. The `offset` values live in
`src/qmini_hardware/config/joint_offsets.yaml` and are produced by the
procedure below.

> **No torque is applied during homing.** The motors freewheel (M1 mode); this
> is purely read-and-record. Offsets are **per-robot** — never copy another
> robot's `joint_offsets.yaml`.

## ⚠️ The GO-M8010-6 zeroes at power-up — pose the robot FIRST

The GO-M8010-6 has no battery-backed absolute reference: it establishes
`motor_q ≈ 0` **at the instant 24 V is applied**, wherever the rotor happens to
sit. This dictates the order of operations:

- **Pose the robot at the all-joints-zero jig pose BEFORE powering the motors.**
  Then the motor reads ~0 at the true zero on *every* boot, and the captured
  offset (a tiny residual) is reproducible and survives power cycles.
- If you instead power up at a random pose and then move to the jig pose, the
  captured offset is only valid for that session — the next power cycle re-zeros
  somewhere else and the saved offset is wrong. **Don't do this.**

So the capture service records only the small residual that remains after a
correct power-up; it is **not** a substitute for posing the robot at power-up.

## Procedure

1. **Hold the robot at the all-joints-zero jig pose** — every joint aligned to
   its 0 rad reference (alignment marks / jig / straight-leg reference). Hold it
   there steadily.

2. **Power on the motors (24 V) while still holding the jig pose.** This sets
   each motor's zero at (or veryw near) the true joint zero. Then launch the bus
   (motors freewheel):

   ```bash
   ros2 launch qmini_bringup motor_bus_test.launch.py
   ```

   On first run you'll see a warning that offsets are UNCALIBRATED — expected.

3. **Capture** (in a second sourced shell), still holding the jig pose:

   ```bash
   ros2 service call /motor_bus_node/capture_home_offsets std_srvs/srv/Trigger
   ```

   The service sets `offset = motor_q / gear_ratio` for every responding motor,
   overwrites `joint_offsets.yaml`, and reports how many of 10 it captured (and
   names any that were not responding).

4. **Verify** — still holding the jig pose. A one-shot echo is enough here:

   ```bash
   ros2 topic echo /joint_states --once
   ```

   All `position` values should now read **≈ 0** (a few mrad of sensor noise is
   fine).

5. **Sign / ratio check** — move a joint a *known* amount and confirm both the
   magnitude and the direction. Use the live monitor so you can watch the value
   change in place instead of re-running `--once` (it auto-redraws and shows
   joint names):

   ```bash
   python3 scripts/joint_monitor.py        # Ctrl-C to quit; --rate N to tune
   ```

   - **Standard ratio (6.33), wide-range joint:** rotate `hip_pitch_l` (or
     `knee_pitch_l`) to a visible **~90° (≈ 1.57 rad)** and confirm the monitor
     reads **≈ 1.57**. This validates the 6.33 ratio used by the 8 non-hip-roll
     joints.
   - **Hip-roll ratio (18.99):** `hip_roll_l`/`hip_roll_r` only travel ~15°
     mechanically, so move it to its stop and confirm the monitor reads the true
     small angle (**~15° ≈ 0.26 rad**). With the *old, wrong* 6.33 ratio it would
     have read **3× too large (~0.79 rad / 45°)** — physically impossible for a
     15°-travel joint, so even this tiny motion proves the 18.99 ratio is
     applied.
   - **Direction:** confirm each joint tracks with the **right sign** (matching
     the URDF joint axis). A wrong sign means the motor's rotation is opposite
     the URDF axis — see Notes.

6. The offsets persist in `joint_offsets.yaml` and load automatically on every
   subsequent launch. Re-run only if the mechanism is re-assembled or a motor's
   zero shifts.

## Confirm reproducibility (do this once)

Because the motor zero is set at power-up, this test decides whether the saved
offsets are trustworthy across reboots:

1. After capturing, **power-cycle the motors** — hold the jig pose, power on
   while holding it, then relaunch the bus.
2. Echo `/joint_states` at the jig pose **without re-running the capture**
   (you are checking the *saved* offsets, not making new ones):
   - **≈ 0** → the zero is reproducible; the committed `joint_offsets.yaml` is
     trustworthy across reboots. You're done.
   - **drifted** → the power-up pose wasn't tight enough, or a motor crossed a
     turn boundary. Re-home and tighten the jig fit. If drift persists, the zero
     is **not** reproducible on this hardware and homing must be re-captured on
     every boot — which changes how the energizing path (M2.3) must start up
     (capture before driving to the standing pose, rather than trusting the
     saved file).

## Notes

- If a motor is ABSENT (not responding) during capture, it is **skipped** and
  keeps its previous offset; the service result lists it. Fix the connection and
  re-run.
- Sign / direction: if a joint reads the wrong sign after homing, the motor's
  rotation is opposite the URDF joint axis. (Direction handling beyond offset is
  tracked for a later step — flag it if you see it.)
- This is the minimal pull-forward of the `qmini_calibration` (M4) "joint zero /
  direction" measurement, done early because it is a hard prerequisite for any
  position command in M2.
