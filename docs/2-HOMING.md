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

## Procedure

1. **Power the motors** (24V) and launch the bus (motors freewheel):

   ```bash
   ros2 launch qmini_bringup motor_bus_test.launch.py
   ```

   On first run you'll see a warning that offsets are UNCALIBRATED — expected.

2. **Place the robot in the all-joints-zero jig pose** — every joint aligned to
   its 0 rad reference (alignment marks / jig / straight-leg reference). Hold it
   there steadily.

3. **Capture** (in a second sourced shell):

   ```bash
   ros2 service call /motor_bus_node/capture_home_offsets std_srvs/srv/Trigger
   ```

   The service sets `offset = motor_q / gear_ratio` for every responding motor,
   overwrites `joint_offsets.yaml`, and reports how many of 10 it captured (and
   names any that were not responding).

4. **Verify** — still holding the jig pose:

   ```bash
   ros2 topic echo /joint_states --once
   ```

   All `position` values should now read **≈ 0**. Move a joint a known amount
   and confirm it tracks with the right sign.

5. The offsets persist in `joint_offsets.yaml` and load automatically on every
   subsequent launch. Re-run only if the mechanism is re-assembled or a motor's
   zero shifts.

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
