# M2.1 Controller Test — PD Packer (NO TORQUE)

Validates `qmini_controllers/pd_packer_node`: the thin command packer that turns
a joint-position target into a `qmini_msgs/MotorCommand`, gated by the deadman.

**This stage applies no torque.** Nothing subscribes to `/motor_command` yet
(that is M2.2), so the controller only *publishes* command packets you inspect.
The motor bus runs read-only, exactly as in the M1 smoke test.

## What the node does

`pd_packer_node`:
- subscribes `/joint_target` (absolute joint targets, from the policy in M5),
  `/joint_states`, and `safety/motion_gate`;
- emits `MotorCommand` on `/motor_command` at 50 Hz **only while
  `MotionGate == ENABLED`** (deadman held); silent in every other state;
- on enable, **ease-out ramps** from the robot's current pose to the target over
  `ramp_duration_s` (default 3 s) so the first command doesn't yank the joints;
- holds the **home / standing pose** until a policy publishes `/joint_target`;
- **clamps every `q_des`** to the per-joint `position_limits` (hard safety bound);
- attaches joint-side `kp`/`kd` (the conversion to motor-side happens later, in
  `qmini_hardware`, at M2.2).

All numeric values are copied verbatim from the authoritative Isaac Lab repo
`qmini_isaaclab` (`qmini.py`):
- gains + limits: `qmini_controllers/config/gains.yaml`
- home pose: `qmini_hardware/config/home_pose.yaml`

## Prerequisites

- M2.0 homing done and verified (see [2-HOMING.md](2-HOMING.md)) — the home pose
  is only meaningful with correct offsets.
- Bluetooth gamepad **on and connected** (see [3-JOYSTICK.md](3-JOYSTICK.md)).
- Motors powered (freewheeling) if running with `use_hardware:=true`.

## Run

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch qmini_bringup controller_test.launch.py
```

Launch args:
- `use_hardware:=false` — bench test, no robot. The ramp then starts from the
  home pose (no `/joint_states`). Default `true` brings up the read-only motor
  bus so the ramp starts from the real pose.
- `log_level:=debug` — more verbose.

Confirm the **joy_node line** names your gamepad, e.g.
`Opened joystick: Xbox Wireless Controller`. If it names a 2.4G dongle instead,
**unplug the dongle** (the BT pad must be `device_id 0`). Filtering by name was
tried and removed — the SDL name match proved too brittle.

## Verify (two terminals, sourced)

1. **Gamepad is the one feeding `/joy`:**
   ```bash
   ros2 topic echo /joy        # press LB / RB -> buttons[4], buttons[5] flip 0<->1
   ```

2. **Command gating + ramp:**
   ```bash
   ros2 topic echo /motor_command
   ```
   - Deadman **released** → no messages (controller silent).
   - Hold **LB + RB together** → `MotionGate -> ENABLED`; `/motor_command`
     publishes ~50 Hz. `q_des` eases from the current pose to the home pose over
     ~3 s, then holds. `kp`/`kd` show the joint-side gains; `dq_des`/`tau_ff` 0.
   - **Release** → publishing stops.

3. **Limits:** every `q_des` stays within `position_limits` from `gains.yaml`.

### Why LB **and** RB (two-handed deadman)

Motion is enabled only while **both** shoulder buttons are held — a two-handed
enable switch. Release = stop (the safe state is un-pressed). Requiring two
buttons on opposite shoulders forces both hands on the controller and resists
accidental enable. This is the deadman layer of the safety stack; see CLAUDE.md
*Safety architecture*.

## Expected log behavior (not errors)

- `MotionGate -> BOOTING` at startup, before the first `/joy` message.
- `MotionGate -> SOFT_STOPPED` once `/joy` arrives with the deadman released.
- `MotionGate -> HARD_STOPPED ... /joy stale` if `/joy` stops for >100 ms (e.g.
  the gamepad sleeps or the wrong device was opened). This is the watchdog
  working; it clears to SOFT_STOPPED when the link returns and the deadman is
  released.

## Build gotchas (this environment)

- **`No module named 'catkin_pkg'`** on `colcon build`: a conda Python is
  shadowing the system one. Build with the system interpreter pinned:
  ```bash
  colcon build --symlink-install \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
  ```
- **Added a new file (config/launch) but it's "not found" at runtime:**
  `--symlink-install` symlinks files at build time, so a brand-new file needs a
  rebuild of that package. Editing an existing file's contents does not. A fresh
  clone + full build is unaffected.

## Next: M2.2

Wire `qmini_hardware` to subscribe `/motor_command` and apply it to the motors
with offset + per-joint ratio + `kp/kd / ratio^2` conversion, gated by the
safety heartbeat. That is the first stage that can apply torque — energizing
(drive-to-home via the deadman) is M2.3.
