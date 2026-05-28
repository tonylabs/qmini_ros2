# Joystick / Teleop Controller — Setup & Troubleshooting

> **Status: WORK IN PROGRESS (started 2026-05-24).** This captures what we
> learned bringing up the controller during M2. The ROS 2 teleop nodes
> (`qmini_joystick`, deadman gating in `qmini_safety`) are **not built yet** —
> sections marked **TODO** get filled in as M2/M6 land. For now this is a
> hardware/OS bring-up + `evtest` troubleshooting reference.

## Prerequisites — install the test tools first

The CLI test tools are **not installed by default** on Ubuntu (`jstest`/`evtest`
will report "command not found"). Install them before anything below:

```bash
sudo apt update
sudo apt install -y joystick evtest    # joystick provides jstest + jscal
```

| Tool | Package | Reads | Use |
|---|---|---|---|
| `jstest /dev/input/jsN` | `joystick` | legacy joydev node | one live-updating line of axes+buttons (easiest) |
| `evtest /dev/input/eventN` | `evtest` | raw evdev | named events (`ABS_X`, `BTN_TL`, …); also dumps the device descriptor |

ROS 2's `joy` package (the `joy_node`) is already installed with
`ros-humble-desktop`; no extra apt step for the ROS path.

## The actual controller (NOT what CLAUDE.md assumes)

CLAUDE.md's safety section was written for a **PS4 DualShock over Bluetooth**.
The real hardware is different:

- **Controller:** TPARTS handheld, talks over a **2.4 GHz USB dongle**
  (`RDMCTMZT Wireless 2.4G Dongle`, USB id `36b0:3002`) — **not** Bluetooth.
- It presents to Linux as a standard **Xbox 360 / XInput pad** (kernel `xpad`
  driver), device name `Microsoft X-Box 360 pad` (`045e:028e`).
- Consequences:
  - The **button/axis layout is the Xbox map, not the PS4 map** in CLAUDE.md.
    Re-map every function against the indices you actually measure here.
  - The **"BT pairing precondition"** watchdog in CLAUDE.md does not apply
    (2.4 GHz dongle, not BT). A 2.4 GHz link is generally more reliable than
    BT, but the `/joy` heartbeat watchdog (100 ms) is still the primary defense.
  - **Battery %** may not be reported over XInput — confirm before relying on
    the battery watchdog.

## The multi-interface gotcha (read this first)

The dongle is a **composite HID device** that splits into several input
nodes — keyboard, mouse, "System Control", "Consumer Control". **None of those
carry the gamepad sticks/buttons.** The gamepad is a *separate* device.

Inspect everything with:

```bash
cat /proc/bus/input/devices
```

What we saw (yours may differ in the `eventN` / `jsN` numbers — they are NOT
stable across reboots or replug, so always re-check):

| Device name | Handlers | What it is |
|---|---|---|
| `RDMCTMZT Wireless 2.4G Dongle` | `event11` kbd | dongle keyboard collection — *not* the gamepad |
| `RDMCTMZT ... Mouse` | `event12 mouse2` | dongle mouse collection |
| `RDMCTMZT ... System Control` | `event13` | power/menu/sleep keys + a HAT; **udev mislabels this `-event-joystick`** |
| `RDMCTMZT ... Consumer Control` | `event14` | media keys |
| `RDMCTMZT ... Keyboard` | `event15` | another kbd collection |
| **`Microsoft X-Box 360 pad`** | **`event27 js0`** | **THE GAMEPAD** — sticks, buttons, triggers, rumble |

**Pick the device whose name is `Microsoft X-Box 360 pad` and that has a `jsN`
handler.** Its capability line shows `ABS=3003f` (X, Y, Z, RX, RY, RZ + HAT)
and a `KEY=` bitmask in the `BTN_GAMEPAD` range. Do **not** test the
`-event-joystick` symlink under `/dev/input/by-id/` for this dongle — udev tags
the System-Control node as a joystick because it has a HAT axis, which is a
red herring.

### Confirmed device descriptor (`evtest /dev/input/event27`, 2026-05-24)

The `Microsoft X-Box 360 pad` reports exactly the standard XInput set:

- **Buttons** (`EV_KEY`): `BTN_SOUTH`(304) `BTN_EAST`(305) `BTN_NORTH`(307)
  `BTN_WEST`(308) `BTN_TL`(310) `BTN_TR`(311) `BTN_SELECT`(314) `BTN_START`(315)
  `BTN_MODE`(316) `BTN_THUMBL`(317) `BTN_THUMBR`(318) — 11 buttons.
- **Axes** (`EV_ABS`): `ABS_X`/`ABS_Y` (left stick, ±32768), `ABS_RX`/`ABS_RY`
  (right stick, ±32768), `ABS_Z` (LT, 0–255), `ABS_RZ` (RT, 0–255),
  `ABS_HAT0X`/`ABS_HAT0Y` (D-pad, −1..1).
- `EV_FF` (rumble) supported.

> Note: at the raw evdev layer the triggers (`ABS_Z`/`ABS_RZ`) are 0–255 and
> rest at **0**. `joy_node` (SDL) remaps them to its own −1..+1 axes that rest
> at **+1.0** — that's why `/joy` `axes[2]`/`axes[5]` read `1.0` untouched.

## Quick functional test (no ROS)

`jsN` exists → use `jstest` (one live-updating line, easiest to read):

```bash
sudo apt install -y joystick
jstest /dev/input/js0          # use the jsN from /proc/bus/input/devices
```

Or read the raw evdev stream with named events:

```bash
sudo apt install -y evtest
sudo evtest /dev/input/event27 # use the eventN of the X-Box 360 pad
```

Actuate every stick / trigger / button. You want to see, on the gamepad node:

- `EV_ABS` events: `ABS_X`, `ABS_Y` (left stick), `ABS_RX`, `ABS_RY` (right
  stick), `ABS_Z` (LT), `ABS_RZ` (RT), `ABS_HAT0X/Y` (D-pad).
- `EV_KEY` events: `BTN_SOUTH`, `BTN_TR`, etc.

`evtest` first prints the **device descriptor** (the capability list above) and
then `Testing ... (interrupt to exit)`. The descriptor alone only proves the
*receiver* enumerated — you must see `Event:` lines stream when you actuate a
control to prove the *transmitter* is live.

## ROS 2 (`joy_node`)

The ROS 2 Humble `joy` package (SDL2-based) is installed and reads `jsN`/evdev
directly — it does **not** need anything beyond a working gamepad node.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run joy joy_node
# in another shell:
ros2 topic echo /joy
```

Expected from the X-Box 360 pad: **8 axes, 11 buttons**. At rest, `axes[2]`
and `axes[5]` sit at **+1.0** — those are LT/RT (SDL convention: rest +1.0,
fully pressed −1.0). Sticks are `axes[0],[1]` (left) and `axes[3],[4]` (right);
D-pad is usually `axes[6],[7]`.

> `joy_node` republishes at ~20 Hz (`autorepeat_rate`) even with no input, so a
> steady stream of *identical* frames is normal — it is NOT proof of live input.
> Confirm liveness by watching a value **change** when you actuate a control.

## Permissions

| Environment | Device access | Action needed |
|---|---|---|
| **Ubuntu Desktop** | `systemd-logind` auto-grants an ACL to the active graphical-seat user (`getfacl /dev/input/eventN` shows `user:<you>:rw-`) | none — works out of the box |
| **Ubuntu Server / headless SSH** (the Pi 5 target) | no graphical seat → **no auto-ACL** → permission denied | add user to `input` group **or** install a udev rule (below) |

Headless fix (preferred — same pattern as `dialout`/udev for the motors):

```bash
# Option A: group membership
sudo usermod -aG input "$USER"
# CRITICAL: log out completely and log back in (or reboot). A new terminal
# alone is NOT enough — children of the shell inherit the group set that was
# fixed at login time, so `joy_node` spawned from a freshly-opened SSH still
# runs with the old groups. `newgrp input` fixes only the current interactive
# shell, not the joy_node you start from it. Verify after re-login:
groups | grep -o input    # must print: input

# Option B: udev rule (fold into config/udev/99-qmini.rules eventually)
echo 'SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="028e", MODE="0660", GROUP="input"' \
  | sudo tee /etc/udev/rules.d/99-qmini-joystick.rules
sudo udevadm control --reload && sudo udevadm trigger
```

> Note: the udev rule above matches the **gamepad** VID/PID (`045e:028e`,
> the xpad device). If a future controller enumerates under the dongle's own
> id (`36b0:3002`) instead, match that. Verify with `lsusb` after plugging in.

## Troubleshooting

### `/joy` (or evtest) shows only neutral/zero values, never changes

> **Observed 2026-05-24:** `jstest /dev/input/js0` opened the `Microsoft X-Box
> 360 pad`, saw all 8 axes / 11 buttons (triggers correctly at −32767), but
> **no value changed on actuation**. This confirmed the entire OS/driver/
> `joy_node` path is healthy — the issue was the **handheld→receiver radio
> link**, not software. (Resolution pending: bind/power on the controller.)

The gamepad node enumerates but the handheld transmitter isn't actually
sending. In order of likelihood:

1. **You're reading the wrong node.** Confirm you're on `Microsoft X-Box 360
   pad` (`jsN`/`event27`), not a dongle kbd/mouse/system-control node. See the
   table above.
2. **Transmitter powered off / asleep** — power it on; look for a *solid*
   (not blinking) status LED.
3. **Transmitter not bound to the dongle** — a blinking dongle/transmitter LED
   usually means "searching/unbound." Run the TPARTS bind procedure (typically
   hold a bind button while powering on; see the controller manual).
4. **Master/arm switch or throttle-lock** in a position that gates output.

### No `/dev/input/jsN` at all

The `joydev` module may not be loaded, or the device enumerated before it:

```bash
lsmod | grep joydev || sudo modprobe joydev
sudo udevadm trigger          # or replug the dongle
```

(SDL `joy_node` can often work off the `eventN` node without `jsN`, but
`jstest` needs `jsN`.)

### Permission denied opening the device

You're headless / not in `input` and have no ACL. See **Permissions** above.

### `joy_node` runs, `/joy` exists, but no messages — `topic hz /joy` is silent

> **Observed 2026-05-29:** `ros2 node list` shows `/joy_node`, `ros2 topic list`
> shows `/joy`, but `ros2 topic echo /joy` and `ros2 topic hz /joy` see nothing
> even on stick input. The `joy_node` debug log shows full ROS initialization
> but **no `Opened joystick:` line** — SDL never opened the device.

Root cause: SDL2's joystick subsystem reads `/dev/input/event*` (evdev), **not**
`/dev/input/jsN` (joydev). When the user is not in the `input` group, SDL
**silently** fails to open the device — no error, no log line. The node stays
up, advertises the topic, and never publishes a frame.

`jstest /dev/input/js0` working does **not** prove `joy_node` will work: joydev
nodes are often more permissive (0660 root:input may already be readable to
a freshly-installed user) than the evdev nodes SDL actually reads.

Fix:

```bash
groups | grep -o input || sudo usermod -aG input "$USER"
# then LOG OUT completely and log back in (or reboot) — see Permissions §
groups | grep -o input    # must print: input
ros2 run joy joy_node     # must print: Opened joystick: Microsoft X-Box 360 pad...
```

### `joy_node` opened the wrong controller

If multiple pads are present (e.g. a Razer device also exposes input nodes),
point `joy_node` at the right one:

```bash
ros2 run joy joy_node --ros-args -p device_id:=0   # try 0,1,... ; check the "Opened joystick:" log line
```

---

## TODO — fill in as M2/M6 progress

- [ ] **Measured button/axis map for the TPARTS controller** (table: function →
      `axes[i]`/`buttons[i]` → resting/active value). Pending live capture.
- [ ] **Deadman control** — which axis/button, hold-to-enable semantics.
      (RC-style transmitters sometimes map switches to **axes**, not buttons —
      but this unit reports a full standard XInput button set, so a real button
      like `BTN_TL` is available.)
- [ ] **Re-mapped safety button table** replacing CLAUDE.md's PS4 map
      (soft-stop, hard-stop, mode, velocity ramp, IMU re-tare).
- [ ] **`qmini_joystick` node** — `/joy` → `qmini_msgs/JoystickCommand`.
- [ ] **`qmini_safety` deadman gating** — `/joy` watchdog (100 ms) + open
      `MotionGate` only while deadman held.
- [ ] **Fold the joystick udev rule into `config/udev/99-qmini.rules`** so a
      fresh Pi flash gets motors + IMU + joystick access in one step.
- [ ] **Confirm whether battery % is available** over this controller's XInput.
- [ ] **Launch integration** — add `joy_node` to the relevant bringup launch.
