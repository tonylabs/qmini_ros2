# M1 Smoke Test — Motor Bus Bring-Up

This document walks through the first hardware-in-the-loop test for the Qmini
ROS 2 stack: bringing up `qmini_safety` + `qmini_hardware` (the motor bus
driver), verifying that the four RS-485 channels open, and confirming that
hand-rotating each motor produces real values in `/joint_states`.

If anything along the way deviates from the **Expected output** boxes below,
skip to the [Troubleshooting](#troubleshooting) section at the bottom.

## What this validates

- The C++ build chain compiles `qmini_hardware` against the vendored Unitree
  SDK.
- The safety heartbeat + latched motion-gate plumbing actually runs.
- All four FTDI RS-485 channels open at the configured baud rate (4 Mbps).
- The motor bus polls every motor and aggregates state into `/joint_states`
  at the configured rate (~200 Hz).
- Per-channel and per-motor mapping in `motor_layout.yaml` matches the
  physical wiring on your bench.

## What this does NOT validate

- The policy (M5) — `qmini_rl` isn't built yet, and `MotionGate` stays
  `HARD_STOPPED` throughout this test.
- The PD packer (M2) — no commands are sent; motors freewheel in FOC mode
  with `kp = kd = q = dq = tau = 0`.
- The IMU pipeline (M3) — `qmini_imu` isn't part of this launch.
- The joystick (M6) — no `/joy` subscription yet.

## Prerequisites

### Hardware

- 1× FTDI FT4232H 4-channel USB-to-RS485 adapter (serial `FTA98W5B`)
- At least 1× Unitree GO-M8010-6 motor (test motor — eventually 10)
- 24 V power supply for the motors (motors will not respond on logic power
  alone)
- USB-C cable for the FTDI adapter
- Pi 5 (deployment target) **or** any Ubuntu 22.04 x86_64 machine (dev). The
  procedure is identical; the SDK's `CMakeLists.txt` auto-selects the right
  prebuilt `.so`.

### Software

- Ubuntu 22.04
- ROS 2 Humble (`ros-humble-desktop` + `ros-dev-tools`)
- The repo cloned and the Unitree SDK submodule initialized:

  ```bash
  cd ~/qmini_ros2
  git submodule update --init --recursive
  ls src/qmini_hardware/third_party/unitree_actuator_sdk/CMakeLists.txt  # must exist
  ```

- Your user in the `dialout` group:

  ```bash
  groups | grep dialout || { sudo usermod -aG dialout "$USER"; echo "Log out and back in."; }
  ```

## Step 1 — Build the workspace

### 1.1 Open a clean shell

Start a fresh terminal. **Do not** start from a shell where conda or `uv` has
been activated — both will hijack the Python that `colcon` invokes, and the
ROS-installed packages (`catkin_pkg`, `empy`, etc.) won't be visible.
Symptoms of this collision look like:

```
ModuleNotFoundError: No module named 'catkin_pkg'
```

with a Python path that is not `/usr/bin/python3` (typically
`~/miniconda3/bin/python3` or `~/.local/bin/python3.12`).

### 1.2 Source ROS 2 + guard PATH

```bash
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:$PATH
which python3   # expected: /usr/bin/python3
```

If `which python3` is anything other than `/usr/bin/python3`, the build will
not find `catkin_pkg`. Run `conda deactivate` (possibly multiple times if
nested envs were active), then re-export the PATH line.

To make this permanent, add an alias to `~/.bashrc`:

```bash
alias ros2dev='export PATH=/usr/bin:$PATH; conda deactivate 2>/dev/null; source /opt/ros/humble/setup.bash; cd ~/qmini_ros2'
```

### 1.3 Install ROS dependencies

```bash
cd ~/qmini_ros2
rosdep install --from-paths src --ignore-src -r -y
```

This installs `yaml-cpp`, `rclcpp`, `sensor_msgs`, etc. that are listed in
each package's `package.xml`.

### 1.4 Compile

```bash
colcon build --symlink-install --packages-up-to qmini_hardware qmini_safety qmini_bringup
```

**`qmini_bringup` must be in the list.** It is *not* a build dependency of
`qmini_hardware` or `qmini_safety`, so `--packages-up-to qmini_hardware
qmini_safety` alone will NOT build it — and then the launch file you need in
Step 3 won't be installed (see [A7](#a7-launch-file-was-not-found-in-the-share-directory)).

The `--symlink-install` flag is important: it lets you edit launch files,
YAML configs, and Python sources without rebuilding. Note the caveat: it
symlinks files that exist *at build time*. A launch/config file added to a
package after its last build is not picked up until you rebuild that package.

**Expected output:**

```
Starting >>> qmini_msgs
Finished <<< qmini_msgs [several s]
Starting >>> qmini_hardware
Starting >>> qmini_safety
Finished <<< qmini_safety [a few s]
Finished <<< qmini_hardware [a few s]
Starting >>> qmini_bringup
Finished <<< qmini_bringup [<1s]

Summary: 4 packages finished
```

(`qmini_description` is not pulled in by this target — it's only needed for
the RViz/URDF launch, not the motor-bus test.)

If any package fails, jump to [A1–A8](#a-build-failures).

### 1.5 Source the workspace overlay

```bash
source install/setup.bash

# Verify the binaries exist:
ls install/qmini_hardware/lib/qmini_hardware/motor_bus_node
ls install/qmini_safety/lib/qmini_safety/safety_node
```

Both should be present and executable.

## Step 2 — Verify USB serial visibility

Plug in the FTDI 4-channel adapter, then:

```bash
ls -la /dev/serial/by-id/ | grep FTA98W5B
```

**Expected output:**

```
... usb-FTDI_USB__-__Serial_Converter_FTA98W5B-if00-port0 -> ../../ttyUSB0
... usb-FTDI_USB__-__Serial_Converter_FTA98W5B-if01-port0 -> ../../ttyUSB1
... usb-FTDI_USB__-__Serial_Converter_FTA98W5B-if02-port0 -> ../../ttyUSB2
... usb-FTDI_USB__-__Serial_Converter_FTA98W5B-if03-port0 -> ../../ttyUSB3
```

Four entries, one per RS-485 channel. The underlying `/dev/ttyUSB[0-3]`
numbers may vary across reboots — that's why `motor_layout.yaml` references
the stable `by-id` paths instead.

Confirm you can actually open the ports:

```bash
[[ -r /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA98W5B-if00-port0 ]] \
  && echo "readable" || echo "PERMISSION ISSUE — see B1"
```

## Step 3 — Phase A: launch with the adapter only (no motors yet)

This phase verifies the software path before bringing motors into the mix.
If anything in the launch itself is broken, you want to find out before
debugging is muddied by motor wiring.

```bash
ros2 launch qmini_bringup motor_bus_test.launch.py
```

**Expected log:**

```
[INFO] [launch]: All log files can be found below /home/.../ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [qmini_safety-1]: qmini_safety started — gate=HARD_STOPPED, heartbeat at 50 Hz.
[INFO] [motor_bus_node-2]: GO-M8010-6 gear ratio = 6.3300 (motor↔joint factor).
[INFO] [motor_bus_node-2]: [hip_yaw_bus] opened ...if00... @ 4000000 baud (2 motors).
[INFO] [motor_bus_node-2]: [hip_roll_bus] opened ...if01... @ 4000000 baud (2 motors).
[INFO] [motor_bus_node-2]: [left_lower_bus] opened ...if02... @ 4000000 baud (3 motors).
[INFO] [motor_bus_node-2]: [right_lower_bus] opened ...if03... @ 4000000 baud (3 motors).
[INFO] [motor_bus_node-2]: motor_bus_node up: 4 channels, 10 motors, /joint_states @ 200.0 Hz.
[INFO] [motor_bus_node-2]: MotionGate -> state=2 (enabled=no) reason="Initial state..."
```

Leave it running.

In a **second terminal**:

```bash
cd ~/qmini_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /joint_states
```

**Expected:** `~200 Hz, ±a few`. If you see something dramatically lower
(e.g. < 50 Hz) jump to [C1](#c1-jointstates-rate-much-lower-than-200-hz).

```bash
ros2 topic echo /joint_states --once
```
> "A message was lost!!!" — benign, ignore it. That's a DDS transport notice from ros2 topic echo, not from your node.


**Expected with no motors connected:**

```yaml
header: { ... }
name: [hip_yaw_l, hip_roll_l, hip_pitch_l, knee_pitch_l, ankle_pitch_l,
       hip_yaw_r, hip_roll_r, hip_pitch_r, knee_pitch_r, ankle_pitch_r]
position: [.nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan]
velocity: [.nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan]
effort:   [.nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan]
```

All NaN is correct: the channels opened, but motors aren't responding
because they aren't connected/powered. The node correctly publishes NaN as
the "no data" sentinel.

Verify the safety topics too:

```bash
ros2 topic echo /safety/motion_gate --once
# state: 2  (== STATE_HARD_STOPPED)
# reason: "Initial state: policy and joystick not wired yet (M1)."

ros2 topic hz /safety/heartbeat
# expected ~50 Hz
```

If Phase A is clean, stop the launch (Ctrl+C in the launch terminal) and
proceed to Phase B.

## Step 4 — Phase B: connect ONE motor

**Do not** wire all ten motors at once. If anything is miswired (data lines
swapped, ID conflict, daisy-chain termination problem), one motor at a time
turns a one-hour debug into five minutes.

> **Partial population is supported.** `motor_bus_node` polls each motor
> independently. A motor that doesn't reply for `motor_absent_threshold`
> consecutive polls (default 3) is marked **ABSENT**, stops being polled
> (which silences the SDK's `does not reply` / `wait time out` spam), and is
> re-probed every `motor_reprobe_period_s` (default 2 s) — so connecting it
> later recovers automatically without restarting the launch. Absent motors
> publish `.nan` in `/joint_states`; connected ones publish real values on the
> same channel. You'll see a short burst of SDK warnings at startup (a few per
> missing motor) and then one `marking ABSENT` line per missing motor — that
> is expected, not an error. If you instead get an *endless* flood, you're
> running an old build; rebuild `qmini_hardware`.

### 4.1 Pick the easiest motor to access

`hip_yaw_l` (channel 1, motor ID 0) is usually convenient. Connect:

- 24V motor power
- A and B differential lines to the channel 1 bus header on the FTDI adapter
- GND between the adapter and the motor's logic GND
- Verify the motor's flashed ID matches `motor_layout.yaml`. If you used the
  Unitree SDK's `changeID` example, double-check what ID you flashed.

### 4.2 Re-launch

```bash
ros2 launch qmini_bringup motor_bus_test.launch.py
```

In the second terminal:

```bash
ros2 topic echo /joint_states --once
```

**Expected:**

```yaml
position: [-0.0123, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan]
velocity: [ 0.0,    .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan]
effort:   [ 0.0,    .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan, .nan]
```

Index `[0]` (hip_yaw_l) has a real value; the other nine remain NaN.

### 4.3 Hand-rotate the motor

While slowly turning the motor shaft by hand:

```bash
ros2 topic echo /joint_states --field position
```

`position[0]` should track the rotation continuously. Watch a full
±π range — it should change smoothly, no jumps or wrap-around glitches.

If `position[0]` stays NaN despite the channel reporting opened, see
[B4](#b4-channel-opens-but-its-motors-stay-nan).

If the wrong index updates instead of `[0]`, see
[B5](#b5-wrong-joint-index-updates).

## Step 5 — Phase C: add the remaining motors

One motor at a time, in this order (lets you verify each channel's full
complement before moving to the next):

```
ch1 (hip_yaw_bus):     hip_yaw_l(id 0)  →  hip_yaw_r(id 1)
ch2 (hip_roll_bus):    hip_roll_l(id 0) →  hip_roll_r(id 1)
ch3 (left_lower_bus):  hip_pitch_l(id 0) → knee_pitch_l(id 1) → ankle_pitch_l(id 2)
ch4 (right_lower_bus): hip_pitch_r(id 0) → knee_pitch_r(id 1) → ankle_pitch_r(id 2)
```

After each motor is added:

```bash
ros2 topic echo /joint_states --once
```

Confirm one more NaN turned into a real number at the expected index, then
hand-rotate to confirm the right index responds.

Once all 10 motors are reporting:

```bash
ros2 topic echo /joint_states --once
# position: [10 real numbers, no NaN]
```

M1 is complete.

## Step 6 — Channel-to-joint mapping verification

If you discover during Phase B/C that **the wrong array index updates** when
you rotate a joint, the FTDI interface number → physical channel mapping on
your adapter board differs from the assumption in `motor_layout.yaml`. Fix
by editing the file:

```bash
$EDITOR src/qmini_hardware/config/motor_layout.yaml
```

Swap the `port:` URLs between affected channels. **No rebuild needed** —
because of `--symlink-install`, the installed YAML is a symlink to the
source. Re-launch and re-test.

Example: if rotating `hip_yaw_l` moves index `[5]` (hip_yaw_r), the if00 and
… wait, actually no — that means the motor ID 0 on channel 1 produces the
*right* leg's hip-yaw value, which would mean channel 1 and channel 2 are
swapped, or the LEFT motor is flashed with ID 1 and the right with ID 0.
Determine which by single-motor isolation: disconnect all but one motor on
that channel and see which joint name responds.

## Stopping

Ctrl+C in the launch terminal. The motor bus node's destructor joins all
polling threads cleanly. The motors should freewheel (they're already at
zero torque) but be aware that any uncommanded motion from gravity will
continue until they're physically braked or powered off.

For absolute confidence:

1. Ctrl+C the launch.
2. `ros2 node list` — should be empty after the launch unwinds.
3. Power off the motors at the 24 V supply before unwiring.

## Cleanup between runs

If you change CMakeLists, package.xml, or .msg files, you need a clean
rebuild (CMake doesn't always pick up these changes):

```bash
rm -rf build/ install/ log/
colcon build --symlink-install --packages-up-to qmini_hardware qmini_safety qmini_bringup
source install/setup.bash
```

For pure source edits (`.cpp`, `.py`, `.yaml`, launch files), `--symlink-install`
means a quicker incremental build is enough:

```bash
colcon build --symlink-install --packages-select qmini_hardware
```

---

## Troubleshooting

### A — Build failures

#### A1. `Unable to locate package ros-humble-desktop`

Cause: the ROS 2 apt sources aren't configured.

Fix:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main' \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

On the Pi 5, use `arch=arm64`.

#### A2. Many `python3-numpy`, `libeigen3-dev`, etc. "not installable"

Cause: Ubuntu `universe` (or `main`) is disabled in your apt sources.

Fix: confirm your `/etc/apt/sources.list.d/ubuntu.sources` (or
`/etc/apt/sources.list`) contains the line:

```
Components: main restricted universe multiverse
```

If `Components` only lists some of those, edit the file with `sudo` to add
the missing ones, then `sudo apt update`.

#### A3. `ModuleNotFoundError: No module named 'catkin_pkg'`

Cause: conda Python or `uv`'s `~/.local/bin/python3.12` is ahead of
`/usr/bin/python3` in PATH. The trace shows the culprit path explicitly.

Fix:

```bash
export PATH=/usr/bin:$PATH
conda deactivate 2>/dev/null
which python3   # must be /usr/bin/python3
rm -rf build/ install/ log/
colcon build --symlink-install \
  --packages-up-to qmini_hardware qmini_safety qmini_bringup
```

The `ros2dev` alias from §1.2 makes this permanent.

#### A4. `unitree_actuator_sdk not found at .../third_party/unitree_actuator_sdk`

Cause: the git submodule isn't initialized.

Fix:

```bash
cd ~/qmini_ros2
git submodule update --init --recursive
ls src/qmini_hardware/third_party/unitree_actuator_sdk/CMakeLists.txt  # must exist
```

If the submodule reference itself is missing, re-add it:

```bash
git submodule add https://github.com/unitreerobotics/unitree_actuator_sdk \
  src/qmini_hardware/third_party/unitree_actuator_sdk
```

#### A5. `yaml-cpp/yaml.h: No such file or directory`

Cause: `libyaml-cpp-dev` not installed.

Fix:

```bash
sudo apt install -y libyaml-cpp-dev
# or, more thorough:
rosdep install --from-paths src --ignore-src -r -y
```

#### A6. Compiler error mentioning C++14/17 mismatch in SDK headers

Cause: rare; the Unitree SDK is built against C++14 and `qmini_hardware`
uses C++17. They normally interoperate, but a particular header inclusion
order can trip on it.

Fix: confirm the order in `motor_bus_node.cpp` includes the SDK headers
after `<rclcpp/rclcpp.hpp>` — that's how we wrote it. If you reordered them
for some reason, undo that.

#### A7. `launch file 'motor_bus_test.launch.py' was not found in the share directory`

Cause: `qmini_bringup` wasn't built (or wasn't rebuilt after the launch file
was added). The launch files live in `qmini_bringup`, and that package is
**not** a dependency of `qmini_hardware`/`qmini_safety` — so a build scoped to
only those two never installs it.

Fix: include `qmini_bringup` in the build, then re-check the install dir:

```bash
colcon build --symlink-install \
  --packages-up-to qmini_hardware qmini_safety qmini_bringup
ls install/qmini_bringup/share/qmini_bringup/launch/
# motor_bus_test.launch.py must be listed
```

If `qmini_bringup` *was* in the build but the file still isn't installed, the
file was added after the last build of that package — `--symlink-install`
only links files present at build time. Force a rebuild of just that package:

```bash
colcon build --symlink-install --packages-select qmini_bringup
```

#### A8. `use of deleted function 'Channel::Channel(Channel&&)'` / `static assertion failed: result type must be constructible from input type`

Cause: a container of a type that holds `std::atomic` or `std::thread` (both
non-copyable and non-movable) — e.g. if you reintroduced a `std::vector<Channel>`.
`std::vector` requires its element type to be movable or copyable to grow or
`reserve`, and `Channel` is neither.

Fix: keep `channels_` a `std::deque<Channel>` (the current design). A deque
never relocates existing elements, so `emplace_back` works without a move
constructor, and `operator[]`/range-for usage is identical to a vector. Do
not switch it back to `std::vector`.

### B — Runtime failures

#### B1. `[hip_yaw_bus] could not open .../if00-port0: Permission denied`

Cause: the user running the node isn't in the `dialout` group, or the port
permissions are wrong.

Fix:

```bash
ls -la /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA98W5B-if00-port0
# Resolve the symlink, check the target device, e.g. /dev/ttyUSB0
# Should be: crw-rw---- root dialout

groups   # should include 'dialout'

# If not in dialout:
sudo usermod -aG dialout "$USER"
# Log out and back in. Group changes don't apply to existing shells.
```

If you're already in dialout and the device is owned by another group
(e.g. `tty` only), it's a udev rule problem. Install the project's
`config/udev/99-qmini.rules`:

```bash
sudo cp src/qmini_hardware/config/udev/99-qmini.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
# Unplug and re-plug the adapter, or reboot.
```

#### B2. All four channels report `could not open ...`

Either the adapter isn't plugged in, or `motor_layout.yaml` has stale paths.

Fix:

```bash
ls -la /dev/serial/by-id/                  # see what's actually present
cat src/qmini_hardware/config/motor_layout.yaml | grep port:
# The two listings should match exactly.
```

If the adapter is detected but the by-id path doesn't match
`FTA98W5B`, that's a different unit; update `motor_layout.yaml` with the
correct serial.

#### B3. One channel fails, others succeed

Most likely a one-off cable/connector issue. Check:

- The if-number reported (`if02` etc.) maps to the physical channel you
  expect.
- The cable for that channel isn't disconnected.
- The kernel hasn't deassigned that port (try `dmesg | tail` after re-plug).

If the failure persists across re-plugs, the FT4232H channel itself may be
damaged. Test by swapping the cable to a known-good channel.

#### B4. Channel opens but a motor stays NaN (`motor id=N silent ... marking ABSENT`)

The serial port is open but per-motor `sendRecv()` is returning `false` (no
valid response), or the motor returns `MotorData.correct == false`. After
`motor_absent_threshold` (default 3) failures the node logs `marking ABSENT`
and stops polling that one motor; it keeps publishing `.nan` for it and
re-probes every `motor_reprobe_period_s` (default 2 s). Other motors on the
same channel are unaffected. Fix the cause below, then just reconnect/power
the motor — it recovers without restarting the launch.

Causes, in order of likelihood:

1. **Motor not powered.** The GO-M8010-6 needs 24 V on its main power; USB
   logic voltage is not enough.
2. **Wrong motor ID.** `motor_layout.yaml` says motor at index 0 of that
   channel has `id: 0`, but the physical motor is flashed with a different
   ID. Use the SDK's `changeID` example to verify or re-flash:

   ```bash
   cd src/qmini_hardware/third_party/unitree_actuator_sdk
   mkdir build && cd build && cmake .. && make changeID
   ./changeID
   # follow prompts; defaults usually work, target your port + desired ID.
   ```

3. **Data lines swapped.** A/B differential pair reversed on the RS-485
   header — many adapters are forgiving but some aren't. Try the reverse
   wiring.
4. **Bus termination missing.** RS-485 wants a 120 Ω termination resistor
   at each end of the bus. For a single short bench cable it usually works
   without, but longer runs (> 50 cm) start to drop frames.
5. **Wrong baud rate.** Default for stock GO-M8010-6 firmware is 4 Mbps.
   If you've reflashed it to a different baud, update
   `motor_layout.yaml`'s `baud_rate` for that channel.

#### B5. Wrong joint index updates

When you rotate motor X by hand, the position changes at index Y (where Y ≠
expected). Two causes:

1. **FTDI interface → physical channel mapping is reversed.** Swap the
   `port:` URLs in `motor_layout.yaml`. No rebuild needed.
2. **Motor IDs are swapped within a channel.** E.g. on channel 1, the
   motor you call hip_yaw_l is flashed with id 1 instead of id 0. Either
   re-flash, or swap the `id:` values in `motor_layout.yaml`.

To diagnose: disconnect all but one motor on the suspected channel and see
which joint index responds to its motion. That isolates whether it's a
port-level or ID-level swap.

#### B6. `error while loading shared libraries: libUnitreeMotorSDK_Linux64.so`

The motor bus node can't find the SDK's .so at runtime, even though the
build succeeded.

Cause: `LD_LIBRARY_PATH` doesn't include `install/lib`.

Fix: `colcon build` should have copied the .so to `install/lib` via the
`install(FILES "${UNITREE_SDK_LIB}" DESTINATION lib)` line in
`qmini_hardware/CMakeLists.txt`. Verify:

```bash
ls install/lib/libUnitreeMotorSDK_*.so
```

If missing, do a clean rebuild (`rm -rf build install log && colcon build ...`).
If present but the loader still can't find it, force the path:

```bash
export LD_LIBRARY_PATH="$(pwd)/install/lib:$LD_LIBRARY_PATH"
ros2 launch qmini_bringup motor_bus_test.launch.py
```

#### B7. Node crashes during startup with `terminate called after throwing`

The Unitree SDK can throw from its `SerialPort` constructor if the port
exists but is held open by another process (e.g. an earlier launch that
didn't unwind cleanly), or if `recvLength` overflows for a malformed
device.

Fix:

```bash
# 1. Confirm no stale processes
pgrep -f motor_bus_node
killall motor_bus_node 2>/dev/null

# 2. Confirm no other apps holding the ports
sudo fuser /dev/ttyUSB[0-3]

# 3. Retry
ros2 launch qmini_bringup motor_bus_test.launch.py
```

### C — Performance issues

#### C1. `/joint_states` rate much lower than 200 Hz

Causes, in order:

1. **Bus contention.** Each `sendRecv()` to a 3-motor channel takes longer
   than to a 2-motor channel. The publish timer ticks at 200 Hz but only
   samples per-motor caches — it doesn't wait for fresh data. Slower
   reporting means some publishes echo the previous sample. The actual
   *fresh-data* rate per motor is lower.

2. **CPU contention.** Especially on the Pi 5, conda/uv processes running
   in the background or thermal throttling can drop rate. Check:

   ```bash
   top -H -p $(pgrep -f motor_bus_node)
   ```

3. **USB-2 fallback.** The FT4232H is USB 2.0 internally; if you plugged it
   into a hub or a USB-2 port, throughput is roughly halved. Use a direct
   USB 3.0 port.

4. **Too-aggressive `min_poll_period_us`.** Lowering it below the actual
   round-trip time wastes CPU. Default 500 µs (2 kHz max) is conservative;
   you can experiment up or down.

#### C2. Position jumps / wrap-around glitches

The Unitree SDK reports motor-side angle as an unbounded float in radians.
`motor_bus_node.cpp` divides by gear ratio for joint-side angle. There's no
wrap or unwrap in M1 — values are passed through. If you see ±2π jumps
during a smooth rotation, the SDK or the motor itself is wrapping; report
it and we add an unwrap step in M2.

#### C3. High CPU usage from `motor_bus_node`

4 polling threads at 2 kHz + a 200 Hz publish timer should sit comfortably
on ~one Pi 5 core. If you see > 50% CPU on a single core, investigate:

```bash
htop                                            # is one core pinned?
strace -p $(pgrep -f motor_bus_node) -c        # what syscalls dominate?
```

Common cause is the serial driver entering polling mode (vs interrupt) when
the kernel thinks the device is "busy." Latency timer setting can help:

```bash
# /dev/ttyUSB0 is whichever the FTDI port resolved to
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

The default is often 16 ms; setting to 1 ms reduces stall but also CPU.

### D — Sanity-check commands

A few one-liners worth remembering:

```bash
# Are the safety topics alive?
ros2 topic list | grep safety
# Expected: /safety/heartbeat  /safety/motion_gate

# What state is the gate in?
ros2 topic echo /safety/motion_gate --once | grep state

# Per-channel data: are all motors responding?
ros2 topic echo /joint_states --once | grep -A1 position

# Is the motor bus seeing the safety heartbeat?
ros2 topic hz /safety/heartbeat   # expected ~50 Hz

# Confirm which SDK library actually loaded
ldd $(ros2 pkg prefix qmini_hardware)/lib/qmini_hardware/motor_bus_node \
  | grep -i unitree
```

---

## Done?

When all ten joints respond, the rate hovers around 200 Hz, and rotating
each joint moves the correct array index — M1 is complete. Mark it in the
project tracker and proceed to M2 (PD packer and the first commanded hold
pose).
