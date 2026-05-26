# IMU Driver — Wheeltec N100 (FDILink)

> 中文版见 [`6-IMU_DRIVER.zh.md`](6-IMU_DRIVER.zh.md)。

The `qmini_imu` package: a native C++ driver for the **Wheeltec N100** IMU
(FDILink serial protocol).
Publishes `sensor_msgs/Imu` on `/imu/data`. Self-contained — opens the serial
port via raw `termios`, no external `serial` / `serial-ros2` dependency — and
runs in its own process so a stalled IMU port can never block the realtime
motor bus (`qmini_hardware`).

## Run

```bash
ros2 launch qmini_imu imu.launch.py
# or override the port (prefer a stable by-id path):
ros2 launch qmini_imu imu.launch.py serial_port:=/dev/serial/by-id/usb-...
```

Edit `qmini_imu/config/imu.yaml` for the port, baud (N100 default 921600),
frame_id, and covariances. Find the port with `ls -la /dev/serial/by-id/`.

## What it publishes

Per AHRS frame: orientation (fused quaternion), `angular_velocity` (raw gyro,
the policy's `imu_ang_vel`), `linear_acceleration` (raw accel). The N100 streams
interleaved IMU (gyro/accel/mag) and AHRS (orientation) frames; we merge the
latest IMU frame into each AHRS publish.

## Frame alignment is NOT final here (→ M4)

This driver publishes the **sensor's own frame**. The Isaac Lab policy expects
the IMU frame to match the sim mount (`ImuCfg` offset
`pos=(-0.04718, 0.0663, 0.11094)`, `rot=(1,0,0,0)` identity rel. `base_link`).
Verifying/fixing that mapping is exactly what **M4 calibration** does; the M5
observation assembler applies the final `base_link` transform. The
`apply_ros_transform` param reproduces the vendor's `device_type==1` ROS-frame
rotation by default — keep it on for sane RViz output, confirm in M4.

## Mount rotation — putting `/imu/data` in `base_link`

The IMU is bolted on at some fixed rotation relative to `base_link`. The driver
corrects for it with the **`mount_rotation`** parameter — a quaternion `[w,x,y,z]`
representing R (`base_link ← sensor`), applied **uniformly** to orientation
(post-multiply), gyro, and accel so `/imu/data` comes out in `base_link`.

- Default (code): identity `[1,0,0,0]` — no correction.
- **This robot (config/imu.yaml): `[0,0,1,0]` = 180° about Y** — equivalent to the
  official SDK's `(-1,+1,-1)` sign flip on x,z. **Verified 2026-05-26:** with it,
  `imu_projected_gravity` reads ≈ `(0,0,-1)` upright; without it, z came out
  *positive* (the policy would think the robot was upside-down).

The value lives in `qmini_imu/config/imu.yaml` (per-robot, like
`joint_offsets.yaml`) and is recorded as the M4 IMU mount in
`calibration_results.yaml`.

## Verifying orientation once mounted

### What "correct" means

The policy does not consume the orientation quaternion directly — it uses only
**`imu_ang_vel`** (gyro) and **`imu_projected_gravity`** (derived from orientation
in `qmini_rl`). Both must be in the **`base_link`** frame Isaac Lab trained with:
**x-forward, y-left, z-up**. So once mounted (with `mount_rotation` set), three
things must hold:

1. **Axis mapping** — tilting/rotating about base X shows up on the X channel.
2. **Sign** — each channel moves the right direction (right-hand rule).
3. **Gravity direction** — `imu_projected_gravity ≈ (0, 0, -1)` upright.

### The decisive check: `projected_gravity`, not raw accel

`imu_projected_gravity` is the actual policy input, and its target is unambiguous:
**`(0, 0, -1)` upright** (gravity points down → **z negative**). Read it from the
calibration node `imu_noise_calib`, which prints `proj_gravity_mean` and
`mount_tilt_from_vertical_deg`, or compute `quat_rotate_inverse(orientation,
(0,0,-1))` from a `/imu/data` message.

> Don't use the raw `linear_acceleration` sign as the reference. Accelerometers
> report one of two conventions (specific-force "+up" vs gravity "−down"), and the
> N100's sign may even *disagree* with `projected_gravity` — that's a sensor
> convention quirk, not a frame error, and the policy ignores `linear_acceleration`
> entirely. Use its **magnitude** (≈ 9.8) as a health check and **axis mapping**
> for the tilt tests, but judge correctness by `projected_gravity`.

### Protocol (robot powered, on the rope/bench)

**A. Static, upright & level** — `imu_projected_gravity ≈ (0, 0, -1)`,
`angular_velocity ≈ 0`, `|linear_acceleration| ≈ 9.8`.

**B. Static tilts** — tilt and confirm gravity bleeds onto the expected axis:
nose-down → `projected_gravity.x` goes **positive**; roll **left** →
`projected_gravity.y` goes **positive**; return upright → back to `(0,0,-1)`.

**C. Rotations** (right-hand rule, watch `angular_velocity`): yaw **left** →
`z` positive; pitch nose-down → `y` positive; roll left-side-down → `x` positive.

**D. Consistency** — while pitching nose-down, `angular_velocity.y` positive **and**
`projected_gravity.x` positive together. If they disagree, the gyro and orientation
aren't in the same frame — fix `mount_rotation` before running the policy.

### If it's wrong — how to correct it

Set `mount_rotation` in `qmini_imu/config/imu.yaml` to the rotation that brings the
sensor into `base_link`, then re-run A–D. For an axis-aligned mount it's one of the
180°/identity quaternions (e.g. `[0,0,1,0]` = 180° about Y = negate x,z;
`[0,1,0,0]` = 180° about X; `[0,0,0,1]` = 180° about Z; `[1,0,0,0]` = identity).
Determine it from the static gravity reading and iterate until
`projected_gravity ≈ (0,0,-1)` upright and the tilt/rotate signs are correct.
Physically mounting the N100 aligned to `base_link` (so `mount_rotation` =
identity) also works and matches the sim's `rot=(1,0,0,0)`.

## Protocol provenance

The FDILink frame layout, struct field order/units, and the CRC-8 / CRC-16/CCITT
lookup tables were derived from the reference driver
[`NDHANA94/ros2_wheeltec_n100_imu`](https://github.com/NDHANA94/ros2_wheeltec_n100_imu)
(no license stated upstream). This package is a **clean reimplementation** — the
parser, serial layer, threading, and ROS wiring are original; only the protocol
constants and the (data-only) CRC tables match the device's framing. Do not
vendor the upstream source.
