# qmini_imu

Native C++ driver for the **Wheeltec N100** IMU (FDILink serial protocol).
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

Edit `config/imu.yaml` for the port, baud (N100 default 921600), frame_id, and
covariances. Find the port with `ls -la /dev/serial/by-id/`.

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

## Protocol provenance

The FDILink frame layout, struct field order/units, and the CRC-8 / CRC-16/CCITT
lookup tables were derived from the reference driver
[`NDHANA94/ros2_wheeltec_n100_imu`](https://github.com/NDHANA94/ros2_wheeltec_n100_imu)
(no license stated upstream). This package is a **clean reimplementation** — the
parser, serial layer, threading, and ROS wiring are original; only the protocol
constants and the (data-only) CRC tables match the device's framing. Do not
vendor the upstream source.
