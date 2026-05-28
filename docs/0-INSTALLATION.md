# Installation — From a Fresh Ubuntu 22.04 to a Built Workspace

First-time setup for the `qmini_ros2` stack. Do this before any of the numbered
bring-up guides (`1-SMOKE_TEST.md` onward).

Two machines, same steps, different onnxruntime arch:

| Machine | Role | Arch | ROS 2 | onnxruntime |
|---|---|---|---|---|
| **Dev PC** | development / build | x86_64 (`amd64`) | Humble | `onnxruntime-linux-x64` |
| **Raspberry Pi 5** | the real robot | aarch64 (`arm64`) | Humble | `onnxruntime-linux-aarch64` |

The build is arch-agnostic — only the onnxruntime release tarball differs.

---

## 0. Prerequisites

- **Ubuntu 22.04 (jammy)** — ROS 2 Humble targets exactly this release.
- On the Pi: a 64-bit Ubuntu 22.04 image (not Raspberry Pi OS).
- Network access (or a Chinese mirror — see *Behind the GFW* below).
- The workspace lives at **`~/qmini_ros2`** (clone/copy it there; all commands
  below assume that path).

### ⚠️ Enable the `jammy-updates` pocket FIRST

Some Ubuntu images (Raspberry Pi images especially) ship with only `jammy` and
`jammy-security` in their apt sources, **missing `jammy-updates`**. That breaks
the ROS install partway through with errors like:

```
liblz4-dev : Depends: liblz4-1 (= 1.9.3-2build1) but 1.9.3-2ubuntu0.1 is to be installed
zlib1g-dev : Depends: zlib1g (= ...ubuntu2)      but ...ubuntu2.1 is to be installed
dpkg-dev   : Depends: bzip2 but it is not installable
```

Each `-dev` package must match its runtime lib's **exact** version; the patched
`…ubuntu0.1` runtime libs live in `jammy-updates`, so without that pocket apt
can't line them up. Check and fix before running the installer:

```bash
# Ubuntu 22.04 uses the legacy /etc/apt/sources.list (not deb822 ubuntu.sources).
grep -E 'jammy-updates|jammy-backports' /etc/apt/sources.list || \
  echo "deb http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse" | \
    sudo tee -a /etc/apt/sources.list
sudo apt update && sudo apt full-upgrade -y
```

The bundled `install_ros2_humble.sh` now does this automatically (step `[0/8]`),
but verify it if you're installing by hand.

---

## 1. Install ROS 2 Humble

Use the bundled script — it sets the locale, adds the ROS 2 apt repo + key,
installs `ros-humble-ros-base` + dev tools + the packages this stack links
against, installs **ONNX Runtime (C++)** to `/opt/onnxruntime`, runs
`rosdep init/update`, and adds the `dialout`/`input` groups + `~/.bashrc`
sourcing.

```bash
cd ~/qmini_ros2
bash scripts/install_ros2_humble.sh
```

Useful environment toggles:

```bash
# Behind the GFW — use a Chinese mirror for Ubuntu-ports + ROS:
USE_CN_MIRROR=1 bash scripts/install_ros2_humble.sh
CN_MIRROR_HOST=mirrors.ustc.edu.cn USE_CN_MIRROR=1 bash scripts/install_ros2_humble.sh

# Skip the onnxruntime install (e.g. you manage it yourself):
INSTALL_ONNX=0 bash scripts/install_ros2_humble.sh

# Pin a different onnxruntime version (default 1.19.2):
ORT_VER=1.19.2 bash scripts/install_ros2_humble.sh
```

**Re-login (or reboot) after the script** — it changes group membership
(`dialout`, `input`) and `~/.bashrc`, which only take effect in a new session.

---

## 2. ONNX Runtime (C++) — what the script installs, and doing it by hand

`qmini_rl/policy_runner_node` links the **C++** ONNX Runtime (not the pip
package). The script installs the official release for your arch to
`/opt/onnxruntime` and registers it with `ldconfig`. To do it manually, or on a
machine where you skipped it:

```bash
VER=1.19.2
ARCH=$(dpkg --print-architecture); case "$ARCH" in
  arm64) ORT=aarch64 ;; amd64) ORT=x64 ;; esac           # Pi -> aarch64, dev PC -> x64
cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v${VER}/onnxruntime-linux-${ORT}-${VER}.tgz
tar xzf onnxruntime-linux-${ORT}-${VER}.tgz
sudo mv onnxruntime-linux-${ORT}-${VER} /opt/onnxruntime
echo /opt/onnxruntime/lib | sudo tee /etc/ld.so.conf.d/onnxruntime.conf && sudo ldconfig
```

CMake defaults `ONNXRUNTIME_DIR` to `/opt/onnxruntime`, so this needs no env var.
No sudo / can't write `/opt`? Extract anywhere and
`export ONNXRUNTIME_DIR=/path/to/onnxruntime-linux-${ORT}-${VER}` before building,
plus add its `lib/` to `LD_LIBRARY_PATH` at runtime. See `8-POLICY_RUNNER.md` for
detail and a verify step. (Do **not** use `pip install onnxruntime` — Python
wheel only, no C++ dev files — or the copy inside `qmini_official_sdk`.)

---

## 3. Build the workspace

```bash
cd ~/qmini_ros2
source /opt/ros/humble/setup.bash

# resolve declared system deps for the packages under src/
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash          # after every fresh build in a new shell
```

For faster iteration on one package:
`colcon build --symlink-install --packages-select qmini_rl`.

A successful `qmini_rl` configure prints
`qmini_rl: ONNX Runtime headers /opt/onnxruntime/include, lib …`; if it instead
says `ONNX Runtime NOT found`, step 2 didn't land where CMake looks.

---

## 4. Device permissions (the real robot)

Motor bus + IMU are USB serial; the joystick is an input device:

```bash
sudo usermod -aG dialout $USER     # /dev/ttyUSB* (motor bus, IMU) — script does input group
# re-login for group changes to take effect
```

Pin device paths with udev rather than hardcoding `/dev/ttyUSB0..4` (USB
enumeration order isn't stable across reboots) — see the per-device guides
(`1-SMOKE_TEST.md`, `6-IMU_DRIVER.md`).

---

## 5. Verify

```bash
ros2 doctor                        # general health
ros2 pkg list | wc -l              # ROS packages visible
colcon test-result --all || true   # if you've run colcon test
```

---

## Troubleshooting

- **`held broken packages` / `-dev` version mismatch during install** — the
  `jammy-updates` pocket is missing. See §0; then `sudo apt full-upgrade -y`.
- **`ModuleNotFoundError: No module named 'catkin_pkg'` during `colcon build`** —
  colcon is using a `~/.local` Python that lacks `catkin_pkg`. Build with the
  system Python: `PATH=/usr/bin:$PATH colcon build --cmake-args
  -DPython3_EXECUTABLE=/usr/bin/python3 …`, or `pip install --user catkin_pkg`.
- **`qmini_rl: ONNX Runtime NOT found`** — install onnxruntime (§2) or point
  `ONNXRUNTIME_DIR` at it.
- **GitHub release download blocked (GFW)** — the CN mirror covers apt/ROS but
  not GitHub release assets; fetch the onnxruntime tarball via a proxy/mirror or
  copy it over manually, then extract to `/opt/onnxruntime`.
