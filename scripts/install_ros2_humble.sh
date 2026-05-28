#!/usr/bin/env bash
# ROS 2 Humble install for Ubuntu 22.04 (Raspberry Pi 5, arm64)
# Used by the dreambo bipedal stack on the Pi 5 onboard computer.
#
# Behind the GFW? Swap to a Chinese mirror for both Ubuntu-ports and ROS 2:
#   USE_CN_MIRROR=1 bash install_ros2_humble.sh
# Pick a different mirror (default: Tsinghua TUNA):
#   CN_MIRROR_HOST=mirrors.ustc.edu.cn      USE_CN_MIRROR=1 bash install_ros2_humble.sh
#   CN_MIRROR_HOST=mirrors.aliyun.com       USE_CN_MIRROR=1 bash install_ros2_humble.sh
#   CN_MIRROR_HOST=repo.huaweicloud.com     USE_CN_MIRROR=1 bash install_ros2_humble.sh
#
# Skip the ONNX Runtime (C++) install (default installs it for the policy node):
#   INSTALL_ONNX=0 bash install_ros2_humble.sh
# Pin a different ONNX Runtime version (default 1.19.2):
#   ORT_VER=1.19.2 bash install_ros2_humble.sh
set -euo pipefail

USE_CN_MIRROR="${USE_CN_MIRROR:-0}"
CN_MIRROR_HOST="${CN_MIRROR_HOST:-mirrors.tuna.tsinghua.edu.cn}"
INSTALL_ONNX="${INSTALL_ONNX:-1}"
ORT_VER="${ORT_VER:-1.19.2}"
NEED_RELOGIN=0

if [ "$(. /etc/os-release && echo "$VERSION_CODENAME")" != "jammy" ]; then
  echo "Warning: ROS 2 Humble targets Ubuntu 22.04 (jammy); detected $(. /etc/os-release && echo "$VERSION_CODENAME")." >&2
fi
if [ "$(dpkg --print-architecture)" != "arm64" ]; then
  echo "Warning: this script is intended for arm64 (Raspberry Pi 5); detected $(dpkg --print-architecture)." >&2
fi

if [ "$USE_CN_MIRROR" = "1" ]; then
  echo "[mirror] Switching Ubuntu ports apt sources to https://${CN_MIRROR_HOST}/ubuntu-ports"
  for f in /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list; do
    if [ -f "$f" ]; then
      sudo cp -n "$f" "$f.bak"
      sudo sed -i \
        -e "s|http://ports.ubuntu.com/ubuntu-ports|https://${CN_MIRROR_HOST}/ubuntu-ports|g" \
        -e "s|https://ports.ubuntu.com/ubuntu-ports|https://${CN_MIRROR_HOST}/ubuntu-ports|g" \
        "$f"
    fi
  done
fi

# Some Ubuntu 22.04 images (Raspberry Pi especially) ship with only the 'jammy'
# and 'jammy-security' apt pockets, missing 'jammy-updates'. The patched runtime
# libs (liblz4-1, libzstd1, zlib1g, ... at versions like 1build1.1) live in
# jammy-updates, and the matching '-dev' packages depend on the EXACT version —
# so without jammy-updates the ROS install fails with "held broken packages".
echo "[0/8] Ensure jammy-updates + jammy-backports pockets are enabled"
SRC=/etc/apt/sources.list.d/ubuntu.sources
LEGACY_SRC=/etc/apt/sources.list
if [ -f "$SRC" ]; then
  if ! grep -qE '^Suites:.*jammy-updates' "$SRC"; then
    sudo cp -n "$SRC" "$SRC.bak"
    sudo sed -i 's/^Suites: jammy$/Suites: jammy jammy-updates jammy-backports/' "$SRC"
    echo "  Added jammy-updates + jammy-backports to $SRC (backup at $SRC.bak)."
  else
    echo "  jammy-updates already enabled."
  fi
elif [ -f "$LEGACY_SRC" ]; then
  # 22.04 typically uses the legacy /etc/apt/sources.list. Ensure jammy-updates
  # and jammy-backports lines exist (most stock images already have them).
  if ! grep -qE '^[^#].*jammy-updates' "$LEGACY_SRC"; then
    sudo cp -n "$LEGACY_SRC" "$LEGACY_SRC.bak"
    echo "deb http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse" \
      | sudo tee -a "$LEGACY_SRC" > /dev/null
    echo "  Appended jammy-updates to $LEGACY_SRC (backup at $LEGACY_SRC.bak)."
  else
    echo "  jammy-updates already enabled."
  fi
else
  echo "  Neither $SRC nor $LEGACY_SRC found — ensure jammy-updates is enabled manually." >&2
fi

echo "[1/8] Locale"
sudo apt-get update
sudo apt-get install -y locales curl gnupg lsb-release software-properties-common ca-certificates
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "[2/8] Universe repo"
sudo add-apt-repository -y universe

echo "[3/8] ROS 2 apt key"
if [ "$USE_CN_MIRROR" = "1" ]; then
  ROS_KEY_URL="https://${CN_MIRROR_HOST}/rosdistro/ros.key"
else
  ROS_KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
fi
sudo curl -fsSL "$ROS_KEY_URL" -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "[4/8] ROS 2 apt repo (humble / jammy / arm64)"
if [ "$USE_CN_MIRROR" = "1" ]; then
  ROS_REPO_URL="https://${CN_MIRROR_HOST}/ros2/ubuntu"
else
  ROS_REPO_URL="http://packages.ros.org/ros2/ubuntu"
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
${ROS_REPO_URL} $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
# full-upgrade (not plain upgrade) so the held-back runtime libs from
# jammy-updates are pulled in, keeping them in lockstep with their -dev packages.
sudo apt-get full-upgrade -y

echo "[5/8] ROS 2 Humble base + dev tools"
sudo apt-get install -y \
  ros-humble-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete

echo "[6/8] Packages commonly needed for the dreambo bipedal stack"
sudo apt-get install -y \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-camera-info-manager \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-tf-transformations \
  ros-humble-control-msgs \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-urdf \
  ros-humble-ackermann-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-diagnostic-msgs \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard

# Non-ROS system deps the workspace links against and tools used at runtime.
#   libeigen3-dev — imu_n100 (Eigen quaternion math)
#   python3-numpy — calibration analyzers / policy tooling
#   python3-pip   — general Python tooling
# Other system deps declared in each package.xml are resolved later with:
#   rosdep install --from-paths src --ignore-src -y
sudo apt-get install -y \
  libeigen3-dev \
  python3-numpy \
  python3-pip

# ONNX Runtime (C++) — qmini_rl/policy_runner_node links the C++ API, NOT the
# Python pip package. Install the official prebuilt release for this arch to
# /opt/onnxruntime (CMake defaults ONNXRUNTIME_DIR there) and register it with
# ldconfig. Opt out with INSTALL_ONNX=0; pin a version with ORT_VER=...
if [ "$INSTALL_ONNX" = "1" ]; then
  case "$(dpkg --print-architecture)" in
    arm64) ORT_ARCH=aarch64 ;;
    amd64) ORT_ARCH=x64 ;;
    *)     ORT_ARCH="" ;;
  esac
  if [ -z "$ORT_ARCH" ]; then
    echo "[onnx] Unknown arch $(dpkg --print-architecture) — skipping ONNX Runtime; install it by hand." >&2
  elif [ -e /opt/onnxruntime/lib/libonnxruntime.so ]; then
    echo "[onnx] /opt/onnxruntime already present — skipping."
  else
    echo "[onnx] Installing ONNX Runtime C++ ${ORT_VER} (${ORT_ARCH}) to /opt/onnxruntime"
    ort_tmp="$(mktemp -d)"
    ort_tgz="onnxruntime-linux-${ORT_ARCH}-${ORT_VER}.tgz"
    curl -fsSL -o "${ort_tmp}/${ort_tgz}" \
      "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VER}/${ort_tgz}"
    tar xzf "${ort_tmp}/${ort_tgz}" -C "${ort_tmp}"
    sudo rm -rf /opt/onnxruntime
    sudo mv "${ort_tmp}/onnxruntime-linux-${ORT_ARCH}-${ORT_VER}" /opt/onnxruntime
    echo /opt/onnxruntime/lib | sudo tee /etc/ld.so.conf.d/onnxruntime.conf > /dev/null
    sudo ldconfig
    rm -rf "${ort_tmp}"
    echo "[onnx] Installed to /opt/onnxruntime (ldconfig registered)."
  fi
fi

echo "[7/8] rosdep init + bashrc sourcing"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi

if [ "$USE_CN_MIRROR" = "1" ]; then
  echo "[mirror] Rewriting rosdep sources to https://${CN_MIRROR_HOST}/rosdistro"
  sudo tee /etc/ros/rosdep/sources.list.d/20-default.list > /dev/null <<EOF
# os-specific listings first
yaml https://${CN_MIRROR_HOST}/rosdistro/rosdep/osx-homebrew.yaml osx

yaml https://${CN_MIRROR_HOST}/rosdistro/rosdep/base.yaml
yaml https://${CN_MIRROR_HOST}/rosdistro/rosdep/python.yaml
yaml https://${CN_MIRROR_HOST}/rosdistro/rosdep/ruby.yaml
EOF
  export ROSDISTRO_INDEX_URL="https://${CN_MIRROR_HOST}/rosdistro/index-v4.yaml"
  if ! grep -q "ROSDISTRO_INDEX_URL" "$HOME/.bashrc"; then
    echo "export ROSDISTRO_INDEX_URL=$ROSDISTRO_INDEX_URL" >> "$HOME/.bashrc"
  fi
fi

rosdep update || true

if grep -q "source /opt/ros/humble/setup.bash" "$HOME/.bashrc"; then
  echo "  ~/.bashrc already sources /opt/ros/humble/setup.bash."
else
  echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
  echo "  Added 'source /opt/ros/humble/setup.bash' to ~/.bashrc."
  NEED_RELOGIN=1
fi

echo "[8/8] Joystick device permissions"
if id -nG "$USER" | grep -qw input; then
  echo "  $USER already in 'input' group."
else
  sudo usermod -a -G input "$USER"
  echo "  Added $USER to 'input' group."
  NEED_RELOGIN=1
fi

echo
echo "==================================================================="
echo " ROS 2 Humble install finished."
echo "==================================================================="
if [ "$NEED_RELOGIN" = "1" ]; then
  echo
  echo " >>> RE-LOGIN REQUIRED <<<"
  echo " ~/.bashrc and/or your group membership changed. Log out and back in"
  echo " (or reboot) before continuing. For a quick test in this shell only:"
  echo "     source /opt/ros/humble/setup.bash"
fi
echo
echo " After re-login, verify with:"
echo "     ros2 doctor"
echo "     ros2 pkg list | wc -l"
echo "     ros2 run joy joy_enumerate_devices    # plug a controller in first"
