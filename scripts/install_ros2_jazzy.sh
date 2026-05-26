#!/usr/bin/env bash
# ROS 2 Jazzy install for Ubuntu 24.04 (Raspberry Pi 5, arm64)
# Used by the dreambo bipedal stack on the Pi 5 onboard computer.
#
# Behind the GFW? Swap to a Chinese mirror for both Ubuntu-ports and ROS 2:
#   USE_CN_MIRROR=1 bash install_ros2_jazzy.sh
# Pick a different mirror (default: Tsinghua TUNA):
#   CN_MIRROR_HOST=mirrors.ustc.edu.cn      USE_CN_MIRROR=1 bash install_ros2_jazzy.sh
#   CN_MIRROR_HOST=mirrors.aliyun.com       USE_CN_MIRROR=1 bash install_ros2_jazzy.sh
#   CN_MIRROR_HOST=repo.huaweicloud.com     USE_CN_MIRROR=1 bash install_ros2_jazzy.sh
#
# Skip the onnxruntime pip install (default is to install it for the policy node):
#   INSTALL_ONNX=0 bash install_ros2_jazzy.sh
set -euo pipefail

USE_CN_MIRROR="${USE_CN_MIRROR:-0}"
CN_MIRROR_HOST="${CN_MIRROR_HOST:-mirrors.tuna.tsinghua.edu.cn}"
INSTALL_ONNX="${INSTALL_ONNX:-1}"
NEED_RELOGIN=0

if [ "$(. /etc/os-release && echo "$VERSION_CODENAME")" != "noble" ]; then
  echo "Warning: ROS 2 Jazzy targets Ubuntu 24.04 (noble); detected $(. /etc/os-release && echo "$VERSION_CODENAME")." >&2
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

echo "[4/8] ROS 2 apt repo (jazzy / noble / arm64)"
if [ "$USE_CN_MIRROR" = "1" ]; then
  ROS_REPO_URL="https://${CN_MIRROR_HOST}/ros2/ubuntu"
else
  ROS_REPO_URL="http://packages.ros.org/ros2/ubuntu"
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
${ROS_REPO_URL} $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get upgrade -y

echo "[5/8] ROS 2 Jazzy base + dev tools"
sudo apt-get install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete

echo "[6/8] Packages commonly needed for the dreambo bipedal stack"
sudo apt-get install -y \
  ros-jazzy-vision-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-camera-info-manager \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf-transformations \
  ros-jazzy-control-msgs \
  ros-jazzy-controller-manager \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-urdf \
  ros-jazzy-ackermann-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-teleop-twist-keyboard

# Non-ROS system deps the workspace links against and tools used at runtime.
#   libeigen3-dev — imu_n100 (Eigen quaternion math)
#   python3-numpy — policy (observation vector construction)
#   python3-pip   — to install onnxruntime (no apt package on Noble);
#                   the pip install is the next step below.
# Other system deps declared in each package.xml are resolved later with:
#   rosdep install --from-paths src --ignore-src -y
sudo apt-get install -y \
  libeigen3-dev \
  python3-numpy \
  python3-pip

# onnxruntime (Python) — used by the policy node. Noble enforces PEP 668 so
# we install with --user (lands in ~/.local) + --break-system-packages
# (acknowledges PEP 668; doesn't actually break anything since onnxruntime
# has no apt counterpart to collide with). Opt out with INSTALL_ONNX=0.
if [ "$INSTALL_ONNX" = "1" ]; then
  echo "[onnx] Installing onnxruntime to ~/.local via pip (user-site)"
  pip install --user --break-system-packages onnxruntime
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

if grep -q "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc"; then
  echo "  ~/.bashrc already sources /opt/ros/jazzy/setup.bash."
else
  echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"
  echo "  Added 'source /opt/ros/jazzy/setup.bash' to ~/.bashrc."
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
echo " ROS 2 Jazzy install finished."
echo "==================================================================="
if [ "$NEED_RELOGIN" = "1" ]; then
  echo
  echo " >>> RE-LOGIN REQUIRED <<<"
  echo " ~/.bashrc and/or your group membership changed. Log out and back in"
  echo " (or reboot) before continuing. For a quick test in this shell only:"
  echo "     source /opt/ros/jazzy/setup.bash"
fi
echo
echo " After re-login, verify with:"
echo "     ros2 doctor"
echo "     ros2 pkg list | wc -l"
echo "     ros2 run joy joy_enumerate_devices    # plug a controller in first"
