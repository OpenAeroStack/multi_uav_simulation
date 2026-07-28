#!/usr/bin/env bash
# pi_setup.sh — one-shot edge-vision setup for the Raspberry Pi 4B (UAV2 node).
#
# Target: Raspberry Pi 4B · Ubuntu Server 22.04 (64-bit) · ROS 2 Humble
# (matches the host exactly, so DDS "just works").
#
# Installs ROS 2 Humble (ros-base), a YOLO venv (ultralytics + yolov8n),
# builds the uav_vision package, and runs a YOLO sanity check.
#
# Usage (on the Pi, or run remotely over SSH):
#   bash pi_setup.sh
#
# Safe to re-run: each step is idempotent-ish (skips what's already done).
set -euo pipefail

REPO_URL="${REPO_URL:-}"                 # optional: git URL to clone if repo missing
REPO_DIR="${REPO_DIR:-$HOME/multi_uav_simulation}"
BRANCH="${BRANCH:-ground-vs-edge-processing-RPi}"
YOLO_ENV="${YOLO_ENV:-$HOME/yolo_env}"

echo "==> [1/6] System update"
sudo apt-get update
sudo apt-get -y full-upgrade

echo "==> [2/6] Locale + ROS 2 apt repo"
sudo apt-get install -y locales curl gnupg lsb-release software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "==> [3/6] Install ROS 2 Humble (ros-base) + build tools"
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base ros-dev-tools \
     python3-colcon-common-extensions python3-venv python3-pip \
     libgl1 libglib2.0-0
grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "==> [4/6] YOLO venv ($YOLO_ENV) with ultralytics"
if [ ! -d "$YOLO_ENV" ]; then
  python3 -m venv "$YOLO_ENV" --system-site-packages
fi
# shellcheck disable=SC1091
source "$YOLO_ENV/bin/activate"
pip install --upgrade pip
# IMPORTANT: install the CPU-only PyTorch FIRST. The default PyPI wheel for arm64
# is a CUDA build (e.g. torch 2.x+cuXXX) whose CPU kernels use SIMD instructions
# (dot-product/fp16) the Pi 4's Cortex-A72 lacks -> "Illegal instruction" (SIGILL)
# during YOLO inference. The +cpu wheels avoid that.
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics
# fetch the small model weights into the venv dir
( cd "$YOLO_ENV" && python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" )
deactivate

echo "==> [5/6] Get + build uav_vision"
if [ ! -d "$REPO_DIR" ]; then
  if [ -z "$REPO_URL" ]; then
    echo "!! $REPO_DIR missing and REPO_URL not set. Clone the repo there first, or"
    echo "   re-run:  REPO_URL=<git-url> bash pi_setup.sh"
    exit 1
  fi
  git clone "$REPO_URL" "$REPO_DIR"
fi
cd "$REPO_DIR"
git checkout "$BRANCH" 2>/dev/null || echo "(staying on current branch)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_vision
grep -q "source $REPO_DIR/install/setup.bash" ~/.bashrc || \
  echo "source $REPO_DIR/install/setup.bash" >> ~/.bashrc

echo "==> [6/6] YOLO sanity check on ARM"
# shellcheck disable=SC1091
source "$YOLO_ENV/bin/activate"
cd "$YOLO_ENV"
python - <<'PY'
from ultralytics import YOLO
m = YOLO('yolov8n.pt')
r = m('https://ultralytics.com/images/bus.jpg')
print("persons detected:", sum(int(c) == 0 for c in r[0].boxes.cls))
PY
deactivate

echo ""
echo "==> DONE. ROS 2 Humble + uav_vision + YOLO are installed on the Pi."
echo "    Open a fresh shell (so ~/.bashrc sourcing takes effect), then the Pi is"
echo "    ready to run camera_relay + detector as the UAV2 edge node (Phase 4)."
