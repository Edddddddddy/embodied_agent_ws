#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/lifecycle_utils.sh"
embodied_resolve_workspace "${BASH_SOURCE[0]}"

sudo apt-get update
sudo apt-get install -y \
  libasound2-plugins \
  nlohmann-json3-dev \
  pulseaudio-utils \
  python3-venv \
  python3-pip \
  portaudio19-dev \
  python3-pytest \
  ros-jazzy-ros-gz \
  ros-jazzy-rviz2 \
  ros-jazzy-turtlebot3-gazebo

python3 -m venv --system-site-packages "$WORKSPACE/.venv"
source "$WORKSPACE/.venv/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "$WORKSPACE/requirements.txt"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WORKSPACE"
colcon build --symlink-install

# 自动建图是当前主演示，不再让用户完成基础构建后才在运行期发现 explorer 缺失。
# 可在仅开发语音模块时显式设为 false，缩短无网络环境的 bootstrap。
if [[ "${EMBODIED_INSTALL_FRONTIER:-true}" == "true" ]]; then
  WORKSPACE="$WORKSPACE" bash "$WORKSPACE/scripts/setup_frontier_exploration.sh"
fi

echo "Bootstrap complete. Run: source $WORKSPACE/scripts/activate.sh"
