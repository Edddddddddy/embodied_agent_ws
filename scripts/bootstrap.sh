#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"

sudo apt-get update
sudo apt-get install -y \
  libasound2-plugins \
  nlohmann-json3-dev \
  pulseaudio-utils \
  python3-venv \
  python3-pip \
  portaudio19-dev \
  python3-pytest

python3 -m venv --system-site-packages "$WORKSPACE/.venv"
source "$WORKSPACE/.venv/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "$WORKSPACE/requirements.txt"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WORKSPACE"
colcon build --symlink-install

echo "Bootstrap complete. Run: source $WORKSPACE/scripts/activate.sh"
