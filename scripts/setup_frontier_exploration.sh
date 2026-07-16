#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/lifecycle_utils.sh"
embodied_resolve_workspace "${BASH_SOURCE[0]}"
REPOSITORY="https://github.com/robo-friends/m-explore-ros2.git"
REVISION="326cf8a0b487c34246bb8f3326afbcd69576dc60"
SOURCE_DIR="$WORKSPACE/third_party/m-explore-ros2"
DRY_RUN=false

if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=true
fi

echo "frontier explorer: $REPOSITORY@$REVISION"
echo "source: $SOURCE_DIR"
if [[ "$DRY_RUN" == "true" ]]; then
  echo "DRY RUN: clone/fetch pinned m-explore-ros2 and build explore_lite_msgs + explore_lite"
  exit 0
fi

if [[ ! -d "$SOURCE_DIR/.git" ]]; then
  mkdir -p "$(dirname "$SOURCE_DIR")"
  git clone "$REPOSITORY" "$SOURCE_DIR"
elif [[ "$(git -C "$SOURCE_DIR" remote get-url origin)" != "$REPOSITORY" ]]; then
  echo "FAIL: $SOURCE_DIR points to an unexpected Git remote" >&2
  exit 2
fi

git -C "$SOURCE_DIR" fetch --tags origin
git -C "$SOURCE_DIR" checkout --detach "$REVISION"

# shellcheck source=activate.sh
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"
colcon build --symlink-install \
  --base-paths "$SOURCE_DIR" \
  --packages-select explore_lite_msgs explore_lite

# ROS/colcon 生成的 setup 脚本会读取未定义的 COLCON_TRACE；本脚本启用了
# `set -u`，因此只在 source 期间临时关闭 nounset，避免“构建成功却验收失败”。
set +u
source "$WORKSPACE/install/setup.bash"
set -u
ros2 pkg executables explore_lite | grep -q 'explore_lite explore'
embodied_workspace_doctor true
echo "PASS: pinned Explore Lite frontier runtime is installed"
