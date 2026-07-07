#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
THIRD_PARTY="${THIRD_PARTY:-$WORKSPACE/third_party}"
SUMMER_TTS_REPO="${SUMMER_TTS_REPO:-https://github.com/huakunyang/SummerTTS.git}"
SUMMER_TTS_REF="${SUMMER_TTS_REF:-main}"
SUMMER_TTS_DIR="${SUMMER_TTS_DIR:-$THIRD_PARTY/SummerTTS}"
SUMMER_TTS_BINARY="${SUMMER_TTS_BINARY:-$SUMMER_TTS_DIR/build/tts_test}"
SUMMER_TTS_MODEL="${SUMMER_TTS_MODEL:-$SUMMER_TTS_DIR/models/single_speaker_fast.bin}"

mkdir -p "$THIRD_PARTY"
touch "$THIRD_PARTY/COLCON_IGNORE"

sudo apt-get update
sudo apt-get install -y build-essential cmake git

if [[ ! -d "$SUMMER_TTS_DIR/.git" ]]; then
  git clone --depth 1 --branch "$SUMMER_TTS_REF" "$SUMMER_TTS_REPO" "$SUMMER_TTS_DIR"
else
  echo "READY: $SUMMER_TTS_DIR"
fi

patch_missing_cstdint() {
  local target="$1"
  python3 - "$target" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
if "#include <cstdint>" in text:
    raise SystemExit(0)
lines = text.splitlines()
for index, line in enumerate(lines):
    if line.startswith("#include"):
        continue
    lines.insert(index, "#include <cstdint>")
    break
else:
    lines.insert(0, "#include <cstdint>")
path.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY
}

# SummerTTS 上游源码在较新的 GCC 上需要显式包含 cstdint。
# third_party 被 gitignore，只在本地部署时打补丁，不把第三方源码 vendoring 进本仓库。
patch_missing_cstdint "$SUMMER_TTS_DIR/src/header/hanzi2phoneid.h"
patch_missing_cstdint "$SUMMER_TTS_DIR/src/header/Hanz2Piny.h"
patch_missing_cstdint "$SUMMER_TTS_DIR/src/header/pinyinmap.h"

cmake -S "$SUMMER_TTS_DIR" -B "$SUMMER_TTS_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SUMMER_TTS_DIR/build" -j "$(nproc)"

python3 "$WORKSPACE/scripts/summer_tts_smoke.py" \
  --binary "$SUMMER_TTS_BINARY" \
  --model "$SUMMER_TTS_MODEL" \
  --preflight-only

cat <<EOF

SummerTTS runtime is ready.

Local source:
  $SUMMER_TTS_DIR

Next checks:
  source scripts/activate.sh
  bash scripts/acceptance_test.sh summer-tts-preflight
  bash scripts/acceptance_test.sh summer-tts-smoke
  bash scripts/acceptance_test.sh summer-pseudo-tts

To use SummerTTS in the offline Agent, set:
  tts_provider:=summer
EOF
