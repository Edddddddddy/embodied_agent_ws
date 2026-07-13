#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
ROOT="${LLAMA_FACTORY_ROOT:-$WORKSPACE/third_party/LLaMA-Factory}"
VENV="${LORA_VENV:-$WORKSPACE/.venv-lora}"
REF="${LLAMA_FACTORY_REF:-ea31c43d806162a7fd98065abfef2d974fff5766}"

if [[ "${1:-}" == "--dry-run" ]]; then
  echo "git clone https://github.com/hiyouga/LLaMA-Factory.git $ROOT"
  echo "git -C $ROOT checkout $REF"
  echo "python3 -m venv $VENV"
  echo "$VENV/bin/pip install --upgrade pip"
  echo "$VENV/bin/pip install -e $ROOT"
  exit 0
fi

if [[ ! -d "$ROOT/.git" ]]; then
  git clone https://github.com/hiyouga/LLaMA-Factory.git "$ROOT"
fi
git -C "$ROOT" fetch --depth 1 origin "$REF"
git -C "$ROOT" checkout --detach "$REF"

python3 -m venv "$VENV"
"$VENV/bin/pip" install --upgrade pip
"$VENV/bin/pip" install -e "$ROOT"

ACTUAL_REF="$(git -C "$ROOT" rev-parse HEAD)"
[[ "$ACTUAL_REF" == "$REF" ]] || { echo "LLaMA-Factory pin mismatch" >&2; exit 1; }
"$VENV/bin/llamafactory-cli" version
echo "READY: LLaMA-Factory $ACTUAL_REF in $VENV"
