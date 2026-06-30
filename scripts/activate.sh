#!/usr/bin/env bash
set -e

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"

# ROS-generated setup files are not safe under `set -u`. Preserve the caller's
# nounset setting while sourcing them, then restore it.
NOUNSET_WAS_ENABLED=false
if [[ $- == *u* ]]; then
  NOUNSET_WAS_ENABLED=true
  set +u
fi
source /opt/ros/jazzy/setup.bash
if [[ -f "$WORKSPACE/.venv/bin/activate" ]]; then
  source "$WORKSPACE/.venv/bin/activate"
  # ROS console scripts are generated with the system-Python shebang. Export
  # the venv packages so those child processes can still import model SDKs.
  VENV_SITE_PACKAGES="$(python -c 'import site; print(site.getsitepackages()[0])')"
  export PYTHONPATH="$VENV_SITE_PACKAGES${PYTHONPATH:+:$PYTHONPATH}"
fi
source "$WORKSPACE/install/setup.bash"
if [[ -f "$WORKSPACE/.env" ]]; then
  ALLEXPORT_WAS_ENABLED=false
  if [[ $- == *a* ]]; then
    ALLEXPORT_WAS_ENABLED=true
  else
    set -a
  fi
  source "$WORKSPACE/.env"
  if [[ "$ALLEXPORT_WAS_ENABLED" != true ]]; then
    set +a
  fi
fi
if [[ "$NOUNSET_WAS_ENABLED" == true ]]; then
  set -u
fi
