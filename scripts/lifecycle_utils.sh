#!/usr/bin/env bash

activate_lifecycle_node() {
  local node_name="$1"
  python3 "$WORKSPACE/scripts/activate_lifecycle_node.py" "$node_name"
}

wait_for_topic_subscribers() {
  local topic="$1"
  local minimum="${2:-1}"
  local attempts="${3:-80}"
  local count
  for _ in $(seq 1 "$attempts"); do
    count="$(ros2 topic info "$topic" 2>/dev/null | \
      awk '/Subscription count:/ {print $3}' || true)"
    if [[ "$count" =~ ^[0-9]+$ ]] && (( count >= minimum )); then
      return 0
    fi
    sleep 0.1
  done
  echo "Topic $topic did not discover $minimum subscriber(s)" >&2
  return 1
}
