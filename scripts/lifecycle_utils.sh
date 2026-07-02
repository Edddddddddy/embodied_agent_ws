#!/usr/bin/env bash

wait_for_lifecycle_node() {
  local node_name="$1"
  local attempts="${2:-80}"
  local state
  for _ in $(seq 1 "$attempts"); do
    state="$(ros2 lifecycle get "/$node_name" 2>/dev/null || true)"
    if [[ "$state" == unconfigured* || "$state" == inactive* || \
          "$state" == active* || "$state" == finalized* ]]; then
      return 0
    fi
    sleep 0.1
  done
  echo "Lifecycle node /$node_name was not discovered" >&2
  return 1
}

activate_lifecycle_node() {
  local node_name="$1"
  wait_for_lifecycle_node "$node_name"
  ros2 lifecycle set "/$node_name" configure >/dev/null
  ros2 lifecycle set "/$node_name" activate >/dev/null
  local state
  state="$(ros2 lifecycle get "/$node_name")"
  [[ "$state" == active* ]]
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
