#!/usr/bin/env bash
# Shared ROS DDS runtime defaults for WSL demos/tests.
#
# FastDDS enables shared-memory transport by default. In WSL, stale
# `/dev/shm/fastrtps_port*` lock files or parallel ROS/Gazebo processes often
# produce:
#   RTPS_TRANSPORT_SHM Error ... Failed init_port fastrtps_port7000
#
# The voice demos run all ROS nodes locally, so UDPv4 is stable enough and avoids
# that SHM lock path. Set EMBODIED_ALLOW_FASTDDS_SHM=true only when explicitly
# debugging FastDDS shared-memory transport.

if [[ "${EMBODIED_ALLOW_FASTDDS_SHM:-false}" != "true" ]]; then
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
fi

