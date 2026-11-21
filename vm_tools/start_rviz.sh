#!/usr/bin/env bash
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: No ROS 2 Jazzy or Humble installation found." >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-${PI_IP:-192.168.8.144}}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/fastdds_no_shm.xml"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 daemon stop >/dev/null 2>&1 || true
exec ros2 run rviz2 rviz2 -d "$HERE/chapt4.rviz" "$@"
