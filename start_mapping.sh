#!/usr/bin/env bash
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CHAPT3_WS="${CHAPT3_WS:-$HOME/chapt3/chapt3_ws}"
CHAPT4_WS="$HERE/chapt4_ws"

source /opt/ros/jazzy/setup.bash

if [[ ! -f "$CHAPT3_WS/install/setup.bash" ]]; then
  echo "ERROR: Chapt3 workspace not found at $CHAPT3_WS" >&2
  echo "Build Chapt3 first or set CHAPT3_WS." >&2
  exit 1
fi
source "$CHAPT3_WS/install/setup.bash"
source "$CHAPT4_WS/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

if [[ -f "$CHAPT3_WS/src/chapt3_bringup/config/fastdds_no_shm.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$CHAPT3_WS/src/chapt3_bringup/config/fastdds_no_shm.xml"
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

exec ros2 launch chapt4_bringup mapping.launch.py "$@"
