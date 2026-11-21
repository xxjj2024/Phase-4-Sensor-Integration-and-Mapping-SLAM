#!/usr/bin/env bash
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CHAPT3_WS="${CHAPT3_WS:-$HOME/chapt3/chapt3_ws}"
CHAPT4_WS="$HERE/chapt4_ws"

source /opt/ros/jazzy/setup.bash
source "$CHAPT3_WS/install/setup.bash"
source "$CHAPT4_WS/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export CHAPT4_MAP_DIR="${CHAPT4_MAP_DIR:-$HERE/maps}"
exec ros2 run chapt4_bringup save_map.sh "$@"
