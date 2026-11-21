#!/usr/bin/env bash
set -eo pipefail

NAME="${1:-room}"
MAP_DIR="${CHAPT4_MAP_DIR:-$HOME/chapt4/maps}"

if [[ "$NAME" == */* ]]; then
  OUT="$NAME"
  mkdir -p "$(dirname "$OUT")"
else
  mkdir -p "$MAP_DIR"
  OUT="$MAP_DIR/$NAME"
fi

if ! ros2 topic list 2>/dev/null | grep -qx '/map'; then
  echo "ERROR: /map topic not found. Start mapping and drive the robot first." >&2
  exit 1
fi

echo "Saving /map to ${OUT}.yaml and ${OUT}.pgm"
timeout 30s ros2 run nav2_map_server map_saver_cli \
  -f "$OUT" \
  --fmt pgm \
  --mode trinary \
  --ros-args \
  -p use_sim_time:=false \
  -p map_subscribe_transient_local:=true

ls -l "${OUT}.yaml" "${OUT}.pgm"
