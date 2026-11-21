#!/usr/bin/env bash
set -eo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: $0 [interface] [lidar_ip]"
  echo "Example: $0 eth0 192.168.10.160"
  exit 0
fi

IFACE="${1:-eth0}"
LIDAR_IP="${2:-192.168.10.160}"
HOST_IP="${LITRA_HOST_IP:-192.168.10.100}"

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "ERROR: interface $IFACE not found." >&2
  echo "Available interfaces:" >&2
  ip -br link >&2
  exit 1
fi

sudo ip link set "$IFACE" up

if ! ip -4 addr show dev "$IFACE" | grep -q 'inet 192\.168\.10\.'; then
  echo "Adding $HOST_IP/24 to $IFACE"
  sudo ip addr add "$HOST_IP/24" dev "$IFACE"
else
  echo "$IFACE already has a 192.168.10.x address"
fi

ip -4 addr show dev "$IFACE"

echo "Pinging Litra at $LIDAR_IP ..."
if ping -c 3 -W 1 "$LIDAR_IP"; then
  echo "Litra is reachable at $LIDAR_IP"
else
  echo "ERROR: $LIDAR_IP is not reachable." >&2
  echo "Check lidar power, Ethernet cable, and device IP (default 192.168.10.160)." >&2
  exit 1
fi
