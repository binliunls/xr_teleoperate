#!/usr/bin/env bash
# Run on the workstation.
# Adds/removes a route so traffic to 192.168.124.0/24 (Sharpa hands)
# goes via Thor (192.168.123.163).
# Nothing is written permanently — resets on reboot.
#
# Usage:
#   sudo ./workstation_route.sh start   # add route
#   sudo ./workstation_route.sh stop    # remove route

set -euo pipefail

HAND_SUBNET="192.168.124.0/24"
THOR_GW="192.168.123.163"

case "${1:-}" in
start)
    if ip route show "$HAND_SUBNET" | grep -q "$THOR_GW"; then
        echo "Route already exists, nothing to do."
    else
        ip route add "$HAND_SUBNET" via "$THOR_GW"
        echo "Added route: $HAND_SUBNET via $THOR_GW"
    fi
    echo "Test with: ping 192.168.124.163"
    ;;

stop)
    if ip route show "$HAND_SUBNET" | grep -q "$THOR_GW"; then
        ip route del "$HAND_SUBNET" via "$THOR_GW"
        echo "Removed route: $HAND_SUBNET via $THOR_GW"
    else
        echo "Route not found, nothing to remove."
    fi
    ;;

*)
    echo "Usage: sudo $0 {start|stop}"
    exit 1
    ;;
esac
