#!/usr/bin/env bash
# Watch rx/tx rates on the network interface that reaches the Sharpa hands.
#
# Usage:
#   bash sharpa_net_monitor.sh                    # auto-detects iface via 192.168.124.10
#   bash sharpa_net_monitor.sh eth11              # explicit interface
#   IFACE=eth11 bash sharpa_net_monitor.sh        # via env var
#
# Expected ranges (with both hands powered on):
#   ~0 Mb/s   — no hand streaming (JPEG disabled or hands quiet)
#   ~37 Mb/s  — one hand streaming tactile JPEG
#   ~74 Mb/s  — both hands streaming (saturates a 100 Mb link)
set -u

IFACE="${1:-${IFACE:-}}"
if [[ -z "$IFACE" ]]; then
    # Try to auto-detect via the route to a known hand IP.
    IFACE=$(ip -o route get 192.168.124.10 2>/dev/null | awk '{print $3; exit}')
fi
if [[ -z "$IFACE" || ! -d "/sys/class/net/$IFACE" ]]; then
    echo "ERROR: could not find the iface for 192.168.124.x" >&2
    echo "Pass it explicitly:  bash $0 <ifname>" >&2
    echo "Available ifaces:" >&2
    ls /sys/class/net/ >&2
    exit 1
fi

echo "Monitoring $IFACE.  Ctrl+C to stop."
trap 'echo; echo done.' EXIT

while true; do
    R1=$(cat "/sys/class/net/$IFACE/statistics/rx_bytes")
    T1=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
    sleep 1
    R2=$(cat "/sys/class/net/$IFACE/statistics/rx_bytes")
    T2=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
    RX=$(( (R2 - R1) * 8 / 1000000 ))
    TX=$(( (T2 - T1) * 8 / 1000000 ))
    printf "rx: %3d Mb/s   tx: %3d Mb/s\n" "$RX" "$TX"
done
