#!/usr/bin/env bash
# Run on Thor (192.168.123.163 / 192.168.124.163).
# Enables IP forwarding, NAT, and UDP broadcast relay for port 54321
# so the workstation can run the Sharpa SDK directly.
# Nothing is written permanently — all settings reset on reboot.
#
# Usage:
#   sudo ./thor_route.sh start [workstation-ip]   # enable (default WS IP: 192.168.123.1)
#   sudo ./thor_route.sh stop                     # undo everything
#
# The UDP relay bridges port 54321 broadcasts in both directions:
#   Hands → broadcast → Thor → workstation:54321   (discovery heartbeats)
#   Workstation → Thor → broadcast → Hands:54321   (host announcements)

set -euo pipefail

HAND_SUBNET="192.168.124.0/24"
HAND_BCAST="192.168.124.255"
RELAY_PORT=54321

HAND_IFACE=$(ip -4 addr | awk '/192\.168\.124\.163/{print $NF}')
if [[ -z "$HAND_IFACE" ]]; then
    echo "ERROR: could not find interface with 192.168.124.163" >&2
    exit 1
fi

WS_IFACE=$(ip -4 addr | awk '/192\.168\.123\.163/{print $NF}')
if [[ -z "$WS_IFACE" ]]; then
    echo "ERROR: could not find interface with 192.168.123.163" >&2
    exit 1
fi

RELAY_PID_FILE="/tmp/sharpa_udp_relay.pid"

start_relay() {
    local ws_ip="$1"
    if ! command -v socat &>/dev/null; then
        echo "WARNING: socat not found — install with: sudo apt install socat"
        echo "         UDP broadcast relay will not be started."
        echo "         You can still use Option A (run scripts on Thor via SSH)."
        return
    fi

    echo "Starting UDP broadcast relay (port $RELAY_PORT) ..."

    # Relay 1: hand broadcasts → workstation (hands announce themselves)
    socat UDP4-RECVFROM:$RELAY_PORT,reuseaddr,broadcast,fork \
          UDP4-SENDTO:"${ws_ip}":$RELAY_PORT &
    echo $! >> "$RELAY_PID_FILE"

    # Relay 2: workstation unicast on port 54322 → hand subnet broadcast
    # (workstation sends host announcements to Thor:54322, Thor rebroadcasts)
    socat UDP4-RECVFROM:54322,reuseaddr,fork \
          UDP4-DATAGRAM:"${HAND_BCAST}":$RELAY_PORT,broadcast &
    echo $! >> "$RELAY_PID_FILE"

    echo "Relay started. On the workstation, set:"
    echo "  export SHARPA_ANNOUNCE_PORT=54322"
    echo "  export SHARPA_ANNOUNCE_HOST=192.168.123.163"
}

stop_relay() {
    if [[ -f "$RELAY_PID_FILE" ]]; then
        echo "Stopping UDP relay..."
        while read -r pid; do
            kill "$pid" 2>/dev/null || true
        done < "$RELAY_PID_FILE"
        rm -f "$RELAY_PID_FILE"
    fi
}

case "${1:-}" in
start)
    WS_IP="${2:-$(ip -4 route | awk '/192\.168\.123\./{print $0}' | grep -oP '192\.168\.123\.[0-9]+' | grep -v '163' | head -1)}"
    if [[ -z "$WS_IP" ]]; then
        echo "ERROR: could not auto-detect workstation IP. Pass it explicitly:"
        echo "  sudo $0 start 192.168.123.x"
        exit 1
    fi
    echo "Workstation IP: $WS_IP"

    echo "Enabling IP forwarding..."
    sysctl -w net.ipv4.ip_forward=1

    echo "Adding FORWARD rules ($WS_IFACE <-> $HAND_IFACE)..."
    iptables -C FORWARD -i "$WS_IFACE"  -o "$HAND_IFACE" -j ACCEPT 2>/dev/null \
        || iptables -A FORWARD -i "$WS_IFACE"  -o "$HAND_IFACE" -j ACCEPT
    iptables -C FORWARD -i "$HAND_IFACE" -o "$WS_IFACE"  -j ACCEPT 2>/dev/null \
        || iptables -A FORWARD -i "$HAND_IFACE" -o "$WS_IFACE"  -j ACCEPT

    echo "Adding MASQUERADE on $HAND_IFACE..."
    iptables -t nat -C POSTROUTING -o "$HAND_IFACE" -j MASQUERADE 2>/dev/null \
        || iptables -t nat -A POSTROUTING -o "$HAND_IFACE" -j MASQUERADE

    start_relay "$WS_IP"

    echo ""
    echo "Done."
    echo "  Option A (recommended): ssh to Thor and run scripts there directly."
    echo "  Option B (relay):       run scripts on workstation — socat relay bridges discovery."
    ;;

stop)
    stop_relay

    echo "Disabling IP forwarding..."
    sysctl -w net.ipv4.ip_forward=0

    echo "Removing FORWARD rules..."
    iptables -D FORWARD -i "$WS_IFACE"  -o "$HAND_IFACE" -j ACCEPT 2>/dev/null || true
    iptables -D FORWARD -i "$HAND_IFACE" -o "$WS_IFACE"  -j ACCEPT 2>/dev/null || true

    echo "Removing MASQUERADE..."
    iptables -t nat -D POSTROUTING -o "$HAND_IFACE" -j MASQUERADE 2>/dev/null || true

    echo "Done. All routing and relay reverted."
    ;;

*)
    echo "Usage: sudo $0 {start [workstation-ip]|stop}"
    exit 1
    ;;
esac
