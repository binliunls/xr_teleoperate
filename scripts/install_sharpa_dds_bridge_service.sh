#!/usr/bin/env bash
# Run on Thor. Installs the Sharpa DDS bridge as a systemd service.
#
# Usage:
#   ./install_sharpa_dds_bridge_service.sh [--bridge PATH] [--sdk-lib DIR] [--no-start]
#   ./install_sharpa_dds_bridge_service.sh --uninstall
#
#   --bridge PATH   built sharpa_dds_bridge binary (see Makefile.sharpa_bridge)
#                   default: ../dds_bridge_5.0.4/sharpa_dds_bridge, else ./sharpa_dds_bridge
#   --sdk-lib DIR   directory holding libsharpa-wave-sdk.so (e.g. <sdk>/lib)
#                   default: ../sharpa_sdk_5.0.4/lib
#   --no-start      install and enable, but do not start now
#   --uninstall     stop, disable and remove the service and /opt/sharpa-dds-bridge
#
# Asks for sudo itself when not run as root. Installs into /opt/sharpa-dds-bridge,
# writes /etc/default/sharpa-dds-bridge (kept if it already exists) and enables the service at boot.
# Starting the service moves the hands to their zero pose.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR=/opt/sharpa-dds-bridge
UNIT=/etc/systemd/system/sharpa_dds_bridge.service
ENV_FILE=/etc/default/sharpa-dds-bridge

# Re-exec under sudo before parsing, since parsing shifts the arguments away.
if [[ $EUID -ne 0 ]]; then
    exec sudo "$0" "$@"
fi

BRIDGE="" SDK_LIB="" START=1 UNINSTALL=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --bridge)    BRIDGE="${2:?missing path for --bridge}"; shift 2 ;;
        --sdk-lib)   SDK_LIB="${2:?missing dir for --sdk-lib}"; shift 2 ;;
        --no-start)  START=0; shift ;;
        --uninstall) UNINSTALL=1; shift ;;
        -h|--help)   sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
    esac
done

if (( UNINSTALL )); then
    systemctl disable --now sharpa_dds_bridge.service 2>/dev/null || true
    rm -f "$UNIT"
    rm -rf "$INSTALL_DIR"
    systemctl daemon-reload
    echo "Removed sharpa_dds_bridge.service (kept $ENV_FILE)."
    exit 0
fi

if [[ -z "$BRIDGE" ]]; then
    for c in "$SCRIPT_DIR/../dds_bridge_5.0.4/sharpa_dds_bridge" "$SCRIPT_DIR/sharpa_dds_bridge"; do
        [[ -x "$c" ]] && { BRIDGE="$c"; break; }
    done
fi
[[ -n "$SDK_LIB" ]] || SDK_LIB="$SCRIPT_DIR/../sharpa_sdk_5.0.4/lib"

[[ -x "$BRIDGE" ]] || { echo "ERROR: --bridge must be an executable file (got '$BRIDGE')" >&2; exit 1; }
[[ -f "$SDK_LIB/libsharpa-wave-sdk.so" ]] || { echo "ERROR: $SDK_LIB/libsharpa-wave-sdk.so not found" >&2; exit 1; }

install -d "$INSTALL_DIR/lib"
install -m 0755 "$BRIDGE" "$INSTALL_DIR/sharpa_dds_bridge"
install -m 0644 "$SDK_LIB/libsharpa-wave-sdk.so" "$INSTALL_DIR/lib/libsharpa-wave-sdk.so"
install -m 0755 "$SCRIPT_DIR/sharpa_dds_bridge_run.sh" "$INSTALL_DIR/sharpa_dds_bridge_run.sh"
install -m 0644 "$SCRIPT_DIR/sharpa_dds_bridge.service" "$UNIT"

if [[ ! -f "$ENV_FILE" ]]; then
    cat > "$ENV_FILE" <<'CONF'
# Settings for sharpa_dds_bridge.service. Restart the service after editing:
#   sudo systemctl restart sharpa_dds_bridge
SHARPA_SIDE=both
SHARPA_SPEED=0.5
SHARPA_STATE_HZ=50
# DDS is pinned to the interface on this subnet (the hand network). Set an IP
# to pick a specific interface, or leave empty to use the CycloneDDS default.
SHARPA_DDS_ADDRESS=192.168.124.0
# SHARPA_EXTRA_ARGS=--tactile
CONF
fi

systemctl daemon-reload
if (( START )); then
    systemctl enable --now sharpa_dds_bridge.service
else
    systemctl enable sharpa_dds_bridge.service
fi

echo "Installed. Check with:"
echo "  systemctl status sharpa_dds_bridge --no-pager"
echo "  journalctl -u sharpa_dds_bridge -f"
