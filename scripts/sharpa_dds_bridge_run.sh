#!/usr/bin/env bash
# Launcher used by sharpa_dds_bridge.service. Settings come from the
# environment (/etc/default/sharpa-dds-bridge):
#   SHARPA_SIDE        left | right | both        (default: both)
#   SHARPA_SPEED       hand speed coefficient     (default: 0.5)
#   SHARPA_STATE_HZ    state publish rate in Hz   (default: 50)
#   SHARPA_EXTRA_ARGS  extra flags, e.g. --tactile
#   SHARPA_DDS_ADDRESS subnet or IP whose interface DDS is pinned to
#                      (default: 192.168.124.0, the hand network; empty = no pinning)
#   INSTALL_DIR        holds sharpa_dds_bridge and lib/  (default: /opt/sharpa-dds-bridge)
set -euo pipefail

INSTALL_DIR="${INSTALL_DIR:-/opt/sharpa-dds-bridge}"
# Pin CycloneDDS to the hand network. Without this it picks whichever interface
# it ranks first (the Wi-Fi adapter on Thor). A CYCLONEDDS_URI set by the
# caller takes precedence.
DDS_ADDRESS="${SHARPA_DDS_ADDRESS-192.168.124.0}"
if [[ -z "${CYCLONEDDS_URI:-}" && -n "$DDS_ADDRESS" ]]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"$DDS_ADDRESS\" multicast=\"true\"/></Interfaces></General></Domain></CycloneDDS>"
fi

# lib/ carries libsharpa-wave-sdk.so; /usr/local/lib carries libddsc/libddscxx.
export LD_LIBRARY_PATH="$INSTALL_DIR/lib:/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# shellcheck disable=SC2086  # SHARPA_EXTRA_ARGS is meant to word-split
exec "$INSTALL_DIR/sharpa_dds_bridge" \
    --side "${SHARPA_SIDE:-both}" \
    --speed "${SHARPA_SPEED:-0.5}" \
    --state-hz "${SHARPA_STATE_HZ:-50}" \
    ${SHARPA_EXTRA_ARGS:-}
