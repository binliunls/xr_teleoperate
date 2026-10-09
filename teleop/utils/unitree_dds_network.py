"""Deployment-specific Unitree DDS addressing for the workstation and Thor."""

from __future__ import annotations

import ipaddress
from collections.abc import Sequence


WORKSTATION_DDS_ADDRESS = "192.168.125.222"
THOR_DDS_ADDRESS = "192.168.125.163"
WORKSTATION_ROBOT_DDS_ADDRESS = "192.168.123.222"
H2_BODY_DDS_ADDRESSES = ("192.168.123.161",)


def _normalize_peer(peer: str) -> str:
    """Validate an IPv4 DDS peer with an optional explicit UDP port."""
    raw = str(peer).strip()
    try:
        return str(ipaddress.ip_address(raw))
    except ValueError:
        host, separator, port_text = raw.rpartition(":")
        if not separator:
            raise
        address = ipaddress.ip_address(host)
        port = int(port_text)
        if address.version != 4 or not 1 <= port <= 65535:
            raise ValueError(f"invalid DDS peer: {peer}")
        return f"{address}:{port}"


def build_address_config(
    local_address: str = WORKSTATION_DDS_ADDRESS,
    peer_address: str | Sequence[str] = THOR_DDS_ADDRESS,
    *,
    max_message_size_bytes: int | None = None,
    allow_multicast: bool | str = True,
) -> str:
    """Build CycloneDDS XML using exact IP addresses, not ambiguous NIC names."""
    local = str(ipaddress.ip_address(local_address))
    peers = [peer_address] if isinstance(peer_address, str) else list(peer_address)
    peer_xml = "".join(
        f'<Peer address="{_normalize_peer(peer)}"/>' for peer in peers
    )
    multicast = str(allow_multicast).lower()
    if isinstance(allow_multicast, bool):
        multicast = "true" if allow_multicast else "false"
    if multicast not in {"true", "false", "spdp"}:
        raise ValueError(f"invalid CycloneDDS multicast mode: {allow_multicast}")
    max_message_size = (
        f"<MaxMessageSize>{int(max_message_size_bytes)}B</MaxMessageSize>"
        if max_message_size_bytes is not None
        else ""
    )
    return (
        '<CycloneDDS><Domain Id="any"><General><Interfaces>'
        f'<NetworkInterface address="{local}"/>'
        f'</Interfaces><AllowMulticast>{multicast}</AllowMulticast>'
        f'{max_message_size}</General>'
        '<Discovery><Peers>'
        f'{peer_xml}'
        '</Peers></Discovery>'
        '<Internal><SocketReceiveBufferSize min="16MB" max="16MB"/></Internal>'
        '</Domain></CycloneDDS>'
    )
