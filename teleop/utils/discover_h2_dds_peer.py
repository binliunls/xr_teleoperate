#!/usr/bin/env python3
"""Discover an H2 process' current CycloneDDS SPDP unicast peer.

This is a passive, standard-library-only probe.  It joins the SPDP multicast
group on the selected Thor interface and reads the process name and advertised
metatraffic unicast locator from participant discovery packets.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import shlex
import socket
import subprocess
import struct
import sys
import time
from collections.abc import Iterator, Sequence


RTPS_HEADER_SIZE = 20
DATA_SUBMESSAGE_ID = 0x15
PID_SENTINEL = 0x0001
PID_METATRAFFIC_UNICAST_LOCATOR = 0x0032
PID_PROPERTY_LIST = 0x0059
PL_CDR_BE = 0x0002
PL_CDR_LE = 0x0003

DEFAULT_H2_PROCESS_NAMES = ("basic_service", "humanoid")
DEFAULT_THOR_SSH_TARGET = "unitree@192.168.125.163"
DEFAULT_THOR_HELPER_PATH = "/home/unitree/.local/bin/discover_h2_dds_peer.py"


class H2PeerDiscoveryError(RuntimeError):
    """The required H2 DDS participants could not be resolved safely."""


def _validate_peer(peer: object, expected_source_ip: str) -> str:
    raw = str(peer).strip()
    host, separator, port_text = raw.rpartition(":")
    if not separator:
        raise H2PeerDiscoveryError(f"discovered peer has no UDP port: {raw!r}")
    try:
        address = ipaddress.ip_address(host)
        port = int(port_text)
    except ValueError as error:
        raise H2PeerDiscoveryError(f"invalid discovered peer: {raw!r}") from error
    if address.version != 4 or str(address) != expected_source_ip:
        raise H2PeerDiscoveryError(
            f"discovered peer {raw!r} is not on expected H2 address "
            f"{expected_source_ip}"
        )
    if not 1 <= port <= 65535:
        raise H2PeerDiscoveryError(f"invalid discovered peer port: {raw!r}")
    return f"{address}:{port}"


def _parse_remote_results(
    output: str,
    process_names: Sequence[str],
    expected_source_ip: str,
) -> list[str]:
    expected = tuple(process_names)
    found: dict[str, str] = {}
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            result = json.loads(line)
        except json.JSONDecodeError as error:
            raise H2PeerDiscoveryError(
                f"unexpected output from Thor DDS discovery: {line!r}"
            ) from error
        process_name = result.get("process_name")
        if process_name not in expected:
            raise H2PeerDiscoveryError(
                f"unexpected H2 process in discovery output: {process_name!r}"
            )
        if process_name in found:
            raise H2PeerDiscoveryError(
                f"ambiguous discovery result for H2 process {process_name!r}"
            )
        found[process_name] = _validate_peer(result.get("peer"), expected_source_ip)

    missing = [name for name in expected if name not in found]
    if missing:
        raise H2PeerDiscoveryError(
            "missing DDS discovery result for H2 process(es): " + ", ".join(missing)
        )
    return [found[name] for name in expected]


def discover_h2_body_peers_via_ssh(
    *,
    ssh_target: str = DEFAULT_THOR_SSH_TARGET,
    helper_path: str = DEFAULT_THOR_HELPER_PATH,
    source_ip: str = "192.168.123.161",
    interface: str = "eth10",
    process_names: Sequence[str] = DEFAULT_H2_PROCESS_NAMES,
    timeout_s: float = 35.0,
) -> list[str]:
    """Resolve current H2 metatraffic endpoints by probing SPDP on Thor.

    Both probes run concurrently through one SSH connection. Results are only
    returned when every requested process produced exactly one validated peer.
    """
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive")
    if not process_names or len(set(process_names)) != len(process_names):
        raise ValueError("process_names must be non-empty and unique")

    probe_commands = []
    for process_name in process_names:
        probe_commands.append(
            shlex.join(
                [
                    helper_path,
                    "--interface",
                    interface,
                    "--source-ip",
                    source_ip,
                    "--process-name",
                    process_name,
                    "--timeout",
                    f"{timeout_s:g}",
                    "--json",
                ]
            )
        )

    launches = []
    waits = []
    for index, command in enumerate(probe_commands):
        launches.append(f"({command}) & peer_probe_{index}=$!")
        waits.append(f'wait "$peer_probe_{index}" || peer_status=1')
    remote_script = "; ".join(
        ["peer_status=0", *launches, *waits, 'exit "$peer_status"']
    )
    ssh_command = [
        "ssh",
        "-T",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=8",
        "-o",
        "ServerAliveInterval=5",
        "-o",
        "ServerAliveCountMax=2",
        ssh_target,
        remote_script,
    ]
    try:
        completed = subprocess.run(
            ssh_command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s + 15.0,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise H2PeerDiscoveryError(
            f"could not run H2 DDS discovery through SSH target {ssh_target!r}: {error}"
        ) from error

    if completed.returncode != 0:
        detail = completed.stderr.strip() or "remote discovery returned no details"
        raise H2PeerDiscoveryError(
            f"H2 DDS discovery failed through {ssh_target!r}: {detail}"
        )
    return _parse_remote_results(completed.stdout, process_names, source_ip)


def _align4(value: int) -> int:
    return (value + 3) & ~3


def _iter_data_payloads(packet: bytes) -> Iterator[bytes]:
    if len(packet) < RTPS_HEADER_SIZE or packet[:4] != b"RTPS":
        return

    offset = RTPS_HEADER_SIZE
    while offset + 4 <= len(packet):
        submessage_id = packet[offset]
        flags = packet[offset + 1]
        endian = "<" if flags & 0x01 else ">"
        submessage_size = struct.unpack_from(endian + "H", packet, offset + 2)[0]
        end = len(packet) if submessage_size == 0 else offset + 4 + submessage_size
        if end > len(packet) or end <= offset + 4:
            return

        if submessage_id == DATA_SUBMESSAGE_ID and offset + 8 <= end:
            content = offset + 4
            inline_qos_offset = struct.unpack_from(endian + "H", packet, content + 2)[0]
            # octetsToInlineQos is relative to the end of the four-byte
            # extraFlags/octetsToInlineQos pair, not to the DATA content start.
            payload = content + 4 + inline_qos_offset

            if flags & 0x02:  # Inline QoS precedes the serialized payload.
                while payload + 4 <= end:
                    parameter_id, parameter_size = struct.unpack_from(
                        endian + "HH", packet, payload
                    )
                    payload += 4
                    if parameter_id == PID_SENTINEL:
                        break
                    payload += _align4(parameter_size)

            if flags & 0x04 and payload < end:  # Serialized data is present.
                yield packet[payload:end]

        offset = end


def _read_cdr_string(data: bytes, offset: int, endian: str) -> tuple[str, int]:
    if offset + 4 > len(data):
        raise ValueError("truncated CDR string length")
    size = struct.unpack_from(endian + "I", data, offset)[0]
    offset += 4
    if size == 0 or offset + size > len(data):
        raise ValueError("invalid CDR string size")
    raw = data[offset : offset + size]
    if raw[-1] == 0:
        raw = raw[:-1]
    offset += _align4(size)
    return raw.decode("utf-8", errors="replace"), offset


def _parse_properties(data: bytes, endian: str) -> dict[str, str]:
    if len(data) < 4:
        return {}
    count = struct.unpack_from(endian + "I", data, 0)[0]
    offset = 4
    properties: dict[str, str] = {}
    try:
        for _ in range(count):
            name, offset = _read_cdr_string(data, offset, endian)
            value, offset = _read_cdr_string(data, offset, endian)
            properties[name] = value
    except ValueError:
        return {}
    return properties


def _parse_spdp(packet: bytes) -> dict[str, object] | None:
    for payload in _iter_data_payloads(packet):
        if len(payload) < 4:
            continue
        encoding = int.from_bytes(payload[:2], "big")
        if encoding == PL_CDR_LE:
            endian = "<"
        elif encoding == PL_CDR_BE:
            endian = ">"
        else:
            continue

        offset = 4  # Skip the CDR encapsulation header.
        properties: dict[str, str] = {}
        locators: list[tuple[str, int]] = []
        while offset + 4 <= len(payload):
            parameter_id, parameter_size = struct.unpack_from(
                endian + "HH", payload, offset
            )
            offset += 4
            if parameter_id == PID_SENTINEL:
                break
            if offset + parameter_size > len(payload):
                break
            value = payload[offset : offset + parameter_size]

            if parameter_id == PID_PROPERTY_LIST:
                properties.update(_parse_properties(value, endian))
            elif (
                parameter_id == PID_METATRAFFIC_UNICAST_LOCATOR
                and len(value) >= 24
            ):
                kind, port = struct.unpack_from(endian + "iI", value, 0)
                if kind == 1:  # LOCATOR_KIND_UDPv4
                    address = socket.inet_ntoa(value[20:24])
                    locators.append((address, port))

            offset += _align4(parameter_size)

        process_name = properties.get("__ProcessName")
        if process_name and locators:
            return {
                "process_name": process_name,
                "pid": properties.get("__Pid", ""),
                "locators": locators,
            }
    return None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Passively discover an H2 CycloneDDS process peer on Thor"
    )
    parser.add_argument("--interface", default="eth10")
    parser.add_argument("--source-ip", default="192.168.123.161")
    parser.add_argument("--process-name", default="basic_service")
    parser.add_argument("--group", default="239.255.0.1")
    parser.add_argument("--port", type=int, default=7400)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--json", action="store_true", dest="as_json")
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        interface_index = socket.if_nametoindex(args.interface)
    except OSError as error:
        print(f"error: interface {args.interface!r} is unavailable: {error}", file=sys.stderr)
        return 2

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", args.port))
        membership = struct.pack(
            "=4s4si",
            socket.inet_aton(args.group),
            socket.inet_aton("0.0.0.0"),
            interface_index,
        )
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            sock.settimeout(min(0.5, max(0.001, deadline - time.monotonic())))
            try:
                packet, source = sock.recvfrom(65535)
            except TimeoutError:
                continue
            if source[0] != args.source_ip:
                continue
            participant = _parse_spdp(packet)
            if not participant or participant["process_name"] != args.process_name:
                continue

            locators = participant.pop("locators")
            address, port = next(
                (locator for locator in locators if locator[0] == args.source_ip),
                locators[0],
            )
            result = {
                **participant,
                "source_ip": source[0],
                "spdp_source_port": source[1],
                "meta_unicast_ip": address,
                "meta_unicast_port": port,
                "peer": f"{address}:{port}",
                "guid_prefix": packet[8:20].hex(),
            }
            if args.as_json:
                print(json.dumps(result, sort_keys=True))
            else:
                print(result["peer"])
            return 0
    finally:
        sock.close()

    print(
        f"error: no SPDP packet for {args.process_name!r} from {args.source_ip} "
        f"within {args.timeout:g}s",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
