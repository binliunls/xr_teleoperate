#!/usr/bin/env bash
set -euo pipefail

readonly DEFAULT_INTERFACE="enx80691a14d263"
readonly DEFAULT_CONNECTION="h2-thor-usb"
readonly THOR_ADDRESS="192.168.125.163/24"

interface="$DEFAULT_INTERFACE"
connection=""
connection_was_explicit=false
activate=false
dry_run=false

usage() {
    cat <<'EOF'
Usage: install_thor_usb_ethernet.sh [OPTIONS]

Create or update Thor's persistent NetworkManager profile for the dedicated
workstation link. If the device already has an active Ethernet profile, that
profile is updated in place so a duplicate profile is not created.

Options:
  --interface NAME   Network interface (default: enx80691a14d263)
  --connection NAME  NetworkManager connection to create or update
                     (default: active profile, otherwise h2-thor-usb)
  --activate         activate the profile now (may interrupt this link)
  --dry-run          print the profile changes without touching the host
  -h, --help         show this help

The installed IPv4 settings are 192.168.125.163/24, manual addressing,
autoconnect enabled, no gateway, no automatic DNS, and never-default enabled.
No UUID or MAC address is copied from another Thor installation.
EOF
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

print_command() {
    printf '+'
    printf ' %q' "$@"
    printf '\n'
}

run() {
    if "$dry_run"; then
        print_command "$@"
    else
        "$@"
    fi
}

while (($#)); do
    case "$1" in
        --interface)
            (($# >= 2)) || die "--interface requires a value"
            interface="$2"
            shift 2
            ;;
        --connection)
            (($# >= 2)) || die "--connection requires a value"
            connection="$2"
            connection_was_explicit=true
            shift 2
            ;;
        --activate)
            activate=true
            shift
            ;;
        --dry-run)
            dry_run=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown argument: $1"
            ;;
    esac
done

[[ -n "$interface" ]] || die "interface must not be empty"
[[ ${#interface} -le 15 ]] || die "interface names are limited to 15 characters"
[[ "$interface" =~ ^[[:alnum:]_.:-]+$ ]] \
    || die "interface contains unsupported characters: $interface"
if "$connection_was_explicit"; then
    [[ -n "$connection" ]] || die "connection must not be empty"
    [[ "$connection" != *$'\n'* ]] || die "connection must not contain a newline"
fi

if "$dry_run"; then
    [[ -n "$connection" ]] || connection="$DEFAULT_CONNECTION"
    printf 'Dry run: configure NetworkManager connection %q on %q.\n' \
        "$connection" "$interface"
    run nmcli connection add type ethernet ifname "$interface" \
        con-name "$connection"
else
    ((EUID == 0)) || die "run as root (for example, with sudo)"
    command -v nmcli >/dev/null 2>&1 || die "nmcli is required"
    [[ -d "/sys/class/net/$interface" ]] \
        || die "network interface does not exist: $interface"

    # Prefer the profile already carrying this device. Updating it avoids two
    # autoconnect candidates fighting over the dedicated USB Ethernet adapter.
    if [[ -z "$connection" ]]; then
        active_connection="$(
            nmcli -g GENERAL.CONNECTION device show "$interface" 2>/dev/null \
                || true
        )"
        if [[ -n "$active_connection" && "$active_connection" != "--" ]]; then
            connection="$active_connection"
        else
            connection="$DEFAULT_CONNECTION"
        fi
    fi

    if nmcli -g connection.type connection show "$connection" \
        >/dev/null 2>&1; then
        connection_type="$(
            nmcli -g connection.type connection show "$connection"
        )"
        [[ "$connection_type" == "802-3-ethernet" ]] \
            || die "connection '$connection' is not an Ethernet profile"

        profile_interface="$(
            nmcli -g connection.interface-name connection show "$connection"
        )"
        if [[ -n "$profile_interface" \
              && "$profile_interface" != "--" \
              && "$profile_interface" != "$interface" ]]; then
            die "connection '$connection' belongs to '$profile_interface', not '$interface'"
        fi
        printf 'Updating NetworkManager connection %q on %q.\n' \
            "$connection" "$interface"
    else
        if "$connection_was_explicit"; then
            printf 'Creating NetworkManager connection %q on %q.\n' \
                "$connection" "$interface"
        else
            printf 'No active profile found; creating %q on %q.\n' \
                "$connection" "$interface"
        fi
        run nmcli connection add type ethernet ifname "$interface" \
            con-name "$connection"
    fi
fi

run nmcli connection modify "$connection" \
    connection.interface-name "$interface" \
    connection.autoconnect yes \
    ipv4.method manual \
    ipv4.addresses "$THOR_ADDRESS" \
    ipv4.gateway "" \
    ipv4.never-default yes \
    ipv4.ignore-auto-dns yes
run nmcli connection reload

if "$activate"; then
    if ! "$dry_run" && [[ -n "${SSH_CONNECTION:-}" ]]; then
        read -r _ _ ssh_server_address _ <<<"$SSH_CONNECTION"
        if [[ "$ssh_server_address" == "${THOR_ADDRESS%/*}" ]]; then
            die "refusing to reactivate the Ethernet link carrying this SSH session; reboot or run from Thor's local console"
        fi
    fi
    run nmcli connection up "$connection" ifname "$interface"
elif "$dry_run"; then
    printf '\nDry run only; no profile was saved or reactivated.\n'
else
    printf '\nProfile saved but not reactivated. Reboot Thor, or activate it from a\n'
    printf 'local console with:\n  sudo nmcli connection up %q ifname %q\n' \
        "$connection" "$interface"
fi

printf '\nAfter activation, verify with:\n'
printf '  ip -4 address show dev %q\n' "$interface"
printf '  nmcli -f connection.id,connection.autoconnect,ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.never-default connection show %q\n' \
    "$connection"
