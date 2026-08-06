#!/usr/bin/env bash
#
# Staged Unitree H2 Thor-side installation and checks.
# This script runs only on Thor/Jetson. It does not stand the robot or automatically chain the camera and hand bridge.
#
# First-time setup:
#   bash setup_h2_thor.sh workflow       # Show the staged procedure
#   bash setup_h2_thor.sh install        # Install base tools and SDK
#   bash setup_h2_thor.sh camera-install # Prepare the GMSL camera environment
#   bash setup_h2_thor.sh sharpa-install # Install the Sharpa ARM SDK
#   bash setup_h2_thor.sh bridge-build   # Build the Sharpa DDS bridge
#   bash setup_h2_thor.sh doctor         # Validate the Thor installation
#
# After every cold start, run these in two separate Thor terminals:
#   bash setup_h2_thor.sh camera-start   # Publish four DDS camera streams
#   bash setup_h2_thor.sh bridge-start   # Connect both Sharpa hands to DDS

set -Eeuo pipefail
IFS=$'\n\t'

DDS_DOMAIN="${DDS_DOMAIN:-0}"
CAMERA_DDS_DOMAIN="${CAMERA_DDS_DOMAIN:-10}"
CAMERA_NETWORK_INTERFACE="${CAMERA_NETWORK_INTERFACE:-}"

CAMERA_ROOT="${CAMERA_ROOT:-$HOME/h2_camera}"
CAMERA_SETUP="${CAMERA_SETUP:-$CAMERA_ROOT/setup_gmsl_cameras.sh}"
CAMERA_PUBLISHER="${CAMERA_PUBLISHER:-$CAMERA_ROOT/dds_camera_publisher.py}"
CAMERA_VENV="${CAMERA_VENV:-$HOME/.venvs/h2_camera_dds}"
XR_ROOT="${XR_ROOT:-$HOME/xr_teleoperate}"
UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-$HOME/unitree_sdk2}"
SHARPA_BRIDGE_DIR="${SHARPA_BRIDGE_DIR:-$HOME/Sharpa/bridge_dds}"
SHARPA_SDK_ROOT="${SHARPA_SDK_ROOT:-/opt/sharpa-wave-sdk}"
SHARPA_INCLUDE_DIR="${SHARPA_INCLUDE_DIR:-$SHARPA_SDK_ROOT/include}"
SHARPA_LIB_DIR="${SHARPA_LIB_DIR:-$SHARPA_SDK_ROOT/lib}"
SHARPA_ARM_URL="${SHARPA_ARM_URL:-https://github.com/sharpa-robotics/sharpa-wave-sdk/releases/download/v5.0.3/SharpaWaveSDK_5.0.1_arm.zip}"

LEFT_HAND_IP="${LEFT_HAND_IP:-192.168.124.10}"
RIGHT_HAND_IP="${RIGHT_HAND_IP:-192.168.124.20}"

CLEANUP_DIR=""
cleanup() {
    [[ -z "${CLEANUP_DIR:-}" ]] || rm -rf -- "$CLEANUP_DIR"
}
trap cleanup EXIT

RED=$'\033[0;31m'
YELLOW=$'\033[0;33m'
GREEN=$'\033[0;32m'
BOLD=$'\033[1m'
RESET=$'\033[0m'

info() { printf '%s\n' "${GREEN}[INFO]${RESET} $*"; }
warn() { printf '%s\n' "${YELLOW}[WARN]${RESET} $*" >&2; }
die()  { printf '%s\n' "${RED}[ERROR]${RESET} $*" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }
need_dir() { [[ -d "$1" ]] || die "Directory does not exist: $1"; }
need_file() { [[ -f "$1" ]] || die "File does not exist: $1"; }

usage() {
    cat <<'EOF'
Usage (must be run on Thor):
  bash setup_h2_thor.sh <command>

Information and configuration
  workflow          Show the Thor first-time installation and cold-start sequence
  config            Show current directory, network, and SDK configuration
  doctor            Read-only checks of architecture, GMSL cameras, DDS, SDK, hand network, and bridge

Optional: migrate proprietary vendor components from an existing Thor
  bundle-export <bundle.tar.gz>
                    Export components that cannot be downloaded publicly from a working Thor
  bundle-import <bundle.tar.gz>
                    Import a migration bundle on a new Thor

First-time installation (run each step manually)
  install           Install build tools and prepare the official unitree_sdk2
  camera-install    Initialize S56x/SHF3L and install the native DDS image environment
  sharpa-install    Install the official Sharpa Wave SDK 5.0.1 ARM64 release
  bridge-build      Build and install the Sharpa DDS bridge

Every cold start (use separate terminals)
  camera-start      Start four JPEG/CycloneDDS image publishers in the foreground
  camera-check      Check devices and the publisher process locally on Thor
  bridge-check      Check bridge shared libraries, process, and both hand networks
  bridge-start      Start the Sharpa DDS bridge in the foreground; Ctrl+C stops it

Common path overrides:
  CAMERA_ROOT=~/h2_camera CAMERA_DDS_DOMAIN=10 CAMERA_NETWORK_INTERFACE=eth0
  XR_ROOT=~/xr_teleoperate
  UNITREE_SDK2_ROOT=~/unitree_sdk2
  SHARPA_SDK_ROOT=/opt/sharpa-wave-sdk SHARPA_BRIDGE_DIR=~/Sharpa/bridge_dds

Notes:
  - Camera scripts are deployed by thor-sync from the workstation; vendor drivers/device trees must already exist on Thor.
  - Images use isolated CycloneDDS domain 10 and do not depend on ROS 2 or Unitree SDK2.
  - The Sharpa ARM64 SDK uses the 5.0.1 ARM zip from the official v5.0.3 release.
  - This script does not control the H2 body, stand the robot, or run a policy.
  - No all/run-everything command is provided; run each next step manually only after its checks pass.
EOF
}

workflow() {
    cat <<'EOF'
New H2 Thor setup workflow (the operator runs each step separately)

1. Deploy the camera and installation scripts from the workstation
  THOR_IP=<new-Thor-IP> bash setup_h2.sh thor-sync

2. Log in to the new Thor and install each component
  ssh unitree@<new-Thor-IP>
  bash ~/setup_h2_thor.sh config

  # install only adds Linux build/diagnostic tools; it does not overwrite Unitree's factory control software.
  bash ~/setup_h2_thor.sh install
  bash ~/setup_h2_thor.sh camera-install
  bash ~/setup_h2_thor.sh sharpa-install
  bash ~/setup_h2_thor.sh bridge-build
  bash ~/setup_h2_thor.sh doctor

  # Use bundle-export/import to migrate proprietary vendor components from an old Thor of the
  # same model only if camera-install reports both the vendor driver package and /dev/video0-3 missing.

3. After every robot cold start
  1. Manually power on the H2 by briefly pressing and then holding the power button.
  2. Thor terminal A:
       bash ~/setup_h2_thor.sh camera-start
  3. Thor terminal B:
       bash ~/setup_h2_thor.sh bridge-start
  4. Check on Thor or locally:
       bash ~/setup_h2_thor.sh camera-check
       bash ~/setup_h2_thor.sh bridge-check
  5. Manually use the wireless remote:
       L2+B (damping) -> L2+Up (prepare) -> R2+Y (stand)
     Wait for the posture to stabilize after each step. This script does not perform these button presses.
  6. Return to the workstation and continue the setup_h2.sh stages for teleoperation, collection, or deployment.

Stopping:
  - camera-start and bridge-start run in the foreground; press Ctrl+C to stop them.
  - After stopping the bridge, verify that both hands no longer receive DDS commands.
EOF
}

print_config() {
    cat <<EOF
Host             : $(hostname)
Architecture     : $(uname -m)
Robot/hand DDS   : $DDS_DOMAIN
Camera DDS       : $CAMERA_DDS_DOMAIN (isolated, native CycloneDDS)
Camera interface : ${CAMERA_NETWORK_INTERFACE:-<auto-select>}
Camera directory : $CAMERA_ROOT
GMSL initialization: $CAMERA_SETUP
DDS publisher    : $CAMERA_PUBLISHER
Camera Python    : $CAMERA_VENV/bin/python
H2 XR source     : $XR_ROOT
Unitree SDK2     : $UNITREE_SDK2_ROOT
Sharpa bridge    : $SHARPA_BRIDGE_DIR/sharpa_dds_bridge
Sharpa headers   : $SHARPA_INCLUDE_DIR
Sharpa shared libraries: $SHARPA_LIB_DIR
Left/right hand IP: $LEFT_HAND_IP / $RIGHT_HAND_IP
EOF
}

bundle_export() {
    [[ $# -eq 1 ]] || die "Usage: bundle-export <output-bundle.tar.gz>"
    local output="$1"
    [[ "$output" == *.tar.gz || "$output" == *.tgz ]] ||
        die "Migration bundle filename must end in .tar.gz or .tgz"
    if [[ -e "$output" && "${H2_BUNDLE_OVERWRITE:-}" != "YES" ]]; then
        die "$output already exists; set H2_BUNDLE_OVERWRITE=YES to confirm overwrite"
    fi

    need_cmd tar
    need_cmd sha256sum
    need_dir "$UNITREE_SDK2_ROOT"
    need_dir "$SHARPA_INCLUDE_DIR"
    need_dir "$SHARPA_LIB_DIR"
    need_file "$SHARPA_LIB_DIR/libsharpa-wave-sdk.so"
    need_file "$SHARPA_BRIDGE_DIR/sharpa_dds_bridge"

    CLEANUP_DIR="$(mktemp -d /tmp/h2-thor-bundle.XXXXXX)"
    local stage="$CLEANUP_DIR/payload"
    mkdir -p "$stage/sharpa" "$stage/xr_bridge_source"

    if [[ -d "$CAMERA_ROOT" ]]; then
        info "Copying camera scripts: $CAMERA_ROOT"
        cp -a "$CAMERA_ROOT" "$stage/camera_project"
    else
        warn "$CAMERA_ROOT not found; the migration bundle will not contain camera scripts. Run thor-sync from the workstation."
    fi
    info "Copying Unitree SDK2: $UNITREE_SDK2_ROOT"
    cp -a "$UNITREE_SDK2_ROOT" "$stage/unitree_sdk2"
    info "Copying proprietary Sharpa headers and shared libraries"
    cp -a "$SHARPA_INCLUDE_DIR" "$stage/sharpa/include"
    cp -a "$SHARPA_LIB_DIR" "$stage/sharpa/lib"
    cp -a "$SHARPA_BRIDGE_DIR/sharpa_dds_bridge" "$stage/sharpa_dds_bridge"

    if [[ -f "$XR_ROOT/scripts/Makefile.sharpa_bridge" &&
          -f "$XR_ROOT/scripts/sharpa_dds_bridge.cpp" ]]; then
        cp -a "$XR_ROOT/scripts/Makefile.sharpa_bridge" \
            "$XR_ROOT/scripts/sharpa_dds_bridge.cpp" \
            "$stage/xr_bridge_source/"
    else
        warn "The old Thor has no bridge source; the bundle still contains the executable, but it cannot be rebuilt on the new Thor"
    fi

    cat >"$stage/manifest.txt" <<EOF
format=h2-thor-bundle-v1
created_at=$(date --iso-8601=seconds)
source_host=$(hostname)
arch=$(uname -m)
kernel=$(uname -r)
camera_source=$CAMERA_ROOT
unitree_sdk2_source=$UNITREE_SDK2_ROOT
sharpa_include_source=$SHARPA_INCLUDE_DIR
sharpa_lib_source=$SHARPA_LIB_DIR
bridge_source=$SHARPA_BRIDGE_DIR/sharpa_dds_bridge
EOF

    mkdir -p "$(dirname "$output")"
    tar -czf "$output" -C "$stage" .
    (
        cd "$(dirname "$output")"
        sha256sum "$(basename "$output")" >"$(basename "$output").sha256"
    )
    CLEANUP_DIR=""
    rm -rf -- "$(dirname "$stage")"
    info "Migration bundle created: $output"
    info "Checksum file created: $output.sha256"
    warn "The migration contains the proprietary Sharpa SDK. Store and transfer it within the license terms; do not upload it to a public repository."
}

bundle_import() {
    [[ $# -eq 1 ]] || die "Usage: bundle-import <migration-bundle.tar.gz>"
    local bundle="$1"
    need_file "$bundle"
    need_cmd tar

    if [[ -f "$bundle.sha256" ]]; then
        info "Verifying migration bundle SHA256"
        (cd "$(dirname "$bundle")" && sha256sum -c "$(basename "$bundle").sha256")
    else
        warn "$bundle.sha256 not found; transfer integrity cannot be verified"
    fi

    if tar -tzf "$bundle" | grep -Eq '(^/|(^|/)\.\.(/|$))'; then
        die "Migration bundle contains an unsafe path"
    fi

    CLEANUP_DIR="$(mktemp -d /tmp/h2-thor-import.XXXXXX)"
    tar -xzf "$bundle" -C "$CLEANUP_DIR"
    need_file "$CLEANUP_DIR/manifest.txt"
    grep -Fxq "format=h2-thor-bundle-v1" "$CLEANUP_DIR/manifest.txt" ||
        die "Unsupported migration bundle format"

    local source_arch
    source_arch="$(awk -F= '$1=="arch" {print $2}' "$CLEANUP_DIR/manifest.txt")"
    [[ "$source_arch" == "$(uname -m)" ]] ||
        die "Architecture mismatch: bundle=$source_arch, new Thor=$(uname -m)"

    cat "$CLEANUP_DIR/manifest.txt"
    cat >&2 <<EOF

${YELLOW}${BOLD}About to copy hardware software from the old Thor to the new Thor${RESET}
This will not modify Unitree's factory motion-control software, but it will install the camera
project, Unitree SDK2, Sharpa SDK files, and bridge. Existing files are preserved by default;
set H2_BUNDLE_OVERWRITE=YES to allow overwriting.
EOF
    read -r -p "Enter IMPORT to continue: " answer
    [[ "$answer" == "IMPORT" ]] || die "Cancelled"

    local overwrite="${H2_BUNDLE_OVERWRITE:-NO}"
    local camera_target="$CAMERA_ROOT"

    if [[ ! -d "$CLEANUP_DIR/camera_project" ]]; then
        warn "The migration bundle contains no camera scripts; run setup_h2.sh thor-sync from the workstation"
    elif [[ -e "$camera_target" && "$overwrite" != "YES" ]]; then
        warn "Preserving existing camera project: $camera_target"
    else
        if [[ -e "$camera_target" ]]; then
            [[ "$camera_target" == "$HOME/"* ]] ||
                die "Refusing to overwrite a camera directory outside HOME: $camera_target"
            rm -rf -- "$camera_target"
        fi
        mkdir -p "$(dirname "$camera_target")"
        cp -a "$CLEANUP_DIR/camera_project" "$camera_target"
        info "Camera project imported: $camera_target"
    fi

    if [[ -e "$UNITREE_SDK2_ROOT" && "$overwrite" != "YES" ]]; then
        warn "Preserving existing Unitree SDK2: $UNITREE_SDK2_ROOT"
    else
        if [[ -e "$UNITREE_SDK2_ROOT" ]]; then
            [[ "$UNITREE_SDK2_ROOT" == "$HOME/"* ]] ||
                die "Refusing to overwrite an SDK2 directory outside HOME: $UNITREE_SDK2_ROOT"
            rm -rf -- "$UNITREE_SDK2_ROOT"
        fi
        mkdir -p "$(dirname "$UNITREE_SDK2_ROOT")"
        cp -a "$CLEANUP_DIR/unitree_sdk2" "$UNITREE_SDK2_ROOT"
        info "Unitree SDK2 imported: $UNITREE_SDK2_ROOT"
    fi

    sudo install -d "$SHARPA_INCLUDE_DIR" "$SHARPA_LIB_DIR"
    if [[ "$overwrite" == "YES" ]]; then
        sudo cp -a "$CLEANUP_DIR/sharpa/include/." "$SHARPA_INCLUDE_DIR/"
        sudo cp -a "$CLEANUP_DIR/sharpa/lib/." "$SHARPA_LIB_DIR/"
    else
        sudo cp -an "$CLEANUP_DIR/sharpa/include/." "$SHARPA_INCLUDE_DIR/"
        sudo cp -an "$CLEANUP_DIR/sharpa/lib/." "$SHARPA_LIB_DIR/"
    fi
    sudo ldconfig
    info "Sharpa SDK files imported; existing-file overwrite policy: $overwrite"

    mkdir -p "$SHARPA_BRIDGE_DIR"
    if [[ ! -e "$SHARPA_BRIDGE_DIR/sharpa_dds_bridge" || "$overwrite" == "YES" ]]; then
        install -m 0755 "$CLEANUP_DIR/sharpa_dds_bridge" \
            "$SHARPA_BRIDGE_DIR/sharpa_dds_bridge"
    else
        warn "Preserving existing bridge: $SHARPA_BRIDGE_DIR/sharpa_dds_bridge"
    fi

    if [[ -f "$CLEANUP_DIR/xr_bridge_source/Makefile.sharpa_bridge" ]]; then
        mkdir -p "$XR_ROOT/scripts"
        if [[ "$overwrite" == "YES" ]]; then
            cp -a "$CLEANUP_DIR/xr_bridge_source/." "$XR_ROOT/scripts/"
        else
            cp -an "$CLEANUP_DIR/xr_bridge_source/." "$XR_ROOT/scripts/"
        fi
        info "Bridge build source imported: $XR_ROOT/scripts"
    fi

    local import_tmp="$CLEANUP_DIR"
    CLEANUP_DIR=""
    rm -rf -- "$import_tmp"
    info "Migration import complete. Next, run install, camera-install, bridge-build, and doctor in order."
}

install_base() {
    need_cmd sudo
    need_cmd git
    info "Installing Thor build and diagnostic tools (hardware services will not be started)"
    sudo apt-get update
    # Do not install python3-opencv with apt: it conflicts with nvidia-opencv-dev/JetPack and triggers downgrades/removals.
    # Keep the OpenCV bundled with JetPack; the camera venv uses --system-site-packages.
    sudo apt-get install -y \
        build-essential cmake pkg-config git curl unzip file iputils-ping \
        gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        v4l-utils python3-venv python3-pip python3-gi gir1.2-gstreamer-1.0 \
        cyclonedds-dev

    if ! python3 -c "import cv2" >/dev/null 2>&1; then
        warn "cv2 was not found in system Python; verify the JetPack OpenCV installation or install a compatible package manually"
    else
        info "Detected system OpenCV: $(python3 -c 'import cv2; print(cv2.__version__)')"
    fi

    if [[ ! -d "$UNITREE_SDK2_ROOT" ]]; then
        info "Cloning the official unitree_sdk2 C++ SDK"
        git clone https://github.com/unitreerobotics/unitree_sdk2.git "$UNITREE_SDK2_ROOT"
    else
        info "Preserving existing Unitree SDK2: $UNITREE_SDK2_ROOT"
    fi

    cat <<EOF

Base tools are ready. Next, run each of the following:
  bash $0 camera-install
  bash $0 sharpa-install
  bash $0 bridge-build

Note: sharpa-install downloads the ARM SDK only from the official Sharpa release URL pinned in this script.
EOF
}

sharpa_install() {
    [[ "$(uname -m)" == "aarch64" ]] ||
        die "The official Sharpa ARM package can only be installed on an aarch64 Thor; current architecture: $(uname -m)"
    need_cmd curl
    need_cmd unzip
    need_cmd file
    need_cmd sudo

    CLEANUP_DIR="$(mktemp -d)"
    local archive="$CLEANUP_DIR/SharpaWaveSDK_5.0.1_arm.zip"
    local unpack="$CLEANUP_DIR/unpack"
    mkdir -p "$unpack"

    info "Downloading SDK 5.0.1 ARM64 from the official Sharpa release"
    curl --fail --location --show-error "$SHARPA_ARM_URL" --output "$archive"
    unzip -q "$archive" -d "$unpack"

    local source_root=""
    local candidate
    shopt -s nullglob
    for candidate in "$unpack" "$unpack"/*; do
        if [[ -f "$candidate/include/SharpaWaveSDK.h" &&
              -f "$candidate/lib/libsharpa-wave-sdk.so" ]]; then
            source_root="$candidate"
            break
        fi
    done
    shopt -u nullglob
    [[ -n "$source_root" ]] ||
        die "The official archive is missing include/SharpaWaveSDK.h or lib/libsharpa-wave-sdk.so"
    file "$source_root/lib/libsharpa-wave-sdk.so" | grep -Eq 'ARM aarch64|ARM64' ||
        die "The shared library in the archive is not aarch64: $(file "$source_root/lib/libsharpa-wave-sdk.so")"
    grep -Eq 'SHARPA_SDK_VERSION[[:space:]]+\"5\.0\.[01]\"' \
        "$source_root/include/SharpaWaveSDK.h" ||
        die "The Sharpa header is not from the SDK 5.0.x required by the bridge"

    info "Installing to $SHARPA_SDK_ROOT"
    sudo mkdir -p "$SHARPA_SDK_ROOT"
    sudo cp -a "$source_root/." "$SHARPA_SDK_ROOT/"
    printf '%s\n' "$SHARPA_LIB_DIR" |
        sudo tee /etc/ld.so.conf.d/sharpa-wave-sdk.conf >/dev/null
    sudo ldconfig

    need_file "$SHARPA_INCLUDE_DIR/SharpaWaveSDK.h"
    need_file "$SHARPA_LIB_DIR/libsharpa-wave-sdk.so"
    info "Sharpa ARM SDK installed: $(grep SHARPA_SDK_VERSION "$SHARPA_INCLUDE_DIR/SharpaWaveSDK.h")"
}

camera_install() {
    need_file "$CAMERA_SETUP"
    need_file "$CAMERA_PUBLISHER"
    need_cmd python3
    local vendor_modules="${UNITREE_MODULES_DIR:-$HOME/.Unitree/YUSHU_4A_AGTH_G2Y_7.1}"
    local setup_extra=()
    if [[ -f "$vendor_modules/ko/kfifo_buf.ko" ]]; then
        info "Found vendor driver package: $vendor_modules"
    elif [[ -c /dev/video0 && -c /dev/video1 && -c /dev/video2 && -c /dev/video3 ]]; then
        warn "$vendor_modules/ko/kfifo_buf.ko not found"
        warn "However, /dev/video0-3 exist; using --controls-only (set controls without reloading kernel modules)"
        setup_extra=(--controls-only)
    else
        die "Vendor driver package $vendor_modules and /dev/video0-3 are missing. Copy ~/.Unitree/YUSHU_4A_AGTH_G2Y_7.1 from the old Thor."
    fi

    cat <<EOF
About to initialize the mixed GMSL cameras:
  head left/right  = Astra S56x via Libargus/nvarguscamerasrc
  wrist left/right = SHF3L via V4L2 /dev/video2 and /dev/video3
  setup            = $CAMERA_SETUP ${setup_extra[*]}

This script sets camera controls and does not modify robot motion control.
A full module reload requires ~/.Unitree/YUSHU_4A_AGTH_G2Y_7.1; if absent, only --controls-only is used.
EOF
    read -r -p "Enter CAMERA to continue: " answer
    [[ "$answer" == "CAMERA" ]] || die "Cancelled"
    sudo -E bash "$CAMERA_SETUP" "${setup_extra[@]}" "$@"

    if [[ ! -x "$CAMERA_VENV/bin/python" ]]; then
        python3 -m venv --system-site-packages "$CAMERA_VENV"
    fi
    "$CAMERA_VENV/bin/python" -m pip install -U pip
    local cyclone_version
    cyclone_version="$(dpkg-query -W -f='${Version}' cyclonedds-dev 2>/dev/null | cut -d- -f1)"
    [[ -n "$cyclone_version" ]] || die "Could not read the cyclonedds-dev version"
    local multiarch cyclone_prefix system_lib
    multiarch="$(gcc -dumpmachine)"
    system_lib="/usr/lib/$multiarch"
    cyclone_prefix="$HOME/.local/cyclonedds-system"
    need_file "$system_lib/libddsc.so"
    need_file "$system_lib/libcycloneddsidl.so"
    need_file "/usr/include/dds/dds.h"
    need_cmd idlc
    mkdir -p "$cyclone_prefix/bin" "$cyclone_prefix/include" "$cyclone_prefix/lib"
    ln -sfn "$(command -v idlc)" "$cyclone_prefix/bin/idlc"
    ln -sfn /usr/include/dds "$cyclone_prefix/include/dds"
    ln -sfn "$system_lib/libddsc.so" "$cyclone_prefix/lib/libddsc.so"
    ln -sfn "$system_lib/libcycloneddsidl.so" \
        "$cyclone_prefix/lib/libcycloneddsidl.so"
    info "Installing Python CycloneDDS $cyclone_version to match the system libraries"
    env "CYCLONEDDS_HOME=$cyclone_prefix" \
        "$CAMERA_VENV/bin/python" -m pip install "cyclonedds==$cyclone_version"
    "$CAMERA_VENV/bin/python" -c \
        "import re, cv2, cyclonedds, gi; gi.require_version('Gst', '1.0'); from gi.repository import Gst; print('camera DDS environment OK', cv2.__version__); info=cv2.getBuildInformation(); print('GStreamer:', 'YES' if re.search(r'GStreamer:\\s+YES', info) else 'check manually')"
    info "Camera and native DDS environment are ready."
}

camera_start() {
    need_file "$CAMERA_SETUP"
    need_file "$CAMERA_PUBLISHER"
    need_file "$CAMERA_VENV/bin/python"
    if pgrep -f '[d]ds_camera_publisher.py' >/dev/null 2>&1; then
        die "dds_camera_publisher.py is already running; do not start another instance"
    fi
    local vendor_modules="${UNITREE_MODULES_DIR:-$HOME/.Unitree/YUSHU_4A_AGTH_G2Y_7.1}"
    local setup_extra=()
    if [[ ! -f "$vendor_modules/ko/kfifo_buf.ko" ]]; then
        if [[ -c /dev/video0 && -c /dev/video1 && -c /dev/video2 && -c /dev/video3 ]]; then
            warn "Vendor driver package is missing; camera-start is using --controls-only"
            setup_extra=(--controls-only)
        else
            die "Vendor driver package is missing and no camera devices are present"
        fi
    fi
    info "Preparing GMSL cameras and starting four JPEG DDS publishers; robot joints are not controlled."
    sudo -E bash "$CAMERA_SETUP" "${setup_extra[@]}"
    # nvargus-daemon may remain "active" while its IPC provider is stale after
    # prior Argus pipeline failures. camera-start owns the Argus cameras, so
    # restart it before opening the two head-camera streams.
    info "Restarting nvargus-daemon to clear stale Argus provider state"
    sudo systemctl restart nvargus-daemon
    local attempt
    for ((attempt = 0; attempt < 30; attempt++)); do
        systemctl is-active --quiet nvargus-daemon && break
        sleep 0.1
    done
    systemctl is-active --quiet nvargus-daemon ||
        die "Failed to restart nvargus-daemon"
    sleep 3
    local net_args=()
    [[ -n "$CAMERA_NETWORK_INTERFACE" ]] &&
        net_args=(--network-interface "$CAMERA_NETWORK_INTERFACE")
    exec "$CAMERA_VENV/bin/python" "$CAMERA_PUBLISHER" \
        --domain "$CAMERA_DDS_DOMAIN" "${net_args[@]}" "$@"
}

camera_check() {
    local missing=0
    local device
    for device in /dev/video0 /dev/video1 /dev/video2 /dev/video3; do
        if [[ -c "$device" ]]; then
            info "Found $device"
        else
            warn "Missing $device"
            missing=$((missing + 1))
        fi
    done
    (( missing == 0 )) || die "Missing $missing camera device(s)"
    if pgrep -f '[d]ds_camera_publisher.py' >/dev/null 2>&1; then
        info "DDS camera publisher is running (domain $CAMERA_DDS_DOMAIN)"
    else
        die "DDS camera publisher is not running; run camera-start in a separate terminal"
    fi
    warn "Thor checks only local devices/processes; run setup_h2.sh camera-check on the workstation to check actual DDS frame rates for all four streams"
}

bridge_build() {
    [[ "$(uname -m)" == "aarch64" ]] ||
        warn "Current architecture is not aarch64; the Makefile is intended for Thor"
    need_cmd make
    need_cmd g++
    need_dir "$UNITREE_SDK2_ROOT"
    need_file "$SHARPA_INCLUDE_DIR/SharpaWaveSDK.h"

    local sharpa_so="$SHARPA_LIB_DIR/libsharpa-wave-sdk.so"
    need_file "$sharpa_so"

    local source_dir="$XR_ROOT/scripts"
    need_file "$source_dir/Makefile.sharpa_bridge"
    need_file "$source_dir/sharpa_dds_bridge.cpp"

    info "Building the Sharpa DDS bridge"
    (cd "$source_dir" && make -f Makefile.sharpa_bridge \
        "UNITREE_SDK2_DIR=$UNITREE_SDK2_ROOT" \
        "SHARPA_INCLUDE_DIR=$SHARPA_INCLUDE_DIR" \
        "SHARPA_LIB_DIR=$SHARPA_LIB_DIR")

    mkdir -p "$SHARPA_BRIDGE_DIR"
    install -m 0755 "$source_dir/sharpa_dds_bridge" \
        "$SHARPA_BRIDGE_DIR/sharpa_dds_bridge"
    info "Installed: $SHARPA_BRIDGE_DIR/sharpa_dds_bridge"
}

bridge_check() {
    local bridge="$SHARPA_BRIDGE_DIR/sharpa_dds_bridge"
    need_file "$bridge"
    [[ -x "$bridge" ]] || die "Bridge is not executable: $bridge"

    info "Checking shared libraries"
    if ldd "$bridge" | grep -q 'not found'; then
        ldd "$bridge" >&2
        die "Bridge has missing shared-library dependencies"
    fi

    if pgrep -x sharpa_dds_bridge >/dev/null 2>&1; then
        info "sharpa_dds_bridge is running"
    else
        warn "sharpa_dds_bridge is not currently running"
    fi

    local ip
    for ip in "$LEFT_HAND_IP" "$RIGHT_HAND_IP"; do
        if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
            info "Sharpa hand reachable: $ip"
        else
            warn "Sharpa hand unreachable: $ip"
        fi
    done
}

bridge_start() {
    local bridge="$SHARPA_BRIDGE_DIR/sharpa_dds_bridge"
    need_file "$bridge"
    [[ -x "$bridge" ]] || die "Bridge is not executable: $bridge"
    if pgrep -x sharpa_dds_bridge >/dev/null 2>&1; then
        die "sharpa_dds_bridge is already running; do not start another instance"
    fi

    cat >&2 <<EOF
${YELLOW}${BOLD}About to connect to and hold both Sharpa hands${RESET}
Confirm that no one is near the hands, nothing is being grasped, and no other Sharpa SDK control program is running on the workstation.
The bridge stays in the foreground; press Ctrl+C to stop it.
EOF
    read -r -p "Enter BRIDGE to continue: " answer
    [[ "$answer" == "BRIDGE" ]] || die "Cancelled"
    cd "$SHARPA_BRIDGE_DIR"
    exec ./sharpa_dds_bridge --side both "$@"
}

doctor() {
    local failures=0
    check_ok()   { printf '%s\n' "${GREEN}[ OK ]${RESET} $*"; }
    check_fail() { printf '%s\n' "${RED}[FAIL]${RESET} $*"; failures=$((failures + 1)); }
    check_warn() { printf '%s\n' "${YELLOW}[WARN]${RESET} $*"; }

    if [[ "$(uname -m)" == "aarch64" ]]; then
        check_ok "Thor/aarch64 architecture"
    else
        check_fail "Current architecture is $(uname -m), not the aarch64 architecture normally used by Thor"
    fi

    if [[ -f "$CAMERA_SETUP" && -f "$CAMERA_PUBLISHER" ]]; then
        check_ok "GMSL initialization and DDS publisher"
    else
        check_fail "Camera scripts are incomplete: $CAMERA_ROOT (run thor-sync on the workstation first)"
    fi
    if [[ -x "$CAMERA_VENV/bin/python" ]] &&
       "$CAMERA_VENV/bin/python" -c "import cv2, cyclonedds, gi; gi.require_version('Gst', '1.0')" >/dev/null 2>&1; then
        check_ok "Camera Python/CycloneDDS environment"
    else
        check_fail "Camera environment is not installed (run camera-install)"
    fi
    if [[ -d "$UNITREE_SDK2_ROOT/include" ]]; then
        check_ok "Unitree SDK2"
    else
        check_fail "Unitree SDK2 is incomplete: $UNITREE_SDK2_ROOT"
    fi
    if [[ -f "$SHARPA_INCLUDE_DIR/SharpaWaveSDK.h" ]]; then
        check_ok "Sharpa C++ headers"
    else
        check_fail "Missing SharpaWaveSDK.h"
    fi
    if [[ -f "$SHARPA_LIB_DIR/libsharpa-wave-sdk.so" ]]; then
        check_ok "Sharpa C++ shared library"
    else
        check_fail "Missing libsharpa-wave-sdk.so"
    fi
    if [[ -x "$SHARPA_BRIDGE_DIR/sharpa_dds_bridge" ]]; then
        check_ok "Sharpa DDS bridge"
        if ldd "$SHARPA_BRIDGE_DIR/sharpa_dds_bridge" | grep -q 'not found'; then
            check_fail "Sharpa bridge has missing shared-library dependencies"
        fi
    else
        check_fail "Bridge has not been built and installed"
    fi
    local ip
    for ip in "$LEFT_HAND_IP" "$RIGHT_HAND_IP"; do
        if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
            check_ok "Sharpa network $ip"
        else
            check_warn "Sharpa hand unreachable: $ip (expected when powered off)"
        fi
    done

    local device
    for device in /dev/video0 /dev/video1 /dev/video2 /dev/video3; do
        if [[ -c "$device" ]]; then
            check_ok "Camera device $device"
        else
            check_warn "$device not found (driver not initialized or camera disconnected)"
        fi
    done

    (( failures == 0 )) || die "doctor found $failures required check(s) that failed"
    info "Required Thor installation checks passed; recheck WARN items after powering on the hardware."
}

main() {
    local command="${1:-help}"
    [[ $# -eq 0 ]] || shift
    case "$command" in
        help|-h|--help) usage ;;
        workflow) workflow ;;
        config) print_config ;;
        bundle-export) bundle_export "$@" ;;
        bundle-import) bundle_import "$@" ;;
        install) install_base "$@" ;;
        camera-install) camera_install "$@" ;;
        camera-start) camera_start "$@" ;;
        camera-check) camera_check "$@" ;;
        sharpa-install) sharpa_install "$@" ;;
        bridge-build) bridge_build "$@" ;;
        bridge-check) bridge_check "$@" ;;
        bridge-start) bridge_start "$@" ;;
        doctor) doctor "$@" ;;
        *) usage >&2; die "Unknown command: $command" ;;
    esac
}

main "$@"
