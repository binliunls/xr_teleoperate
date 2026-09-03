#!/usr/bin/env bash
#
# Staged H2 + Sharpa workstation operations entry point.
#
# Default topology (each value can be overridden by its corresponding environment variable):
#   Workstation <192.168.123.x> -- DDS/SSH --> H2 Thor 192.168.123.163
#   Thor    <192.168.124.163> ---------> Sharpa L/R 192.168.124.10/.20
#   Cameras: native CycloneDDS JPEG on Thor, isolated on domain 10 (no ROS/Unitree SDK2 dependency)
#   Hands: sharpa_dds_bridge on Thor, DDS domain 0
# Thor-side installation, cameras, and hand bridge are managed separately by setup_h2_thor.sh.
#
# Recommended first-time sequence (run `bash setup_h2.sh help` for every command):
#   bash setup_h2.sh workflow # Show the complete staged procedure
#   bash setup_h2.sh install  # Build the local tv env, install H2 Python packages, and build MANUS
#   bash setup_h2.sh doctor   # Validate the completed installation

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "$SCRIPT_DIR/teleop" && -d "$SCRIPT_DIR/scripts" ]]; then
    WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(dirname "$SCRIPT_DIR")}"
    DEFAULT_XR_ROOT="$SCRIPT_DIR"
else
    WORKSPACE_ROOT="${WORKSPACE_ROOT:-$SCRIPT_DIR}"
    DEFAULT_XR_ROOT="$WORKSPACE_ROOT/xr_teleoperate"
fi

# ----------------------------- User configuration ----------------------------
# Each repository has a <NAME>_ROOT checkout path, a <NAME>_REPO URL, and a
# <NAME>_VERSION ref. Third-party repositories are pinned to the exact commit
# this setup was validated against; our own forks track a branch so that
# ongoing work can be pulled without editing this script. The two forks are
# private, so they use SSH and need a GitHub key on the workstation.
XR_ROOT="${XR_ROOT:-$DEFAULT_XR_ROOT}"
XR_REPO="${XR_REPO:-git@github.com:mingxueg-nv/xr_teleoperate.git}"
XR_VERSION="${XR_VERSION:-mingxueg/change_camera_streams_on_dds}"

UNITREE_LEROBOT_ROOT="${UNITREE_LEROBOT_ROOT:-$WORKSPACE_ROOT/unitree_lerobot}"
UNITREE_LEROBOT_REPO="${UNITREE_LEROBOT_REPO:-git@github.com:mingxueg-nv/unitree_lerobot.git}"
UNITREE_LEROBOT_VERSION="${UNITREE_LEROBOT_VERSION:-mingxueg/dds_deploy}"

SDK2_ROOT="${SDK2_ROOT:-$WORKSPACE_ROOT/unitree_sdk2_python}"
SDK2_REPO="${SDK2_REPO:-https://github.com/unitreerobotics/unitree_sdk2_python.git}"
SDK2_VERSION="${SDK2_VERSION:-e4cd91f051aaa77a70600e3d2bf7f50889db1980}"

GROOT_ROOT="${GROOT_ROOT:-$WORKSPACE_ROOT/Isaac-GR00T}"
GROOT_REPO="${GROOT_REPO:-https://github.com/NVIDIA/Isaac-GR00T.git}"
# The commit this workstation trains and serves with. It is a few commits past
# the n1.7-release tag, which is what the models here were built against.
GROOT_VERSION="${GROOT_VERSION:-ab88b50c718f6528e1df9dcbaf75865d1b604760}"
THOR_SETUP_SCRIPT="${THOR_SETUP_SCRIPT:-$XR_ROOT/setup_h2_thor.sh}"

XR_ENV="${XR_ENV:-tv}"

THOR_USER="${THOR_USER:-unitree}"
THOR_IP="${THOR_IP:-192.168.123.163}"
DDS_DOMAIN="${DDS_DOMAIN:-0}"
CAMERA_DDS_DOMAIN="${CAMERA_DDS_DOMAIN:-10}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-}"

SHARPA_TELEOP_ROOT="${SHARPA_TELEOP_ROOT:-$WORKSPACE_ROOT/sharpa-teleop}"
SHARPA_TELEOP_REPO="${SHARPA_TELEOP_REPO:-https://github.com/isaac-for-healthcare/sharpa-teleop.git}"
SHARPA_TELEOP_VERSION="${SHARPA_TELEOP_VERSION:-ce5a2635f7d3ad84041d279868f009e9ceeba6c3}"
SHARPA_RETARGET_DIR="${SHARPA_RETARGET_DIR:-$SHARPA_TELEOP_ROOT/workstation/sharpa-manus-sdk-1.1.0/retargeting_alg_release_V5.0}"

MANUS_ROOT="${MANUS_ROOT:-$WORKSPACE_ROOT/Sharpa/sharpa-manus-sdk}"
MANUS_REPO="${MANUS_REPO:-https://github.com/sharpa-robotics/sharpa-manus-sdk.git}"
MANUS_REPO_VERSION="${MANUS_REPO_VERSION:-b87bff567c82905dd591d91f08be59fb90a604df}"
MANUS_CLIENT_DIR="${MANUS_CLIENT_DIR:-$MANUS_ROOT/client}"
# Official MANUS Core SDK archive; unrelated to the sharpa-manus-sdk git pin above.
MANUS_SDK_VERSION="${MANUS_SDK_VERSION:-3.1.1}"
MANUS_SDK_URL="${MANUS_SDK_URL:-https://static.manus-meta.com/resources/manus_core_3/sdk/MANUS_Core_${MANUS_SDK_VERSION}_SDK.zip}"

RED=$'\033[0;31m'
YELLOW=$'\033[0;33m'
GREEN=$'\033[0;32m'
BOLD=$'\033[1m'
RESET=$'\033[0m'

info() { printf '%s\n' "${GREEN}[INFO]${RESET} $*"; }
warn() { printf '%s\n' "${YELLOW}[WARN]${RESET} $*" >&2; }
die()  { printf '%s\n' "${RED}[ERROR]${RESET} $*" >&2; exit 1; }

usage() {
    cat <<'EOF'
Usage:
  bash setup_h2.sh <command> [arguments...]

Configuration and installation
  workflow                         Print manual and automated steps from setup through deployment
  config                           Show current robot, network, and directory configuration
  repos                            Clone the core repositories at their pinned versions (GR00T: install-groot)
  install                          Build tv; install XR/SDK2/LeRobot packages and MANUS SDK/client
  install-groot                    Install the Isaac-GR00T uv/CUDA Python environment
  manus-install                    Install the official MANUS Integrated SDK and build the acquisition client
  manus-start                      Start the MANUS client in the foreground and publish skeletons on localhost:2044
  manus-check                      Check the MANUS client, SDK shared libraries, and port 2044 listener
  thor-sync [local-xr-root]        Deploy Thor files from <xr-root>/scripts
  cert [workstation-IP]            Generate a self-signed XR HTTPS certificate
  doctor                           Check directories, environments, DDS, Thor, cameras, and GPU
  ik-test                          Test H2 IK in Meshcat only; do not send commands to the robot

Local hardware checks
  camera-check                     Check all four native DDS image streams and receive rates
  camera-visualize                 Display all four DDS camera streams; press q to exit
  hand-retarget                    Start Manus -> Sharpa DDS hand mapping

Collection, training, and deployment
  Review and run the explicit commands in deploy_dds.sh manually.

Common overrides:
  THOR_IP=192.168.123.163 DDS_DOMAIN=0 CAMERA_DDS_DOMAIN=10 NETWORK_INTERFACE=enp3s0
  Checkout paths : XR_ROOT, UNITREE_LEROBOT_ROOT, SDK2_ROOT, SHARPA_TELEOP_ROOT, MANUS_ROOT, GROOT_ROOT
  Clone sources  : XR_REPO, UNITREE_LEROBOT_REPO, SDK2_REPO, SHARPA_TELEOP_REPO, MANUS_REPO, GROOT_REPO
  Pinned versions: XR_VERSION, UNITREE_LEROBOT_VERSION, SDK2_VERSION, SHARPA_TELEOP_VERSION,
                   MANUS_REPO_VERSION, GROOT_VERSION, MANUS_SDK_VERSION

EOF
}

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }
need_dir() { [[ -d "$1" ]] || die "Directory does not exist: $1"; }
need_file() { [[ -f "$1" ]] || die "File does not exist: $1"; }

# ensure_repo <label> <checkout-dir> <url> <ref>
#
# Clones <url> at <ref> when the directory is missing, and sets REPO_WAS_CLONED
# so callers can run first-time-only fixups. An existing checkout is reported
# but never touched: these trees carry hand-applied source fixes, glove
# calibration bound to the physical hardware, and submodules that intentionally
# run ahead of the recorded gitlinks, so forcing them onto a pinned ref would
# break a working install.
REPO_WAS_CLONED=0
ensure_repo() {
    local label="$1" dir="$2" url="$3" ref="$4"
    REPO_WAS_CLONED=0
    need_cmd git

    if [[ ! -d "$dir" ]]; then
        info "Cloning $label into $dir at $ref"
        mkdir -p "$(dirname "$dir")"
        git clone "$url" "$dir" || die "Could not clone $label from $url"
        git -C "$dir" checkout --quiet "$ref" ||
            die "Cloned $label but could not check out '$ref'"
        REPO_WAS_CLONED=1
        info "$label is now at $(git -C "$dir" rev-parse --short HEAD)"
        return
    fi

    git -C "$dir" rev-parse --git-dir >/dev/null 2>&1 ||
        die "$dir exists but is not a git checkout; move it aside or point the matching *_ROOT elsewhere"

    local head branch dirty=""
    head="$(git -C "$dir" rev-parse --short HEAD)"
    branch="$(git -C "$dir" rev-parse --abbrev-ref HEAD)"
    [[ -n "$(git -C "$dir" status --porcelain)" ]] && dirty=" with local changes"
    info "$label already present at $branch@$head$dirty"

    # Compare against the pin only when the ref already resolves locally, so an
    # offline install does not need a fetch to get past this check.
    local want
    if want="$(git -C "$dir" rev-parse --verify --quiet "$ref^{commit}")"; then
        [[ "$want" == "$(git -C "$dir" rev-parse HEAD)" ]] ||
            warn "$label is not on the pinned ref $ref; keeping the current checkout"
    fi
}

# Populates xr_teleoperate submodules by path, and only when the directory is
# empty. A blanket `git submodule update` would check out the recorded gitlink,
# which on an established workstation silently rolls teleimager back to an
# older commit than the one actually installed into the environment.
ensure_xr_submodules() {
    git -C "$XR_ROOT" rev-parse --git-dir >/dev/null 2>&1 || {
        warn "$XR_ROOT is not a git checkout; skipping submodule initialization"
        return
    }
    local path
    while read -r path; do
        [[ -n "$path" ]] || continue
        if [[ -n "$(ls -A "$XR_ROOT/$path" 2>/dev/null)" ]]; then
            info "Submodule already populated: $path"
            continue
        fi
        info "Initializing submodule $path"
        git -C "$XR_ROOT" submodule update --init --recursive -- "$path" ||
            die "Could not initialize submodule $path"
    done < <(git -C "$XR_ROOT" config --file .gitmodules \
                 --get-regexp '^submodule\..*\.path$' 2>/dev/null | awk '{print $2}' || true)
}

# The MANUS client links against system ncurses/zmq and includes Eigen, none of
# which ship with the repository.
check_build_deps() {
    local missing=()
    [[ -f /usr/include/ncurses.h || -f /usr/include/curses.h ]] || missing+=(libncurses-dev)
    [[ -f /usr/include/zmq.h ]] || missing+=(libzmq3-dev)
    [[ -f /usr/include/eigen3/Eigen/Core ]] || missing+=(libeigen3-dev)
    if (( ${#missing[@]} > 0 )); then
        die "Missing development packages: ${missing[*]}
Install them first: sudo apt-get install -y ${missing[*]}"
    fi
}

ensure_all_repos() {
    if [[ "$XR_ROOT" == "$SCRIPT_DIR" ]]; then
        info "xr_teleoperate is the checkout this script runs from: $XR_ROOT"
    else
        ensure_repo xr_teleoperate "$XR_ROOT" "$XR_REPO" "$XR_VERSION"
    fi
    ensure_xr_submodules
    ensure_repo unitree_lerobot "$UNITREE_LEROBOT_ROOT" "$UNITREE_LEROBOT_REPO" "$UNITREE_LEROBOT_VERSION"
    ensure_repo unitree_sdk2_python "$SDK2_ROOT" "$SDK2_REPO" "$SDK2_VERSION"
    ensure_repo sharpa-teleop "$SHARPA_TELEOP_ROOT" "$SHARPA_TELEOP_REPO" "$SHARPA_TELEOP_VERSION"
    ensure_repo sharpa-manus-sdk "$MANUS_ROOT" "$MANUS_REPO" "$MANUS_REPO_VERSION"
    manus_apply_local_fixes "$REPO_WAS_CLONED"
    need_dir "$SHARPA_RETARGET_DIR"
}

# Two fixes that upstream sharpa-manus-sdk still needs for the H2 + Sharpa
# setup. Each is guarded by a content check, so re-running is a no-op and a
# checkout that already carries the fix by hand is left alone.
manus_apply_local_fixes() {
    local fresh_clone="${1:-0}"
    local client_cpp="$MANUS_CLIENT_DIR/SharpaManusClient.cpp"
    local makefile="$MANUS_CLIENT_DIR/Makefile"
    need_file "$client_cpp"
    need_file "$makefile"

    # Upstream rotates the four pinky nodes into the thumb slots, which shifts
    # every finger by one -- index motion ends up driving the Sharpa middle
    # finger and open/close is corrupted. The retargeting order is wrist,
    # thumb, index, middle, ring, pinky, i.e. the identity permutation.
    local rotated='reorderedIndices = {0, 21, 22, 23, 24, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};'
    local identity='reorderedIndices = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};'
    if grep -qF -- "$identity" "$client_cpp"; then
        info "Finger order is already correct in SharpaManusClient.cpp"
    elif grep -qF -- "$rotated" "$client_cpp"; then
        info "Applying the finger-order fix to SharpaManusClient.cpp"
        sed -i "s|$rotated|$identity|" "$client_cpp"
        grep -qF -- "$identity" "$client_cpp" || die "The finger-order fix did not apply"
    else
        warn "No known reorderedIndices line in $client_cpp; verify the finger order by hand"
    fi

    # Upstream bakes in a relative rpath, so the client only resolves
    # libManusSDK_Integrated.so when it is started from client/. Anchor the
    # lookup to the binary instead.
    local rpath_old='-Wl,-rpath=./ManusSDK/lib'
    local rpath_new="-Wl,-rpath='\$\$ORIGIN/ManusSDK/lib'"
    if grep -qF -- "$rpath_new" "$makefile"; then
        info 'Makefile rpath is already anchored to $ORIGIN'
    elif grep -qF -- "$rpath_old" "$makefile"; then
        info 'Anchoring the Makefile rpath to $ORIGIN'
        sed -i "s|$rpath_old|$rpath_new|" "$makefile"
        grep -qF -- "$rpath_new" "$makefile" || die "The rpath fix did not apply"
    else
        warn "No known rpath in $makefile; verify it by hand"
    fi

    if [[ "$fresh_clone" == "1" ]]; then
        warn "Fresh sharpa-manus-sdk clone: the Calibration_left.mcal/Calibration_right.mcal files are upstream defaults, not this workstation's gloves. Recalibrate both gloves in client/CalibrationGUI before collecting data."
    fi
}

conda_env_exists() {
    conda env list | awk '{print $1}' | grep -Fxq "$1"
}

xr_run() {
    conda run --no-capture-output -n "$XR_ENV" \
        env "PYTHONPATH=$XR_ROOT:$UNITREE_LEROBOT_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$@"
}

print_config() {
    cat <<EOF
Robot           : Unitree H2, dual 14-DoF arms
End effectors   : Two Sharpa hands, 22 DoF each (DDS bridge)
Observation/action: 58 = 14 arm + 22 left hand + 22 right hand
Cameras         : Stereo head + left/right wrists, JPEG/CycloneDDS, 30 Hz
Thor            : $THOR_USER@$THOR_IP
DDS domain      : $DDS_DOMAIN
Camera DDS      : $CAMERA_DDS_DOMAIN (isolated from robot/hands)
Network interface: ${NETWORK_INTERFACE:-<auto-select>}

Repositories (checkout @ pinned version)
  xr_teleoperate     : $XR_ROOT @ $XR_VERSION
  unitree_lerobot    : $UNITREE_LEROBOT_ROOT @ $UNITREE_LEROBOT_VERSION
  unitree_sdk2_python: $SDK2_ROOT @ $SDK2_VERSION
  sharpa-teleop      : $SHARPA_TELEOP_ROOT @ $SHARPA_TELEOP_VERSION
  sharpa-manus-sdk   : $MANUS_ROOT @ $MANUS_REPO_VERSION
  Isaac-GR00T        : $GROOT_ROOT @ $GROOT_VERSION
MANUS Core SDK  : $MANUS_SDK_VERSION
XR conda environment: $XR_ENV
EOF
}

workflow() {
    cat <<'EOF'
Complete H2 + Sharpa workflow

1. Hardware/network configuration (manual)
   - Connect the workstation and Thor to 192.168.123.x; Thor defaults to 192.168.123.163.
   - Connect both Sharpa hands to Thor's 192.168.124.x interface (left .10, right .20).
   - Install the stereo head and both wrist cameras; verify that left/right wiring matches the topic names.
   - Prepare a physical emergency stop, wireless remote, safety frame/operator, and a clear workspace.

2. Workstation software
   bash setup_h2.sh install
   bash setup_h2.sh install-groot
   bash setup_h2.sh cert <workstation-IP-on-robot-subnet>
   bash setup_h2.sh doctor
   bash setup_h2.sh ik-test

3. Thor software migration and installation on a new H2
   - Deploy the public scripts first: bash setup_h2.sh thor-sync
   - Camera drivers/device trees come from the Thor vendor system.
   - setup_h2_thor.sh sharpa-install installs the Sharpa ARM SDK from the official release.
     bundle-export/import is reserved for vendor components that cannot be downloaded publicly.
   - View the complete step-by-step commands for old/new Thor systems on the workstation:
     bash setup_h2_thor.sh workflow
   - On the new Thor, run install, camera-install, sharpa-install, bridge-build, and doctor in order.
     This does not overwrite Unitree's factory motion-control software or start the robot automatically.

4. After every robot cold start
   - Power on by briefly pressing and then holding the power button.
   - Thor terminal A: bash ~/setup_h2_thor.sh camera-start (camera DDS domain 10)
   - Thor terminal B: bash ~/setup_h2_thor.sh bridge-start (keep it running)
   - Local terminal: bash setup_h2.sh camera-check
   - On the remote, press in order: L2+B (damping) -> L2+Up (prepare) -> R2+Y (stand).
     Wait for the posture to stabilize after each step; use --motion for standing collection/deployment.

5. Teleoperation, collection, training, and deployment
   Terminal C: bash setup_h2.sh manus-start
   Terminal D: bash setup_h2.sh hand-retarget
   Terminal E: run the explicit teleop command from deploy_dds.sh
   - On the first PICO cold start, you may need to visit the local URL, exit and restart teleop, then visit the vuer URL.
   - The operator holds the reference pose and presses c to calibrate; press r/B to begin tracking.
   - Press s/Y to start an episode and again to save it; press q/A to exit safely.
   - Check every trajectory for success, collisions, black frames, and a consistent initial pose.
   - Run training and real-robot deployment from reviewed, task-specific command files.
     deploy_dds.sh is the current H2 + Sharpa + native DDS reference.

Note: This script only orchestrates the existing H2 implementation. It does not perform mechanical
installation, joint-zero calibration, camera intrinsic/extrinsic calibration, initial PICO/Manus
authorization, or standing operations. When changing hands, cameras, or joint ordering, update the
control/recording structures in xr_teleoperate, ROBOT_CONFIGS in
unitree_lerobot/utils/constants.py, and the observation/action slices in the deployment script.
Changing command-line arguments alone is insufficient.
EOF
}

thor_sync() {
    [[ $# -le 1 ]] || die "Usage: thor-sync [local-xr-root]"
    local local_xr_root="${1:-$XR_ROOT}"
    local source_dir="$local_xr_root/scripts"
    need_cmd ssh
    need_cmd scp
    need_file "$THOR_SETUP_SCRIPT"
    need_file "$source_dir/setup_gmsl_cameras.sh"
    need_file "$source_dir/dds_camera_publisher.py"
    need_file "$source_dir/Makefile.sharpa_bridge"
    need_file "$source_dir/sharpa_dds_bridge.cpp"
    local remote="$THOR_USER@$THOR_IP"
    info "Deploying Thor files from $source_dir to $remote"
    ssh "$remote" 'mkdir -p "$HOME/h2_camera" "$HOME/xr_teleoperate/scripts"'
    scp "$THOR_SETUP_SCRIPT" "$remote:~/setup_h2_thor.sh"
    scp \
        "$source_dir/setup_gmsl_cameras.sh" \
        "$source_dir/dds_camera_publisher.py" \
        "$remote:~/h2_camera/"
    scp \
        "$source_dir/Makefile.sharpa_bridge" \
        "$source_dir/sharpa_dds_bridge.cpp" \
        "$remote:~/xr_teleoperate/scripts/"
    ssh "$remote" \
        'chmod +x "$HOME/setup_h2_thor.sh" "$HOME/h2_camera/setup_gmsl_cameras.sh" "$HOME/h2_camera/dds_camera_publisher.py"'
    info "Deployment complete. Next, log in to Thor and run: bash ~/setup_h2_thor.sh workflow"
}

install_all() {
    need_cmd conda
    need_cmd git
    check_build_deps
    ensure_all_repos

    if ! conda_env_exists "$XR_ENV"; then
        info "Creating XR environment $XR_ENV"
        conda create -y -n "$XR_ENV" python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
    fi
    conda install -y -n "$XR_ENV" ffmpeg=7.1.1 -c conda-forge

    info "Installing XR, H2 control, and DDS dependencies"
    xr_run python -m pip install -U pip
    xr_run python -m pip install -e "$XR_ROOT/teleop/teleimager" --no-deps
    xr_run python -m pip install -e "$XR_ROOT/teleop/televuer"
    xr_run python -m pip install -e "$XR_ROOT/teleop/robot_control/dex-retargeting"
    xr_run python -m pip install -r "$XR_ROOT/requirements.txt"
    xr_run python -m pip install -e "$SDK2_ROOT"
    xr_run python -m pip install -e "$UNITREE_LEROBOT_ROOT"
    xr_run python -m pip install cyclonedds==0.10.2 msgpack msgpack-numpy pyzmq pyyaml opencv-python

    manus_install
    info "Installation complete. Next: bash '$0' cert <workstation-IP> && bash '$0' doctor"
}

manus_install() {
    [[ "$(uname -m)" == "x86_64" ]] ||
        die "The MANUS Linux SDK package requires an x86_64 workstation; current architecture: $(uname -m)"
    need_cmd curl
    need_cmd unzip
    need_cmd make
    need_cmd g++
    check_build_deps
    ensure_repo sharpa-manus-sdk "$MANUS_ROOT" "$MANUS_REPO" "$MANUS_REPO_VERSION"
    manus_apply_local_fixes "$REPO_WAS_CLONED"
    ensure_repo sharpa-teleop "$SHARPA_TELEOP_ROOT" "$SHARPA_TELEOP_REPO" "$SHARPA_TELEOP_VERSION"
    need_file "$SHARPA_RETARGET_DIR/retargeting_manus_demo_dds.py"

    # The repository ships its own older ManusSDK copy, so a fresh clone looks
    # complete even though the client needs Core 3.1.x. CoreSdk_SetLogLocation
    # was added in 3.1.x, which distinguishes the two without a version macro.
    local sdk_dir="$MANUS_CLIENT_DIR/ManusSDK"
    if [[ -f "$sdk_dir/include/ManusSDK.h" &&
          -f "$sdk_dir/lib/libManusSDK_Integrated.so" ]] &&
       grep -q 'CoreSdk_SetLogLocation' "$sdk_dir/include/ManusSDK.h"; then
        info "Reusing the installed MANUS SDK: $sdk_dir"
    else
        local archive
        archive="$(mktemp --suffix=.zip)"
        info "Downloading MANUS Core SDK $MANUS_SDK_VERSION (approximately 208 MiB), replacing any bundled copy"
        curl --fail --location --show-error "$MANUS_SDK_URL" --output "$archive"
        mkdir -p "$sdk_dir/include" "$sdk_dir/lib"
        unzip -jo "$archive" \
            "ManusSDK_v${MANUS_SDK_VERSION}/SDKClient_Linux/ManusSDK/include/*.h" \
            -d "$sdk_dir/include"
        unzip -jo "$archive" \
            "ManusSDK_v${MANUS_SDK_VERSION}/SDKClient_Linux/ManusSDK/lib/libManusSDK_Integrated.so" \
            -d "$sdk_dir/lib"
        rm -f "$archive"
    fi

    info "Installing MANUS retargeting Python dependencies"
    xr_run python -m pip install \
        numpy==1.26.4 scipy==1.14.0 casadi==3.7.0 pyzmq==26.2.0 \
        protobuf==3.20.3 rich==14.0.0 matplotlib==3.7.5 mujoco==3.3.6

    info "Building the Sharpa MANUS acquisition client"
    make -C "$MANUS_CLIENT_DIR" clean
    make -C "$MANUS_CLIENT_DIR" -j"$(nproc)" all
    if ldd "$MANUS_CLIENT_DIR/SharpaManusClient.out" | awk '/not found/{found=1} END{exit !found}'; then
        ldd "$MANUS_CLIENT_DIR/SharpaManusClient.out" >&2
        die "The MANUS client has missing shared-library dependencies"
    fi
    xr_run python "$SHARPA_RETARGET_DIR/retargeting_manus_demo_dds.py" --help >/dev/null
    info "MANUS SDK/client installed; insert the Sensor Dongle and an SDK(integrated)-enabled License Key, then run manus-start"
}

# Only report processes whose executable really is the client binary; matching the
# command line alone also hits editors, shells, and wrappers that mention the path.
manus_client_running() {
    local target pid exe
    target="$(readlink -f "$MANUS_CLIENT_DIR/SharpaManusClient.out" 2>/dev/null)" || return 1
    [[ -n "$target" ]] || return 1
    for pid in $(pgrep -f 'SharpaManusClient\.out' 2>/dev/null || true); do
        exe="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
        [[ "$exe" == "$target" ]] && return 0
    done
    return 1
}

manus_start() {
    local client="$MANUS_CLIENT_DIR/SharpaManusClient.out"
    need_file "$client"
    [[ -x "$client" ]] || die "MANUS client is not executable; run manus-install first"
    if manus_client_running; then
        die "SharpaManusClient.out is already running; do not start another instance"
    fi
    cat >&2 <<EOF
${YELLOW}${BOLD}Starting MANUS Core Integrated${RESET}
Insert both the MetaglovePro Sensor Dongle and a License Key with the SDK(integrated) feature.
Start hand-retarget only after seeing Prime1 Dongle license data and continuous "glove ... is published" messages.
The process stays in the foreground; press Ctrl+C to stop it.
EOF
    cd "$MANUS_CLIENT_DIR"
    exec ./SharpaManusClient.out
}

manus_check() {
    local client="$MANUS_CLIENT_DIR/SharpaManusClient.out"
    need_file "$client"
    need_file "$MANUS_CLIENT_DIR/ManusSDK/lib/libManusSDK_Integrated.so"
    need_cmd ss
    if ldd "$client" | awk '/not found/{found=1} END{exit !found}'; then
        ldd "$client" >&2
        die "The MANUS client has missing shared-library dependencies"
    fi
    manus_client_running ||
        die "SharpaManusClient.out is not running; run manus-start in a separate terminal"
    ss -ltn | awk '$4 ~ /:2044$/ {found=1} END {exit !found}' ||
        die "The MANUS client is running but is not listening on TCP port 2044"
    info "The MANUS client is running and listening on localhost:2044"
}

install_groot() {
    need_cmd git
    # Cloned in full rather than --depth 1: the pin is a commit, which a shallow
    # clone of a single branch cannot check out.
    ensure_repo Isaac-GR00T "$GROOT_ROOT" "$GROOT_REPO" "$GROOT_VERSION"
    need_file "$GROOT_ROOT/pyproject.toml"
    if ! command -v uv >/dev/null 2>&1; then
        need_cmd conda
        info "Installing uv"
        conda install -y -n base -c conda-forge uv
    fi
    command -v nvidia-smi >/dev/null 2>&1 ||
        warn "nvidia-smi was not found; GR00T can be installed, but training/inference requires a compatible NVIDIA GPU"
    info "Installing GR00T $GROOT_VERSION from its lockfile (Python 3.10/CUDA dependencies; large download)"
    (cd "$GROOT_ROOT" && uv sync --python 3.10)
    (cd "$GROOT_ROOT" && uv run python -c \
        "import gr00t, torch; print('GR00T OK; torch=', torch.__version__, 'cuda=', torch.cuda.is_available())")
}

make_cert() {
    need_cmd openssl
    local host_ip="${1:-}"
    if [[ -z "$host_ip" ]]; then
        host_ip="$(ip -4 route get "$THOR_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')"
    fi
    [[ "$host_ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] ||
        die "Could not determine the workstation IPv4 address; specify it explicitly: bash $0 cert 192.168.123.x"

    local out="$HOME/.config/xr_teleoperate"
    mkdir -p "$out"
    # The XR headset browser is Chromium-based, which refuses a server
    # certificate that claims CA:TRUE or omits the serverAuth usage -- and it
    # refuses it without offering the "proceed anyway" interstitial, so the
    # page just comes up blank. Pin the leaf-certificate extensions.
    openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
        -keyout "$out/key.pem" -out "$out/cert.pem" \
        -subj "/CN=$host_ip" \
        -addext "subjectAltName=IP:$host_ip,IP:127.0.0.1,DNS:localhost" \
        -addext "basicConstraints=critical,CA:FALSE" \
        -addext "keyUsage=critical,digitalSignature,keyEncipherment" \
        -addext "extendedKeyUsage=serverAuth"
    chmod 600 "$out/key.pem"
    info "Certificate written to $out; it must still be trusted manually on the first XR browser visit."
}

doctor() {
    local failures=0
    check_ok()   { printf '%s\n' "${GREEN}[ OK ]${RESET} $*"; }
    check_fail() { printf '%s\n' "${RED}[FAIL]${RESET} $*"; failures=$((failures + 1)); }
    check_warn() { printf '%s\n' "${YELLOW}[WARN]${RESET} $*"; }

    if [[ -f "$XR_ROOT/teleop/teleop_hand_and_arm.py" ]]; then
        check_ok "xr_teleoperate"
    else
        check_fail "$XR_ROOT is incomplete"
    fi
    if [[ -f "$XR_ROOT/h2_description/H2.urdf" ]]; then
        check_ok "H2 URDF"
    else
        check_fail "Missing H2 URDF"
    fi
    if [[ -f "$UNITREE_LEROBOT_ROOT/unitree_lerobot/utils/constants.py" ]]; then
        check_ok "unitree_lerobot"
        if grep -q '"Unitree_H2_Sharpa"' "$UNITREE_LEROBOT_ROOT/unitree_lerobot/utils/constants.py"; then
            check_ok "Unitree_H2_Sharpa data configuration"
        else
            check_fail "Missing H2 Sharpa ROBOT_CONFIGS"
        fi
    else
        check_fail "$UNITREE_LEROBOT_ROOT is incomplete"
    fi

    if command -v conda >/dev/null 2>&1; then
        if conda_env_exists "$XR_ENV"; then
            check_ok "conda:$XR_ENV"
            if xr_run python -c \
                "import cv2, cyclonedds, unitree_sdk2py, msgpack, zmq, logging_mp" \
                >/dev/null 2>&1; then
                check_ok "$XR_ENV real-robot runtime dependencies"
            else
                check_fail "$XR_ENV is missing cv2/cyclonedds/unitree_sdk2py/msgpack/zmq/logging_mp"
            fi
        else
            check_fail "Missing conda:$XR_ENV"
        fi
    else
        check_fail "conda is not installed"
    fi
    if [[ -f "$GROOT_ROOT/pyproject.toml" ]]; then
        check_ok "Isaac-GR00T source"
        if [[ -x "$GROOT_ROOT/.venv/bin/python" ]]; then
            check_ok "Isaac-GR00T uv environment"
        else
            check_warn "GR00T is not installed yet (run install-groot)"
        fi
    else
        check_warn "Isaac-GR00T not found: $GROOT_ROOT"
    fi

    if [[ -f "$HOME/.config/xr_teleoperate/cert.pem" ]]; then
        check_ok "XR certificate"
    else
        check_warn "XR certificate has not been generated (run cert)"
    fi
    if [[ -f "$XR_ROOT/teleop/utils/dds_image_client.py" ]]; then
        check_ok "Native DDS image client"
    else
        check_fail "Missing DDSImageClient"
    fi
    if [[ -x "$MANUS_CLIENT_DIR/SharpaManusClient.out" ]] &&
       [[ -f "$MANUS_CLIENT_DIR/ManusSDK/lib/libManusSDK_Integrated.so" ]] &&
       ! ldd "$MANUS_CLIENT_DIR/SharpaManusClient.out" | awk '/not found/{found=1} END{exit !found}'; then
        check_ok "MANUS Integrated SDK/client"
    else
        check_fail "MANUS SDK/client installation is incomplete (run manus-install)"
    fi
    if xr_run python "$SHARPA_RETARGET_DIR/retargeting_manus_demo_dds.py" --help >/dev/null 2>&1; then
        check_ok "MANUS retargeting V5.0 Python environment"
    else
        check_fail "MANUS retargeting V5.0 dependencies failed validation (run manus-install)"
    fi
    if command -v nvidia-smi >/dev/null 2>&1; then
        check_ok "NVIDIA tools available"
    else
        check_warn "nvidia-smi not detected (training may be CPU-only)"
    fi

    if ping -c 1 -W 1 "$THOR_IP" >/dev/null 2>&1; then
        check_ok "Thor reachable: $THOR_IP"
    else
        check_warn "Thor unreachable: $THOR_IP (expected when powered off or off the robot network)"
    fi

    (( failures == 0 )) || die "doctor found $failures required check(s) that failed"
    info "Required software checks passed. Recheck WARN items after enabling the corresponding hardware."
}

ik_test() {
    need_file "$XR_ROOT/teleop/test_h2.py"
    (cd "$XR_ROOT/teleop" && xr_run python test_h2.py --ik-only)
}

camera_check() {
    need_file "$XR_ROOT/teleop/utils/dds_image_client.py"
    info "Checking camera CycloneDDS domain $CAMERA_DDS_DOMAIN (approximately 10 seconds)"
    local net_args=()
    [[ -n "$NETWORK_INTERFACE" ]] && net_args=(--network-interface "$NETWORK_INTERFACE")
    xr_run python "$XR_ROOT/teleop/utils/dds_image_client.py" \
        --domain "$CAMERA_DDS_DOMAIN" "${net_args[@]}" --duration 5
}

camera_visualize() {
    need_file "$XR_ROOT/teleop/utils/dds_image_client.py"
    info "Displaying camera streams from CycloneDDS domain $CAMERA_DDS_DOMAIN; press q to exit"
    xr_run python - "$CAMERA_DDS_DOMAIN" "$NETWORK_INTERFACE" <<'PY'
import sys

import cv2

from teleop.utils.dds_image_client import DDSImageClient

domain_id = int(sys.argv[1])
network_interface = sys.argv[2] or None
client = DDSImageClient(
    domain_id=domain_id,
    network_interface=network_interface,
    warmup_timeout=10,
    require_warmup=True,
)

try:
    while True:
        frames = {
            "head_left": client.get_left_head_frame(),
            "head_right": client.get_right_head_frame(),
            "left_wrist": client.get_left_wrist_frame(),
            "right_wrist": client.get_right_wrist_frame(),
        }
        for name, frame in frames.items():
            if frame.bgr is not None:
                cv2.imshow(name, frame.bgr)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    client.close()
    cv2.destroyAllWindows()
PY
}

hand_retarget() {
    # V5.0 has a dedicated DDS entrypoint. Haptics/tactile collection is opt-in,
    # so this command deliberately omits --haptics and every --haptic-* option.
    need_file "$SHARPA_RETARGET_DIR/retargeting_manus_demo_dds.py"
    local dds_args=(
        --dds
        --side both
        --mocap-address tcp://localhost:2044
        --dds-domain "$DDS_DOMAIN"
    )
    [[ -n "$NETWORK_INTERFACE" ]] && dds_args+=(--dds-interface "$NETWORK_INTERFACE")
    (cd "$SHARPA_RETARGET_DIR" &&
        xr_run python retargeting_manus_demo_dds.py "${dds_args[@]}" "$@")
}

main() {
    local command="${1:-help}"
    [[ $# -eq 0 ]] || shift
    case "$command" in
        help|-h|--help) usage ;;
        workflow) workflow ;;
        config) print_config ;;
        repos) ensure_all_repos ;;
        thor-sync) thor_sync "$@" ;;
        install) install_all "$@" ;;
        install-groot) install_groot "$@" ;;
        manus-install) manus_install "$@" ;;
        manus-start) manus_start "$@" ;;
        manus-check) manus_check "$@" ;;
        cert) make_cert "$@" ;;
        doctor) doctor "$@" ;;
        ik-test) ik_test "$@" ;;
        camera-check) camera_check "$@" ;;
        camera-visualize) camera_visualize "$@" ;;
        hand-retarget) hand_retarget "$@" ;;
        *) usage >&2; die "Unknown command: $command" ;;
    esac
}

main "$@"
