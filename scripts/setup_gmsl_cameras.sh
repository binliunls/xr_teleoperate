#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
# All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -Eeuo pipefail

readonly trigger_pin="0x00020007"

unitree_i2c_script="${UNITREE_I2C_SCRIPT:-${HOME}/.Unitree/set_i2c3.sh}"
unitree_modules_dir="${UNITREE_MODULES_DIR:-${HOME}/.Unitree/YUSHU_4A_AGTH_G2Y_7.1}"
trigger_source="free-run"
enable_3g_0="0,0,0,0"
enable_3g_1="0,0,0,0"
device_timeout_seconds=30
controls_only=false
dry_run=false
skip_clock_boost=false
skip_i2c=false
skip_imu=false
argus_infinite_timeout=false

usage() {
  cat <<'EOF'
Prepare the mixed Astra S56x + SHF3L GMSL cameras for the Holoscan application.

Usage:
  sudo ./setup_gmsl_cameras.sh [options]

Options:
  --driver-dir PATH           Vendor JP7.1 driver package directory.
  --i2c-script PATH           Optional board I2C setup script.
  --trigger SOURCE            free-run, internal, or external (default: free-run).
  --enable-3g-0 CSV           Four CAM0-CAM3 link flags (default: 0,0,0,0).
  --enable-3g-1 CSV           Four CAM4-CAM7 link flags (default: 0,0,0,0).
  --device-timeout SECONDS    Wait for video0-video3 (default: 30, maximum: 120).
  --controls-only             Do not stop Argus or reload kernel modules.
  --argus-infinite-timeout    Add a reboot-scoped Argus timeout override.
  --skip-clock-boost          Do not run the vendor boost_clock.sh.
  --skip-i2c                  Do not run the optional board I2C setup script.
  --skip-imu                  Do not load the S56 BMI088 IMU modules.
  --dry-run                   Print privileged operations without executing them.
  -h, --help                  Show this help.

Camera mapping and controls:
  video0  Astra S56x head left   sensor_mode=0
  video1  Astra S56x head right  sensor_mode=0
  video2  SHF3L wrist left       sensor_mode=1 (1920x1536)
  video3  SHF3L wrist right      sensor_mode=1 (1920x1536)

Trigger behavior:
  free-run  Sets trig_mode=0 and disables the internal PWM if it exists.
  internal  Sets trig_mode=1 and configures pwmchip4/pwm0 for 30 Hz, 10% duty.
  external  Sets trig_mode=1, disables internal PWM, and expects 30 Hz on CN4.

This script does not install or select a device-tree overlay. The mixed overlay must
already describe S56 on CAM0/CAM1 and sgx-yuv-gmsl2 (SHF3L) on CAM2/CAM3.
EOF
}

log() {
  printf '[gmsl-setup] %s\n' "$*"
}

fail() {
  printf '[gmsl-setup] ERROR: %s\n' "$*" >&2
  exit 1
}

if [[ "${EUID}" -eq 0 && -n "${SUDO_USER:-}" ]]; then
  invoking_user_home="$(getent passwd "${SUDO_USER}" | cut -d: -f6)"
  [[ -n "${invoking_user_home}" ]] || fail "Could not resolve the home directory for ${SUDO_USER}."
  if [[ -z "${UNITREE_I2C_SCRIPT+x}" ]]; then
    unitree_i2c_script="${invoking_user_home}/.Unitree/set_i2c3.sh"
  fi
  if [[ -z "${UNITREE_MODULES_DIR+x}" ]]; then
    unitree_modules_dir="${invoking_user_home}/.Unitree/YUSHU_4A_AGTH_G2Y_7.1"
  fi
fi

print_command() {
  printf '[gmsl-setup] Running:'
  printf ' %q' "$@"
  printf '\n'
}

run() {
  print_command "$@"
  if [[ "${dry_run}" == false ]]; then
    "$@"
  fi
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "Required command '$1' is not installed."
}

require_file() {
  [[ -f "$1" ]] || fail "Required file not found: $1"
}

module_loaded() {
  [[ -d "/sys/module/$1" ]]
}

unload_module() {
  local module_name="$1"
  if module_loaded "${module_name}"; then
    run rmmod "${module_name}"
  else
    log "Kernel module ${module_name} is not loaded"
  fi
}

write_sysfs() {
  local value="$1"
  local path="$2"

  log "Writing ${value} to ${path}"
  if [[ "${dry_run}" == false ]]; then
    [[ -e "${path}" ]] || fail "Expected sysfs node is missing: ${path}"
    printf '%s\n' "${value}" >"${path}"
  fi
}

wait_for_path() {
  local path="$1"
  local attempts=$((device_timeout_seconds * 10))
  local attempt

  if [[ "${dry_run}" == true ]]; then
    log "Would wait up to ${device_timeout_seconds}s for ${path}"
    return
  fi

  for ((attempt = 0; attempt < attempts; ++attempt)); do
    if [[ -e "${path}" ]]; then
      return
    fi
    sleep 0.1
  done
  fail "Timed out waiting for ${path}; check the installed mixed-camera overlay and kernel log."
}

validate_link_flags() {
  local value="$1"
  [[ "${value}" =~ ^[01],[01],[01],[01]$ ]] ||
    fail "Link flags must contain four comma-separated 0/1 values: ${value}"
}

configure_internal_pwm() {
  local pwm_chip="/sys/class/pwm/pwmchip4"
  local pwm_channel="${pwm_chip}/pwm0"

  if [[ "${dry_run}" == false ]]; then
    [[ -d "${pwm_chip}" ]] || fail "Internal-trigger PWM controller is missing: ${pwm_chip}"
  fi

  if [[ "${dry_run}" == true || ! -d "${pwm_channel}" ]]; then
    write_sysfs 0 "${pwm_chip}/export"
    wait_for_path "${pwm_channel}"
  fi

  if [[ "${dry_run}" == true || -e "${pwm_channel}/enable" ]]; then
    write_sysfs 0 "${pwm_channel}/enable"
  fi
  write_sysfs 33333333 "${pwm_channel}/period"
  write_sysfs 3333333 "${pwm_channel}/duty_cycle"
  write_sysfs 1 "${pwm_channel}/enable"
  log "Internal 30 Hz camera trigger is enabled"
}

disable_internal_pwm() {
  local enable_node="/sys/class/pwm/pwmchip4/pwm0/enable"
  if [[ "${dry_run}" == true || -e "${enable_node}" ]]; then
    write_sysfs 0 "${enable_node}"
    log "Internal camera trigger PWM is disabled"
  fi
}

configure_argus_override() {
  local dropin_dir="/run/systemd/system/nvargus-daemon.service.d"
  local dropin_file="${dropin_dir}/gmsl-cameras.conf"

  log "Configuring reboot-scoped nvargus-daemon infinite-timeout override"
  if [[ "${dry_run}" == false ]]; then
    install -d -m 0755 "${dropin_dir}"
    printf '%s\n' \
      '[Service]' \
      'Environment="NVCAMERA_NITO_PATH=CONFIG"' \
      'Environment="enableCamInfiniteTimeout=1"' >"${dropin_file}"
  fi
  run systemctl daemon-reload
}

apply_camera_controls() {
  local trigger_mode=0
  local camera_spec
  local camera_name
  local device
  local sensor_mode
  local -a camera_specs=(
    "head-left:/dev/video0:0"
    "head-right:/dev/video1:0"
    "wrist-left:/dev/video2:1"
    "wrist-right:/dev/video3:1"
  )

  if [[ "${trigger_source}" != "free-run" ]]; then
    trigger_mode=1
  fi

  for camera_spec in "${camera_specs[@]}"; do
    IFS=: read -r camera_name device sensor_mode <<<"${camera_spec}"
    wait_for_path "${device}"
    run v4l2-ctl -d "${device}" \
      -c "sensor_mode=${sensor_mode},trig_pin=${trigger_pin},trig_mode=${trigger_mode}"
    log "Configured ${camera_name} (${device}): sensor_mode=${sensor_mode}, trig_mode=${trigger_mode}"
  done
}

verify_camera_controls() {
  local device
  for device in /dev/video0 /dev/video1 /dev/video2 /dev/video3; do
    if [[ "${dry_run}" == true ]]; then
      print_command v4l2-ctl -d "${device}" --get-ctrl=sensor_mode,trig_pin,trig_mode
    else
      log "Controls reported by ${device}:"
      v4l2-ctl -d "${device}" --get-ctrl=sensor_mode,trig_pin,trig_mode
    fi
  done
}

while (($# > 0)); do
  case "$1" in
    --driver-dir)
      (($# >= 2)) || fail "--driver-dir requires a path"
      unitree_modules_dir="$2"
      shift 2
      ;;
    --i2c-script)
      (($# >= 2)) || fail "--i2c-script requires a path"
      unitree_i2c_script="$2"
      shift 2
      ;;
    --trigger)
      (($# >= 2)) || fail "--trigger requires free-run, internal, or external"
      trigger_source="$2"
      shift 2
      ;;
    --enable-3g-0)
      (($# >= 2)) || fail "--enable-3g-0 requires four comma-separated flags"
      enable_3g_0="$2"
      shift 2
      ;;
    --enable-3g-1)
      (($# >= 2)) || fail "--enable-3g-1 requires four comma-separated flags"
      enable_3g_1="$2"
      shift 2
      ;;
    --device-timeout)
      (($# >= 2)) || fail "--device-timeout requires a number of seconds"
      device_timeout_seconds="$2"
      shift 2
      ;;
    --controls-only)
      controls_only=true
      shift
      ;;
    --argus-infinite-timeout)
      argus_infinite_timeout=true
      shift
      ;;
    --skip-clock-boost)
      skip_clock_boost=true
      shift
      ;;
    --skip-i2c)
      skip_i2c=true
      shift
      ;;
    --skip-imu)
      skip_imu=true
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
      fail "Unknown option: $1"
      ;;
  esac
done

case "${trigger_source}" in
  free-run|internal|external) ;;
  *) fail "Unsupported trigger source '${trigger_source}'; use free-run, internal, or external." ;;
esac

[[ "${device_timeout_seconds}" =~ ^[0-9]+$ ]] || fail "Device timeout must be an integer."
((device_timeout_seconds >= 1 && device_timeout_seconds <= 120)) ||
  fail "Device timeout must be between 1 and 120 seconds."
validate_link_flags "${enable_3g_0}"
validate_link_flags "${enable_3g_1}"

if [[ "${dry_run}" == false && "${EUID}" -ne 0 ]]; then
  fail "Run this host setup with sudo, or use --dry-run to inspect it."
fi

require_command v4l2-ctl
if [[ "${controls_only}" == false ]]; then
  require_command busybox
  require_command insmod
  require_command rmmod
  require_command systemctl

  require_file "${unitree_modules_dir}/ko/max96712.ko"
  require_file "${unitree_modules_dir}/ko/sgcam-gmsl2.ko"
  require_file "${unitree_modules_dir}/ko/pwm-gpio.ko"
  if [[ "${skip_imu}" == false ]]; then
    require_file "${unitree_modules_dir}/ko/kfifo_buf.ko"
    require_file "${unitree_modules_dir}/ko/bmi088.ko"
  fi
  if [[ "${skip_clock_boost}" == false ]]; then
    require_file "${unitree_modules_dir}/boost_clock.sh"
  fi
fi

log "Preparing S56 head cameras on video0/1 and SHF3L wrist cameras on video2/3"
log "Trigger source: ${trigger_source}"

if [[ "${controls_only}" == false ]]; then
  if [[ "${skip_i2c}" == false ]]; then
    if [[ -f "${unitree_i2c_script}" ]]; then
      run bash "${unitree_i2c_script}"
    else
      log "Optional board I2C setup script not found; continuing: ${unitree_i2c_script}"
    fi
  fi

  run systemctl stop nvargus-daemon.service

  run busybox devmem 0x810c2810a8 w 0x203000
  run busybox devmem 0x810c2810a0 w 0x203000

  unload_module bmi088
  unload_module kfifo_buf
  unload_module pwm_gpio
  unload_module sgcam_gmsl2
  unload_module max96712

  run insmod "${unitree_modules_dir}/ko/max96712.ko"
  run insmod "${unitree_modules_dir}/ko/sgcam-gmsl2.ko" \
    "enable_3G_0=${enable_3g_0}" "enable_3G_1=${enable_3g_1}"
  run insmod "${unitree_modules_dir}/ko/pwm-gpio.ko"

  if [[ "${skip_clock_boost}" == false ]]; then
    run bash "${unitree_modules_dir}/boost_clock.sh"
  fi

  if [[ "${skip_imu}" == false ]]; then
    run insmod "${unitree_modules_dir}/ko/kfifo_buf.ko"
    run insmod "${unitree_modules_dir}/ko/bmi088.ko"
  fi
fi

case "${trigger_source}" in
  free-run)
    disable_internal_pwm
    ;;
  external)
    disable_internal_pwm
    ;;
esac

apply_camera_controls

case "${trigger_source}" in
  internal)
    configure_internal_pwm
    ;;
  external)
    log "External mode expects a 30 Hz, 3.3 V, 10% duty trigger on CN4 CAM-FSYNC1"
    ;;
esac

if [[ "${argus_infinite_timeout}" == true ]]; then
  require_command systemctl
  configure_argus_override
fi

if [[ "${controls_only}" == false || "${argus_infinite_timeout}" == true ]]; then
  run systemctl restart nvargus-daemon.service
fi

if [[ "${dry_run}" == false ]]; then
  systemctl is-active --quiet nvargus-daemon.service ||
    fail "nvargus-daemon did not become active; inspect 'journalctl -u nvargus-daemon'."
  log "nvargus-daemon is active"
fi

verify_camera_controls
log "Camera host setup completed; the Holoscan application can now be started"
