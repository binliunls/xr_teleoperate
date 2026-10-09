#!/usr/bin/env bash
# cleanup_teleop.sh — tear down the whole H2 + Sharpa teleop / data-capture stack,
# on the workstation AND on Thor, in reverse of the startup order.
#
# Run ON THE WORKSTATION:
#   ./scripts/cleanup_teleop.sh            # workstation, then Thor
#   ./scripts/cleanup_teleop.sh -n         # dry run: list what would be killed
#   ./scripts/cleanup_teleop.sh --ws-only
#   ./scripts/cleanup_teleop.sh --thor-only
#
# Workstation (steps 7, 5):  teleop_hand_and_arm.py (Vuer teleop), rerun viewer,
#                            televuer / ROS image client, avatar_hand_dds_bridge.py
# Thor        (steps 4..1):  sharpa_dds_bridge (hand DDS bridge), sharpa180 container
#                            (tactile_zmq_180.py), sharpa180-recorder container,
#                            camera publishers (run_all_3cam.sh / gscam_main /
#                            gst-launch), any tmux sessions.
# Containers are stopped, not removed: 'docker start sharpa180' brings them back.
#
# Thor SSH: key auth if available, else sshpass with the password in $THOR_SSH_PASS.

set -u

THOR_HOST="${THOR_HOST:-unitree@192.168.125.163}"
THOR_SSH_PASS="${THOR_SSH_PASS:-}"
DRY=0; WS=1; THOR=1
for a in "$@"; do
  case "$a" in
    -n|--dry-run) DRY=1 ;;
    --ws-only)    THOR=0 ;;
    --thor-only)  WS=0 ;;
    -h|--help)    sed -n '2,19p' "$0"; exit 0 ;;
    *) echo "unknown arg: $a"; exit 2 ;;
  esac
done

# Command-line substrings identifying each side's teleop processes.
WS_PATTERNS=(
  teleop_hand_and_arm.py
  avatar_hand_dds_bridge.py
  "rerun --port"
  televuer
  ros_image_client
)
THOR_PATTERNS=(
  sharpa_dds_bridge
  run_all_3cam
  gscam_main
  gst-launch-1.0
  calibrate_sharpa_tactile.py
)
THOR_CONTAINERS=(sharpa180 sharpa180-recorder)

say() { printf '%s\n' "$*"; }

# Shell function body shared by both sides (sent to Thor as text).
# kill_patterns <label> <dry> <pattern>...  — TERM matches, then KILL survivors.
KILL_FN='
kill_patterns() {
  local label="$1" dry="$2"; shift 2
  local pids="" p pid left
  for p in "$@"; do pids="$pids $(pgrep -f -- "$p" 2>/dev/null | grep -vx "$$" || true)"; done
  pids=$(echo "$pids" | tr " " "\n" | grep -E "^[0-9]+$" | sort -un | tr "\n" " ")
  if [ -z "${pids// /}" ]; then echo "  $label: none running"; return 0; fi
  for pid in $pids; do echo "  $label: pid $pid  $(ps -o args= -p "$pid" 2>/dev/null | cut -c1-90)"; done
  [ "$dry" = 1 ] && return 0
  kill $pids 2>/dev/null || true; sleep 3
  left=$(for pid in $pids; do kill -0 "$pid" 2>/dev/null && echo "$pid"; done)
  if [ -n "$left" ]; then echo "  $label: still alive after TERM, sending KILL: $left"; kill -9 $left 2>/dev/null || true; sleep 1; fi
  for p in "$@"; do pgrep -f -- "$p" >/dev/null 2>&1 && echo "  $label: STILL RUNNING: $p"; done
  return 0
}'
eval "$KILL_FN"

if [ "$WS" = 1 ]; then
  say "=== Workstation ($(hostname))$( [ "$DRY" = 1 ] && echo ' [DRY RUN]')"
  kill_patterns "ws" "$DRY" "${WS_PATTERNS[@]}"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn 2>/dev/null | grep -q ':8012 ' && say "  ws: WARNING port 8012 (Vuer) still bound" || say "  ws: port 8012 (Vuer) free"
  fi
fi

if [ "$THOR" = 1 ]; then
  say ""
  say "=== Thor ($THOR_HOST)$( [ "$DRY" = 1 ] && echo ' [DRY RUN]')"
  SSH=(ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=6)
  if ! ssh -o BatchMode=yes -o ConnectTimeout=6 -o StrictHostKeyChecking=accept-new "$THOR_HOST" true 2>/dev/null; then
    if [ -n "$THOR_SSH_PASS" ] && command -v sshpass >/dev/null 2>&1; then
      SSH=(sshpass -p "$THOR_SSH_PASS" "${SSH[@]}")
    else
      say "  no key auth to Thor: export THOR_SSH_PASS=<password> (needs sshpass) or set up ssh-copy-id"; exit 1
    fi
  fi
  remote=$(cat <<EOF
$KILL_FN
DRY=$DRY
echo "-- tmux session 'teleop' (start_teleop_thor.sh)"
if command -v tmux >/dev/null && tmux has-session -t teleop 2>/dev/null; then
  tmux list-windows -t teleop -F "  window: #{window_name}  #{pane_current_command}" 2>/dev/null
  [ "\$DRY" = 1 ] || tmux kill-session -t teleop
else echo "  none"; fi
echo "-- hand bridge / cameras / calibration"
kill_patterns "thor" "\$DRY" $(printf "'%s' " "${THOR_PATTERNS[@]}")
echo "-- docker containers"
for c in ${THOR_CONTAINERS[*]}; do
  if docker ps --format "{{.Names}}" 2>/dev/null | grep -qx "\$c"; then
    echo "  stopping \$c  (\$(docker top "\$c" -o pid,args 2>/dev/null | tail -n +2 | head -1 | cut -c1-70))"
    [ "\$DRY" = 1 ] || docker stop "\$c" >/dev/null
  else echo "  \$c: not running"; fi
done
left=\$(docker ps --format "{{.Names}}" 2>/dev/null | grep -E "^sharpa180" || true)
[ -z "\$left" ] && echo "  containers: clean (stopped ones kept; docker start <name> restores)" || echo "  containers STILL RUNNING: \$left"
EOF
)
  "${SSH[@]}" "$THOR_HOST" "bash -s" <<<"$remote"
fi
