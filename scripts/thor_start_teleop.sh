#!/usr/bin/env bash
# start_teleop_thor.sh — Thor-side teleop / data-capture stack in a tmux session.
# Covers startup steps 2-4 in one tmux session "teleop", one window per step.
# Cameras (step 1) are NOT included.
#
# Run ON THOR (unitree@192.168.125.163):
#   ~/start_teleop_thor.sh            start (if not already running) and attach
#   ~/start_teleop_thor.sh --status   last lines of each window, no attach
#   ~/start_teleop_thor.sh --stop     kill the tmux session (Ctrl-C equivalent for all 3)
#
# Windows:  recorder : docker start -a sharpa180-recorder
#           tactile  : docker start sharpa180 && docker exec -it sharpa180 ... tactile_zmq_180.py
#           bridge   : ~/Sharpa_Haochen/new_bridge_dds_native_capture_v1/sharpa_dds_bridge --side both ...
# tmux keys: Ctrl-b n / p = next / previous window,  Ctrl-b d = detach (everything keeps
# running),  Ctrl-C inside a window = stop that step (window stays open with a shell).
# Full teardown incl. containers: workstation scripts/cleanup_teleop.sh.

S=teleop
RECORDER='docker start -a sharpa180-recorder'
TACTILE="docker start sharpa180 && docker exec -it sharpa180 bash -lc 'source /root/.bashrc && cd /workspace && exec python3 tactile_zmq_180.py --config /workspace/config/tactile_180_dual.json'"
BRIDGE='cd ~/Sharpa_Haochen/new_bridge_dds_native_capture_v1 && ./sharpa_dds_bridge --side both --state-hz 30 --dds-interface enx80691a14d263 --external-tactile --tactile-port 0 --telemetry-port 48011'

# Run a step, then drop into a shell so the output stays readable after exit.
win() { printf 'echo "$ %s"; %s; echo; echo "[%s exited — window kept open]"; exec bash' "$2" "$2" "$1"; }

case "${1:-}" in
  --status)
    tmux has-session -t $S 2>/dev/null || { echo "tmux session '$S' not running"; exit 1; }
    for w in recorder tactile bridge; do
      echo "== $w"; tmux capture-pane -p -t $S:$w 2>/dev/null | grep -v '^\s*$' | tail -4 | cut -c1-110
    done; exit 0 ;;
  --stop)  tmux kill-session -t $S 2>/dev/null && echo "session '$S' killed" || echo "session '$S' not running"; exit 0 ;;
  -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
  "") ;;
  *) echo "unknown arg: $1"; exit 2 ;;
esac

if tmux has-session -t $S 2>/dev/null; then
  echo "tmux session '$S' already running — attaching (Ctrl-b d to detach)"
else
  tmux new-session  -d -s $S -n recorder "$(win recorder "$RECORDER")"
  sleep 3
  tmux new-window   -t $S    -n tactile  "$(win tactile  "$TACTILE")"
  sleep 5
  tmux new-window   -t $S    -n bridge   "$(win bridge   "$BRIDGE")"
  tmux select-window -t $S:recorder
  echo "started tmux session '$S' (windows: recorder, tactile, bridge). Wait for:"
  echo "  recorder : [control] REP tcp://192.168.125.163:48010  +  [relay] 30 Hz-compatible PUB ..."
  echo "  tactile  : [ZMQ] calibration RPC tcp://127.0.0.1:48009   (do NOT calibrate yet)"
  echo "  bridge   : [left] hand ready.  and  [right] hand ready."
  echo "Then: workstation Avatar bridge (5) -> calibrate here (6) -> teleop (7)."
fi
[ -t 0 ] && exec tmux attach -t $S
exit 0
