# Sharpa hands over DDS: read, control and setup

How the two Sharpa Wave hands on the H2 are read and controlled from the workstation,
and how to set the whole chain up from a fresh robot image.

## 1. Architecture

```
 Workstation (eno2 192.168.124.120)            Thor (br1 192.168.124.163)             Hands
┌──────────────────────────────┐   DDS     ┌────────────────────────────┐   Sharpa SDK   ┌────────────────┐
│ sharpa_joint_sliders.py      │ domain 0  │ sharpa_dds_bridge          │  TCP 59999 +   │ left  .124.10  │
│ teleop_hand_and_arm.py       │◄─────────►│ (systemd service)          │◄──UDP 50000/──►│ right .124.20  │
│ SharpaHandDDSClient (Python) │ rt/sharpa │ sharpa-wave-sdk 5.0.4 (C++)│  54321         └────────────────┘
└──────────────────────────────┘           └────────────────────────────┘
```

- The hands sit on the 192.168.124.0/24 network behind Thor (`eth11`, bridged as `br1`).
  Thor is the only machine that talks to them with the Sharpa SDK.
- The **DDS bridge** (`scripts/sharpa_dds_bridge.cpp`) runs on Thor. It connects to the hands
  with the Sharpa C++ SDK and republishes them as DDS topics, so the workstation needs no
  Sharpa SDK.
- Clients on the workstation read `rt/sharpa/<side>/state` and write `rt/sharpa/<side>/cmd`
  with the Unitree DDS Python SDK (`unitree_sdk2py`).
- `scripts/sharpa_dds_bridge.py` is an older Python version of the bridge. The C++ one is what
  the service runs.

## 2. Topics and message contract

DDS domain **0**. Message types come from `unitree_hg` (`HandCmd_`, `HandState_`, `MotorCmd_`).

| Topic | Direction | Type | Content |
|---|---|---|---|
| `rt/sharpa/left/state` | bridge → clients | `HandState_` | `motor_state[i].q` for 22 joints, **degrees**, 50 Hz |
| `rt/sharpa/right/state` | bridge → clients | `HandState_` | same, right hand |
| `rt/sharpa/left/cmd` | clients → bridge | `HandCmd_` | `motor_cmd[i].q` for 22 joints, **radians** |
| `rt/sharpa/right/cmd` | clients → bridge | `HandCmd_` | same, right hand |

Rules that are easy to get wrong:

- **Units differ**: commands are radians, state is degrees.
- **Every command carries all 22 joints.** The bridge replaces its whole target vector with each
  message; joints you leave out become 0. To move one joint, send the others at their current
  value (the slider window does this by syncing the sliders to the state first).
- **The bridge holds the last command** and re-sends it to the hand at 50 Hz. A client that
  stops publishing leaves the hand at its last target; it does not return to zero.
- **Last writer wins** if several clients publish to the same `cmd` topic.
- The bridge sends all joints to **0 at startup and at shutdown**, so the hands move to their zero
  pose whenever the service starts or stops.
- Only `motor_cmd[i].q` is used. `mode`, `dq`, `tau`, `kp`, `kd` are ignored.

### Joint order (22 per hand)

Ranges below are the test ranges from the Sharpa SDK sample (`sharpa_wave_example.py`), not the
hardware limits. The slider window uses the same ranges.

| # | Joint | Range (deg) | # | Joint | Range (deg) |
|---|---|---|---|---|---|
| 0 | Thumb CMC flex/ext | 0 to 50 | 11 | Middle PIP | 0 to 20 |
| 1 | Thumb CMC abd/add | 0 to 10 | 12 | Middle DIP | 0 to 20 |
| 2 | Thumb MCP flex/ext | 0 to 30 | 13 | Ring MCP flex/ext | 0 to 20 |
| 3 | Thumb MCP abd/add | 0 to 10 | 14 | Ring MCP abd/add | -20 to 20 |
| 4 | Thumb DIP | 0 to 40 | 15 | Ring PIP | 0 to 20 |
| 5 | Index MCP flex/ext | 0 to 20 | 16 | Ring DIP | 0 to 20 |
| 6 | Index MCP abd/add | -20 to 20 | 17 | Pinky CMC | 0 to 10 |
| 7 | Index PIP | 0 to 20 | 18 | Pinky MCP flex/ext | 0 to 20 |
| 8 | Index DIP | 0 to 20 | 19 | Pinky MCP abd/add | -20 to 20 |
| 9 | Middle MCP flex/ext | 0 to 20 | 20 | Pinky PIP | 0 to 20 |
| 10 | Middle MCP abd/add | -20 to 20 | 21 | Pinky DIP | 0 to 20 |

## 3. What the bridge does

`sharpa_dds_bridge [--side left|right|both] [--speed 0.5] [--state-hz 50] [--tactile]`

1. Starts Sharpa device discovery (UDP heartbeat on port 54321) and waits up to 300 s for a hand.
   It exits with an error if none appears.
2. Connects each requested side and sets position control mode, speed coefficient (`--speed`),
   current coefficient 0.6 and control source SDK, then calls `start()`.
3. Disables the hand's tactile JPEG stream unless `--tactile` is given. One hand streams about
   37 Mb/s of tactile data, so two would saturate a 100 Mb link.
4. Commands every joint to 0, then runs two loops per hand: a state loop that publishes
   `HandState_` at `--state-hz`, and a command loop that sends the latest target to the hand at 50 Hz.
5. On SIGINT/SIGTERM it commands zero again, stops the hand and disconnects.

Known device facts (this robot): left hand `Wave-L-01` at 192.168.124.10, right hand
`Wave-R-01` at 192.168.124.20, both firmware 3.0.7.

## 4. Which network interface DDS uses (important)

Unitree's `ChannelFactory` without an interface name lets CycloneDDS choose one. On Thor it
chose the **Wi-Fi adapter**, not the hand network, so a workstation on a different network never
finds the bridge. Pin both ends to the 192.168.124.x interface:

- **Thor (bridge)**: the service launcher sets
  `CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.124.0" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'`
  The setting is controlled by `SHARPA_DDS_ADDRESS` in `/etc/default/sharpa-dds-bridge`
  (default `192.168.124.0`; a subnet or an IP; empty disables pinning). A `CYCLONEDDS_URI` already
  in the environment takes precedence.
- **Workstation (clients)**: pass the interface that holds a 192.168.124.x address.
  Here that is `eno2`:
  `ChannelFactoryInitialize(0, "eno2")`, or `--network-interface eno2` for `teleop_hand_and_arm.py`.
  `sharpa_joint_sliders.py` finds the interface itself.

## 5. Setup

### 5.1 Thor: Sharpa SDK 5.0.4

SDK 5.0.4 matches hand firmware 3.0.7. SDK 5.0.0 works but logs "Firmware version mismatch".

```bash
cd /user-data/binliu_workspace/Sharpa
curl -LO https://github.com/sharpa-robotics/sharpa-wave-sdk/releases/download/v5.0.4/SharpaWaveSDK_5.0.4_arm.zip
mkdir sharpa_sdk_5.0.4 && unzip -q SharpaWaveSDK_5.0.4_arm.zip -d sharpa_sdk_5.0.4
```

Note: the `config/models/*.onnx` files in that zip are 133-byte Git LFS pointers, not the real
models. That does not matter for the bridge (tactile is off). Fetch the real files from the
`sharpa-wave-sdk` repo if you use `--tactile` or the tactile samples.

### 5.2 Thor: build the bridge

Needs `g++` and the `unitree_sdk2` source tree (headers plus `thirdparty/` DDS libraries).

```bash
cd /user-data/binliu_workspace/Sharpa/dds_bridge_5.0.4     # copy of scripts/sharpa_dds_bridge.cpp + Makefile.sharpa_bridge
make -f Makefile.sharpa_bridge \
  UNITREE_SDK2_DIR=/user-data/binliu_workspace/unitree_sdk2 \
  SHARPA_LIB_DIR=/user-data/binliu_workspace/Sharpa/sharpa_sdk_5.0.4/lib \
  SHARPA_INCLUDE_DIR=/user-data/binliu_workspace/Sharpa/sharpa_sdk_5.0.4/include
```

The binary links `libsharpa-wave-sdk.so` and the DDS libraries dynamically. To run it by hand:

```bash
export LD_LIBRARY_PATH=/user-data/binliu_workspace/Sharpa/sharpa_sdk_5.0.4/lib:/usr/local/lib
./sharpa_dds_bridge --side both
```

### 5.3 Thor: install as a service

Copy `sharpa_dds_bridge.service`, `sharpa_dds_bridge_run.sh` and
`install_sharpa_dds_bridge_service.sh` into one folder next to `dds_bridge_5.0.4/` and
`sharpa_sdk_5.0.4/` (the installer looks for the binary and library at those relative paths), then:

```bash
./install_sharpa_dds_bridge_service.sh          # asks for sudo; installs, enables and starts it
```

- Installs into `/opt/sharpa-dds-bridge/` (binary, `lib/libsharpa-wave-sdk.so`, launcher),
  `/etc/systemd/system/sharpa_dds_bridge.service` and `/etc/default/sharpa-dds-bridge`
  (an existing settings file is kept).
- `--bridge PATH` and `--sdk-lib DIR` override the default paths. `--no-start` installs without
  starting (starting moves the hands to zero). `--uninstall` removes it.
- Settings in `/etc/default/sharpa-dds-bridge`: `SHARPA_SIDE` (both), `SHARPA_SPEED` (0.5),
  `SHARPA_STATE_HZ` (50), `SHARPA_DDS_ADDRESS` (192.168.124.0), optional `SHARPA_EXTRA_ARGS`
  (for example `--tactile`). After editing: `sudo systemctl restart sharpa_dds_bridge`.
- If the bridge cannot find a hand it exits after 300 s and systemd restarts it every 5 s.

```bash
systemctl status sharpa_dds_bridge --no-pager
journalctl -u sharpa_dds_bridge -f
```

### 5.4 Workstation

Use a Python environment that has `unitree_sdk2py`, `cyclonedds` and `tkinter` (here: conda env `tv`).

```bash
conda activate tv
python scripts/sharpa_joint_sliders.py --check     # prints msg/s for each hand; sends nothing
python scripts/sharpa_joint_sliders.py             # the slider window (section 6)
```

Expect about 50 msg/s per hand. The workstation's 192.168.124.x address must be reachable from
Thor (`ping 192.168.124.163`).

## 6. Reading and controlling

### Slider window: `scripts/sharpa_joint_sliders.py`

```
python scripts/sharpa_joint_sliders.py [--side left|right|both] [--interface eno2] [--domain 0] [--check]
```

- One tab per hand. Each joint has a **slider** (the target you command) and a **bar** (the state
  the hand reports), with the numbers next to them. The top bar shows the state rate per hand.
- It starts in **monitor-only** mode and sends nothing. Tick **Enable control** to start sending
  (about 20 Hz); the banner turns green. Ticking it first syncs the sliders to the current state so
  nothing jumps. **Sync sliders to state** repeats that, **All zero** sets every slider to 0.
- Closing the window stops commands; the hands keep their last target.
- A 14° move on a 0 to 50° joint is hard to see. Use the bars to confirm the hand followed.

### Python client: `teleop/robot_control/robot_hand_sharpa_dds.py`

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from teleop.robot_control.robot_hand_sharpa_dds import SharpaHandDDSClient

ChannelFactoryInitialize(0, "eno2")          # once per process, same domain as the bridge
left = SharpaHandDDSClient("left")
_, deg = left.get_joint_position_degree()    # 22 values, degrees
cmd = [0.0] * 22                             # all joints, radians
cmd[7] = 0.1745                              # index PIP to 10 degrees
left.set_joint_position(cmd)                 # publish; repeat at ~20-50 Hz to hold the target
```

### Teleop

`teleop/teleop_hand_and_arm.py --ee sharpa` only **reads** the hand state
(`robot_hand_sharpa.py`, a daemon thread). Commands come from a separate retargeting process on the
same DDS domain. Relevant options: `--sharpa-dds-domain` (default 0) and
`--network-interface` (set it to the 192.168.124.x interface, for example `eno2`). The main process
calls `ChannelFactoryInitialize` once, and the Sharpa reader reuses it.

## 7. Test tools

| Tool | What it does |
|---|---|
| `sharpa_joint_sliders.py --check` | state rate and first joints per hand, from the workstation |
| `sharpa_dds_state_check.cpp` | C++ subscriber: `state_check <topic> <seconds>` prints msg/s and the last state |
| `sharpa_dds_cmd_test.cpp` | `cmd_test <left\|right> <joint> <deg> [hold_s]` moves one joint (refuses above 20°) and prints the state following |
| `test_sharpa_hand.py --mode dds --side left` | gesture sequence through the bridge (fist, open, finger curls) |
| `sharpa_net_monitor.sh` | rx/tx rate on the hand network (about 37 Mb/s per hand with tactile on) |

Build the two C++ tools with the command in each file's header (they link the `unitree_sdk2`
source tree).

## 8. Troubleshooting

| Symptom | Check |
|---|---|
| Workstation sees 0 msg/s | Is the bridge running (`systemctl status sharpa_dds_bridge`)? Same domain (0)? Both ends pinned to 192.168.124.x (section 4)? |
| `Host key verification failed` / `Permission denied` over SSH after a reimage | New host key and empty `authorized_keys`; run `ssh-copy-id unitree@192.168.124.163`. |
| Bridge exits: no devices found | Hands powered? `ping 192.168.124.10` / `.20` from Thor, check `br1` has 192.168.124.163. |
| Manual run fails with a port in use | The service already holds UDP 50000/50005/54321; `sudo systemctl stop sharpa_dds_bridge` first. |
| Slider window moves nothing | The banner must be green (Enable control ticked). Confirm commands reach the topic with a subscriber on `rt/sharpa/<side>/cmd`. |
| Hands jump to zero | Expected at service start and stop. |
| "Firmware version mismatch" in the log | Use SDK 5.0.4 (section 5.1). |
| `libsharpa-wave-sdk.so: not found` | `LD_LIBRARY_PATH` must include the SDK `lib` dir (the service launcher sets it). |
| Network saturates with `--tactile` | Two hands stream about 74 Mb/s; leave tactile off unless needed. |

## 9. Files

| File | Role |
|---|---|
| `sharpa_dds_bridge.cpp`, `Makefile.sharpa_bridge` | the bridge and its Makefile |
| `sharpa_dds_bridge_run.sh` | service launcher: DDS pin, library path, flags from the environment |
| `sharpa_dds_bridge.service` | systemd unit |
| `install_sharpa_dds_bridge_service.sh` | installs or removes the service |
| `sharpa_joint_sliders.py` | workstation slider window and `--check` |
| `sharpa_dds_state_check.cpp`, `sharpa_dds_cmd_test.cpp` | C++ DDS test tools |
| `sharpa_net_monitor.sh` | network rate monitor |
| `thor_route.sh`, `workstation_route.sh` | older approach: NAT and UDP relay so the workstation runs the Sharpa SDK directly; not needed with the bridge |
| `../teleop/robot_control/robot_hand_sharpa_dds.py` | Python DDS client (state and commands) |
| `../teleop/robot_control/robot_hand_sharpa.py` | read-only state reader used by teleop |
