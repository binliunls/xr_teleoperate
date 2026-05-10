# Connecting to Sharpa Hands on Thor

## Network Layout

```
Workstation ──[192.168.123.x]── Thor ──[192.168.124.x]── Sharpa Hands
                            123.163   124.163
```

Thor has two ethernet ports:

| Interface | IP | Connected to |
|---|---|---|
| eth (123) | 192.168.123.163 | Workstation |
| eth (124) | 192.168.124.163 | Sharpa hands |

The workstation cannot reach the Sharpa hands directly. Traffic must be routed through Thor.

---

## Why simple routing is not enough for the SDK

The Sharpa SDK uses **UDP broadcast** for device discovery in both directions:

| Direction | Packet destination | Crosses subnet? |
|---|---|---|
| Host → Hands (announcement) | `192.168.124.255:54321` | No — broadcast stays on 124.x |
| Hands → Host (heartbeat) | `255.255.255.255:54321` | No — global broadcast stays local |

IP routing (MASQUERADE) only handles unicast — it cannot forward broadcasts. Even with routing enabled, the SDK on the workstation will time out during discovery because the heartbeat packets never arrive.

**Two options:**

- **Option A** — Run scripts on Thor via SSH. Thor is on `192.168.124.x` so broadcasts reach it directly. No extra setup.
- **Option B** — Run scripts on the workstation. Requires `socat` on Thor to relay broadcasts across the subnet boundary.

---

## Setup (after every reboot)

### Option A — Run on Thor (recommended)

No workstation routing needed. Just SSH to Thor and run scripts there:

```bash
ssh user@192.168.123.163
cd ~/binliu/xr_teleoperate
python scripts/test_sharpa_left_hand.py
```

### Option B — Run on workstation (requires socat relay)

Install `socat` on Thor if not present:
```bash
sudo apt install socat
```

**On Thor:**
```bash
sudo ./scripts/thor_route.sh start [workstation-ip]
# e.g. sudo ./scripts/thor_route.sh start 192.168.123.50
```

This enables IP forwarding, MASQUERADE, and starts two `socat` relay processes:
- Relays hand heartbeat broadcasts → workstation port 54321
- Relays workstation announcements → hand subnet broadcast

**On the workstation:**
```bash
sudo ./scripts/workstation_route.sh start
```

### Verify

```bash
ping 192.168.124.10   # left hand
ping 192.168.124.20   # right hand
```

---

## Undo

### On Thor

```bash
sudo ./scripts/thor_route.sh stop
```

Stops the relay processes and removes all routing/NAT rules.

### On the workstation

```bash
sudo ./scripts/workstation_route.sh stop
```

---

## DDS bridge (recommended for workstation control)

Instead of fighting the UDP broadcast discovery problem, run a bridge service on Thor that owns the SDK connection and exposes the hands over DDS. The workstation then only needs `unitree_sdk2py` — no Sharpa SDK required.

### Architecture

```
Workstation                    Thor                       Sharpa Hands
  eval_h2_groot.py  ──DDS──►  sharpa_dds_bridge  ──SDK──►  192.168.124.10
  test_sharpa_*.py  ◄──DDS──  (rt/sharpa/{left,right}/{cmd,state})  192.168.124.20
```

The bridge is a native C++ binary (`scripts/sharpa_dds_bridge.cpp`) that uses the
ARM C++ Sharpa SDK. The Python SDK has no ARM binaries so Python cannot run the bridge.

### DDS topics

| Topic | Direction | Type | Unit |
|---|---|---|---|
| `rt/sharpa/left/cmd` | workstation → Thor | `HandCmd_` | radians |
| `rt/sharpa/right/cmd` | workstation → Thor | `HandCmd_` | radians |
| `rt/sharpa/left/state` | Thor → workstation | `HandState_` | degrees |
| `rt/sharpa/right/state` | Thor → workstation | `HandState_` | degrees |

### Build the bridge (on Thor)

Dependencies:
- Sharpa C++ SDK: install the ARM deb or copy `libsharpa-wave-sdk.so` +
  `SharpaWaveSDK.h` to `/usr/lib/sharpa-wave-sdk/` and `/usr/include/sharpa-wave-sdk/`
- unitree_sdk2: clone to `~/unitree_sdk2` (contains aarch64 prebuilt `.a` and thirdparty `.so`)

```bash
cd ~/binliu/xr_teleoperate/scripts
make -f Makefile.sharpa_bridge          # auto-detects ~/unitree_sdk2
# or with explicit path:
make -f Makefile.sharpa_bridge UNITREE_SDK2_DIR=~/unitree_sdk2
```

### Start the bridge (on Thor)

```bash
cd ~/binliu/xr_teleoperate/scripts
./sharpa_dds_bridge
# custom speed/rate:
./sharpa_dds_bridge --speed 0.3 --state-hz 50
```

### Use from workstation

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
ChannelFactoryInitialize(0)
from teleop.robot_control.robot_hand_sharpa_dds import SharpaHandDDSClient

left  = SharpaHandDDSClient("left")
right = SharpaHandDDSClient("right")

left.set_joint_position([0.0] * 22)          # radians
_, angles_deg = left.get_joint_position_degree()
```

### Test script

```bash
# On Thor (direct SDK):
python scripts/test_sharpa_left_hand.py --mode sdk

# On workstation (DDS bridge must be running on Thor):
python scripts/test_sharpa_left_hand.py --mode dds
```

---

## SDK version

The Sharpa SDK path is controlled by the `SHARPA_SDK_PATH` environment variable.

| Version | Path | How to select |
|---|---|---|
| 5.0 (default) | `/usr/lib/sharpa-wave-sdk/python` | unset `SHARPA_SDK_PATH` |
| 4.6.6 | `~/Sharpa/SharpaWaveSDK_4.6.6/python` | `export SHARPA_SDK_PATH=~/Sharpa/SharpaWaveSDK_4.6.6/python` |

To switch permanently, add the export to `~/.bashrc`.
