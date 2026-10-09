# H2 + Thor teleop network and signal map

This document describes the verified stage-one deployment for the current H2
with the new Thor. Stage one keeps the existing 30 Hz tactile path; it does not
enable the separate 180 Hz capture path.

## What is persistent now

- The workstation keeps `192.168.123.222`, `192.168.124.222`, and
  `192.168.125.222` on `eno1` through NetworkManager.
- The workstation keeps a host route for `192.168.123.161` through
  `192.168.125.163`, with source address `192.168.123.222`.
- Thor keeps `192.168.125.163` on its USB Ethernet adapter.
- `h2-dds-routing.service` restores Thor's `192.168.123.163` alias, the return
  route to the workstation, IP forwarding, proxy ARP, and two address-scoped
  firewall rules after every Thor boot.
- The existing `sharpa-tactile-ips.service` continues to restore the two
  independent `192.168.124.x` Sharpa links.
- The camera launcher explicitly selects Thor `192.168.125.163`, workstation
  peer `192.168.125.222`, CycloneDDS, and ROS domain 0. It refuses to start if
  the expected Thor address is absent.
- Thor follows the H2 controller's NTP clock, and the workstation follows Thor.
- Physical H2 teleop discovers the current `basic_service` and `humanoid` DDS
  ports at startup. Five-digit ports no longer belong in the teleop command.

The post-reboot acceptance check verified all of these conditions. The `.125`
link negotiated at 2.5 Gb/s full duplex on both ends.

## Topology

```text
 Left Sharpa hand                         H2 body controller
 192.168.124.10                           192.168.123.161
       | vendor SDK                            | DDS + NTP
       |                                       |
 Thor eth11                              Thor eth10
 192.168.124.1               192.168.124.240/32 + 192.168.123.163/24
       \                                      /
        \                                    /
         +----------- Thor ------------------+
                   USB Ethernet
                 192.168.125.163
                        |
                  2.5 Gb/s link
                        |
               Workstation eno1
       192.168.125.222 + 192.168.123.222
             (+ 192.168.124.222 unused)

 Right Sharpa hand 192.168.124.20 is reached from Thor eth10 using
 Thor source address 192.168.124.240.
```

The H2 body packets keep workstation source address `192.168.123.222`:

```text
workstation .123.222
  -> host route via Thor .125.163
  -> Thor forwards between USB Ethernet and eth10
  -> H2 .123.161
```

## Address ownership

| Machine/interface | Address | Role |
|---|---:|---|
| Workstation `eno1` | `192.168.123.222/24` | Source address for the Unitree body/hand recording DDS participant |
| Workstation `eno1` | `192.168.125.222/24` | ROS cameras, Avatar hand-control DDS, tactile clients, SSH, and NTP client |
| Workstation `eno1` | `192.168.124.222/24` | Configured but unused by this teleop path |
| Thor USB `enx80691a14d263` | `192.168.125.163/24` | Cameras, Sharpa DDS bridge, tactile publisher, SSH, and NTP server |
| Thor `eth10` | `192.168.123.163/24` | Gateway/proxy toward the H2 controller |
| H2 controller | `192.168.123.161` | H2 body DDS and upstream NTP server |
| Thor `eth11` | `192.168.124.1/24` | Left Sharpa link |
| Left Sharpa hand | `192.168.124.10` | Vendor SDK device |
| Thor `eth10` | `192.168.124.240/32` | Right Sharpa link, coexisting with `.123.163` |
| Right Sharpa hand | `192.168.124.20` | Vendor SDK device |

## End-to-end signal map

All DDS and ROS 2 participants below use domain 0.

| Signal | Publisher -> subscriber | Topic/type or protocol | Effective rates |
|---|---|---|---|
| H2 body/arm state | H2 `.123.161` -> teleop `.123.222` | `rt/lowstate`, Unitree `LowState_` | H2 source observed near 1 kHz; workstation checks every 2 ms; recorder samples the latest value at 30 Hz |
| H2 arm/head command | teleop `.123.222` -> H2 `.123.161` | With `--motion`: `rt/arm_sdk`, Unitree `LowCmd_`; otherwise `rt/lowcmd` | IK creates a new target at 30 Hz; the low-level writer repeats the latest target at 250 Hz |
| H2 locomotion | workstation Unitree `LocoClient` -> H2 | Unitree generated DDS request/reply topics | Called from the 30 Hz teleop loop |
| Sharpa desired joints | Avatar bridge `.125.222` -> Thor bridge `.125.163`; recorder `.123.222` also subscribes passively | `rt/sharpa/left/cmd` and `rt/sharpa/right/cmd`, `HandCmd_`, 22 radians | New glove targets at 30 Hz; Thor repeats the latest physical-hand write at 50 Hz |
| Sharpa measured joints | Thor bridge `.125.163` -> Avatar `.125.222` and recorder `.123.222` | `rt/sharpa/left/state` and `rt/sharpa/right/state`, `HandState_`; degrees on wire and radians in the dataset | 30 Hz |
| Legacy tactile | Thor `.125.163:7779` -> teleop and optionally Avatar haptics | ZMQ/TCP custom binary containing deformation, F6, contact points, frame ID, SDK timestamp, and optional 22-joint trailer | About 30 hardware ticks/s; 10 fingertip messages per tick |
| Head image | Thor `.125.163` -> workstation ROS client `.125.222` | `/head/left/image_raw`, `sensor_msgs/msg/Image`, RGB8 640 x 480 | 30 Hz |
| Wrist images | Thor `.125.163` -> workstation ROS client `.125.222` | `/wrist/left/image_raw` and `/wrist/right/image_raw`, `sensor_msgs/msg/Image`, RGB8 640 x 480 | 30 Hz each |
| Camera calibration | Thor -> optional ROS consumers | Corresponding `/camera_info`, `sensor_msgs/msg/CameraInfo` | Published by gscam; current teleop does not subscribe |
| Clock | H2 `.123.161:123/UDP` -> Thor chrony -> workstation | NTP; Thor serves the workstation at `.125.163:123/UDP` | Adaptive clock polling, independent of the 30 Hz data loop |
| Dynamic H2 discovery | H2 SPDP -> helper on Thor -> teleop over SSH | Helper listens on `239.255.0.1:7400/UDP`; result returns over `.125.163:22/TCP` | Once before physical H2 control initialization |
| Avatar gloves | USB receiver `/dev/ttyACM0` -> Avatar SDK | USB serial; CLI `--transport wireless` maps to `usb_serial` | SDK-internal input rate; DDS output is limited to 30 Hz |
| XR arm input | PICO/Vuer session -> teleop process | Web/Vuer input, outside robot DDS | Sampled by teleop at 30 Hz |

“Everything at 30 Hz” refers to new hand actions, recorded states, camera frames,
legacy tactile ticks, and dataset rows. The 250 Hz arm writer and 50 Hz hand
writer are internal repeat loops: they resend the most recent 30 Hz target for
control continuity and do not create additional dataset actions.

## Port classes

### Fixed application ports

| Port | Use |
|---:|---|
| `22/TCP` | Workstation SSH to Thor; also transports the startup discovery result |
| `123/UDP` | H2 -> Thor -> workstation NTP chain |
| `7779/TCP` | Current 30 Hz Sharpa tactile ZMQ publisher |
| `7400/UDP` on multicast group `239.255.0.1` | DDS domain-0 SPDP discovery |

### Discovered H2 ports

The two five-digit H2 metatraffic locators are OS/DDS assigned. The helper
extracts the advertised `PID_METATRAFFIC_UNICAST_LOCATOR` for both required
processes before any robot controller is initialized.

At the last verification they were:

| H2 process | Current example only |
|---|---:|
| `basic_service` | `192.168.123.161:59905` |
| `humanoid` | `192.168.123.161:34339` |

Earlier H2 boots advertised different values. Never copy these examples into a
command. If either process is missing or the result is not from `.123.161`,
teleop exits before it creates the robot controller. Operationally, exactly one
live participant with each process name is required; if duplicate same-named H2
participants are ever intentionally started, stop the duplicate before teleop.

### DDS-negotiated ports

Camera DDS, hand DDS, and DDS user-data locators do not have fixed application
ports in these scripts. CycloneDDS advertises its metatraffic and user-data
locators during discovery. The Thor firewall therefore permits all IP protocols
and ports only for this exact pair:

```text
192.168.123.222 <-> 192.168.123.161
```

It does not open general forwarding between the two networks.

### Physical Sharpa ports

The `.124` addresses and left/right routes are fixed, but the vendor Sharpa SDK
learns session ports from its discovery/heartbeat exchange. Do not copy the
Avatar glove JSON ports (`50020`, `50030`, `50002`, or `50012`) into the Wave
hand topology: those values belong to Avatar's optional Ethernet glove
transport and are inactive under the current USB-serial glove command.

## Multicast boundary

Generic multicast is intentionally not forwarded between `.123` and `.125`.

- Cameras and Avatar/Thor hand DDS communicate on the same `.125` Ethernet.
- H2 cross-network discovery is converted into explicit unicast peers at
  startup.
- H2 DDS data then follows advertised unicast locators.
- The Unitree participant uses `AllowMulticast=spdp` plus explicit peers.

A future participant that relies only on multicast discovery across
`.123 <-> .125` will still fail by design. Give it explicit peers or deploy a
deliberate DDS router. Blanket multicast forwarding was not added because it
would enlarge the traffic and failure boundary.

## Camera DDS configuration

Thor's camera launcher now forces:

- publisher address `192.168.125.163`;
- workstation peer `192.168.125.222`;
- `ROS_DOMAIN_ID=0`;
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`;
- DDS maximum message fragment size 1400 bytes;
- 16 MiB DDS socket receive buffer;
- no fallback to automatic interface selection.

The workstation ROS image client separately forces domain 0, local address
`.125.222`, and Thor peer `.125.163`. Its image subscriptions are best-effort,
keep-last depth 2. The Unitree control/hand participant remains independently
bound to `.123.222`, so creating the camera participant cannot silently move
body control onto the wrong address.

## Where recording is saved

The active 30 Hz path saves the episode on the workstation, not Thor:

```text
<TASK_DIR>/<TASK_NAME>/episode_NNNN/
  data.json
  colors/
  depths/
  audios/
  tactiles/
```

Images are JPEG files under `colors/`. Each deformation map is a lossless PNG
under `tactiles/`. Scalar/vector values and relative file paths are stored in
`data.json`.

Each recorded row now contains the following timing evidence:

- row sampling time on workstation monotonic and realtime clocks;
- each camera's ROS header stamp and workstation receive monotonic/realtime;
- H2 LowState workstation receive times, opaque `unitree_tick`, and source
  sequence when present;
- H2 target-set time and the latest successful DDS `Write` time;
- Sharpa measured-state receive times per side;
- Sharpa desired-command receive times per side;
- every fingertip's SDK `ts`, `frame_id`, and workstation receive times;
- `joint_qpos` in radians inside a tactile entry when the bridge supplied the
  aligned 22-joint trailer.

The actual glove target observed on DDS is additive data at:

```text
actions.left_ee.desired_qpos
actions.right_ee.desired_qpos
timestamps.sharpa_hand_desired_command.left
timestamps.sharpa_hand_desired_command.right
```

For compatibility, legacy `actions.*_ee.qpos` remains the shifted
measured-state proxy. It is not relabeled as the desired command.

Timestamp interpretation remains deliberately honest:

- camera header time is Thor ROS publisher time, not hardware exposure time;
- `unitree_tick` is preserved without assuming its unit or epoch;
- Sharpa SDK `ts` is preserved without assuming it is an NTP timestamp;
- arm publish time means successful workstation DDS `Write`, not H2 application;
- desired hand time proves what the recorder observed, not what Thor applied;
- monotonic clocks are comparable only within one host boot;
- this is timestamped asynchronous latest-sample recording, not hardware-triggered
  synchronization.

After the reboot test, Thor reported approximately 20 microseconds offset from
the H2 source and the workstation reported approximately 0.1 millisecond offset
from Thor. That is suitable NTP-quality alignment for the current 30 Hz pipeline,
but it is not PTP or hardware timestamping.

## Normal startup commands

Run these in four terminals.

### 1. Thor cameras

```bash
cd ~/dev/sensing_wrist_gstreamer
./scripts/run_all_3cam.sh
```

### 2. Thor hand bridge and legacy tactile

```bash
cd ~/Sharpa_Haochen/new_bridge_dds_new
./sharpa_dds_bridge \
  --side both \
  --state-hz 30 \
  --dds-interface enx80691a14d263 \
  --tactile-port 7779
```

Do not add `--tactile` here for the current path. That option keeps the separate
device JPEG stream enabled; the bridge's ZMQ deformation/F6 path on port 7779
already runs without it.

### 3. Workstation Avatar glove control

For hand control plus glove force feedback:

```bash
conda activate tv
cd ~/Projects_Haochen/xr_teleoperate/teleop
python avatar_hand_dds_bridge.py \
  --side both \
  --transport wireless \
  --rate-hz 30
```

For recording with less duplicate traffic and no glove force feedback, add
`--no-haptics`. It does not disable glove-to-hand control and does not disable
teleop's tactile recording:

```bash
python avatar_hand_dds_bridge.py \
  --side both \
  --transport wireless \
  --rate-hz 30 \
  --no-haptics
```

### 4. Workstation arm teleop and recording

```bash
conda activate tv
cd ~/Projects_Haochen/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --frequency 30 \
  --arm H2 \
  --ee sharpa \
  --motion \
  --camera-source ros \
  --record \
  --task-name "sharpa_try" \
  --arm-side both \
  --head-mode mono \
  --task-dir ~/Projects_Haochen/datasets_H2_sharpa
```

Do not pass `--body-dds-peer`: the two current H2 endpoints are discovered
automatically. The remaining network defaults already select:

```text
body/control local address: 192.168.123.222
camera local address:       192.168.125.222
Thor camera/hand peer:      192.168.125.163
legacy tactile:             192.168.125.163:7779
```

In the teleop terminal, use the existing interaction sequence:

1. Press `c` to calibrate.
2. Press `r` to start arm synchronization.
3. Press `s` to start an episode and `s` again to save it.
4. Press `q` to stop safely.

Do not run the standalone camera visualizer during normal recording unless it is
needed for diagnosis: it creates another full camera subscriber. Likewise,
Avatar haptics creates a second full port-7779 tactile subscriber. The three raw
camera streams carry about 664 Mb/s of image payload; one full tactile subscriber
adds roughly 138 Mb/s of deformation payload. The 2.5 Gb/s link is not saturated,
but extra subscribers add substantial serialization, fragmentation, TCP, and CPU
load.

## Source-controlled deployment files

- Thor install destinations, commands, and reboot acceptance checks:
  [`../deploy/thor/README.md`](../deploy/thor/README.md)
- Thor persistent USB Ethernet installer:
  `teleop/deploy/thor/install_thor_usb_ethernet.sh`
- Thor H2 route script: `teleop/deploy/thor/h2-dds-routing`
- Thor H2 route service: `teleop/deploy/thor/h2-dds-routing.service`
- Thor H2 chrony source: `teleop/deploy/thor/50-h2-source.conf`
- Thor chrony client allow rule: `teleop/deploy/thor/60-h2-workstation.conf`
- Workstation timesyncd configuration: `teleop/deploy/workstation/60-h2-thor.conf`
- Dynamic H2 discovery helper: `teleop/utils/discover_h2_dds_peer.py`
- Thor camera CycloneDDS configuration:
  `../sensing_h2_ros_gstreamer/config/cyclonedds/thor_125.xml`
- Thor camera launcher:
  `../sensing_h2_ros_gstreamer/scripts/run_all_3cam.sh`
