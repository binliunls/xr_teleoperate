# Reproduce the H2 + Sharpa workstation

This guide reproduces the verified workstation side of the H2 teleoperation
stack with ROS cameras, 30 Hz episode recording, Avatar glove hand control and
haptics, and the linked Thor-local 180 Hz Sharpa tactile capture.

It assumes the existing Thor has already been configured with the companion
`sharpa-teleop` native-capture implementation and the camera publisher. A new
Thor requires the separate Thor deployment described in
[`sharpa_native_180_capture.md`](sharpa_native_180_capture.md).

## Required repositories

### Workstation teleop

Clone or update this repository and select the feature branch:

```bash
cd ~/Projects_Haochen
git clone https://github.com/binliunls/xr_teleoperate.git
cd xr_teleoperate
git fetch origin
git switch --track origin/haochens/180hz-tactile-capture
git submodule update --init --recursive
```

If the branch already exists locally, use:

```bash
git switch haochens/180hz-tactile-capture
git pull --ff-only
git submodule update --init --recursive
```

Install the `teleimager` and `televuer` submodules as described in the root
[`README.md`](../../README.md). `teleimager` is imported at program startup
even when `--camera-source ros` is selected.

### Camera repository

The camera publisher and standalone visualizer belong to a separate repository:

```text
https://gitlab-master.nvidia.com/jeleong/sensing_h2_ros_gstreamer.git
```

The workstation teleop process does not import that repository. It constructs
its own exact CycloneDDS camera configuration. A workstation checkout is needed
only for standalone camera visualization and diagnostics.

For the verified `.125` camera setup, ensure the camera checkout contains:

```text
config/cyclonedds/thor_125.xml
config/cyclonedds/workstation_125.xml
scripts/run_all_3cam.sh
scripts/run_visualize_3cam.sh
scripts/visualize_camera_streams.py
```

The first two Thor-side files must also be present in the camera checkout used
on Thor. These files are not supplied by `xr_teleoperate`; copying or cloning
only this repository cannot reproduce the camera publisher.

### Thor native-capture repository

The `sharpa-teleop` repository is not a workstation runtime dependency when the
same Thor remains deployed. Its compatible native-capture implementation is:

```text
repository: https://github.com/isaac-for-healthcare/sharpa-teleop.git
branch:     haochens/180hz-tactile-capture
commit:     cdd8985bf64a24be95efc13c8b1d9b0031a93bb2
```

It must be deployed on Thor if Thor is being provisioned from scratch.

## Workstation software

Recreate the `tv` environment and the dependencies from the root installation
guide. The verified H2 path additionally requires:

- ROS 2 Humble and `rmw_cyclonedds_cpp`;
- `unitree_sdk2py`;
- the Avatar SDK installed under `/opt/avatar-sdk`;
- the Avatar USB/udev setup and access to the glove receiver;
- OpenCV and ROS image messages for the ROS camera client;
- working TeleVuer certificates;
- Git submodules initialized and installed.

Before using the robot, verify that both gloves are discovered and that the
Avatar bridge can open the USB receiver. Do not assume that copying the Python
repository also installs the vendor SDK or udev rules.

## Dedicated Ethernet configuration

Only one workstation may use the following addresses on the robot link at a
time. Disconnect the previous workstation before connecting another machine.

Configure the USB/Ethernet adapter with these persistent addresses:

```text
192.168.125.222/24   Thor SSH, cameras, Sharpa DDS and tactile control
192.168.123.222/24   local source address for H2 body DDS
```

Add this persistent host route:

```text
destination: 192.168.123.161/32
gateway:     192.168.125.163
source:      192.168.123.222
```

`192.168.124.222` was present on the original workstation but is unused by the
verified path and is not required. Adding only `192.168.125.222` is not enough:
physical H2 control initializes a separate DDS participant bound explicitly to
`192.168.123.222`.

For a temporary, non-persistent check, replace `ROBOT_NIC` with the adapter's
actual interface name:

```bash
sudo ip address replace 192.168.125.222/24 dev ROBOT_NIC
sudo ip address replace 192.168.123.222/24 dev ROBOT_NIC
sudo ip route replace 192.168.123.161/32 \
  via 192.168.125.163 dev ROBOT_NIC src 192.168.123.222
```

Make the equivalent addresses and route persistent using NetworkManager or the
new workstation's normal network manager. Do not rely on the temporary `ip`
commands across a reboot.

Verify the result:

```bash
ip -4 address show dev ROBOT_NIC
ip route get 192.168.123.161 from 192.168.123.222
ping -c 2 192.168.125.163
```

The route lookup must use Thor `.125.163` as the gateway and `.123.222` as the
source.

## CycloneDDS receive buffers

The committed CycloneDDS configurations require a 16 MiB receive buffer. Add a
persistent sysctl file on the workstation:

```bash
sudo tee /etc/sysctl.d/60-ros-image-streams.conf >/dev/null <<'EOF'
net.core.rmem_max=16777216
net.core.rmem_default=16777216
EOF
sudo sysctl --system
```

Verify both values before starting ROS:

```bash
sysctl net.core.rmem_max net.core.rmem_default
```

Without this setting, CycloneDDS may refuse to create a participant instead of
silently using an undersized socket buffer.

## SSH for dynamic H2 discovery and capture fetching

Teleop discovers the H2 body's current DDS ports through Thor at startup. The
helper intentionally uses non-interactive SSH, so passwordless authentication
must work:

```bash
ssh-copy-id unitree@192.168.125.163
ssh -o BatchMode=yes unitree@192.168.125.163 true
```

The same SSH path is used when the finalized native sidecar is fetched from
Thor. Do not hardcode previously observed five-digit DDS ports; they can change
after an H2 reboot.

## Workstation clock synchronization

Install the committed systemd-timesyncd drop-in so the workstation follows
Thor, which in turn follows the H2 controller:

```bash
sudo install -D -m 0644 \
  teleop/deploy/workstation/60-h2-thor.conf \
  /etc/systemd/timesyncd.conf.d/60-h2-thor.conf
sudo systemctl restart systemd-timesyncd
timedatectl timesync-status
```

If the workstation uses another time daemon, configure its equivalent NTP
source as `192.168.125.163` and avoid running two competing time clients. Clock
synchronization is required for useful cross-host realtime alignment; the
capture also retains monotonic clocks and four-timestamp alignment exchanges.

## Acceptance checks

Before teleoperation, confirm all of the following:

1. `192.168.125.222` and `192.168.123.222` are on the dedicated adapter.
2. `192.168.123.161/32` routes through Thor `192.168.125.163`.
3. the DDS receive-buffer sysctls both report `16777216` or greater;
4. passwordless SSH to `unitree@192.168.125.163` succeeds;
5. the workstation clock is synchronized to Thor;
6. ROS 2 Humble can import `rclpy` and `rmw_cyclonedds_cpp` is installed;
7. the Avatar SDK sees both gloves;
8. Thor is already running the camera and native-capture services in the order
   documented in [`sharpa_native_180_capture.md`](sharpa_native_180_capture.md).

For a camera-only diagnostic, run the visualizer from the camera repository:

```bash
./scripts/run_visualize_3cam.sh
```

Do not keep this extra full-rate camera subscriber running during normal data
collection unless it is needed for diagnosis.

## Normal workstation startup

Start the Avatar glove bridge:

```bash
conda activate tv
cd ~/Projects_Haochen/xr_teleoperate/teleop
python avatar_hand_dds_bridge.py \
  --side both --transport wireless --rate-hz 30
```

Do not add `--no-haptics` when glove force feedback is wanted.

Then start teleop and linked recording:

```bash
conda activate tv
cd ~/Projects_Haochen/xr_teleoperate
bash teleop/run_teleop.sh TASK --sharpa-native-capture
```

The workstation episode remains a 30 Hz dataset. The authoritative native
tactile stream remains on Thor until it is finalized and fetched with:

```bash
python -m teleop.utils.fetch_sharpa_native_capture \
  ~/datasets/TASK/episode_NNNN
```

Copying only `xr_teleoperate`, or adding only the `.125` address, is therefore
not sufficient. Reproduction requires the software environment, both local DDS
addresses, the H2 host route, receive-buffer configuration, SSH, clock sync,
and an already-compatible Thor deployment.
