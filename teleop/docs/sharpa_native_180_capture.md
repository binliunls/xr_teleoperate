# H2 teleop with Thor-local Sharpa 180 Hz capture

This mode keeps the existing H2 arm, hand, camera, and 30 Hz episode pipeline,
and adds a loss-sensitive native-rate Sharpa sidecar on Thor. The workstation
sends only low-bandwidth episode-control and alignment markers. It does not
stream, decode, or resample the 180 Hz tactile payload while teleop is running.

## What runs where

| Component | Host | Purpose |
|---|---|---|
| `sharpa180-recorder` | Thor | Records native tactile and hand telemetry, serves capture control on port 48010, and publishes the legacy tactile view on port 7779 |
| `sharpa180` | Thor | Runs the Sharpa 5.0.1 tactile publisher at a nominal 180 Hz per tactile channel |
| `sharpa_dds_bridge` | Thor | Applies DDS hand commands and emits desired/applied/measured hand telemetry |
| Avatar glove bridge | Workstation | Publishes the same 30 Hz DDS hand commands and consumes the same port-7779 haptic feedback used before native capture was added |
| `teleop_hand_and_arm.py` | Workstation | Runs H2 teleop and writes the existing episode at 30 Hz, plus capture identifiers and alignment markers |

The complete Thor placement and provisioning procedure is documented in the
paired `sharpa-teleop` repository at `thor/deploy/README.md`; the runtime and
capture details are in `thor/docker_5_0_1/README.md` and the deployed Thor copy
at `/home/unitree/sharpa180/README.md`. The runtime commands below assume that
setup has already been completed.

The compatible Thor-side implementation is `sharpa-teleop` commit
`03ef34376daff2ba4f3fd332bc387ce50a515011` on branch
`haochens/180hz-tactile-capture`. Keep that revision paired with this
workstation branch when deploying or reviewing the capture path.

## Start the services

Use separate terminals and keep each foreground process running. Start them in
this order so the recorder is listening before either high-rate source begins.

### 1. Native recorder on Thor

```bash
ssh unitree@192.168.125.163
docker start -a sharpa180-recorder
```

The recorder uses these endpoints:

- capture control: `tcp://192.168.125.163:48010`
- 30 Hz compatibility tactile PUB: `tcp://192.168.125.163:7779`
- native tactile input from the Docker publisher: `tcp://127.0.0.1:48008`
- native hand telemetry input from the DDS bridge: `tcp://127.0.0.1:48011`
- persistent host storage: `/home/unitree/sharpa_recordings`

### 2. Native tactile publisher on Thor

Make sure the long-lived SDK container is running, then start its publisher:

```bash
docker start sharpa180
docker exec -it sharpa180 bash -lc \
    'source /root/.bashrc && cd /workspace && exec python3 tactile_zmq_180.py --config /workspace/config/tactile_180_dual.json'
```

This is the patched, loss-detectable publisher mounted from the Thor host. Do
not substitute the earlier `/root/docker_tactile_zmq_pub/tactile_zmq.py`
prototype; that process does not implement this recorder's metadata/barrier
contract.

Before starting the hand bridge or haptics, keep both hands motionless with
every fingertip completely unloaded and tare the host-inference session:

```bash
docker exec -w /workspace sharpa180 python3 calibrate_sharpa_tactile.py
```

Both replies must contain `"ok": true` and `"result": true`. The tare is
process-local and must be repeated whenever `tactile_zmq_180.py` restarts.
Never run it while a fingertip is touching an object.

### 3. DDS hand bridge on Thor

```bash
cd ~/Sharpa_Haochen/new_bridge_dds_native_capture_v1
./sharpa_dds_bridge --side both --state-hz 30 \
    --dds-interface enx80691a14d263 \
    --external-tactile --tactile-port 0 --telemetry-port 48011
```

`--external-tactile` leaves the tactile devices and native stream to the Docker
publisher. Therefore this bridge must not also publish tactile on port 7779;
the recorder provides that compatibility endpoint instead.

The bridge still owns hand control. It subscribes to
`rt/sharpa/{left,right}/cmd`, applies commands at 50 Hz, and publishes
`rt/sharpa/{left,right}/state` at the established 30 Hz. Its SHT1 telemetry also
contains event-driven desired commands, 50 Hz applied-command attempts, and
30 Hz measured states. DDS remains pinned to Thor's `.125` USB interface,
matching the normal bridge across runs and reboots.

### 4. Avatar glove-to-Sharpa bridge on the workstation

```bash
conda activate tv
cd ~/Projects_Haochen/xr_teleoperate/teleop
python avatar_hand_dds_bridge.py --side both --transport wireless --rate-hz 30
```

This is the same hand-control and haptic path as before: the Avatar bridge
publishes radians on the Sharpa DDS command topics, consumes F6 from port 7779,
and the Thor bridge drives the hands. Do not add `--no-haptics`.

### 5. Teleop and episode recording on the workstation

```bash
conda activate tv
cd /home/haochen/Projects_Haochen/xr_teleoperate
bash teleop/run_teleop.sh TASK --sharpa-native-capture
```

Replace `TASK` with the dataset folder name. The flag without an address uses
`tcp://192.168.125.163:48010`. It also derives the compatibility tactile source
as `tcp://192.168.125.163:7779`. An explicit control endpoint remains possible:

```bash
bash teleop/run_teleop.sh TASK \
    --sharpa-native-capture tcp://192.168.125.163:48010
```

Any additional teleop arguments are forwarded unchanged. For example,
`--frequency 30` can be supplied explicitly, although 30 Hz is already the
default.

Start and stop an episode with the existing `s` key or Y-button workflow. A
native START acknowledgement is required before the local episode is created.
On stop, local serialization and Thor finalization run independently; another
episode cannot start until both have completed.

## Meaning of 30, 50, and 180 Hz

These rates describe different streams and should not be forced onto one
timeline during live teleop.

| Rate | Stream and semantics |
|---|---|
| 30 Hz | The workstation teleop loop and episode rows: arm/body state and action, hand feedback, camera snapshots, and the latest cached legacy tactile sample. This is controlled by `--frequency` and remains the existing dataset format. |
| 50 Hz | The bridge's internal SDK command-application rate and exact applied-attempt sidecar. It does not change the 30 Hz DDS state/episode rate. |
| 180 Hz | The nominal rate of each of the 10 tactile channels, five per hand. It is not an aggregate 180 Hz divided among the fingers. Actual counts, rates, and gaps must be read from the finalized manifest. |

`--state-hz 30` preserves the established measured-state stream; the internal
write-attempt stream remains independently 50 Hz. Port 7779 is also still intentional. It preserves the
existing per-row tactile fields in `data.json`; it is a latest-cache,
compatibility/downsampled view and is not the authoritative 180 Hz recording.
The authoritative tactile stream is stored directly on Thor.

The compatibility relay appends the same optional 22-float measured-joint
trailer as the established bridge, so existing per-fingertip `joint_qpos`
fields remain available. The 30 Hz wire/schema, port, subscriber commands, and
haptics are preserved. Its DEFORM values come from the CUDA inference path in
native mode rather than the hand's prior on-device path; no numerical
byte-equality claim is made between two different inference owners.

The Docker publisher sends serialized `proto.Tactile` messages unchanged to
the recorder. The recorder writes those protobuf bytes to manifest-declared
`.shc` chunks. There is no live interpolation, frame duplication, or 6:1
matching between the 30 Hz rows and tactile frames.

## Data and units

The original workstation episode remains under:

```text
~/datasets/TASK/episode_NNNN/
```

It still contains `data.json`, camera images, and the legacy tactile snapshots.
Native mode adds `sharpa_native_capture.json`, which identifies the matching
Thor capture and records control-clock evidence.

The relevant units are:

- workstation arm, body, and hand `qpos`/`action`: radians;
- DDS desired hand command and SHT1 desired/applied command: radians;
- DDS and SHT1 measured Sharpa hand state: degrees at the bridge; the
  workstation hand controller converts its local state to radians;
- tactile deformation: processed `240 x 240` `uint8`, unitless deformation
  values;
- F6: six `float32` values, with the first three representing force and the
  next three torque in the SDK's native numeric units;
- contact point: SDK-native `float32` values/coordinates;
- native recorder, SHT1, and workstation alignment timestamps: nanoseconds;
- legacy tactile SDK `ts`: seconds.

The recorder preserves F6 and contact-point values without calibration or unit
conversion. Do not assign SI units downstream unless the matching SDK/device
calibration establishes them.

One existing schema detail matters for learning pipelines: the workstation
Sharpa hand `action[t]` is a feedback proxy populated from `state[t+1]`. It is
retained unchanged for compatibility. Each row also records the latest desired
DDS command observed by the recorder under
`actions.{left,right}_ee.desired_qpos`, with per-side workstation receive times
under `timestamps.sharpa_hand_desired_command`. This proves what the recorder
observed on `rt/sharpa/{left,right}/cmd`; it does not prove that Thor applied
that command. Use the native SHT1 applied stream when application matters.

## Timestamp and alignment contract

Each workstation `data.json` row contains:

- `timestamps.workstation_monotonic_ns` and
  `timestamps.workstation_realtime_ns`;
- per-camera timing under `timestamps.cameras`;
- H2 LowState, arm command, Sharpa state/action-proxy, desired-command, and
  legacy tactile receive timing in their explicitly named timestamp fields;
- `native_capture.protocol`, `native_capture.capture_id`, and the exact row
  `native_capture.idx`.

For ROS cameras, the ROS header stamp is preserved separately from workstation
receive monotonic/realtime time. A ROS header clock is not assumed to be
synchronized with either workstation clock. For ZMQ cameras, only workstation
receive times are available, so no source timestamp is invented.

For every local row, teleop sends a SAMPLE marker with the same capture UUID,
row index, and workstation timestamps. Thor records these markers in
`clock_samples.jsonl` together with Thor receive `CLOCK_MONOTONIC_RAW` and
`CLOCK_REALTIME` timestamps. SHT1 events also retain their intrinsic monotonic
and realtime timestamps. START, STOP, and downsampled SAMPLE requests retain
four-point workstation/Thor clock exchanges in the local capture metadata.

An offline reader should align the streams by capture UUID and row-index
markers, using the clock anchors to map between hosts. It should retain the
native event times and select an explicit offline policy such as nearest,
windowed, or interpolation only for the downstream model that needs it. It
must not assume exact 30:180 divisibility, exact 6:1 correspondence, or a shared
ROS/workstation/Thor monotonic clock.

## Finalize, fetch, and validate

Thor writes the native capture under:

```text
/home/unitree/sharpa_recordings/<capture_id>
```

This host path is the SCP source of truth. `/recordings/...` is the container
mount path and must not be used from the workstation.

After the episode has stopped and both save/finalize operations have finished,
fetch it manually:

```bash
conda activate tv
cd /home/haochen/Projects_Haochen/xr_teleoperate
python -m teleop.utils.fetch_sharpa_native_capture \
    ~/datasets/TASK/episode_NNNN
```

The command fetches from `unitree@192.168.125.163`, verifies the authoritative
manifest and hashes, rejects partial/unlisted files and unsafe paths, and then
atomically installs:

```text
~/datasets/TASK/episode_NNNN/sharpa_native_capture/
```

It refuses to overwrite an existing installed capture.

`sharpa_native_capture.json` is coordination/reference metadata. Its finalized
STOP summary is authoritative (`response_valid_is_preliminary=false`) because
Thor closes and fsyncs the clock log and manifest before replying. It is still
not sufficient integrity proof by itself: fetching and verifying the manifest,
chunks, and hashes is required. Accept an episode only when the manifest is
finalized and valid, contains no invalid reasons, passes all chunk/clock-file
hashes, and has acceptable actual per-stream counts, rates, and gap evidence.
The current recorder validity gates also require at least 150 Hz for each
tactile channel and at least 20 Hz for each periodic hand-telemetry stream;
these are health thresholds, not claimed recording rates.

START is rejected if tactile or SHT1 input is not fresh. A dropped SAMPLE,
control failure, queue overflow, source gap, writer error, or failed Thor
validation marks the capture degraded/invalid. Do not silently fall back to
the 30 Hz compatibility tactile as if it were the 180 Hz stream.

## Downstream pipeline rule

Keep the 30 Hz episode and the fetched native capture as two linked artifacts.
Future conversion or training code should join them by `capture_id` and SAMPLE
`idx`, expose the SHT1 desired/applied/measured distinction, and decode tactile
chunks offline. This preserves the old teleop dataset unchanged while allowing
a future pipeline to choose its own tactile windowing and alignment policy
without having discarded native frames or hand-control provenance.
