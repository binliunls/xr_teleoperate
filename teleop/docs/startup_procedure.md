# H2 + Sharpa Teleop — Startup Procedure

End-to-end checklist for bringing the system up from cold. Each step is small
so we can pause/resume at any point.

## 1. Power on the robot

Short-press the power button on the robot, then long-press to turn it on.

## 2. Start the camera services on the robot (Thor)

SSH to Thor as `unitree@192.168.123.163`, then:

```bash
ssh unitree@192.168.123.163
cd ~/dev/sensing_wrist_gstreamer/scripts

# One-time-style setup script (safe to run on every boot)
./setup_sensing_cameras_boot.sh
# When it asks:
#   Apply the one-time nvargus-daemon systemd override now?
#   This restarts nvargus-daemon. [y/N]
# answer: n

# Start all four camera services (head left/right + wrist left/right)
./run_all.sh
```

After `run_all.sh` returns, the four ROS image topics should be publishing:

- `/head/left/image_raw`
- `/head/right/image_raw`
- `/wrist/left/image_raw`
- `/wrist/right/image_raw`

Quick verification from the workstation:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /head/left/image_raw   # expect ~30 Hz
```

## 3. Verify camera streams from the workstation (x86)

Run the visualizer to confirm all four cameras are publishing:

```bash
conda activate tv
cd ~/jeleong/sensing_h2_ros_gstreamer/scripts
./visualize_camera_streams.py
```

A tiled OpenCV window should appear showing all four streams (head left, head
right, wrist left, wrist right) with FPS counters. Press `q` or Esc to close.

If any panel is black or missing, go back to Thor and check the matching
service in `~/dev/sensing_wrist_gstreamer/scripts` (one of the
`run_gscam2_*.sh` scripts) before continuing.

## 4. Start the Sharpa hand bridge on Thor

Back on Thor, start the C++ DDS bridge that owns the Sharpa hands via the SDK
and exposes them on DDS topics for the workstation:

```bash
ssh unitree@192.168.123.163
cd ~/Sharpa/bridge_dds
./sharpa_dds_bridge --side both
```

Watch for the per-hand startup log lines:

```
[left] tactile JPEG stream disabled
[left] hand ready.
[right] tactile JPEG stream disabled
[right] hand ready.
Bridge running. Ctrl+C to stop.
```

The bridge publishes `rt/sharpa/{left,right}/state` and subscribes to
`rt/sharpa/{left,right}/cmd` on DDS domain 0. Leave it running.

If the second hand fails to connect (TCP timeout / "Hand not ready"), the
100 Mb link is being saturated by tactile JPEG traffic. See
`teleop/docs/sharpa_hands_thor.md` for the power-on-one-at-a-time workaround.

## 5. Bring the robot to standing

Use the wireless controller in the following order:

| Step | Buttons   | Result                                |
|------|-----------|---------------------------------------|
| 5.1  | `L2 + B`  | Damp mode (joints go limp / safe)     |
| 5.2  | `L2 + ↑`  | Preparation mode                      |
| 5.3  | `R2 + Y`  | Stand up                              |

Wait for each transition to complete (the robot's posture stabilises) before
pressing the next combo. After 5.3 the robot is standing on its legs with the
built-in WBC holding balance — ready for arm teleop in motion mode.

## 6. Start teleop on the workstation

```bash
conda activate tv
cd ~/binliu/xr_teleoperate/teleop
bash run_teleop.sh assemble_trocar_05121740
```

`run_teleop.sh` is a thin wrapper that calls:

```bash
python teleop_hand_and_arm.py \
    --arm H2 --ee sharpa --motion \
    --camera-source ros \
    --record --task-dir ~/datasets --task-name "$1"
```

The single argument is the **task / folder name** under `~/datasets/`. Use a
descriptive name that includes the task and a date/time stamp,
e.g. `assemble_trocar_05121740`. Each recorded episode lands under
`~/datasets/<task-name>/episode_NNNN/`.

Watch the log for these checkpoints:
- `🟢 Press [r] on keyboard or [B button] on right controller to start syncing.`
- After `r` / B-button: `🚀 start Tracking 🚀`
- After `s` / Y-button (start recording): `==> New episode created: ...`
- After `s` / Y-button (save): `==> Episode saved successfully to .../data.json`

Key bindings during teleop:

| Action                              | Keyboard | VR controller (right) |
|-------------------------------------|----------|-----------------------|
| Start tracking (enables arm motion) | `r`      | B button              |
| Start / save a recorded episode     | `s`      | Y button (left ctrl)  |
| Stop and exit                       | `q`      | A button              |

Leave the teleop program running and have the operator standing by.

## 7. Connect the PICO headset (two-pass dance)

The PICO does not work on the first try after a cold boot — there is a known
quirk where the local `192.168.x.x` URL has to be visited once to "wake up"
the connection before the public `vuer.ai` URL becomes usable.

1. **Power on the PICO** and put it on.
2. In the PICO browser, navigate to the **`192.168.x.x:...` URL** printed by
   teleop (or whichever local address televuer advertises). This triggers
   the initial connection handshake.
3. **Close the PICO browser page** and **stop the teleop program** (`q` or
   Ctrl-C) on the workstation.
4. **Restart teleop** with the same command as in step 6:
   ```bash
   bash run_teleop.sh assemble_trocar_05121740
   ```
5. In the PICO browser, navigate to the **`https://vuer.ai/...` URL**
   instead. This time the session connects properly and hand / controller
   tracking starts streaming.

The operator can now press `r` (B button) to start tracking and begin the
demonstration.

## 8. Calibrate the operator's pose

Have the operator assume the **preparation posture** (arms in the neutral
"reference" position the system expects to map onto the robot's home pose),
then press **`c`** to run the calibration. This aligns the VR headset / hand
tracking origin with the robot's frame so subsequent motions are correctly
referenced.

Hold the prep pose steady until the calibration completes, then the operator
is free to move.

## 9. Start tracking and bring up the hand retargeting daemon

1. In the teleop terminal (or via VR controller B button), press **`r`** to
   start tracking. The robot's arms will now follow the operator's wrists.

2. In a **separate terminal** on the workstation, start the Manus → Sharpa
   retargeting daemon that publishes hand commands over DDS:
   ```bash
   conda activate tv
   cd ~/Sharpa/retargeting_alg_release_V4.0
   python retargeting_manus_demo_dds.py --side both
   ```

   This subscribes to Manus mocap on `tcp://localhost:2044`, retargets each
   frame to 22 Sharpa joint angles per hand, and publishes them to
   `rt/sharpa/{left,right}/cmd`. The bridge from step 4 receives those
   commands and drives the hands.

   Watch for `[diag] mocap_frames_seen=N left_pushed=N right_pushed=N` log
   lines confirming end-to-end flow. If `mocap_frames_seen=0`, the Manus
   software on the workstation is not publishing — check that side first.

## 10. Convert Unitree dataset to LeRobot

Once you have one or more recorded datasets under `~/datasets/`, convert them
to the LeRobot format. The converter reads everything under `~/datasets/` and
writes the result to `~/lerobot/`, so make sure **both** `~/datasets/` (the
intended source) and `~/lerobot/` are in the expected state first — typically
`~/lerobot/` should be empty (or removed) before each run so a clean dataset
is written.

```bash
conda activate tv
cd ~/binliu/unitree_lerobot
# (optional) wipe previous output:
rm -rf ~/lerobot
bash convert.sh
```

Result: a v2.x LeRobot dataset at `~/lerobot/` with `data/`, `images/`,
`videos/`, and `meta/` subdirectories.

If you want waist (`body.qpos`) included in state only, edit `convert.sh` to
pass `--include-waist-state`.

## 11. Upload converted dataset to Hugging Face

Push the LeRobot dataset under a named sub-folder of a Hub repo:

```bash
python -m unitree_lerobot.utils.upload_lerobot_dataset \
    --local-path ~/lerobot \
    --repo-id nvidia/orca-template1-dev \
    --path-in-repo <your_dataset_name>
```

Pick `<your_dataset_name>` to be descriptive and unique, e.g.
`0512_1751_liming_collection`. The sub-folder is created automatically on
the Hub if it doesn't exist; re-running uploads only new/changed files.

Auth: pass `--token`, or set `$HF_TOKEN`, or `huggingface-cli login` once.

## 12. Download a trained model from Hugging Face

```bash
conda activate tv
cd ~/binliu/ORCA-Galbot/utils
python hf_download.py nvidia/orca-h2-dev \
    --subdir assemble_trocar \
    --exclude "*/global*/*" \
    --exclude "*/*/global*/*" \
    --local-dir /home/nvidia/models/
```

Replace `--subdir` with the model bundle you want. The two `--exclude`
patterns skip checkpoint shards under a `global*` folder (LFS-heavy and
not needed for inference).

## 13. Start the GR00T policy server

GR00T runs in its own uv-managed environment. In a fresh terminal:

```bash
cd ~/binliu/Isaac-GR00T
uv run python gr00t/eval/run_gr00t_server.py \
    --model-path /home/nvidia/models/assemble_trocar/0512-101data-bs256-tune-visual-checkpoint-29400/ \
    --embodiment-tag new_embodiment \
    --port 5555
```

Adjust `--model-path` to whichever checkpoint you downloaded in step 12.
The server listens on `tcp://localhost:5555` over ZMQ + msgpack and stays up
until you Ctrl-C.

## 14. Run inference

With the bridge, cameras, and GR00T server already up, launch the inference
wrapper from `unitree_lerobot`:

```bash
cd ~/binliu/unitree_lerobot/unitree_lerobot/eval_robot
bash run_eval_h2_groot.sh "assemble_the_trocar_and_place_it_on_the_table." 640
```

Two positional args: **task description** and **total steps per episode**
(here, 640 frames at 30 Hz ≈ 21 s per episode).

If you have an init-posture YAML, point the wrapper at it so each episode
starts from the recorded average starting pose:

```bash
bash run_eval_h2_groot.sh "assemble_the_trocar_and_place_it_on_the_table." 640 \
    --init-state-yaml <your_init_file> \
    --init-state-hold-s 5
```

Alternatively, set the env var `INIT_STATE_YAML=<path>` once and the
wrapper picks it up automatically.

## 15. Generate an init-posture YAML (one-time, optional)

To build an init-posture file from your recorded training data — averages
the first N frames of `states.*.qpos` across every episode in the given
dataset(s):

```bash
bash teleop/utils/run_compute_average_initial_state.sh \
    ~/Assemble_trocar_h2_sharpa/assemble_trocar_05121740 \
    ~/Assemble_trocar_h2_sharpa/assemble_trocar_05121948 \
    -- --first-n 3 \
       --output ~/binliu/unitree_lerobot/unitree_lerobot/eval_robot/init_post_0512.yaml
```

Pass as many dataset directories as you want before the `--` separator —
they are pooled into a single average. `--first-n 3` says "use the first 3
frames of every episode." Plug the resulting YAML into step 14 with
`--init-state-yaml`.
