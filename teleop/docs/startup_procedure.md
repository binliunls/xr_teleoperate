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
