# H2 + Sharpa Teleoperation and GR00T N1.7 Deployment

This guide covers runtime operation only. It does not include installation, dependency setup, SDK setup, or driver setup.

The commands assume this simple home-directory layout:

```text
~/xr_teleoperate
~/unitree_lerobot
~/Isaac-GR00T
~/models
~/datasets
```

Adjust the paths directly if your checkouts are stored elsewhere.

## Choose the workflow

- **Teleoperation and data collection:** complete Sections **1, 2, and 3**. Do not run Section 4.
- **Policy deployment and evaluation:** complete Sections **1, 2, and 4**. Do not run Section 3.

> **CRITICAL SAFETY WARNING — CLEAR THE FRONT WORKSPACE**
>
> Starting the teleoperation control script or the real-robot evaluation script causes both robot arms to rise. Before starting either script, move the table and every other object out of the area in front of and within reach of the robot. Do not start until the full arm workspace is clear.
>
> **DAMPING WARNING:** Before pressing `L2+B`, operators must already be securely supporting both robot arms/hands from safe positions. Continue supporting them while entering damping because the arms can drop when active joint support is removed.

## 1. Safety requirements

1. Enter robot modes in this order:
   - *After the robot output "零力矩模式"*
   - While operators securely support both robot arms/hands, press `L2+B`: damping
   - `L2+Up`: preparation pose
   - `R2+Y`: standing/control mode
2. Wait for each transition to finish before continuing.
3. If teleoperation or policy behavior becomes abnormal:
   - Immediately press `Ctrl+C` to stop the active teleoperation or deployment script.
   - Operators must then securely support both robot arms/hands from safe positions and, while continuing to support them, press `L2+B` to enter damping.
   - Damping removes active joint support, so unsupported arms can fall and damage the hands, robot, or surrounding equipment.
   - Keep the robot in damping before debugging the software.
4. Teleoperation and policy evaluation must never run at the same time.

## 2. Start Thor services after every cold boot

### 2.1 Thor terminal A: native DDS cameras

```bash
ssh unitree@192.168.123.163

CAMERA_NETWORK_INTERFACE=eth10 \
CAMERA_DDS_DOMAIN=10 \
  bash ~/setup_h2_thor.sh camera-start
```

> **IMPORTANT — ALL FOUR CAMERA STREAMS ARE REQUIRED**
>
> Keep this terminal running and confirm that it continuously publishes fresh images from all four cameras:

- `head_left`
- `head_right`
- `wrist_left`
- `wrist_right`

> If any stream is missing or not updating, press `Ctrl+C` to stop `camera-start`, then run the same `camera-start` command again. Do not continue to teleoperation or deployment unless all four streams are healthy. If repeated restarts do not recover all four cameras, stop testing and inspect the camera power, data cables, GMSL connections, and physical connectors.

### 2.2 Thor terminal B: Sharpa DDS bridge

```bash
ssh unitree@192.168.123.163

DDS_DOMAIN=0 \
  bash ~/setup_h2_thor.sh bridge-start
```

Keep this terminal running. Confirm that both hands report ready and that there are no TCP timeout errors.

## 3. Teleoperation and data collection

Skip this section during policy evaluation. Do not run the policy server or evaluation client during teleoperation.

### 3.1 Workstation terminal C: MANUS client

```bash
conda activate tv
cd ~/xr_teleoperate
bash setup_h2.sh manus-start
```

Keep the terminal running. Confirm that data from both gloves is published continuously.


### 3.2 Workstation terminal D: MANUS-to-Sharpa retargeting

```bash
conda activate tv
cd ~/xr_teleoperate

NETWORK_INTERFACE=eno1 \
DDS_DOMAIN=0 \
  bash setup_h2.sh hand-retarget
```

Keep this terminal running. Confirm:

- DDS domain is 0.
- Left and right hand state/command messages refresh continuously.
- No haptics options are enabled.

### 3.3 Put the robot into teleoperation mode

With the robot safely supported:

1. Securely support both robot arms/hands, then press `L2+B` for damping while continuing to support them.
2. Press `L2+Up` for preparation mode.
3. Press `R2+Y` for standing/control mode.

### 3.4 Workstation terminal E: start teleoperation and recording

> **WARNING — BOTH ARMS WILL RISE:** Before running the command below, move the table and all other objects away from the front and full reach of the robot. Confirm that no person is inside the arm workspace.

The following example records a PNP trocar task:

```bash
conda activate tv
cd ~/xr_teleoperate/teleop

export AIOHTTP_NOSENDFILE=1
# export DISPLAY="${DISPLAY:-:0}" # if need image server

python teleop_hand_and_arm.py \
  --arm H2 \
  --ee sharpa \
  --motion \
  --sharpa-dds-domain 0 \
  --network-interface eno1 \
  --camera-source dds \
  --camera-dds-domain 10 \
  --camera-watchdog-timeout-s 0.5 \
  --record \
  --task-dir ~/datasets \
  --task-name pnp_trocar \
  --task-goal "Pick up the trocar and place it in the tray." \
  --task-desc "Pick up the trocar and place it in the tray." \
  --task-steps "step1: approach the trocar, step2: grasp the trocar, step3: move to the tray, step4: place and release the trocar."
```

The recording watchdog monitors all four cameras. If any stream stops advancing for more than 0.5 seconds:

- Teleoperation stops.
- The active episode is quarantined as invalid.
- The arm target is held instead of performing a large automatic homing motion.

### 3.5 Connect the PICO headset

Use the workstation IP address that is reachable from the PICO headset. 
1. Start teleoperation and keep it running.
2. In the PICO browser, first open:

   ```text
   https://192.168.5.22:8012
   ```

3. If prompted, select the advanced option and continue to accept the workstation's self-signed certificate. This first page is only used to establish certificate trust; do not enter VR from it.
4. In the PICO browser, open:

   ```text
   https://vuer.ai/?ws=wss://192.168.5.22:8012
   ```

5. Enter Virtual Reality from the `vuer.ai` page.
6. Hold the standard reference pose and press `c` to calibrate.
7. Press keyboard `r` to start tracking.
8. Press keyboard `s` to control start and end of an episode.
9. Press `q` or controller `A` to stop teleoperation.
10. After the script stops, securely support both robot arms/hands and, while continuing to support them, press `L2+B` to enter damping. Keep supporting the arms until the robot is stable.

### 3.7 Convert raw recordings to LeRobot format

Make sure the destination does not contain data that must be preserved. The converter may recreate an existing output directory.

```bash
conda activate tv
cd ~/unitree_lerobot

python unitree_lerobot/utils/convert_unitree_json_to_lerobot.py \
  --raw-dir ~/datasets \
  --repo-id ~/datasets/pnp_trocar_lerobot \
  --robot-type Unitree_H2_Sharpa
```

Verify:

- `data/`, `videos/`, and `meta/` exist.
- The converted episode count matches the valid raw episode count.
- The 58-dimensional layout is left arm 7, right arm 7, left hand 22, right hand 22.

## 4. GR00T N1.7 real-robot deployment

### 4.1 Workstation terminal F: start the GR00T N1.7 policy server

Confirm that the model directory exists:

```bash
test -d ~/models/gr00t-n1.7-h2-sharpa-trocar-ckpt44200
```

Start the server:

```bash
cd ~/Isaac-GR00T
source .venv/bin/activate

CUDA_VISIBLE_DEVICES=1 python gr00t/eval/run_gr00t_server.py \
  --model-path ~/models/gr00t-n1.7-h2-sharpa-trocar-ckpt44200 \
  --embodiment-tag new_embodiment \
  --host 127.0.0.1 \
  --port 5555
```

Keep the terminal running. Confirm that the server loads the checkpoint and listens on `127.0.0.1:5555`.

### 4.2 Put the robot into evaluation mode

With the robot safely supported:

1. Securely support both robot arms/hands, then press `L2+B` for damping while continuing to support them.
2. Press `L2+Up` for preparation mode.
3. Press `R2+Y` for standing/control mode.
4. Confirm that the arms, hands, tool, and task objects have enough clearance.

### 4.3 Workstation terminal G: start GR00T N1.7 evaluation

> **WARNING — BOTH ARMS WILL RISE:** Before running the command below, move the table and all other objects away from the front and full reach of the robot. Confirm that no person is inside the arm workspace.

```bash
conda activate tv
cd ~/unitree_lerobot/unitree_lerobot/eval_robot

test -f "$PWD/init_post_pnp_trocar.yaml"

XR_TELEOPERATE_ROOT=~/xr_teleoperate \
INIT_STATE_YAML="$PWD/init_post_pnp_trocar.yaml" \
bash run_eval_h2_groot.sh \
  "Pick up the trocar and place it in the tray." \
  250 \
  --policy-host localhost \
  --policy-port 5555 \
  --action-horizon 15 \
  --hold-s 12 \
  --init-state-hold-s 5 \
  --dds-domain 0 \
  --image-source dds \
  --camera-dds-domain 10 \
  --camera-network-interface eno1 \
  --camera-watchdog-timeout-s 0.6 \
  --eval-diagnostics-dir ~/eval_output/eval_diagnostics/pnp_trocar_n17 \
  --head-eye right \
  --video-key-style head_right \
  --head-pitch-home 0.6 \
  --swap-wrist-cams
```

### Head-eye selection

`--head-eye` selects which physical head camera image is sent to the policy:

- `--head-eye right`: use the physical right-eye camera.
- `--head-eye left`: use the physical left-eye camera.

`--video-key-style head_right` controls the observation key name expected by this N1.7 checkpoint; it does not select the physical eye. With the command above, the physical right-eye image is sent as `video.head_right`.

### Wrist-camera mapping

The runtime wrist-camera mapping must match the mapping used when the training dataset was prepared:

- If the training data intentionally swapped the left and right wrist views, add `--swap-wrist-cams`.
- If the training data kept the original left/right wrist views and the runtime camera topics are also correct, do not add `--swap-wrist-cams`.
- If physical runtime wiring is reversed relative to the training data, add `--swap-wrist-cams` to restore the training-time mapping.

The flag swaps which physical wrist stream is assigned to the policy's left and right wrist observation keys. It does not alter the head image.

For the command above, the checkpoint input keys are:

- The right head image as `video.head_right`.
- The left wrist image as `video.wrist_left`.
- The right wrist image as `video.wrist_right`.

Remove the final `--swap-wrist-cams` line when the training data did not swap wrist views and the runtime wiring is already correct. When removing it, also remove the trailing `\` from the preceding `--head-pitch-home 0.6` line.

## 5. Optional evaluation features

### 5.1 Diagnostic logs

The evaluation command uses:

```bash
--eval-diagnostics-dir ~/eval_output/eval_diagnostics/pnp_trocar_n17
```

Each episode writes a compressed NPZ containing:

- Raw 58-dimensional model output.
- Final 58-dimensional command target.
- Robot state before and after each command.
- Left/right hand DDS sequence IDs and message age.
- Right-hand assist score and per-joint adjustment.

### 5.2 Initial pose

The command loads:

```text
init_post_pnp_trocar.yaml
```

The initial-pose file must match the task and robot joint convention. For a newly generated pose:

1. Test with the robot safely supported.
2. Watch both arms throughout the transition.
3. If the target appears inconsistent, stop the script immediately. Securely support both robot arms/hands and, while continuing to support them, press `L2+B` to enter damping.
4. Do not average clearly different initial-pose modes unless that midpoint has been physically reviewed.

## 6. Stopping evaluation

Normal stop:

1. Press `Ctrl+C` to start the normal shutdown and automatic homing sequence.
2. Wait for the homing sequence and script cleanup to finish.
3. Securely support both robot arms/hands and, while continuing to support them, press `L2+B` to enter damping.
4. Keep supporting both arms/hands until the robot is stable; otherwise they can drop and damage the hands or surrounding equipment.
