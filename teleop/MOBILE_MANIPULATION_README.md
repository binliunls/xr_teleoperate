# 🤖 Mobile Manipulation Teleoperation

This script enables simultaneous locomotion and manipulation control in Isaac Sim using VR controllers.

## 🎮 Controller Mapping

### Joysticks
- **Left Joystick:**
  - ↕️ **Y-axis**: Forward/Backward movement (max: ±1.0 m/s forward, ±0.6 m/s backward)
  - ↔️ **X-axis**: Strafe Left/Right (max: ±0.5 m/s)

- **Right Joystick:**
  - ↕️ **Y-axis**: Height Up/Down (range: 0.3 to 0.8 m from ground)
  - ↔️ **X-axis**: Turn Left/Right (max: ±1.57 rad/s)

### Buttons
- 🟢 **Y Button (Left Controller)**: Start teleoperation
- 🔵 **X Button (Left Controller)**: Start recording
- 🔴 **B Button (Right Controller)**: Stop recording
- 🔴 **A Button (Right Controller)**: Emergency stop & exit

### Triggers
- **Left/Right Triggers**: Control grippers (when using `--ee=dex1`)

## 📋 Prerequisites

### 1. Start Isaac Sim with Locomotion Support

```bash
cd ~/unitree_sim_isaaclab
conda activate unitree_sim_env
python sim_main.py --device cpu --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --enable_dex3_dds --robot_type g129
```

**Note:** Make sure the simulation task supports whole-body control (locomotion + manipulation).

### 2. Ensure SSL Certificates Exist

```bash
cd ~/xr_teleoperate/teleop/televuer
# Check if cert.pem and key.pem exist
ls -l *.pem

# If not, generate them:
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem -subj "/C=US/ST=State/L=City/O=Organization/CN=localhost"
```

## 🚀 Usage

### Basic Usage (with Dex1 gripper)

```bash
cd ~/xr_teleoperate/teleop/
python teleop_mobile_manipulation.py --ee=dex1
```

### With Recording

```bash
python teleop_mobile_manipulation.py --ee=dex1 --record --task-name="mobile_pick_and_place"
```

### With Different Arms/End-Effectors

```bash
# G1 29DoF with Dex3 hand
python teleop_mobile_manipulation.py --arm=G1_29 --ee=dex3 --record

# H1_2 with Dex1 gripper
python teleop_mobile_manipulation.py --arm=H1_2 --ee=dex1 --record
```

### Headless Mode (no GUI display)

```bash
python teleop_mobile_manipulation.py --ee=dex1 --headless --record
```

## 📝 Step-by-Step Instructions

### 1. Launch Isaac Sim
First terminal:
```bash
cd ~/unitree_sim_isaaclab
conda activate unitree_sim_env
python sim_main.py --device cpu --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --enable_dex3_dds --robot_type g129
```

Wait for: `controller started, start main loop...`

### 2. Start Mobile Manipulation Script
Second terminal:
```bash
cd ~/xr_teleoperate/teleop/
conda activate tv
python teleop_mobile_manipulation.py --ee=dex1 --record
```

### 3. Connect Quest Headset

1. Put on your Quest headset
2. Ensure Quest is on the same WiFi as your host PC
3. Open browser on Quest
4. Navigate to: `https://<YOUR_HOST_IP>:8012?ws=wss://<YOUR_HOST_IP>:8012`
   - Replace `<YOUR_HOST_IP>` with your computer's IP (check with `ifconfig`)
   - Example: `https://192.168.5.15:8012?ws=wss://192.168.5.15:8012`
5. Bypass security warning (click "Advanced" → "Proceed to...")
6. Click "Virtual Reality" button
7. Allow all permissions

### 4. Start Teleoperation

1. You'll see the robot's camera view in your headset
2. **Press Y button on left controller** to start
3. Use joysticks to control locomotion
4. Use controller poses to control arms
5. Use triggers to control grippers

### 5. Recording Data

1. **Press X button on left controller** to start recording
   - Terminal shows: `📹 Recording started`
2. Perform your task
3. **Press B button on right controller** to stop recording
   - Terminal shows: `💾 Recording saved`
4. Repeat as needed for multiple episodes

### 6. Exit

- **Press A button on right controller** for emergency stop & clean exit
- Or press `q` in the terminal window

## 📊 Recorded Data Format

Data is saved in `./utils/data/<task-name>/` with the following structure:

```
states:
  body:
    qpos: [x_vel, y_vel, yaw_vel, height]  # Locomotion state
  left_arm:
    qpos: [7 joint positions]
  right_arm:
    qpos: [7 joint positions]
  left_ee:
    qpos: [gripper/hand state]
  right_ee:
    qpos: [gripper/hand state]

actions:
  body:
    qpos: [x_vel, y_vel, yaw_vel, height]  # Locomotion commands
  left_arm:
    qpos: [7 joint commands]
  right_arm:
    qpos: [7 joint commands]
  left_ee:
    qpos: [gripper/hand commands]
  right_ee:
    qpos: [gripper/hand commands]
```

## ⚙️ Command Line Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--arm` | str | `G1_29` | Robot arm type: `G1_29`, `G1_23`, `H1_2`, `H1` |
| `--ee` | str | None | End-effector: `dex1`, `dex3`, `inspire1`, `brainco` |
| `--frequency` | float | `30.0` | Data recording frequency (Hz) |
| `--record` | flag | False | Enable data recording |
| `--headless` | flag | False | Run without GUI display |
| `--task-name` | str | `mobile_manipulation` | Task name for saving data |
| `--task-desc` | str | `Mobile manipulation task` | Task description |
| `--task-dir` | str | `./utils/data/` | Directory for saving data |

## 🐛 Troubleshooting

### Quest won't connect
- Check both devices are on same WiFi
- Verify SSL certificates exist in `teleop/televuer/`
- Try regenerating certificates if connection fails
- Check firewall isn't blocking port 8012

### No locomotion response
- Ensure Isaac Sim task supports locomotion
- Check DDS communication is initialized
- Verify joystick values are being read (check debug logs)

### Arms not moving
- Make sure controllers are tracked properly in VR
- Check that arm IK is solving successfully (debug logs)
- Verify robot is in correct mode in Isaac Sim

### Recording fails
- Check disk space is available
- Verify `--record` flag is set
- Ensure task directory exists and is writable

## 🔍 Debug Mode

Enable debug logging for more information:

```python
# Edit the script and change:
logging_mp.basic_config(level=logging_mp.DEBUG)
```

This will show:
- IK solve times
- Locomotion commands
- Recording status
- Controller button states

## 📚 Differences from Original `teleop_hand_and_arm.py`

1. **Forces controller mode** (hand tracking disabled)
2. **Adds locomotion control** via joysticks
3. **Different button mapping** optimized for mobile manipulation
4. **Records body state/action** including locomotion commands
5. **Only works in simulation** (requires Isaac Sim locomotion support)

## 🎯 Example Tasks

### Pick and Navigate
1. Navigate to object using left joystick
2. Adjust height with right joystick
3. Use arms/grippers to pick object
4. Navigate to target location
5. Place object

### Mobile Manipulation with Obstacles
1. Use right joystick to turn and scout
2. Navigate around obstacles
3. Approach target from different angles
4. Perform manipulation task

## 📞 Support

For issues or questions:
- Check the main README: `/xr_teleoperate/README.md`
- Isaac Sim documentation: [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab)
- Original teleoperation guide: [Unitree Teleoperation Docs](https://support.unitree.com/home/zh/Teleoperation)

