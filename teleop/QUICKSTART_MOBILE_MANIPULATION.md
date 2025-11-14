# 🚀 Quick Start: Mobile Manipulation

## 1️⃣ Terminal 1: Start Isaac Sim

```bash
cd ~/unitree_sim_isaaclab
conda activate unitree_sim_env
python sim_main.py --device cpu --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --enable_dex3_dds --robot_type g129
```

Wait for: `controller started, start main loop...`

## 2️⃣ Terminal 2: Start Mobile Manipulation Script

```bash
cd ~/xr_teleoperate/teleop/
conda activate tv
python teleop_mobile_manipulation.py --ee=dex1 --record
```

## 3️⃣ Connect Quest

1. **Put on Quest headset**
2. **Open browser** (Quest Browser)
3. **Navigate to**: `https://192.168.5.15:8012?ws=wss://192.168.5.15:8012`
   - Replace `192.168.5.15` with your host IP
4. **Bypass security warning**: Click "Advanced" → "Proceed"
5. **Click "Virtual Reality"**
6. **Allow all permissions**

## 4️⃣ Control the Robot

### Start Teleoperation
- Press **Y button** (upper button on left controller)

### Locomotion Control
- **Left Joystick**:
  - Push forward/back → Move forward/backward
  - Push left/right → Strafe left/right
- **Right Joystick**:
  - Push up/down → Adjust height
  - Push left/right → Turn left/right

### Arm & Gripper Control
- **Controller position** → Controls arm position
- **Triggers** → Open/close grippers

### Recording
- Press **X button** (lower button on left controller) → Start recording
- Press **B button** (upper button on right controller) → Stop recording

### Exit
- Press **A button** (lower button on right controller) → Emergency stop & exit

## 🎮 Controller Reference

```
Left Controller              Right Controller
     [Y]  ← Upper                 [B]  ← Upper
     [X]  ← Lower                 [A]  ← Lower
     
Y = Start            A = Exit
X = Record Start     B = Record Stop
```

## 📝 Data Location

Recorded data: `~/xr_teleoperate/teleop/utils/data/mobile_manipulation/`

## 🐛 Troubleshooting

**Quest won't connect?**
```bash
cd ~/xr_teleoperate/teleop/televuer
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem -subj "/C=US/ST=State/L=City/O=Organization/CN=localhost"
```

**No locomotion?**
- Check Isaac Sim is running
- Verify joysticks are moving in VR

**Arms not responding?**
- Make sure controllers are tracked
- Check VR session is active

## 📚 Full Documentation

See: `MOBILE_MANIPULATION_README.md` for detailed information

