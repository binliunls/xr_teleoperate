# ALVR Motion Tracker Integration

Use ALVR to stream PICO Motion Tracker data to your PC for robot teleoperation.

## Architecture

```
┌─────────────────────────────────────┐
│        PICO Headset                 │
│  ┌─────────────────────────────┐    │
│  │   ALVR Client               │    │
│  │   - Reads Motion Trackers   │    │
│  │   - Streams to PC via WiFi  │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
                │
                │ WiFi (ALVR Protocol)
                ▼
┌─────────────────────────────────────┐
│        PC (Linux)                   │
│  ┌─────────────────────────────┐    │
│  │   SteamVR + ALVR Streamer   │    │
│  │   - Receives tracker data   │    │
│  │   - Exposes via OpenVR API  │    │
│  └─────────────────────────────┘    │
│               │                     │
│               ▼                     │
│  ┌─────────────────────────────┐    │
│  │   alvr_tracker_bridge.py    │    │
│  │   - Reads from OpenVR       │    │
│  │   - Provides pose data      │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
                │
                │ DDS
                ▼
┌─────────────────────────────────────┐
│        Unitree Robot                │
└─────────────────────────────────────┘
```

## Prerequisites

- **Steam** installed on Linux PC
- **SteamVR** installed via Steam (ALVR requires SteamVR as backend)
- PICO headset with Motion Trackers
- Same WiFi network for PC and PICO

## Setup Instructions

### Step 1: Install Steam and SteamVR

```bash
# Install Steam (if not installed)
sudo apt install steam

# Launch Steam and install SteamVR from the Library
steam &
# In Steam: Library → Search "SteamVR" → Install
```

### Step 2: Install ALVR on PC

```bash
# Download ALVR release (v20.14.1 or newer)
cd ~/binliu
wget https://github.com/alvr-org/ALVR/releases/download/v20.14.1/alvr_streamer_linux.tar.gz

# Extract
tar -xzf alvr_streamer_linux.tar.gz

# Install dependencies (Ubuntu)
sudo apt install libvulkan1 libgtk-3-0 libasound2
```

### Step 3: Configure SteamVR Launch Options

1. Open Steam → Library → Right-click **SteamVR** → Properties
2. In **Launch Options**, add:
   ```
   /home/nvidia/.steam/debian-installation/steamapps/common/SteamVR/bin/vrmonitor.sh %command%
   ```
3. Launch SteamVR once to initialize config files

### Step 4: Run ALVR Dashboard

```bash
cd /home/nvidia/binliu/alvr_streamer_linux
./bin/alvr_dashboard
```

If you have firewall issues:
```bash
sudo ./libexec/alvr/alvr_fw_config.sh
```

### Step 5: Install ALVR Client on PICO

#### 5.1 Download the APK
Download `alvr_client_android.apk` from [ALVR releases](https://github.com/alvr-org/ALVR/releases/tag/v20.14.1)

#### 5.2 Enable Developer Mode on PICO 4 Ultra
1. Go to **Settings → General → Developer**
2. Enable **Developer Mode**
3. Enable **USB Debugging**

#### 5.3 Install ADB on PC (if not installed)
```bash
# Ubuntu/Debian
sudo apt install adb

# Or download Android Platform Tools from Google
# https://developer.android.com/studio/releases/platform-tools
```

#### 5.4 Connect and Install
1. Connect PICO to PC with a data-capable USB cable
2. Put on headset and **Allow USB debugging** when prompted (tick "Always allow")
3. Verify connection:
   ```bash
   adb devices
   # Should show: XXXXXXXX device
   ```
4. Install the APK:
   ```bash
   adb install alvr_client_android.apk
   # Or if updating: adb install -r alvr_client_android.apk
   ```
5. The app appears in PICO library under **Unknown Sources / Third-party apps**

### Step 6: Pair PICO with PC

1. Connect PICO to same WiFi as PC
2. Launch ALVR app on PICO
3. In ALVR Dashboard on PC:
   - PICO should appear under "New Wireless Devices"
   - Click to trust it
   - Status should change to "Streaming" (green)

### Step 7: Configure Motion Trackers on PICO

1. Pair Motion Trackers in PICO Settings → Body Tracking
2. Put on the trackers and calibrate

**Important:** Wrist Motion Trackers appear as **"VRLink Hand Tracker"** devices in SteamVR, not as generic trackers. The bridge script handles this automatically.

### Step 8: Install Python Dependencies

```bash
conda activate tv
pip install openvr numpy
```

### Step 9: Test the Tracker Bridge

```bash
cd ~/binliu/xr_teleoperate/alvr_tracker

# Test mode - shows tracker positions in terminal
python alvr_tracker_bridge.py --test

# Visualization - opens 3D viewer
python visualize_trackers.py
```

Expected output for wrist trackers:
```
--- WRIST TRACKERS (for robot arms) ---
  LEFT wrist: VALID
    Position: [ -0.721,   1.003,   0.304]
    Velocity: 0.000 m/s
  RIGHT wrist: VALID
    Position: [ -0.603,   0.949,   0.231]
    Velocity: 0.000 m/s
```

### Step 10: Run Robot Teleoperation

```bash
python alvr_tracker_teleop.py --arm=H2
```

## Troubleshooting

### ALVR says "SteamVR Linux Files missing"

ALVR requires SteamVR to be installed. Install it via Steam:
```bash
steam steam://install/250820
```

### Trackers show "INVALID" status

- **Full-body trackers** (chest, waist, etc.): Require physical trackers at those positions
- **Wrist trackers**: Should show as "VRLink Hand Tracker" devices
- Check that trackers are paired and calibrated in PICO settings

### Tracking stops when headset is removed (PICO 4 Ultra)

PICO pauses tracking when the proximity sensor detects no face. Unfortunately, **PICO 4 Ultra does not offer a full "stay awake off-head" option** in stock firmware, but you can minimize interruptions:

#### Disable Auto Wake
1. Put on headset → **Settings** (gear icon)
2. Go to **Display & brightness** (or **Display**)
3. Find **Auto Wake** (sometimes shown as "AO WE") → Turn it **OFF**

#### Set Maximum Screen Timeout
1. In **Settings → Display & brightness**
2. Find **Screen timeout**
3. Set to **Never** (or longest available, e.g., 30 minutes)

#### Limitations
- Even with these settings, PICO 4 Ultra may still dim/sleep after detecting "no face"
- The screen timeout only works while wearing the headset
- Custom Settings APKs that work on Pico 4 do **not** reliably work on Pico 4 Ultra

#### Workarounds
- **Tape over proximity sensor** (between the lenses) - crude but sometimes effective
- **Keep headset on** during teleoperation sessions
- **Press power button** to wake if it sleeps, then continue

### OpenVR initialization fails

Create/update the OpenVR paths config:
```bash
mkdir -p ~/.config/openvr
cat > ~/.config/openvr/openvrpaths.vrpath << 'EOF'
{
  "config" : ["/home/nvidia/.config/openvr"],
  "external_drivers" : ["/home/nvidia/binliu/alvr_streamer_linux/lib64/alvr"],
  "jsonid" : "vrpathreg",
  "log" : ["/home/nvidia/.config/openvr"],
  "runtime" : ["/home/nvidia/binliu/alvr_streamer_linux"],
  "version" : 1
}
EOF
```

### High latency

- Use 5GHz WiFi (not 2.4GHz)
- Reduce video resolution in ALVR Settings
- Position PC closer to WiFi router

### Check connected devices

Run this to see all OpenVR devices:
```bash
conda activate tv
python -c "
import openvr
vr = openvr.init(openvr.VRApplication_Other)
for i in range(16):
    dc = vr.getTrackedDeviceClass(i)
    if dc != 0:
        sn = vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
        model = vr.getStringTrackedDeviceProperty(i, openvr.Prop_ModelNumber_String)
        print(f'{i}: {model} ({sn})')
openvr.shutdown()
"
```

## Files

| File | Description |
|------|-------------|
| `alvr_tracker_bridge.py` | Core module - reads tracker data from OpenVR |
| `alvr_tracker_teleop.py` | Robot teleoperation using tracker data |
| `visualize_trackers.py` | 3D visualization using Rerun |
| `README.md` | This documentation |

## Usage in Code

```python
from alvr_tracker_bridge import ALVRTrackerBridge

bridge = ALVRTrackerBridge()
bridge.initialize()

while True:
    bridge.update()
    arm_poses = bridge.get_arm_poses()

    # Get wrist positions for robot IK
    left_pos = arm_poses["left"]["position"]   # [x, y, z] numpy array
    right_pos = arm_poses["right"]["position"]
    left_rot = arm_poses["left"]["rotation"]   # 3x3 rotation matrix
    right_rot = arm_poses["right"]["rotation"]

    # Check if tracking is valid
    if arm_poses["left"]["valid"]:
        # Use left wrist pose for robot control
        pass
```
