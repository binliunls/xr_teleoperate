# ROS 2 camera source for xr_teleoperate

`teleop_hand_and_arm.py` can read cameras from either the existing **ZMQ** image
server (teleimager) or from **ROS 2** image topics published by the Jetson
gstreamer pipeline (`sensing_h2_ros_gstreamer`). Selection is via
`--camera-source {zmq, ros}` (default: `zmq`).

## Topics consumed (ROS source)

| Role                | Topic                       | Notes                          |
|---------------------|-----------------------------|--------------------------------|
| Head — left eye     | `/head/left/image_raw`      | stereo, stitched into one frame |
| Head — right eye    | `/head/right/image_raw`     | stereo, stitched into one frame |
| Left wrist          | `/wrist/left/image_raw`     | mono                           |
| Right wrist         | `/wrist/right/image_raw`    | mono                           |

Message type: `sensor_msgs/Image`. Encodings supported: `rgb8`, `bgr8`, `mono8`.
QoS matches the visualizer: `BEST_EFFORT`, `KEEP_LAST`, depth 2.

## Files

- `teleop/utils/ros_image_client.py` — `ROSImageClient` (drop-in API match for
  `teleimager.ImageClient`). Subscribes to the four topics, decodes images,
  stitches the two head eyes side-by-side, and reports a synthetic `camera_config`
  dict with `head_camera.binocular=True`.
- `teleop_hand_and_arm.py` — adds the `--camera-source` flag and branches between
  `ImageClient` (ZMQ) and `ROSImageClient` (ROS) at startup.

## Stitched head image layout

```
        head_img.bgr  (shape: H × 2W × 3)
        ┌──────────────────────┬──────────────────────┐
        │     left eye         │     right eye        │
        │    (H × W × 3)       │    (H × W × 3)       │
        └──────────────────────┴──────────────────────┘
              color_0                   color_1
```

The existing recording path already handles `binocular=True` by splitting on the
middle column:

```python
colors[f"color_{0}"] = head_img.bgr[:, : W // 2, :]
colors[f"color_{1}"] = head_img.bgr[:, W // 2 :, :]
```

So no changes to the recorder are required — both eyes get saved as
`color_0` / `color_1` automatically.

## Usage

### Prerequisites

ROS 2 (Jazzy) and the CycloneDDS RMW must be on `PATH`:

```bash
source /opt/ros/jazzy/setup.bash
```

The publisher pipeline (`sensing_h2_ros_gstreamer`) must be running on the
Jetson so the four topics are available. Verify from the workstation:

```bash
ros2 topic list | grep image_raw
ros2 topic hz /head/left/image_raw   # expect ~30 Hz
```

### Run teleop with ROS cameras

```bash
python teleop/teleop_hand_and_arm.py \
    --arm H2 --ee sharpa \
    --camera-source ros
```

### Run teleop with the old ZMQ source (default)

```bash
python teleop/teleop_hand_and_arm.py \
    --arm H2 --ee sharpa \
    --camera-source zmq \
    --img-server-ip 192.168.124.162
```

## How the API is matched

`ROSImageClient` exposes the same methods used by `teleop_hand_and_arm.py`:

| Method                       | ZMQ source (`ImageClient`)            | ROS source (`ROSImageClient`)        |
|------------------------------|---------------------------------------|--------------------------------------|
| `get_cam_config()`           | reads server YAML                     | synthetic dict, `binocular=True`     |
| `get_head_frame()`           | single-camera frame                   | stitched stereo frame                |
| `get_left_wrist_frame()`     | wrist frame                           | wrist frame                          |
| `get_right_wrist_frame()`    | wrist frame                           | wrist frame                          |
| `close()`                    | closes ZMQ                            | shuts down rclpy node                |

Both clients return frame objects with `.bgr` (numpy BGR ndarray) and `.fps`,
and behave truthy only when image data is available.

## Cyclone DDS configuration

If you need a custom CycloneDDS XML (e.g. larger UDP receive buffer like the
visualizer uses), pass it via `CYCLONEDDS_URI`:

```bash
export CYCLONEDDS_URI=file:///home/nvidia/jeleong/sensing_h2_ros_gstreamer/config/cyclonedds/image_streams.xml
python teleop/teleop_hand_and_arm.py --camera-source ros ...
```

`ROSImageClient` sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` by default but
respects whatever `CYCLONEDDS_URI` is already in the environment.

## Behavior when topics are silent

If a topic hasn't produced a frame yet, the corresponding `get_*_frame()` call
returns a `ROSFrame` with `bgr=None` (and is falsy). The teleop main loop
already guards with `if head_img is not None:` / `if left_wrist_img is not None:`,
so silent topics simply skip recording for that step — same behavior as the
ZMQ path when a stream is disabled in `cam_config_client.yaml`.

The client also has a **warm-up** step at construction: it waits up to 5 s for
the first frame on both head topics before returning, so `get_cam_config()` can
report the actual image dimensions. Wrists are not blocking. Pass
`require_warmup=True` to `ROSImageClient(...)` if you want a hard error when
head topics never arrive.
