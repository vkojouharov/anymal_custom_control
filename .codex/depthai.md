# DepthAI / OAK-D S2

This note documents the current DepthAI setup in `anymal_custom_control`, the
production ROS-backed OAK-D owner, and the standalone RGB / aligned-depth test
pipelines.

## Current environment

- Base image: `ros:noetic-ros-base-focal`
- Python: system `python3` from Ubuntu 20.04 / ROS Noetic
- DepthAI package: `depthai==2.29.0.0`
- Camera target: Luxonis OAK-D S2

The repo is using the legacy `depthai` 2.x line because the container is tied
to ROS Noetic / Python 3.8.

## Install path

### Python dependency

`requirements.txt` includes:

```txt
depthai==2.29.0.0
```

This version is compatible with the current Noetic / Python 3.8 container.

### Docker image

The image already includes the core runtime pieces needed by DepthAI:

- `python3-pip`
- `python3-opencv`
- `python3-numpy`
- `libusb-1.0-0-dev`
- `udev`

The current `Dockerfile` installs Python dependencies with:

```bash
pip3 install --no-cache-dir -r /requirements.txt
```

### USB passthrough

The working Docker setup depends on full USB bus passthrough, not only a single
device entry.

Current `docker-compose.yml` pieces that matter:

```yaml
privileged: true
volumes:
  - /run/udev:/run/udev:ro
  - /dev/bus/usb:/dev/bus/usb
device_cgroup_rules:
  - 'c 189:* rmw'
```

This is what fixed the earlier DepthAI boot / re-enumeration failure inside the
container.

## Bring-up workflow

### 1. Build / start container

```bash
cd ~/anymal_custom_control
docker compose build
docker compose up -d
```

### 2. Enter container

```bash
docker exec -it anymal_custom_control bash
```

### 3. Probe camera

Use:

```bash
python3 /catkin_ws/src/anymal_custom_control/scripts/camera/depthai_probe.py
```

Expected healthy output:

- one visible device
- handshake succeeds
- connected cameras list is populated
- connected IMU is populated
- USB speed reports `UsbSpeed.SUPER`

If it reports `UsbSpeed.HIGH`, the camera is on USB 2.0, not USB 3.x.

## Scripts

DepthAI scripts currently live in:

`catkin_ws/src/anymal_custom_control/scripts/camera/`

Production OAK-D ownership lives in:

`catkin_ws/src/anymal_custom_control/scripts/run/run_oakd_sensor_node.py`

Relevant files:

- `depthai_probe.py`
- `depthai_rgb_stream.py`
- `depthai_depth_stream.py`
- `../run/run_oakd_sensor_node.py`

## Production OAK-D Node

`run_oakd_sensor_node.py` is the production OAK-D owner. In `operator_station
--mode full`, this is the only process that opens `dai.Device`.

Purpose:

- publish RGB visualization frames to ROS
- publish colorized aligned-depth visualization frames to ROS
- run AprilTag detection on RGB before frame compression
- compare AprilTag RGB pose depth against masked aligned depth
- publish `GAME_ROTATION_VECTOR`
- publish derived camera-Y level error for stabilizing the wrist/boom endpoint

Run directly:

```bash
python3 /catkin_ws/src/anymal_custom_control/scripts/run/run_oakd_sensor_node.py
```

Production operator-station launch uses:

```bash
python3 /catkin_ws/src/anymal_custom_control/scripts/run/run_oakd_sensor_node.py --wait-for-arm-state
```

The `--wait-for-arm-state` option waits for the first `/giraf_arm/state` before
opening the OAK-D. This avoids starting the OAK-D pipeline before the arm
controller has completed initial hardware bring-up and published state.

### Production OAK-D Topics

- `/oakd/rgb/image_color/compressed`
- `/oakd/depth/image_colorized/compressed`
- `/oakd/apriltag/stats_json`
- `/oakd/imu/game_rotation_vector`
- `/oakd/camera_y_axis_fused`
- `/oakd/camera_y_level_error`

The operator web console consumes the compressed image and AprilTag stats
topics. Stabilized teleop consumes `/oakd/camera_y_level_error`.

### Production RGB / Depth Pipeline

- `ColorCamera` on `CAM_A`
- sensor resolution: `1080p`
- preview output: `640x360`
- `previewKeepAspectRatio(False)`
- `BGR`
- `30 FPS`
- mono stereo on `CAM_B` / `CAM_C`
- mono resolution: `400P`
- `StereoDepth` preset: `HIGH_DETAIL`
- `setDepthAlign(CAM_A)`
- `setOutputSize(640, 360)`
- `setLeftRightCheck(True)`
- `setSubpixel(True)`

### Production IMU / Stabilization Signal

- IMU source: `GAME_ROTATION_VECTOR`
- requested IMU rate: `200 Hz`
- derived level error topic: `/oakd/camera_y_level_error`
- error definition: `asin(camera_y_axis_fused.z)`, in radians
- current camera-Y mapping: `+Y` in the OAK-D IMU frame

Yaw initialization/drift of `GAME_ROTATION_VECTOR` does not affect this level
metric because the signal uses only the vertical component of camera Y.

### Production AprilTag / Depth Comparison

The node runs `pupil_apriltags` on RGB frames and publishes JSON stats on
`/oakd/apriltag/stats_json`.

Stats include:

- detector enabled/disabled
- detection FPS
- detection count
- RGB pose-depth summary from AprilTag pose estimation
- depth-mask summary from raw aligned depth under the tag polygon

Current AprilTag settings:

- family: `tag16h5`
- decision margin threshold: `50`
- quad decimate: `1.0`
- threads: `2`
- tag size: `0.0956 m`

### Direct OAK-D Ownership Rule

Only one process should open the OAK-D at a time.

In production full mode:

- `run_oakd_sensor_node.py` opens the OAK-D
- `run_operator_console.py --no-camera` consumes ROS topics and does not open
  the OAK-D
- `run_teleop_stabilized.py` consumes `/oakd/camera_y_level_error` and does not
  open the OAK-D

The older camera dashboards below still open the OAK-D directly. They are useful
as standalone diagnostics but should not run at the same time as the production
OAK-D node.

## `depthai_probe.py`

Purpose:

- enumerate visible DepthAI devices inside the container
- open the first one
- print USB speed, connected cameras, and IMU presence

This is the first script to run after rebuilds, USB changes, or cable / port
changes.

## Standalone `depthai_rgb_stream.py`

Purpose:

- run a simple RGB-only OAK stream
- expose it as MJPEG over HTTP
- show live FPS and image size on the page

Run:

```bash
python3 /catkin_ws/src/anymal_custom_control/scripts/camera/depthai_rgb_stream.py
```

Default URL:

```txt
http://localhost:5002
```

### Current RGB pipeline

- `ColorCamera` on `CAM_A`
- sensor resolution: `1080p`
- preview output: `640x360`
- `BGR`
- `30 FPS`
- output path: `ColorCamera.preview -> XLinkOut("rgb")`

### Current RGB web behavior

- one MJPEG feed at `/feed`
- one stats endpoint at `/stats`
- page shows:
  - current RGB FPS
  - current RGB image size

## Standalone `depthai_depth_stream.py`

Purpose:

- run RGB and stereo depth together
- align depth to the RGB camera perspective
- colorize depth for browser viewing
- show RGB and aligned depth side-by-side in the browser
- show live FPS and image size for both streams

Run:

```bash
python3 /catkin_ws/src/anymal_custom_control/scripts/camera/depthai_depth_stream.py
```

Default URL:

```txt
http://localhost:5003
```

### Current RGB pipeline inside depth stream

- `ColorCamera` on `CAM_A`
- sensor resolution: `1080p`
- preview output: `640x360`
- `previewKeepAspectRatio(False)`
- `BGR`
- `30 FPS`
- output path: `ColorCamera.preview -> XLinkOut("rgb")`

### Current depth pipeline

- `MonoCamera` left on `CAM_B`
- `MonoCamera` right on `CAM_C`
- mono resolution: `400P`
- mono FPS: `30`
- `StereoDepth` preset: `HIGH_DETAIL`
- `setDepthAlign(CAM_A)` so depth matches RGB perspective
- `setOutputSize(640, 360)` so aligned depth matches displayed RGB size
- `setLeftRightCheck(True)`
- `setSubpixel(True)`

### Current depth post-processing

Configured through `stereo.initialConfig`:

- confidence threshold: `220`
- speckle filter: enabled
  - speckle range: `100`
- temporal filter: enabled
  - alpha: `0.6`
  - delta: `40`
- spatial filter: enabled
  - hole filling radius: `4`
  - num iterations: `2`
  - alpha: `0.6`
  - delta: `40`
- threshold filter:
  - min range: `10 mm`
  - max range: `1000 mm`

### Current depth colorization

The depth map is colorized on the host after receipt from DepthAI:

- clipping range: `10 mm` to `1000 mm`
- values outside range saturate at the colormap ends
- zero-depth pixels are painted black
- colormap: `cv2.COLORMAP_JET`

Important: the `10 mm` lower bound is only the display colorization range. It
does not mean the camera can physically produce accurate 1 cm depth.

### Current depth web behavior

The browser page has two panes:

- RGB
- aligned colorized depth

It exposes:

- `/feed/rgb`
- `/feed/depth`
- `/stats`

The page polls `/stats` every 500 ms and shows:

- RGB FPS and size
- depth FPS and size

## Practical notes

### Why this repo uses web streaming instead of `cv2.imshow`

`cv2.imshow` inside the container worked only when X11 auth was correct. The
browser-based MJPEG approach is simpler and avoids host X11 issues.

### USB speed matters

The camera initially enumerated as `UsbSpeed.HIGH` until it was moved to a real
USB 3.x path. Current expected / preferred state is:

- host `lsusb -t` shows the OAK on a SuperSpeed path
- `depthai_probe.py` reports `UsbSpeed.SUPER`

This gives enough bandwidth headroom for RGB + aligned depth at `640x360` and
`30 FPS`.

### If DepthAI sees the device but fails to open it

The failure mode previously observed was:

- device visible as unbooted
- handshake failed during firmware boot / re-enumeration

The fix was the Docker USB configuration described above:

- bind mount `/dev/bus/usb`
- bind mount `/run/udev`
- add USB cgroup rule
- run privileged

## Current defaults summary

- `depthai==2.29.0.0`
- OAK-D S2 on USB 3.x
- probe script for bring-up
- production OAK-D owner:
  - `run_oakd_sensor_node.py`
  - publishes ROS RGB/depth/AprilTag/IMU/stabilization topics
  - waits for `/giraf_arm/state` in full operator-station mode
- RGB stream:
  - `640x360`
  - `30 FPS`
  - MJPEG on port `5002`
- RGB + aligned depth stream:
  - RGB `640x360`
  - aligned depth `640x360`
  - mono stereo at `400P`
  - `HIGH_DETAIL`
  - smoothing / fill filters enabled
  - depth colorized over `10 mm .. 1000 mm`
  - MJPEG on port `5003`

## Likely next improvements

- expose stereo preset as a CLI flag
- expose depth colorization min / max as CLI flags
- expose smoothing parameters as CLI flags
- publish raw aligned depth and camera info topics if downstream ROS consumers
  need metric depth, not only colorized visualization frames
- add stabilization telemetry to the web console explicitly
