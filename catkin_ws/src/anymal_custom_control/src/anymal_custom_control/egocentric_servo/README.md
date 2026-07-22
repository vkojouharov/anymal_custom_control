# ANYmal Egocentric Servo

Reusable ANYmal base visual-servo stack for approaching large `tag16h5`
AprilTags with the forward Oak-D RGB camera.

## Current Architecture

- `run_oakd_sensor_node.py` remains the single DepthAI/Oak-D owner.
- `anymal_custom_control.egocentric_servo.node` consumes AprilTag detections,
  legged odometry, and RGB video.
- `anymal_custom_control.egocentric_servo.console` provides the web console.
- `scripts/run/run_anymal_egocentric_servo.py` is the only stack entry point;
  it launches the camera, servo node, and console as separate processes.
- Control uses AprilTag pose only through a 5 Hz receding-horizon MPC.
- Legged odometry is a reference-only logging signal for comparison against
  AprilTag localization.
- Base commands go through `MovementController` and `/anyjoy/operator`.
- Mode requests go through `ModeController`.

## Critical Constant

The physical AprilTag edge length is a code constant:

```python
APRILTAG_TAG_LENGTH_M = 0.20066  # 7.9 in
```

This must match the printed black-square tag edge. Pose scale is wrong if this
number is wrong.

## Topics

- Input:
  - `/oakd/apriltag/detections_json`
  - `/legged_odometry/pose_in_odom`
- Command:
  - `/anymal/egocentric_servo/command_json`
- Status:
  - `/anymal/egocentric_servo/status_json`
  - `/anymal/egocentric_servo/trajectory_json`

## Commands

Commands are JSON strings sent to `/anymal/egocentric_servo/command_json`:

```json
{"command": "mode", "mode": "Stand"}
{"command": "mode", "mode": "Walk"}
{"command": "arm"}
{"command": "start"}
{"command": "pause"}
{"command": "resume"}
{"command": "stop"}
{"command": "select_tag", "tag_id": 12}
```

`Arm` never moves the robot. `Start` moves only if a fresh AprilTag pose is
available. After `Start`, the node collects 1 second of AprilTag detections,
uses the median pose to initialize the MPC frame, then begins 5 Hz MPC tracking.

## Field-Test Flow

1. Place a large `tag16h5` tag in the forward Oak-D view.
2. Confirm `APRILTAG_TAG_LENGTH_M` matches the measured tag.
3. Start the normal ANYmal ROS stack.
4. Run:

   ```bash
   python3 /catkin_ws/src/anymal_custom_control/scripts/run/run_anymal_egocentric_servo.py
   ```

5. Open `http://<robot-host-ip>:5004`.
6. Confirm RGB stream, tag pose, plausible range, fresh odometry, and zero command.
7. Press `Stand`, `Walk`, `Arm`, then `Start Trajectory`.
8. Use `Pause`, `Resume`, and `Stop` as needed.

## Logging

Runs are written under `/experiments/egocentric_servo_runs` by default. Each run folder is named `egocentric_servo_MMDDYY_HHMMSS` and contains:

- `metadata.json`
- `trajectory.csv`
- `trajectory_rgb.mp4` and `video_frames.csv` when RGB video recording is enabled

The logger writes one synchronized `trajectory.csv` row per MPC tick, currently
5 Hz. Each row stores raw AprilTag translation, full `rotation_camera_tag`, the
AprilTag-derived MPC pose in the tag frame, the legged-odometry pose projected
into the same tag frame, tag-minus-odom drift, physical body velocity commands,
normalized ANYmal movement-axis commands, solver status, solve time, and the
current MPC predicted state/control horizon as JSON columns.

The fixed top-down frame is snapped at `Start Trajectory` from the median
initial AprilTag pose. AprilTag localization and legged odometry are both
reported in this same MPC tag frame, so their difference is the direct
slip/localization disagreement signal.

To visualize the latest run as static plots:

```bash
python3 /experiments/egocentric_servo_runs/visualize_trajectory.py
```

To render a 10 Hz MPC rollout animation:

```bash
python3 /experiments/egocentric_servo_runs/visualize_MPC_planning.py
```

## Validation Notes

- All motion should stop on stale tag pose, explicit `Stop`, node shutdown, or
  process failure via joy-manager timeout. Brief tag dropouts continue along
  the previous MPC horizon for at most two 5 Hz ticks before pausing.
