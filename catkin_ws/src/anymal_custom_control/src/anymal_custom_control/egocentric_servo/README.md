# ANYmal Egocentric Servo

Reusable ANYmal base visual-servo stack for approaching large `tag16h5`
AprilTags with the forward Oak-D RGB camera.

## Current Architecture

- `run_oakd_sensor_node.py` remains the single DepthAI/Oak-D owner.
- `anymal_custom_control.egocentric_servo.node` consumes AprilTag detections,
  legged odometry, and Oak-D fused IMU when available.
- Control uses AprilTag pose only.
- Legged odometry and IMU are reference-only logging signals.
- Base commands go through `MovementController` and `/anyjoy/operator`.
- Mode requests go through `ModeController`.

## Critical Constant

The physical AprilTag edge length is a code constant:

```python
APRILTAG_TAG_LENGTH_M = 0.14605  # 5.75 in
```

This must match the printed black-square tag edge. Pose scale is wrong if this
number is wrong.

## Topics

- Input:
  - `/oakd/apriltag/detections_json`
  - `/legged_odometry/pose_in_odom`
  - `/oakd/imu/game_rotation_vector`
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
available.

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

Runs are written under `/tmp/anymal_egocentric_servo_runs` by default. Each run
contains:

- `metadata.json`
- `trajectory.csv`

The logger stores raw and start-relative AprilTag pose, raw and start-relative
legged odometry, Oak-D quaternion when available, command values, servo state,
and status message.

## Validation Notes

- The BNO086 Oak-D supports fused quaternion output.
- BMI270 Oak-D units can still run AprilTag servoing, but fused IMU topics may
  be unavailable; visual servo control must not depend on Oak-D IMU.
- All motion should stop on stale tag pose, explicit `Stop`, node shutdown, or
  process failure via joy-manager timeout.
