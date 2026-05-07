# `scripts/run`

Preferred user-facing entrypoints live here. These scripts should stay thin:
they launch the intended runtime path and delegate control/perception logic to
package modules or focused nodes.

## Production Entrypoint

- `run_operator_station.py`: preferred full operator station launcher.

In `--mode full`, it starts:

- `run_oakd_sensor_node.py`: the single OAK-D owner for RGB/depth visualization,
  AprilTag stats, and fused-IMU stabilization topics.
- `run_teleop_stabilized.py`: joystick teleop publisher with camera-Y level
  stabilization enabled by default.
- `run_operator_console.py --no-camera`: web console consuming ROS camera,
  AprilTag, stabilization, and arm telemetry topics without opening OAK-D.

## Focused Runnables

- `run_oakd_sensor_node.py`: ROS OAK-D sensor node. Publishes compressed RGB,
  colorized aligned depth, AprilTag stats, `GAME_ROTATION_VECTOR`, and
  `/oakd/camera_y_level_error`. Default depth mode is mono `400p` at `30 Hz`;
  test higher detail with `--mono-resolution 800p --depth-fps 10`.
- `run_teleop_stabilized.py`: starts `giraf_arm_controller` and publishes
  joystick task-space commands plus stabilization angular velocity from
  `/oakd/camera_y_level_error`. It expects `run_oakd_sensor_node.py` to be
  running separately.
- `run_giraf_arm_controller.py`: lower-level controller node entrypoint.
- `run_giraf_arm_teleop.py`: lower-level plain joystick teleop node entrypoint.
- `run_arm_jparse_teleop.py`: fallback launcher for controller + plain teleop,
  without OAK-D stabilization.
- `run_operator_console.py`: web console wrapper. In production full mode it is
  launched with `--no-camera` so it consumes shared ROS camera topics.
- `run_perception_dashboard.py`: older camera-only OAK-D dashboard fallback.

Historical `RUN_*` scripts live under `scripts/legacy/`. Older IMU balance
experiments live under `scripts/imu/` and are not part of the production
operator-station path.
