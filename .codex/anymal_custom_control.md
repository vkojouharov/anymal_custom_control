# anymal_custom_control

## Standard Usage

This package is now organized around a small set of preferred runnables under
`catkin_ws/src/anymal_custom_control/scripts/run/`.

The main arm teleop path is:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_arm_jparse_teleop.py
```

That launches:

- `giraf_arm_controller`
- `giraf_arm_teleop`

The current camera-only dashboard path is:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_perception_dashboard.py
```

The current unified operator console path is:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_operator_console.py
```

The combined launcher path is:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_operator_station.py
```

Useful mode flags:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_operator_station.py --mode teleop
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_operator_station.py --mode perception
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_operator_station.py --mode full
```

## Bring-Up

Before running the new stack:

1. Source the catkin workspace and any robot environment you normally use.
2. Ensure `ROS_MASTER_URI` is pointed at the intended ANYmal ROS master.
3. Start any required motor / hardware support processes such as `candle_ros`.
4. Ensure the Dynamixel adapter and joystick are available to the container or
   host process running the scripts.

Important ROS note:

- Node registration goes through the current `ROS_MASTER_URI`.
- Topic traffic is peer-to-peer after discovery.
- The new arm controller and teleop nodes are expected to run against the same
  ROS master you already use for ANYmal.

## Current Processes

### Teleop

Node:

- `giraf_arm_teleop`

Run directly with:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_giraf_arm_teleop.py
```

Responsibilities:

- read joystick input
- publish teleop task-space velocity commands
- publish gripper velocity commands
- send stop request on `X`
- publish zero command whenever motion is not actively enabled

Current joystick behavior:

- deadman is `LB + RB`
- `LY -> vx`
- `LX -> vy`
- `RT/LT -> vz`
- `RY -> wy`
- `RX -> wz`
- `A/B -> gripper velocity`
- `X -> stop request and exit`

Current limitation:

- the controller accepts full 6-DoF task velocity, but the current teleop
  publisher does not bind `wx` yet

### Control

Node:

- `giraf_arm_controller`

Run directly with:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_giraf_arm_controller.py
```

Responsibilities:

- subscribe to teleop and auto task-space commands
- choose the active command source
- run the J-PARSE task-space to joint-space control loop
- enforce joint, boom, wrist, and gripper limits
- command MD80 and Dynamixel hardware
- publish controller state and debug output
- shut down motors on stop / shutdown

Current source-selection behavior:

- default command source is `teleop`
- auto commands are subscribed and stored, but not selected by default yet

### Perception / Web

There are now two browser-facing paths:

- `run_operator_console.py`
- `run_perception_dashboard.py`

Use `run_operator_console.py` for the current operator-facing web UI.

That currently wraps:

- `scripts/operator_console/operator_console.py`

The operator console currently shows:

- GIRAF arm telemetry from `/giraf_arm/state`
- GIRAF event/debug log from `/giraf_arm/debug`
- OAK-D RGB
- aligned depth
- AprilTag summaries
- ANYmal placeholder panel

Use `run_perception_dashboard.py` only when you want the older camera-only
dashboard.

That still wraps:

- `scripts/camera/depthai_apriltag_stream.py`

So the operator station now points at the unified operator console, while the
older perception dashboard remains available as a fallback.

## ROS Interfaces

Current arm command topics:

- `/giraf_arm/teleop_task_velocity_cmd`
- `/giraf_arm/auto_task_velocity_cmd`
- `/giraf_arm/teleop_gripper_velocity_cmd`
- `/giraf_arm/auto_gripper_velocity_cmd`
- `/giraf_arm/stop`

Current controller outputs:

- `/giraf_arm/state`
- `/giraf_arm/debug`

Command message types:

- task velocity: `geometry_msgs/TwistStamped`
- gripper velocity: `std_msgs/Float64`
- stop: `std_msgs/Bool`

Current safety behavior:

- stale selected command -> zero task velocity
- deadman released in teleop -> zero publish
- `X` in teleop -> publish zero, send stop request, exit teleop
- stop request in controller -> latch stop and shut down motors

Telemetry note:

- non-finite values in web-facing JSON are sanitized to `null`
- for example, an untouched auto command age appears as empty/null in the web
  console rather than `Infinity`

## Scripts Layout

### `scripts/run`

Preferred launchers live here.

Important files:

- `run_arm_jparse_teleop.py`
- `run_giraf_arm_controller.py`
- `run_giraf_arm_teleop.py`
- `run_operator_console.py`
- `run_operator_station.py`
- `run_perception_dashboard.py`

### `scripts/operator_console`

Unified operator-facing web apps live here.

Current app:

- `operator_console.py`

### `scripts/tools`

Diagnostics and validation tools live here.

Examples:

- `depthai_probe.py`
- `depthai_imu_terminal_debug.py`
- `depthai_accel_gyro_test.py`
- `discover_topics.py`
- `monitor.py`
- `probe_msg.py`
- `preflight_cbf_bench.py`
- `test_jparse_controller.py`

### `scripts/legacy`

Old flat runnable scripts were moved here as reference.

Examples:

- historical `RUN_*` teleop scripts
- historical GIRAF combined teleop scripts
- `AUTO_anymal_arm_traj.py`

These are no longer the preferred interface.

## What To Try First

For the new arm stack:

1. confirm the ROS environment is sourced and pointed at the intended master
2. bring up motor dependencies like `candle_ros`
3. run:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_arm_jparse_teleop.py
```

Then inspect:

- controller / teleop terminal output
- `rostopic list | grep giraf_arm`
- `rostopic echo /giraf_arm/state`

For perception only:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_perception_dashboard.py
```

For the unified operator console:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_operator_console.py
```

For the full operator station:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_operator_station.py
```

Then inspect:

- the browser console at `http://localhost:5004`
- `/stats` if the web UI appears stale
- `rostopic echo /giraf_arm/state -n 1` if arm telemetry is missing

## Known Next Steps

- add explicit teleop / auto mode switching inside the controller
- decide whether `stop` should become a service instead of a topic
- bind `wx` in teleop if needed
- extract shared perception modules out of the current camera dashboard script
- add legacy wrappers only if you still need old paths to remain callable
