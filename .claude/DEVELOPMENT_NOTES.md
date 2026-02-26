# ANYmal Custom Control — Development Notes

## Project Overview

Custom teleop and control scripts for an ANYmal D robot (anymal-d181).
Runs inside a Docker container (`anymal_custom_control`) that connects to the robot over the network.

## Architecture

### Docker Setup
- **Dockerfile**: `~/anymal_custom_control/Dockerfile` — based on `ros:noetic-ros-base-focal`
- **docker-compose.yml**: `~/anymal_custom_control/docker-compose.yml`
  - `network_mode: host`, `ROS_MASTER_URI: http://anymal-d181-lpc:11311`, `ROS_IP: 192.168.0.225`
  - Volume mount: `./catkin_ws/src/anymal_custom_control` → `/catkin_ws/src/anymal_custom_control:ro` (live editing without rebuild)
  - Other packages (message definitions) are baked into the image via `COPY catkin_ws/src` + `catkin_make`
  - **Rebuild required** when adding new catkin packages (e.g., message definitions) or changing `package.xml`/`CMakeLists.txt`. Script changes in `anymal_custom_control` are live via the volume mount.

### Catkin Workspace (`catkin_ws/src/`)
- `anymal_custom_control/` — custom scripts and Python library (volume-mounted, live editing)
- `joy_manager_msgs/` — copied from ANYmal research software (provides `AnyJoy.msg`)
- `operational_mode_manager_msgs/` — **created by us** (reverse-engineered from robot, see below)

### Catkin Package Structure (`anymal_custom_control/`)
```
anymal_custom_control/
├── scripts/                        # Executable scripts (run directly)
│   ├── teleop_switchmodes.py       # Main teleop: WASD + mode switching
│   ├── discover_topics.py          # List available camera/control/state topics
│   ├── monitor.py                  # Live topic dashboard (read-only)
│   └── probe_msg.py                # Extract msg definitions from publishers
├── src/anymal_custom_control/      # Python library (importable)
│   ├── __init__.py                 # Exports: ModeController, MovementController, CameraReceiver
│   ├── movement.py                 # MovementController — velocity via AnyJoy at 10 Hz
│   ├── modes.py                    # ModeController — operational mode switching
│   ├── camera.py                   # CameraReceiver — ROS Image topic → numpy arrays
│   └── joystick_driver.py          # Physical gamepad input via pygame + tkinter GUI
├── setup.py                        # Catkin Python package setup
├── package.xml                     # ROS package manifest
└── CMakeLists.txt                  # Build config
```

- **`scripts/`** = things you *run* (`python3 teleop_switchmodes.py`)
- **`src/anymal_custom_control/`** = things you *import* (`from anymal_custom_control import ModeController`)
- The two `src/` levels are a catkin convention: `catkin_ws/src/` holds packages, `<package>/src/` holds Python libraries

### ANYmal Research Software
- Located at `~/ANYmal/anymal-research-software/`
- Contains source for joy_manager, motion controllers, example controllers, etc.
- `joy_manager_msgs` originated from: `~/ANYmal/anymal-research-software/user_interface/input_devices/any_joy/joy_manager_msgs/`

## ROS Topics & Interfaces

### Velocity Control
- **Topic**: `/anyjoy/operator`
- **Type**: `joy_manager_msgs/AnyJoy`
- **Axis mapping** (from `Output.hpp`):
  - `axes[0]` = LATERAL (left/right strafe)
  - `axes[1]` = HEADING (forward/backward)
  - `axes[2]` = ROLL
  - `axes[3]` = TURNING (rotation)
  - `axes[4]` = VERTICAL
  - `axes[5]` = PITCH
- The joy_manager processes these axes and publishes Twist/Pose outputs
- joy_manager has a **0.5s timeout** — must publish continuously or robot stops

### Mode Switching (Operational Modes)
- **Topic**: `/operational_mode_manager/switch_operational_mode/goal`
- **Type**: `operational_mode_manager_msgs/SwitchOperationalModeActionGoal`
- **Goal structure**: `msg.goal.target.name = "Stand"` (string), plus `msg.goal.requester_id` (string, optional)
- **Known mode names** (case-sensitive):
  - `"Rest"` — sit/rest (key 1 in teleop)
  - `"Stand"` — standing (key 2)
  - `"Walk"` — locomotion, enables WASD movement (key 3)
- Published as a one-shot message (not repeated like velocity)
- The `operational_mode_manager` node runs on the NPC (`anymal-d181-npc`) and internally sends goals to `/motion_state_deduction/go_to_motion_state/goal`

### Motion State Deduction
- **Topic**: `/motion_state_deduction/go_to_motion_state/goal`
- **Type**: `motion_transitioner_msgs/GoToMotionStateActionGoal`
- Used internally by the `operational_mode_manager` — generally don't publish here directly
- Motion states include: `stand`, `freeze`, `square_up`, `torso_control`, `_stand_upright`, etc.
- Configured via `motion_transitioner` YAML (see oscillator controller config for example)

### State Notification
- **Topic**: `/operational_mode_manager/state_notification`
- **Type**: `operational_mode_manager_msgs/OperationalModeNotification`
- Publishes current operational mode state

## Python Library (`anymal_custom_control`)

### `MovementController` (movement.py)
Publishes AnyJoy velocity commands at 10 Hz to `/anyjoy/operator`.
```python
from anymal_custom_control import MovementController

mc = MovementController(topic='/anyjoy/operator')
mc.start()                  # start 10 Hz publish thread
mc.forward(0.3)             # 30% max forward velocity
mc.set_velocity(heading=0.5, turning=0.3)  # direct axis control
mc.stop()                   # zero all velocities
mc.shutdown()               # stop publish thread
```
- Thread-safe, context manager support (`with MovementController() as mc:`)
- Convenience methods: `forward()`, `backward()`, `strafe_left()`, `strafe_right()`, `turn_left()`, `turn_right()`
- Values are [-1.0, 1.0] scale factors, clamped automatically

### `ModeController` (modes.py)
Switches operational modes via the `operational_mode_manager`.
```python
from anymal_custom_control import ModeController, MovementController

mc = MovementController()
mc.start()
modes = ModeController(movement_controller=mc)  # auto-zeros velocity on switch
modes.switch_mode(ModeController.WALK)
modes.switch_mode(ModeController.REST)
print(modes.current_mode)   # "Rest"
```
- Class constants: `ModeController.REST`, `.STAND`, `.WALK`
- Optional `movement_controller` param — if provided, `stop()` is called before every mode switch (safety)
- One-shot publishes (no background thread)
- Raises `ValueError` for invalid mode names

### `CameraReceiver` (camera.py)
Subscribes to ROS Image topics and provides frames as numpy arrays.
```python
from anymal_custom_control import CameraReceiver

cam = CameraReceiver('/depth_camera_front/color/image_raw')
frame = cam.get_frame()      # numpy array (H, W, 3) BGR uint8
depth = cam.get_depth()      # numpy array (H, W) uint16 for depth topics
```
- Not yet actively used — scaffolding for future sensor work

## Message Package: operational_mode_manager_msgs

**Created locally** because this package only exists as pre-built binaries on the robot (proprietary ANYbotics software). Not in the research software source tree.

### How it was reverse-engineered
1. `rostopic type` confirmed the message type name
2. `rosnode info /operational_mode_manager` showed all pub/sub topics and types
3. A TCPROS probe script (`scripts/probe_msg.py`) connected directly to the publisher with `md5sum=*` wildcard and extracted the full message definition from the connection header
4. The definition was used to create matching `.msg` and `.action` files

### Message definitions
- `msg/OperationalMode.msg`: `string name`
- `action/SwitchOperationalMode.action`:
  ```
  # Goal
  OperationalMode target
  string requester_id
  ---
  # Result
  bool success
  ---
  # Feedback
  string status
  ```
- **MD5 (ActionGoal)**: `980d2edba015b77191e76e680d8c91b6`
- First attempt was missing `requester_id` field, causing MD5 mismatch. The probe script was key to diagnosing this.

### Probe Script
`scripts/probe_msg.py` — reusable tool for extracting message definitions from any ROS topic publisher via TCPROS. Useful when message packages aren't installed locally. Usage:
```bash
python3 probe_msg.py /some/topic/name
```

## Scripts

### `scripts/teleop_switchmodes.py` (main teleop)
- WASD teleop + mode switching (1/2/3 keys)
- Uses library modules: `MovementController` for velocity, `ModeController` for mode switching
- ~150 lines — thin UI wrapper around the library

### `scripts/discover_topics.py`
- Connects to ROS master and lists available camera, control, and state topics
- Prints usage hints with topic names for `CameraReceiver` and `MovementController`

### `scripts/monitor.py`
- Live dashboard — subscribes to key topics and prints stats (Hz, values)
- Read-only / safe — never publishes

### `scripts/probe_msg.py`
- Diagnostic tool to extract ROS message definitions from publishers via TCPROS
- Useful for reverse-engineering message formats from proprietary packages on the robot

## Joy Manager Internals

- Located at `~/ANYmal/anymal-research-software/user_interface/input_devices/any_joy/joy_manager/`
- Uses **pluginlib** to load modules dynamically
- The `modules`/`commands` fields in AnyJoy are for sending commands to loaded plugins
- **The `lpc` module approach does NOT work** — no `lpc` plugin is loaded in our configuration
- The joy_manager launch file defaults to `modules="[]"` (empty)
- For mode switching, use the `operational_mode_manager` action instead (see above)
- The joy_manager config (`joy_manager.yaml`) defines: joystick input topic (`/anyjoy/operator`), twist/pose outputs, and a `protective_stop` service at `/motion_control_manager/protective_stop`

## Key Lessons Learned

1. **Mode switching uses the operational_mode_manager, NOT joy_manager modules/commands.** The joy_manager `modules`/`commands` fields require loaded plugins which aren't present in our config.
2. **Message packages from ANYbotics proprietary stack** aren't in the research software. They exist only on the robot. Use the probe script to reverse-engineer definitions.
3. **MD5 sums must match exactly.** Even one missing field causes ROS to drop the connection. The TCPROS probe approach is the reliable way to get exact definitions.
4. **Docker rebuild is needed for new catkin packages** (or `package.xml`/`CMakeLists.txt` changes) but not for script changes (volume mount).
5. **Robot nodes**: LPC (`anymal-d181-lpc`) runs user interaction relay; NPC (`anymal-d181-npc`) runs operational_mode_manager, motion_control_manager, etc.

## Future Work / Notes

- Sensor topics (cameras, LiDAR, point clouds, SLAM maps) — not yet explored. Use `discover_topics.py` to find available topics.
- The `motion_transitioner_msgs` package may also need to be created locally if direct motion state control is needed
- Available ROS services for controller info:
  - `anymal_msgs/SwitchController.srv` — `string name`, `bool block` → `int8 status`
  - `anymal_msgs/GetActiveController.srv` — returns `active_controller_name`, `active_controller_locomotion_mode`
  - `anymal_msgs/GetAvailableControllers.srv` — returns `string[] available_controllers`
