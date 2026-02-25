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
  - **Rebuild required** when adding new catkin packages (e.g., message definitions). Script changes in `anymal_custom_control` are live via the volume mount.

### Catkin Workspace (`catkin_ws/src/`)
- `anymal_custom_control/` — custom scripts and source (volume-mounted, live editing)
- `joy_manager_msgs/` — copied from ANYmal research software (provides `AnyJoy.msg`)
- `operational_mode_manager_msgs/` — **created by us** (reverse-engineered from robot, see below)

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

## Joy Manager Internals

- Located at `~/ANYmal/anymal-research-software/user_interface/input_devices/any_joy/joy_manager/`
- Uses **pluginlib** to load modules dynamically
- The `modules`/`commands` fields in AnyJoy are for sending commands to loaded plugins
- **The `lpc` module approach does NOT work** — no `lpc` plugin is loaded in our configuration
- The joy_manager launch file defaults to `modules="[]"` (empty)
- For mode switching, use the `operational_mode_manager` action instead (see above)
- The joy_manager config (`joy_manager.yaml`) defines: joystick input topic (`/anyjoy/operator`), twist/pose outputs, and a `protective_stop` service at `/motion_control_manager/protective_stop`

## Scripts

### `scripts/teleop.py`
- Basic WASD teleop — velocity only, no mode switching
- Uses `MovementController` from `src/anymal_custom_control/movement.py`
- Requires robot to already be in locomotion mode (set via GUI)

### `scripts/teleop_switchmodes.py`
- WASD teleop + mode switching (1/2/3 keys)
- Two separate publishers:
  1. `AnyJoy` at 10 Hz on `/anyjoy/operator` for velocity
  2. `SwitchOperationalModeActionGoal` on `/operational_mode_manager/switch_operational_mode/goal` for mode changes
- Self-contained (does not use `MovementController`)

### `scripts/probe_msg.py`
- Diagnostic tool to extract ROS message definitions from publishers via TCPROS
- Useful for reverse-engineering message formats from proprietary packages on the robot

## Key Lessons Learned

1. **Mode switching uses the operational_mode_manager, NOT joy_manager modules/commands.** The joy_manager `modules`/`commands` fields require loaded plugins which aren't present in our config.
2. **Message packages from ANYbotics proprietary stack** aren't in the research software. They exist only on the robot. Use the probe script to reverse-engineer definitions.
3. **MD5 sums must match exactly.** Even one missing field causes ROS to drop the connection. The TCPROS probe approach is the reliable way to get exact definitions.
4. **Docker rebuild is needed for new catkin packages** but not for script changes (volume mount).
5. **Robot nodes**: LPC (`anymal-d181-lpc`) runs user interaction relay; NPC (`anymal-d181-npc`) runs operational_mode_manager, motion_control_manager, etc.

## Future Work / Notes

- Sensor topics (cameras, LiDAR, point clouds, SLAM maps) — not yet explored
- The `motion_transitioner_msgs` package may also need to be created locally if direct motion state control is needed
- Available ROS services for controller info:
  - `anymal_msgs/SwitchController.srv` — `string name`, `bool block` → `int8 status`
  - `anymal_msgs/GetActiveController.srv` — returns `active_controller_name`, `active_controller_locomotion_mode`
  - `anymal_msgs/GetAvailableControllers.srv` — returns `string[] available_controllers`
