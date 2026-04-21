# Dynamixel Stack Report

## Executive summary

This repo uses the Python `dynamixel-sdk` package as the low-level transport for Dynamixel motors. That SDK is wrapped by a small in-repo module at `catkin_ws/src/anymal_custom_control/src/anymal_custom_control/dynamixel/`, and the active teleop scripts call that wrapper rather than talking to the SDK directly.

In the current stack, Dynamixels are used for the GIRAF wrist and gripper, while the larger arm joints use the separate MD80 stack over `candle_ros`. The active wrist scripts compute desired joint motion in software, convert those desired wrist states into Dynamixel ticks, and push a single sync-write packet each control cycle. The scripts do not currently use Dynamixel feedback in the control loop; they track desired state locally and rely on the servo's onboard position controller to follow the commanded targets.

The current implementation is centered around a 3-joint wrist plus 1 gripper:

- Wrist joint IDs: `11`, `12`, `13`
- Gripper ID: `14`
- Default serial device: `/dev/ttyUSB0`
- Protocol: `2.0`
- Default library baud: `57600`
- Active wrist teleop baud: `1_000_000`

## Dependency and runtime path

The dependency chain is straightforward:

1. `requirements.txt` declares `dynamixel-sdk`.
2. `Dockerfile` copies `requirements.txt` and runs `pip3 install -r /requirements.txt`.
3. `docker-compose.yml` passes the U2D2 serial adapter through as `/dev/ttyUSB0`.
4. Python code imports `dynamixel_sdk` from the installed package.

So the effective runtime path is:

`requirements.txt` -> `pip install dynamixel-sdk` -> in-repo wrapper module -> teleop scripts

This is not a ROS-native Dynamixel stack. The serial bus is driven directly from Python through the SDK. ROS is involved elsewhere in the project, but the Dynamixel layer itself is not built as a ROS action/server abstraction.

## Where the current implementation lives

The current Dynamixel code is concentrated in these files:

- `catkin_ws/src/anymal_custom_control/src/anymal_custom_control/dynamixel/__init__.py`
- `catkin_ws/src/anymal_custom_control/src/anymal_custom_control/dynamixel/control_table.py`
- `catkin_ws/src/anymal_custom_control/src/anymal_custom_control/dynamixel/controller.py`
- `catkin_ws/src/anymal_custom_control/src/anymal_custom_control/dynamixel/driver.py`
- `catkin_ws/src/anymal_custom_control/scripts/dynamixel/scan_ids.py`

The main current script entry points using that stack are:

- `catkin_ws/src/anymal_custom_control/scripts/RUN_arm_wrist_teleop.py`
- `catkin_ws/src/anymal_custom_control/scripts/RUN_arm_wrist_CBF_teleop.py`
- `catkin_ws/src/anymal_custom_control/scripts/RUN_giraf_wrist_teleop.py`

There is also an older script:

- `catkin_ws/src/anymal_custom_control/scripts/RUN_task_space_teleop.py`

That older script appears to belong to a previous 3-finger gripper implementation and does not match the current package layout.

## Layer 1: public package API

`anymal_custom_control.dynamixel.__init__.py` is the public entry point. It re-exports:

- lifecycle helpers: `dynamixel_connect`, `dynamixel_disconnect`
- command helpers: `dynamixel_drive`, `dynamixel_drive_arm`, `dynamixel_drive_gripper`, `dynamixel_home`
- feedback helpers: `dynamixel_read`, `dynamixel_status`
- conversion helpers: `radians_to_ticks`, `ticks_to_radians`
- constants: `ARM_IDS`, `GRIPPER_IDS`, `ALL_IDS`, `ARM_HOME`, `GRIPPER_OPEN`, `GRIPPER_STROKE`, `TICKS_PER_REV`
- low-level escape hatch: `DynamixelController`

The design intent is explicit in the docstring: the Dynamixel API is shaped to feel similar to `motor_driver.py` for the MD80 arm. That lets teleop scripts hold one MD80 context and one Dynamixel context side-by-side with similar calling conventions.

## Layer 2: control table and project configuration

`control_table.py` combines two concerns:

- the X-series register map and operating mode constants
- this project's actual motor IDs and home/open reference positions

### Register definitions

The file stores register metadata as `(address, byte_length)` tuples, for example:

- `OPERATING_MODE = (11, 1)`
- `TORQUE_ENABLE = (64, 1)`
- `GOAL_POSITION = (116, 4)`
- `PRESENT_POSITION = (132, 4)`
- `PRESENT_VELOCITY = (128, 4)`
- `PWM_LIMIT = (36, 2)`

These tuples are consumed by the low-level controller and by the sync-read/sync-write handle factories.

### Operating mode

The driver uses `OP_EXTENDED_POSITION = 4` for all configured motors. That means:

- goals are sent as 4-byte signed values
- commands are not restricted to `0..4095`
- negative values and multi-turn values are valid

This is why the driver can treat arm joints and gripper motion uniformly as position targets in ticks, even if the final values exceed a single-turn range.

### Current hardware map

As of the actual code in `control_table.py`, the current ground truth is:

- `ARM_IDS = (11, 12, 13)`
- `GRIPPER_IDS = (14,)`
- `ALL_IDS = (11, 12, 13, 14)`

Current reference ticks:

- `ARM_HOME = {11: 2057, 12: 2331, 13: 1060}`
- `ARM_TICK_LIMITS = {11: (57, 4057), 12: (1000, 3200), 13: (-940, 3060)}`
- `GRIPPER_OPEN = {14: 2330}`
- `GRIPPER_CLOSED = {14: 5910}`
- `GRIPPER_STROKE = -3580`
- `GOAL_TICK_LIMITS = {11: (57, 4057), 12: (1000, 3200), 13: (-940, 3060), 14: (2330, 5910)}`

Gripper close is implemented as:

`closed_ticks = GRIPPER_OPEN[id] - GRIPPER_STROKE`

Because `GRIPPER_STROKE` is negative, closing increases the commanded tick value.

## Layer 3: low-level SDK wrapper

`controller.py` is the thin wrapper around the upstream SDK classes:

- `PortHandler`
- `PacketHandler`
- `GroupSyncWrite`
- `GroupSyncRead`

The class `DynamixelController` is intentionally small. It mainly owns:

- opening and closing the serial port
- setting the baudrate
- simple single-register `READ` and `WRITE`
- construction of sync group handles

### What it does

- `__init__` opens the serial device and sets the baudrate
- `WRITE` chooses 1-, 2-, or 4-byte SDK write calls based on the control-table tuple
- `READ` does the same for reads and converts returned values to signed integers when needed
- `make_sync_write` and `make_sync_read` create reusable group handles
- `close` closes the serial device

### Important detail: signed conversion

The helper `_to_signed(value, length)` converts raw SDK readback from unsigned integers into signed values using two's complement. This matters because:

- present position and velocity may need signed interpretation
- extended position mode can yield values beyond normal single-turn unsigned expectations

The low-level wrapper itself contains almost no policy. It does not know about arm joints, grippers, home positions, or kinematics. Those are handled one layer up.

## Layer 4: functional driver

`driver.py` is the actual policy layer and the main thing scripts use.

It converts "this project wants to command a wrist and gripper" into actual SDK bus operations.

### `dynamixel_connect`

`dynamixel_connect(...)` performs the full initialization sequence:

1. Create `DynamixelController(port, baudrate, protocol)`
2. Build `all_ids = arm_ids + gripper_ids`
3. Optionally reboot each configured motor
4. Sleep for `settle_time`
5. Torque off every motor
6. Write `OPERATING_MODE = OP_EXTENDED_POSITION` to every motor
7. Read back `OPERATING_MODE` from every motor and verify it stuck
8. Write `PWM_LIMIT` for gripper IDs
9. Torque on every motor
10. Prebuild one `GroupSyncWrite` for `GOAL_POSITION`
11. Prebuild two `GroupSyncRead` handles for `PRESENT_POSITION` and `PRESENT_VELOCITY`

This function returns a plain Python context dict, not a class instance. The returned context currently includes:

- `controller`
- `arm_ids`
- `gripper_ids`
- `all_ids`
- `sync_write_pos`
- `sync_read_pos`
- `sync_read_vel`

That dict is the common handle passed to all other `dynamixel_*` helpers.

The final transmitted goals are also clamped against `GOAL_TICK_LIMITS`, so
the driver now acts as the last safety envelope even if a caller overshoots the
configured wrist or gripper range.

### `dynamixel_disconnect`

This function iterates over `ctx['all_ids']`, writes `TORQUE_ENABLE = 0`, and closes the port. It is written to be safe in `finally:` blocks and tolerant of partially initialized state.

### Drive functions

There are three drive layers:

- `dynamixel_drive(ctx, ticks)`
- `dynamixel_drive_arm(ctx, j1, j2, j3)`
- `dynamixel_drive_gripper(ctx, g=1.0)`

`dynamixel_drive` is the lowest active layer. It expects one tick target per motor, in `ctx['all_ids']` order, and writes the full set in one sync-write packet.

`dynamixel_drive_arm` is a convenience wrapper that:

- takes arm deltas in radians from home
- converts radians to ticks with `4096 ticks / rev`
- adds each result to `ARM_HOME`

`dynamixel_drive_gripper` is another convenience wrapper that:

- takes an open fraction in `[0, 1]`
- clamps it
- converts it into absolute goal ticks from `GRIPPER_OPEN` and `GRIPPER_STROKE`

### Internal sync-write behavior

All three drive paths eventually use `_drive_subset(...)`, which:

1. adds each target as a 4-byte signed little-endian value to the SDK group write handle
2. sends one packet with `txPacket()`
3. clears the group parameters immediately after sending

That means the project's intended normal mode is batched bus writes, not one register write per motor.

### Feedback helpers

`dynamixel_read(ctx)` sync-reads:

- `PRESENT_POSITION`
- `PRESENT_VELOCITY`

and returns a dict keyed by motor ID. Missing responses are returned as `None`, not raised as exceptions. This is a deliberate API choice: callers can decide whether to retry, degrade gracefully, or treat missing motors as fatal.

`dynamixel_status(ctx)` is a simple printer around `dynamixel_read(ctx)`.

### Self-test

`driver.py` also includes a `main()` function that:

- connects
- homes the motors
- waits
- prints status
- disconnects

That is effectively a minimal smoke test for wiring and basic motion.

## Layer 5: direct SDK utility script

`scripts/dynamixel/scan_ids.py` bypasses the custom wrapper and uses the SDK directly. That is intentional: it is a bus-level diagnostics tool, not a teleop/control layer.

It uses:

- `PortHandler`
- `PacketHandler`
- `broadcastPing`
- optional per-ID `ping`

Its purpose is to answer:

- is the adapter visible
- which motor IDs respond
- what baudrate is the bus currently using
- which model numbers are present

This is the fastest script to run after wiring changes or motor reconfiguration.

## How the active control scripts actually use the stack

The current active wrist scripts all follow the same pattern:

- connect joystick input
- connect MD80 arm motors through `motor_driver`
- connect Dynamixels through `dynamixel_connect(...)`
- solve for desired wrist motion in software
- integrate desired wrist joint states locally
- convert desired wrist state to absolute Dynamixel ticks
- issue one combined `dynamixel_drive(...)` per cycle

### Active scripts

The current active scripts are:

- `RUN_arm_wrist_teleop.py`
- `RUN_arm_wrist_CBF_teleop.py`
- `RUN_giraf_wrist_teleop.py`

The difference between them is mainly in how they compute the desired joint velocities:

- `RUN_arm_wrist_teleop.py` uses a Jacobian pseudoinverse
- `RUN_arm_wrist_CBF_teleop.py` uses a CBF-QP controller for singularity avoidance
- `RUN_giraf_wrist_teleop.py` combines ANYmal and arm control modes, but the final Dynamixel command path is the same

### Current wrist command flow

Inside these scripts:

1. Joystick commands become desired end-effector or wrist velocities.
2. The script solves for desired wrist joint velocities.
3. The script integrates those velocities into local variables such as `theta4_pos`, `theta5_pos`, `theta6_pos`, and `gripper_pos`.
4. The scripts clamp `theta4_pos`, `theta5_pos`, and `theta6_pos` against radian limits derived from `ARM_TICK_LIMITS`.
5. The helper `_dxl_ticks(...)` converts that desired state into absolute ticks.
6. The script calls `dynamixel_drive(dxl_ctx, ticks)` once per loop.

The loop period is `DT = 0.005`, so the command rate is about `200 Hz`.

### Important control characteristic: open-loop at the script layer

The current wrist scripts do not call `dynamixel_read()` inside their main control loops. They do not close the loop on measured wrist position from the bus.

Instead, they:

- maintain a desired internal state in Python
- send that desired state to the servos
- rely on the Dynamixel internal controller to track the commanded `GOAL_POSITION`

So the software stack is "closed-loop inside the servo, open-loop in the Python teleop layer."

That is an important design point for anyone adding safety checks, calibration logic, or higher-accuracy control.

### Why the scripts use `dynamixel_drive(...)` instead of higher-level helpers

The active wrist scripts need to send arm wrist joints and gripper position together in one packet. For that reason they build a combined tick list with `_dxl_ticks(...)` and call the low-level batched function directly.

They do not currently use `dynamixel_drive_arm(...)` and `dynamixel_drive_gripper(...)` as separate calls.

## Relationship to the MD80 stack

The overall arm stack is split across two independent actuator systems:

- MD80s for roll, pitch, and boom over `candle_ros`
- Dynamixels for wrist joints and gripper over USB serial and `dynamixel-sdk`

The in-repo Dynamixel API was deliberately shaped to mirror `motor_driver.py`, which is why the teleop scripts can do:

- `motor_connect()` / `motor_drive()` / `motor_disconnect()`
- `dynamixel_connect()` / `dynamixel_drive()` / `dynamixel_disconnect()`

This is a useful mental model: the repo treats MD80 and Dynamixel as sibling actuator backends with similar Python ergonomics but completely different transports and runtime dependencies.

## Current hardware and configuration assumptions

The current code assumes:

- a U2D2 or compatible serial adapter is available at `/dev/ttyUSB0`
- Protocol 2.0 compatible X-series motors
- all configured motors can be put into extended position mode
- gripper force limiting is done with `PWM_LIMIT`

Important baudrate nuance:

- `dynamixel_connect` defaults to `57600`
- the current active wrist scripts explicitly call `dynamixel_connect(baudrate=1_000_000)`
- `scan_ids.py` defaults to `57600`, but can scan all common bauds

So if motors have been reconfigured to `1_000_000`, the active teleop scripts are aligned with that setup, but some tools and defaults still assume factory baud unless told otherwise.

## Safety and cleanup behavior

The Dynamixel driver itself does not implement a high-level safety state machine, but it does provide several important safety-related behaviors:

- torque is disabled before operating mode changes
- operating mode is read back and verified
- gripper PWM is capped at connect time
- disconnect torque-offs every configured motor
- active scripts place disconnect in `finally:` blocks

The wrist teleop scripts add their own higher-level safety behavior:

- dead-man switch logic
- emergency stop button
- joint/gripper clamping
- thread shutdown and cleanup

## Known legacy area

The main remaining place where developers should be careful not to confuse old
assumptions with the current stack is `RUN_task_space_teleop.py`.

### `RUN_task_space_teleop.py` looks like an older generation

This script imports:

- `dynamixel_driver`
- `control_table`

and refers to a 3-finger gripper setup using constants such as `MOTOR100_*`, `MOTOR101_*`, and `MOTOR102_*`.

That does not match the current package structure or current hardware map. It should be treated as a historical reference, not as the authoritative current control path.

## What a new developer should know first

If you are new to this stack, the shortest accurate mental model is:

1. Install dependencies from `requirements.txt`; that gives you `dynamixel-sdk`.
2. The real implementation lives in `anymal_custom_control.dynamixel`.
3. `controller.py` wraps the SDK primitives.
4. `driver.py` adds project policy: IDs, mode setup, sync-read/write, home/open conventions, gripper PWM cap.
5. Current active scripts command wrist joints and gripper by computing desired state in Python, clamping against wrist limits derived from `ARM_TICK_LIMITS`, converting to ticks, and sync-writing `GOAL_POSITION` at about `200 Hz`.
6. The active script layer is not using measured Dynamixel feedback for closed-loop control.
7. `control_table.py` is the current source of truth for IDs and home/open positions.
8. The package README is now aligned with the current 1-gripper hardware; the main remaining legacy reference is `RUN_task_space_teleop.py`.

## Suggested maintenance priorities

If this stack is going to keep evolving, the highest-value cleanup items are:

- either remove or clearly mark `RUN_task_space_teleop.py` as legacy
- decide whether the default library baud should be changed from `57600` to `1_000_000` to match current active scripts
- decide whether wrist control should remain open-loop at the script layer or start incorporating `dynamixel_read()` feedback
