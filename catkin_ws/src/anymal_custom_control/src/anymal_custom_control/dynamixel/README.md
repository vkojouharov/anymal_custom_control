# Dynamixel driver (`anymal_custom_control.dynamixel`)

Python wrapper around the ROBOTIS `dynamixel-sdk` for the GIRAF wrist and
single gripper. The API is intentionally shaped to parallel
`anymal_custom_control.motor_driver` so higher-level teleop code can manage
the MD80 arm and the Dynamixel wrist as separate but similarly-structured
actuator backends.

---

## Current hardware map

This is the current source-of-truth configuration from `control_table.py`.

| Role | Motor ID | Home / reference tick | Absolute tick limits |
| ---- | -------- | --------------------- | -------------------- |
| Arm joint 1 (`th4`) | `11` | `2057` | `57 .. 4057` |
| Arm joint 2 (`th5`) | `12` | `2331` | `1000 .. 3200` |
| Arm joint 3 (`th6`) | `13` | `1060` | `-940 .. 3060` |
| Gripper | `14` | `2330` open | `2330 .. 5910` |

Gripper travel is represented as:

- fully open: `2330`
- fully closed: `5910`
- `GRIPPER_STROKE = -3580`

The wrist control scripts work in radians from each motor's home tick. The
driver converts those commands back to absolute ticks and clamps every goal
against the configured limits before transmitting.

---

## Dependency and runtime path

The stack is:

1. `requirements.txt` installs `dynamixel-sdk`
2. `Dockerfile` installs Python dependencies with `pip3`
3. `docker-compose.yml` maps `/dev/ttyUSB0` into the container
4. `anymal_custom_control.dynamixel` wraps the SDK for project use
5. Active teleop scripts call that wrapper

This is a direct Python SDK integration, not a ROS-native Dynamixel node.

---

## Public API

```python
from anymal_custom_control.dynamixel import (
    dynamixel_connect,
    dynamixel_disconnect,
    dynamixel_drive,
    dynamixel_drive_arm,
    dynamixel_drive_gripper,
    dynamixel_home,
    dynamixel_read,
    dynamixel_status,
    radians_to_ticks,
    ticks_to_radians,
    DynamixelController,
    ARM_IDS,
    ARM_HOME,
    ARM_TICK_LIMITS,
    GRIPPER_IDS,
    GRIPPER_OPEN,
    GRIPPER_CLOSED,
    GRIPPER_STROKE,
    GOAL_TICK_LIMITS,
)
```

### `dynamixel_connect(...)`

```python
ctx = dynamixel_connect(
    port='/dev/ttyUSB0',
    baudrate=57600,
    protocol=2.0,
    arm_ids=ARM_IDS,
    gripper_ids=GRIPPER_IDS,
    gripper_pwm_limit=300,
    reboot=True,
    settle_time=2.0,
)
```

Connect sequence:

1. Open the serial port
2. Optionally reboot all configured motors
3. Torque-off every motor
4. Set `OPERATING_MODE = OP_EXTENDED_POSITION`
5. Read back and verify that mode
6. Apply the gripper `PWM_LIMIT`
7. Torque-on every motor
8. Create reusable sync-write and sync-read handles

### `dynamixel_drive(ctx, ticks)`

Low-level goal-position sync-write. `ticks` must match `ctx['all_ids']` order.
Before transmission, each target is clamped against the configured per-motor
tick limits in `GOAL_TICK_LIMITS`.

### `dynamixel_drive_arm(ctx, j1, j2, j3)`

Command the three wrist joints in radians from the home ticks in `ARM_HOME`.
The driver converts radians to ticks, adds the home offsets, and then applies
the final per-motor tick clamp.

### `dynamixel_drive_gripper(ctx, g=1.0)`

Command the single gripper as an open fraction:

- `1.0` = fully open
- `0.0` = fully closed

The driver maps that fraction onto the configured open/closed tick range and
then applies the final tick clamp.

### `dynamixel_read(ctx)`

Sync-reads present position and present velocity for every configured motor and
returns a dict keyed by motor ID. Missing responses are returned as `None`.

### `dynamixel_disconnect(ctx)`

Torque-offs every configured motor and closes the serial port. Safe to call in
`finally:` blocks.

---

## Module layout

### `control_table.py`

Contains:

- Dynamixel X-series register tuples
- operating mode constants
- project motor IDs
- home ticks
- wrist and gripper tick limits

### `controller.py`

Thin wrapper around:

- `PortHandler`
- `PacketHandler`
- `GroupSyncWrite`
- `GroupSyncRead`

This layer provides basic register read/write access and sync group creation.

### `driver.py`

Project-level policy layer. It handles:

- connect/disconnect lifecycle
- mode setup and verification
- sync-write/sync-read handle construction
- radians-to-ticks conversion
- gripper fraction-to-ticks conversion
- final goal clamping against configured hardware limits

---

## Active script usage

The active wrist scripts are:

- `scripts/RUN_arm_wrist_teleop.py`
- `scripts/RUN_arm_wrist_CBF_teleop.py`
- `scripts/RUN_giraf_wrist_teleop.py`

Their pattern is:

1. solve desired wrist motion in software
2. integrate desired wrist joint state locally
3. clamp the wrist joints against the configured tick-derived joint limits
4. convert the desired wrist state to absolute ticks
5. call `dynamixel_drive(...)` once per control cycle

These scripts currently run at `DT = 0.005`, or about `200 Hz`.

The script layer is not closing the loop on measured Dynamixel position. It
tracks desired state locally and relies on the servo's own onboard position
control to track the commanded goals.

---

## Diagnostics

Use `scripts/dynamixel/scan_ids.py` to validate the bus after wiring or
hardware changes. It talks directly to the SDK and can scan one baud or try
all common bauds.

Examples:

```bash
python3 scripts/dynamixel/scan_ids.py --baudrate 1000000
python3 scripts/dynamixel/scan_ids.py --all-bauds
```

---

## Notes and gotchas

- Factory baud is still usually `57600`; active wrist teleop scripts currently
  connect at `1_000_000`, so the motors must already be configured for that.
- The driver uses extended position mode, so negative and multi-turn tick
  values are valid at the protocol level.
- The configured wrist and gripper tick limits are the final safety envelope
  before each sync-write.
- Put `dynamixel_disconnect(ctx)` in a `finally:` block; otherwise `/dev/ttyUSB0`
  may remain occupied until the process exits cleanly.
