# Dynamixel driver (`anymal_custom_control.dynamixel`)

Python wrapper around the ROBOTIS Dynamixel SDK for the GIRAF arm +
three-finger gripper on ANYmal. Designed to look and feel like the MAB MD80
driver (`motor_driver.py`), so teleop scripts can mix both buses without
context-switching on API style.

---

## Hardware assumed

| Role         | Motor ID | Model         | Home / open ticks |
| ------------ | -------- | ------------- | ----------------- |
| Arm joint 1  | 11       | XH-430-W250-T | 2044              |
| Arm joint 2  | 12       | XH-430-W250-T | 3860              |
| Arm joint 3  | 13       | XH-430-W250-T | 4160              |
| Gripper 1    | 100      | XH-430-W250-T | 1700 (open)       |
| Gripper 2    | 101      | XH-430-W250-T |  100 (open)       |
| Gripper 3    | 102      | XH-430-W250-T | 2100 (open)       |

- U2D2 (or compatible) USB-to-TTL adapter, enumerating at `/dev/ttyUSB0`.
- Protocol 2.0, default baud `57600`. All motors in **extended position
  control** (mode 4) — supports single- and multi-turn targets.
- Gripper close stroke: `OPEN - 4000` ticks (~one revolution of range).

If your IDs or home positions differ, edit [`control_table.py`](control_table.py)
or override per-call with `dynamixel_connect(arm_ids=..., gripper_ids=...)`.

---

## Install / prerequisites

```bash
pip install dynamixel-sdk
```

Linux serial permission (inside the Docker container or host):

```bash
sudo usermod -aG dialout $USER      # once, then log out / log back in
# or, per-session:
sudo chmod 666 /dev/ttyUSB0
```

When running from the `anymal_custom_control` container, make sure the compose
file maps the device:

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
```

---

## Public API

```python
from anymal_custom_control.dynamixel import (
    dynamixel_connect,         # -> ctx
    dynamixel_disconnect,      # ctx
    dynamixel_drive,           # ctx, ticks[6]                     low-level
    dynamixel_drive_arm,       # ctx, j1, j2, j3                   radians
    dynamixel_drive_gripper,   # ctx, g1=1, g2=1, g3=1             0=closed, 1=open
    dynamixel_home,            # ctx                               all → home
    dynamixel_read,            # ctx -> {id: {'position', 'velocity'}}
    dynamixel_status,          # ctx — prints human-readable table
    radians_to_ticks,
    ticks_to_radians,
    DynamixelController,       # low-level SDK wrapper
    ARM_IDS, GRIPPER_IDS, ARM_HOME, GRIPPER_OPEN, GRIPPER_STROKE,
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
    gripper_pwm_limit=300,   # 0..885; caps grip force
    reboot=True,             # adds ~2 s — skip on warm reconnects
    settle_time=2.0,
)
```

Sequence: open port → (optional) reboot all motors → torque-off →
set operating mode to `EXTENDED_POSITION` and read-back verify →
apply gripper PWM cap → torque-on → build `GroupSyncRead/Write` handles.

Raises `RuntimeError` if any motor is missing or misconfigured.

### `dynamixel_drive_arm(ctx, j1, j2, j3)`

Arm joint command in **radians from home**. `j=0` holds the reference
kinematic configuration. CCW positive when viewed from the motor output shaft.

### `dynamixel_drive_gripper(ctx, g1=1.0, g2=1.0, g3=1.0)`

Per-finger open fraction. `1.0` = fully open (`GRIPPER_OPEN[id]` ticks),
`0.0` = fully closed (`GRIPPER_OPEN[id] - GRIPPER_STROKE`). Values are clamped.

### `dynamixel_drive(ctx, ticks)`

Low-level sync-write. `ticks` must have `len(ctx['all_ids'])` entries in the
same order; values are int, encoded 4-byte signed (extended position can go
negative / multi-turn).

### `dynamixel_read(ctx)`

Single `GroupSyncRead` round-trip for **present position** + **present
velocity**. Returns `{mid: {'position': int_or_None, 'velocity': int_or_None}}`
— values are `None` for motors that didn't respond this cycle (bus drop,
timeout, etc.), so the caller can decide whether to retry or fail.

### `dynamixel_disconnect(ctx)`

Torque-off every motor and close the serial port. Idempotent. Safe to call
from a `finally:` block even if `dynamixel_connect` raised partway through.

---

## Quick start

```python
import time
from anymal_custom_control.dynamixel import (
    dynamixel_connect, dynamixel_drive_arm, dynamixel_drive_gripper,
    dynamixel_status, dynamixel_disconnect,
)

ctx = dynamixel_connect()
try:
    # Hold home for a beat
    time.sleep(1)

    # Lift j2 by 0.3 rad, close all three fingers halfway
    dynamixel_drive_arm(ctx, 0.0, 0.3, 0.0)
    dynamixel_drive_gripper(ctx, 0.5, 0.5, 0.5)
    time.sleep(2)

    dynamixel_status(ctx)
finally:
    dynamixel_disconnect(ctx)
```

Run the built-in self-test (drive to home, wait, disconnect):

```bash
python3 -m anymal_custom_control.dynamixel.driver
```

---

## Using it alongside MAB MD80 / ANYmal movement

The API intentionally mirrors `motor_driver.py`. A GIRAF-style teleop script
can hold both contexts side-by-side:

```python
from anymal_custom_control import MovementController, ModeController
from anymal_custom_control.motor_driver import (
    motor_connect, motor_drive, motor_disconnect,
)
from anymal_custom_control.dynamixel import (
    dynamixel_connect, dynamixel_drive_gripper, dynamixel_disconnect,
)

mc   = MovementController(); mc.start()
mode = ModeController(movement_controller=mc)

md80 = motor_connect()            # boom/roll/pitch on CAN bus (candle_ros)
dxl  = dynamixel_connect()        # wrist joints + grippers on USB serial

try:
    mode.switch_mode(ModeController.STAND)
    mc.set_velocity(heading=0.2)
    motor_drive(md80, 0.0, 0.1, -1.0)
    dynamixel_drive_gripper(dxl, 0.0, 0.0, 0.0)   # close gripper
    ...
finally:
    dynamixel_disconnect(dxl)
    motor_disconnect()
    mc.shutdown()
```

The MD80 driver runs on the CAN bus through `candle_ros`; Dynamixels run on
USB-TTL through the SDK. They share no state — feel free to treat them as
independent subsystems.

---

## Low-level escape hatch

For ad-hoc register access (tuning PID gains, reading temperature, custom
sync groups, etc.):

```python
from anymal_custom_control.dynamixel import DynamixelController
from anymal_custom_control.dynamixel.control_table import (
    PRESENT_TEMPERATURE, POSITION_P_GAIN,
)

dxl = DynamixelController('/dev/ttyUSB0', 57600)
try:
    temp = dxl.READ(11, PRESENT_TEMPERATURE)  # °C
    dxl.WRITE(11, POSITION_P_GAIN, 1200)
finally:
    dxl.close()
```

`DynamixelController.make_sync_write(addr_tuple)` and
`make_sync_read(addr_tuple, ids)` return SDK handles you can drive directly.

---

## Convention notes / gotchas

- **Baudrate on fresh motors.** Factory default is 57600. If you've permanently
  set motors to 1 M via Dynamixel Wizard, pass `baudrate=1_000_000` — the
  driver does **not** change the motor's stored baud automatically.
- **Operating mode requires torque-off first.** `dynamixel_connect` handles
  this; if you write `OPERATING_MODE` from your own code, do the same.
- **Extended position mode** means goal values can exceed `[0, 4095]` and
  can be negative. That's how gripper "closed" positions below zero work.
- **`reboot=True` takes ~2 s.** Skip it for fast dev reconnects:
  `dynamixel_connect(reboot=False)`.
- **Read returns may contain `None`.** A dropped packet is not an exception;
  callers should tolerate `None` in `dynamixel_read` output or retry.
- **Gripper force is capped via `PWM_LIMIT`**, not current control. 300/885
  (~34%) is a safe default for compliant grip; raise cautiously.
- **Port close on `Ctrl-C`.** Put `dynamixel_disconnect(ctx)` in a `finally:`
  — the SDK does **not** release the serial port otherwise, and the next
  connect will fail with "Failed to open port."

---

## Troubleshooting

| Symptom                                   | Likely cause                                         |
| ----------------------------------------- | ---------------------------------------------------- |
| `Failed to open port /dev/ttyUSB0`        | Previous session didn't `dynamixel_disconnect`; or permission denied. `lsof /dev/ttyUSB0` to see who holds it; `sudo chmod 666 /dev/ttyUSB0`. |
| `Failed to set baudrate`                  | U2D2 not present, or cable unplugged.                |
| `reboot ID X: [TxRxResult] Port busy`     | Another process owns the bus (often a stray REPL).   |
| One motor silently missing from `dynamixel_read` | Usually a daisy-chain break — check TTL wiring between that motor and the previous one. |
| Operating mode verify fails               | Motor didn't accept mode write (torque still on, or wrong model). |
| Grippers overshoot and stall              | `GRIPPER_STROKE` exceeds actual mechanical range; reduce it in `control_table.py` for that hardware. |
