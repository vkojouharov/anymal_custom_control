"""Functional Dynamixel driver — mirrors motor_driver.py (MD80) conventions.

Typical use from a script::

    from anymal_custom_control.dynamixel import (
        dynamixel_connect, dynamixel_drive_arm, dynamixel_drive_gripper,
        dynamixel_read, dynamixel_home, dynamixel_disconnect,
    )

    ctx = dynamixel_connect()
    try:
        dynamixel_drive_arm(ctx, 0.0, 0.1, -0.2)        # radians from home
        dynamixel_drive_gripper(ctx, 0.0, 0.0, 0.0)     # 0=closed, 1=open
        state = dynamixel_read(ctx)                     # bulk read positions
    finally:
        dynamixel_disconnect(ctx)

See README.md in this folder for the full guide.
"""
import math
import time

from dynamixel_sdk import COMM_SUCCESS

from .control_table import (
    ARM_HOME, ARM_IDS,
    GOAL_POSITION, GRIPPER_IDS, GRIPPER_OPEN, GRIPPER_STROKE,
    OP_EXTENDED_POSITION, OPERATING_MODE, PRESENT_POSITION,
    PRESENT_VELOCITY, PWM_LIMIT, TICKS_PER_REV, TORQUE_ENABLE,
)
from .controller import DynamixelController, _to_signed


# ── Lifecycle ───────────────────────────────────────────────────────────────

def dynamixel_connect(
    port='/dev/ttyUSB0',
    baudrate=57600,
    protocol=2.0,
    arm_ids=ARM_IDS,
    gripper_ids=GRIPPER_IDS,
    gripper_pwm_limit=300,
    reboot=True,
    settle_time=2.0,
):
    """Open port, reboot motors, configure extended-position mode, enable torque.

    Args:
        port: Serial device (default ``/dev/ttyUSB0``; U2D2 typically enumerates here).
        baudrate: Bus baud. Factory default is 57600; bump to 1_000_000 for
            higher control rates once all motors have been set to that baud.
        protocol: Dynamixel protocol version (2.0 for X-series).
        arm_ids, gripper_ids: Override if your hardware uses non-default IDs.
        gripper_pwm_limit: Cap gripper PWM (0..885) to limit grip force.
        reboot: Reboot every motor during connect for a clean startup state.
            Set False to skip the 2s delay when reconnecting mid-session.
        settle_time: Seconds to wait after reboot before writing configuration.

    Returns:
        ctx dict consumed by the other ``dynamixel_*`` functions.

    Raises:
        RuntimeError on port/baud failure or missing motors.
    """
    controller = DynamixelController(port, baudrate, protocol)

    all_ids = tuple(arm_ids) + tuple(gripper_ids)

    if reboot:
        for mid in all_ids:
            comm, err = controller.packet_handler.reboot(controller.port_handler, mid)
            if comm != COMM_SUCCESS:
                print(f"[dxl] reboot ID {mid}: "
                      f"{controller.packet_handler.getTxRxResult(comm)}")
            elif err != 0:
                print(f"[dxl] reboot ID {mid} packet error: "
                      f"{controller.packet_handler.getRxPacketError(err)}")
        time.sleep(settle_time)

    # Torque OFF is required before writing OPERATING_MODE (firmware-enforced).
    for mid in all_ids:
        controller.WRITE(mid, TORQUE_ENABLE, 0)

    # Extended position on everything (single-turn or multi-turn positional).
    for mid in all_ids:
        if not controller.WRITE(mid, OPERATING_MODE, OP_EXTENDED_POSITION):
            raise RuntimeError(f"Failed to set operating mode on motor {mid}")
        mode_readback = controller.READ(mid, OPERATING_MODE)
        if mode_readback != OP_EXTENDED_POSITION:
            raise RuntimeError(
                f"Motor {mid} operating mode verify failed: "
                f"wrote {OP_EXTENDED_POSITION}, read {mode_readback}"
            )

    # Force cap on grippers (current-based grip limiting).
    for mid in gripper_ids:
        controller.WRITE(mid, PWM_LIMIT, int(gripper_pwm_limit))

    # Torque ON — motors will hold position.
    for mid in all_ids:
        if not controller.WRITE(mid, TORQUE_ENABLE, 1):
            raise RuntimeError(f"Failed to enable torque on motor {mid}")

    sync_write_pos = controller.make_sync_write(GOAL_POSITION)
    sync_read_pos  = controller.make_sync_read(PRESENT_POSITION, all_ids)
    sync_read_vel  = controller.make_sync_read(PRESENT_VELOCITY, all_ids)

    return {
        'controller':     controller,
        'arm_ids':        tuple(arm_ids),
        'gripper_ids':    tuple(gripper_ids),
        'all_ids':        all_ids,
        'sync_write_pos': sync_write_pos,
        'sync_read_pos':  sync_read_pos,
        'sync_read_vel':  sync_read_vel,
    }


def dynamixel_disconnect(ctx):
    """Torque-off every motor and close the serial port.

    Safe to call more than once; safe to call with a partially-initialized ctx.
    """
    controller = ctx.get('controller')
    if controller is None:
        return
    for mid in ctx.get('all_ids', ()):
        try:
            controller.WRITE(mid, TORQUE_ENABLE, 0)
        except Exception as e:
            print(f"[dxl] disconnect: failed to disable motor {mid}: {e}")
    controller.close()


# ── Drive ───────────────────────────────────────────────────────────────────

def dynamixel_drive(ctx, ticks):
    """Low-level goal-position sync write.

    Args:
        ticks: iterable of ints, one per motor, in ``ctx['all_ids']`` order.
            Values are encoded as 4-byte signed (extended position mode
            supports multi-turn — both signs and magnitudes beyond 0..4095
            are legal).

    Returns:
        True if the packet went out, False on SDK error.
    """
    ids = ctx['all_ids']
    ticks = list(ticks)
    if len(ticks) != len(ids):
        raise ValueError(f"dynamixel_drive expects {len(ids)} ticks, got {len(ticks)}")

    return _drive_subset(ctx, ids, ticks)


def dynamixel_drive_arm(ctx, j1, j2, j3):
    """Command arm joints in radians from home.

    ``j=0`` holds the reference kinematic home. Positive radians = CCW when
    viewed from the motor output shaft (standard Dynamixel convention).
    """
    arm_ids = ctx['arm_ids']
    ticks = [
        ARM_HOME[arm_ids[0]] + _rad_to_ticks(j1),
        ARM_HOME[arm_ids[1]] + _rad_to_ticks(j2),
        ARM_HOME[arm_ids[2]] + _rad_to_ticks(j3),
    ]
    return _drive_subset(ctx, arm_ids, ticks)


def dynamixel_drive_gripper(ctx, g1=1.0, g2=1.0, g3=1.0):
    """Command grippers as open-fraction in [0, 1].

    ``1.0`` = fully open (home pose), ``0.0`` = fully closed (OPEN - STROKE).
    Values outside [0, 1] are clamped.
    """
    gripper_ids = ctx['gripper_ids']
    ticks = [
        _gripper_ticks(gripper_ids[0], g1),
        _gripper_ticks(gripper_ids[1], g2),
        _gripper_ticks(gripper_ids[2], g3),
    ]
    return _drive_subset(ctx, gripper_ids, ticks)


def dynamixel_home(ctx):
    """Drive every motor to its home / fully-open position."""
    ids = ctx['all_ids']
    ticks = [
        (ARM_HOME[m] if m in ARM_HOME else GRIPPER_OPEN[m])
        for m in ids
    ]
    return dynamixel_drive(ctx, ticks)


# ── Feedback ────────────────────────────────────────────────────────────────

def dynamixel_read(ctx):
    """Bulk-read present position (ticks) + velocity (raw) for every motor.

    Returns:
        dict keyed by motor ID::

            {11: {'position': 2044, 'velocity': 0}, 12: {...}, ...}

        Values are None for motors that did not respond this cycle.
    """
    ids = ctx['all_ids']
    out = {mid: {'position': None, 'velocity': None} for mid in ids}

    for handle, key, ct in (
        (ctx['sync_read_pos'], 'position', PRESENT_POSITION),
        (ctx['sync_read_vel'], 'velocity', PRESENT_VELOCITY),
    ):
        comm = handle.txRxPacket()
        if comm != COMM_SUCCESS:
            print(f"[dxl] sync-read {key}: "
                  f"{ctx['controller'].packet_handler.getTxRxResult(comm)}")
            continue
        addr, length = ct
        for mid in ids:
            if handle.isAvailable(mid, addr, length):
                raw = handle.getData(mid, addr, length)
                out[mid][key] = _to_signed(raw, length)

    return out


def dynamixel_status(ctx):
    """Print a human-readable status line for every motor."""
    state = dynamixel_read(ctx)
    for mid in ctx['all_ids']:
        pos = state[mid]['position']
        vel = state[mid]['velocity']
        role = 'arm    ' if mid in ctx['arm_ids'] else 'gripper'
        pos_str = f"{pos:>6d}" if pos is not None else "  ----"
        vel_str = f"{vel:>6d}" if vel is not None else "  ----"
        print(f"  ID {mid:3d} ({role})  pos={pos_str} ticks  vel={vel_str}")


# ── Conversions ─────────────────────────────────────────────────────────────

def radians_to_ticks(rad):
    """Convert a radian delta to a tick delta (XH-series, 4096 ticks/rev)."""
    return _rad_to_ticks(rad)


def ticks_to_radians(ticks):
    """Inverse of ``radians_to_ticks``."""
    return ticks * (2 * math.pi) / TICKS_PER_REV


# ── Internals ───────────────────────────────────────────────────────────────

def _rad_to_ticks(rad):
    return int(round(rad * TICKS_PER_REV / (2 * math.pi)))


def _gripper_ticks(mid, open_fraction):
    f = max(0.0, min(1.0, float(open_fraction)))
    return int(round(GRIPPER_OPEN[mid] - GRIPPER_STROKE * (1.0 - f)))


def _drive_subset(ctx, ids_subset, ticks):
    """Sync-write goal position for a subset of motors sharing the ctx bus."""
    gsw = ctx['sync_write_pos']
    for mid, t in zip(ids_subset, ticks):
        if not gsw.addParam(mid, int(t).to_bytes(4, 'little', signed=True)):
            gsw.clearParam()
            print(f"[dxl] sync-write addParam failed for motor {mid}")
            return False

    comm = gsw.txPacket()
    gsw.clearParam()
    if comm != COMM_SUCCESS:
        print(f"[dxl] sync-write txPacket: "
              f"{ctx['controller'].packet_handler.getTxRxResult(comm)}")
        return False
    return True


# ── Self-test ───────────────────────────────────────────────────────────────

def main():
    """Connect, home, pause, disconnect — validates wiring and control table."""
    ctx = dynamixel_connect()
    try:
        print("\033[93mDYNAMIXEL: Connected, driving to HOME for 5 s…\033[0m")
        dynamixel_home(ctx)
        time.sleep(5)
        dynamixel_status(ctx)
    finally:
        dynamixel_disconnect(ctx)
        print("\033[93mDYNAMIXEL: Disconnected, torque OFF\033[0m")


if __name__ == "__main__":
    main()
