#!/usr/bin/env python3
"""All-in-one balance runtime with manual pitch Dynamixel control.

This file owns:
    - joystick teleop for the MD80 roll / pitch / boom joints
    - OAK-D IMU readout to the terminal
    - balance control on Dynamixel motor ID 15
    - manual up/down control of Dynamixel motor ID 11

Edit the config block below directly in code.
"""

from __future__ import annotations

import math
import sys
import time

import depthai as dai
import rospy
from dynamixel_sdk import COMM_SUCCESS

from anymal_custom_control.dynamixel import (
    ARM_HOME,
    dynamixel_connect,
    dynamixel_disconnect,
    radians_to_ticks,
)
from anymal_custom_control.dynamixel.control_table import PRESENT_POSITION
from anymal_custom_control.joystick_driver import (
    joystick_connect,
    joystick_disconnect,
    joystick_read,
)
from anymal_custom_control.motor_driver import (
    motor_connect,
    motor_disconnect,
    motor_drive,
)


LOOP_HZ = 200.0

IMU_RATE_ACCEL_HZ = 125
IMU_RATE_GYRO_HZ = 100
IMU_RATE_ROTATION_VECTOR_HZ = 100
IMU_BATCH_THRESHOLD = 1
IMU_MAX_BATCH_REPORTS = 10

# Balance motor on the twist axis.
BALANCE_DXL_ENABLE = True
DXL_PORT = "/dev/ttyUSB0"
DXL_BAUDRATE = 1_000_000
BALANCE_DXL_MOTOR_ID = 15
BALANCE_DXL_INITIAL_TICKS = 3025
BALANCE_DXL_GOAL_TICK_MIN = 1000
BALANCE_DXL_GOAL_TICK_MAX = 5000
BALANCE_DXL_GAIN = 5.0
BALANCE_DXL_DAMPING_GAIN = 0.5
BALANCE_DXL_MOTOR_SIGN = -1.0
BALANCE_DXL_INITIAL_SETTLE_SEC = 0.2
BALANCE_DXL_INITIAL_POSITION_TOL_TICKS = 15
BALANCE_DXL_INITIAL_POSITION_TIMEOUT_SEC = 5.0
BALANCE_TARGET_SAMPLE_COUNT = 20
BALANCE_CONTROL_AXIS = "z"
BALANCE_ERROR_DEADBAND_DEG = 2
BALANCE_MAX_STEP_TICKS = 30

# Manual pitch motor. Uses DPAD_Y for up/down.
PITCH_DXL_ENABLE = True
PITCH_DXL_MOTOR_ID = 11
PITCH_DXL_INITIAL_TICKS = ARM_HOME[11]
PITCH_DXL_GOAL_TICK_MIN = 1900
PITCH_DXL_GOAL_TICK_MAX = 3850
PITCH_DXL_STEP_TICKS = 8
PITCH_DPAD_SIGN = -1
PITCH_DXL_INITIAL_POSITION_TOL_TICKS = 50

# Wrist hold motors. These match the GIRAF wrist startup home positions.
WRIST_HOLD_DXL_ENABLE = True
WRIST_HOLD_MOTOR_IDS = (12, 13)
WRIST_HOLD_INITIAL_TICKS = {
    12: ARM_HOME[12],
    13: ARM_HOME[13],
}
WRIST_HOLD_GOAL_LIMITS = {
    12: (950, 3090),
    13: (-1075, 3075),
}
WRIST_HOLD_INITIAL_POSITION_TOL_TICKS = {
    12: 50,
    13: 50,
}

# Legacy joint-space teleop behavior for the MD80s.
PITCH_MIN = 0.0
BOOM_MIN = -30.0
BOOM_MAX = 0.0
ROLL_RATIO_NUM = 0.005
PITCH_RATIO_NUM = 0.005
BOOM_DRIVE_RATIO = 0.025


def quaternion_normalize(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    i, j, k, real = quat
    norm = math.sqrt(i * i + j * j + k * k + real * real)
    if norm <= 1e-9:
        raise ValueError("Invalid zero-length quaternion from IMU")
    return (i / norm, j / norm, k / norm, real / norm)


def quaternion_conjugate(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    i, j, k, real = quat
    return (-i, -j, -k, real)


def quaternion_multiply(
    lhs: tuple[float, float, float, float],
    rhs: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    li, lj, lk, lr = lhs
    ri, rj, rk, rr = rhs
    return (
        lr * ri + li * rr + lj * rk - lk * rj,
        lr * rj - li * rk + lj * rr + lk * ri,
        lr * rk + li * rj - lj * ri + lk * rr,
        lr * rr - li * ri - lj * rj - lk * rk,
    )


def quaternion_average(
    samples: list[tuple[float, float, float, float]],
) -> tuple[float, float, float, float]:
    if not samples:
        raise ValueError("Cannot average an empty quaternion sample set")

    ref = samples[0]
    accum_i = 0.0
    accum_j = 0.0
    accum_k = 0.0
    accum_r = 0.0
    for sample in samples:
        si, sj, sk, sr = sample
        if (ref[0] * si + ref[1] * sj + ref[2] * sk + ref[3] * sr) < 0.0:
            si, sj, sk, sr = -si, -sj, -sk, -sr
        accum_i += si
        accum_j += sj
        accum_k += sk
        accum_r += sr
    return quaternion_normalize((accum_i, accum_j, accum_k, accum_r))


def quaternion_to_roll_deg(i: float, j: float, k: float, real: float) -> float:
    sinr_cosp = 2.0 * (real * i + j * k)
    cosr_cosp = 1.0 - 2.0 * (i * i + j * j)
    return math.degrees(math.atan2(sinr_cosp, cosr_cosp))


def quaternion_to_pitch_deg(i: float, j: float, k: float, real: float) -> float:
    sinp = 2.0 * (real * j - k * i)
    if abs(sinp) >= 1.0:
        return math.degrees(math.copysign(math.pi / 2.0, sinp))
    return math.degrees(math.asin(sinp))


def quaternion_to_yaw_deg(i: float, j: float, k: float, real: float) -> float:
    siny_cosp = 2.0 * (real * k + i * j)
    cosy_cosp = 1.0 - 2.0 * (j * j + k * k)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def accel_to_roll_deg(x: float, y: float, z: float) -> float:
    return math.degrees(math.atan2(y, z))


def axis_vector(axis_name: str) -> tuple[float, float, float]:
    if axis_name == "x":
        return (1.0, 0.0, 0.0)
    if axis_name == "y":
        return (0.0, 1.0, 0.0)
    if axis_name == "z":
        return (0.0, 0.0, 1.0)
    raise ValueError(f"Unsupported BALANCE_CONTROL_AXIS={axis_name!r}")


def quaternion_twist_deg(
    quat: tuple[float, float, float, float],
    axis_name: str,
) -> float:
    axis = axis_vector(axis_name)
    qi, qj, qk, qr = quaternion_normalize(quat)
    dot = qi * axis[0] + qj * axis[1] + qk * axis[2]
    proj_i = dot * axis[0]
    proj_j = dot * axis[1]
    proj_k = dot * axis[2]
    twist = quaternion_normalize((proj_i, proj_j, proj_k, qr))
    return math.degrees(2.0 * math.atan2(dot, twist[3]))


def unwrap_angle_deg(previous_unwrapped: float | None, wrapped_deg: float) -> float:
    if previous_unwrapped is None:
        return wrapped_deg

    delta = wrapped_deg - ((previous_unwrapped + 180.0) % 360.0 - 180.0)
    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0
    return previous_unwrapped + delta


def build_imu_pipeline(imu_type: str) -> dai.Pipeline:
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("imu")

    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, IMU_RATE_ACCEL_HZ)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, IMU_RATE_GYRO_HZ)

    if imu_type == "BNO086":
        imu.enableIMUSensor(
            dai.IMUSensor.ROTATION_VECTOR,
            min(IMU_RATE_ROTATION_VECTOR_HZ, 100),
        )

    imu.setBatchReportThreshold(IMU_BATCH_THRESHOLD)
    imu.setMaxBatchReports(IMU_MAX_BATCH_REPORTS)
    imu.out.link(xout.input)
    return pipeline


def packet_rotation_quaternion(packet) -> tuple[float, float, float, float] | None:
    rotation = getattr(packet, "rotationVector", None)
    if rotation is None:
        return None
    return quaternion_normalize(
        (
            float(rotation.i),
            float(rotation.j),
            float(rotation.k),
            float(rotation.real),
        )
    )


def packet_roll_deg(packet) -> float | None:
    quat = packet_rotation_quaternion(packet)
    if quat is not None:
        return quaternion_to_roll_deg(
            quat[0],
            quat[1],
            quat[2],
            quat[3],
        )

    accel = getattr(packet, "acceleroMeter", None)
    if accel is not None:
        return accel_to_roll_deg(accel.x, accel.y, accel.z)

    return None


def packet_euler_deg(packet) -> tuple[float, float, float] | None:
    quat = packet_rotation_quaternion(packet)
    if quat is None:
        return None
    return (
        quaternion_to_roll_deg(quat[0], quat[1], quat[2], quat[3]),
        quaternion_to_pitch_deg(quat[0], quat[1], quat[2], quat[3]),
        quaternion_to_yaw_deg(quat[0], quat[1], quat[2], quat[3]),
    )


def relative_axis_deg(
    target_quat: tuple[float, float, float, float],
    current_quat: tuple[float, float, float, float],
) -> float:
    relative_quat = quaternion_multiply(quaternion_conjugate(target_quat), current_quat)
    relative_quat = quaternion_normalize(relative_quat)
    return quaternion_twist_deg(relative_quat, BALANCE_CONTROL_AXIS)


def maybe_lock_target_quaternion(
    target_quat: tuple[float, float, float, float] | None,
    target_quat_samples: list[tuple[float, float, float, float]],
    latest_quat: tuple[float, float, float, float] | None,
) -> tuple[float, float, float, float] | None:
    if latest_quat is None or target_quat is not None:
        return target_quat

    target_quat_samples.append(latest_quat)
    if len(target_quat_samples) < max(BALANCE_TARGET_SAMPLE_COUNT, 1):
        return None
    return quaternion_average(target_quat_samples)


def validate_initial_ticks(label: str, goal_ticks: int, lo: int, hi: int) -> None:
    if not (lo <= goal_ticks <= hi):
        raise ValueError(
            f"{label} initial ticks {goal_ticks} are outside configured limits [{lo}, {hi}]"
        )


def clamp_goal_ticks(goal_ticks: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(goal_ticks)))


def dynamixel_drive_absolute_ticks(ctx, motor_id: int, goal_ticks: int, lo: int, hi: int) -> bool:
    clamped_ticks = clamp_goal_ticks(goal_ticks, lo, hi)
    sync_write = ctx["sync_write_pos"]
    if not sync_write.addParam(motor_id, int(clamped_ticks).to_bytes(4, "little", signed=True)):
        sync_write.clearParam()
        print(f"[dxl] sync-write addParam failed for motor {motor_id}")
        return False

    comm = sync_write.txPacket()
    sync_write.clearParam()
    if comm != COMM_SUCCESS:
        controller = ctx["controller"]
        print(
            f"[dxl] sync-write txPacket for motor {motor_id}: "
            f"{controller.packet_handler.getTxRxResult(comm)}"
        )
        return False
    return True


def dynamixel_read_present_ticks(ctx, motor_id: int) -> int | None:
    return ctx["controller"].READ(motor_id, PRESENT_POSITION)


def wait_for_initial_position(
    ctx,
    motor_id: int,
    goal_ticks: int,
    tol_ticks: int,
    timeout_sec: float,
) -> int:
    deadline = time.monotonic() + timeout_sec
    last_ticks = dynamixel_read_present_ticks(ctx, motor_id)
    while time.monotonic() < deadline and not rospy.is_shutdown():
        present_ticks = dynamixel_read_present_ticks(ctx, motor_id)
        if present_ticks is not None:
            last_ticks = present_ticks
            if abs(present_ticks - goal_ticks) <= tol_ticks:
                return present_ticks
        time.sleep(0.02)

    raise RuntimeError(
        f"Motor {motor_id} failed to reach startup tick {goal_ticks} "
        f"within {timeout_sec:.1f}s; last={last_ticks}"
    )


def teleop_step(
    joystick_data: dict[str, float],
    roll_pos: float,
    pitch_pos: float,
    boom_pos: float,
    pitch_goal_ticks: int,
) -> tuple[float, float, float, int, bool]:
    lx = joystick_data["LX"]
    ly = joystick_data["LY"]
    lt = joystick_data["LT"]
    rt = joystick_data["RT"]
    xb = joystick_data["XB"]
    lb = joystick_data["LB"]
    rb = joystick_data["RB"]
    dpad_y = joystick_data["DPAD_Y"]

    if xb:
        return roll_pos, pitch_pos, boom_pos, pitch_goal_ticks, False

    drive_scale = max((4.0 - boom_pos) / 4.0, 1e-6)
    roll_drive_ratio = ROLL_RATIO_NUM / drive_scale
    pitch_drive_ratio = PITCH_RATIO_NUM / drive_scale

    if lb and rb:
        roll_pos -= roll_drive_ratio * lx
        pitch_pos += pitch_drive_ratio * ly
        if rt and not lt:
            boom_pos -= BOOM_DRIVE_RATIO * rt
        elif lt and not rt:
            boom_pos += BOOM_DRIVE_RATIO * lt

    if dpad_y != 0.0:
        pitch_goal_ticks += int(PITCH_DPAD_SIGN * dpad_y * PITCH_DXL_STEP_TICKS)
        pitch_goal_ticks = clamp_goal_ticks(
            pitch_goal_ticks,
            PITCH_DXL_GOAL_TICK_MIN,
            PITCH_DXL_GOAL_TICK_MAX,
        )

    pitch_pos = max(pitch_pos, PITCH_MIN)
    boom_pos = max(min(boom_pos, BOOM_MAX), BOOM_MIN)
    return roll_pos, pitch_pos, boom_pos, pitch_goal_ticks, True


def main() -> int:
    rospy.init_node("run_balance_pitch_only", anonymous=False)

    joystick = None
    md80_ctx = None
    dxl_ctx = None

    try:
        joystick = joystick_connect()
        md80_ctx = motor_connect()

        wrist_hold_goal_ticks = dict(WRIST_HOLD_INITIAL_TICKS)

        if BALANCE_DXL_ENABLE or PITCH_DXL_ENABLE or WRIST_HOLD_DXL_ENABLE:
            if BALANCE_DXL_ENABLE:
                validate_initial_ticks(
                    "Balance motor",
                    BALANCE_DXL_INITIAL_TICKS,
                    BALANCE_DXL_GOAL_TICK_MIN,
                    BALANCE_DXL_GOAL_TICK_MAX,
                )
            if PITCH_DXL_ENABLE:
                validate_initial_ticks(
                    "Pitch motor",
                    PITCH_DXL_INITIAL_TICKS,
                    PITCH_DXL_GOAL_TICK_MIN,
                    PITCH_DXL_GOAL_TICK_MAX,
                )
            if WRIST_HOLD_DXL_ENABLE:
                for motor_id in WRIST_HOLD_MOTOR_IDS:
                    lo, hi = WRIST_HOLD_GOAL_LIMITS[motor_id]
                    validate_initial_ticks(
                        f"Wrist hold motor {motor_id}",
                        WRIST_HOLD_INITIAL_TICKS[motor_id],
                        lo,
                        hi,
                    )

            arm_ids = []
            if BALANCE_DXL_ENABLE:
                arm_ids.append(BALANCE_DXL_MOTOR_ID)
            if PITCH_DXL_ENABLE:
                arm_ids.append(PITCH_DXL_MOTOR_ID)
            if WRIST_HOLD_DXL_ENABLE:
                arm_ids.extend(mid for mid in WRIST_HOLD_MOTOR_IDS if mid not in arm_ids)

            dxl_ctx = dynamixel_connect(
                port=DXL_PORT,
                baudrate=DXL_BAUDRATE,
                arm_ids=tuple(arm_ids),
                gripper_ids=(),
            )

            if BALANCE_DXL_ENABLE:
                if not dynamixel_drive_absolute_ticks(
                    dxl_ctx,
                    BALANCE_DXL_MOTOR_ID,
                    BALANCE_DXL_INITIAL_TICKS,
                    BALANCE_DXL_GOAL_TICK_MIN,
                    BALANCE_DXL_GOAL_TICK_MAX,
                ):
                    raise RuntimeError(
                        f"Failed to command initial position on motor {BALANCE_DXL_MOTOR_ID}"
                    )
                wait_for_initial_position(
                    dxl_ctx,
                    BALANCE_DXL_MOTOR_ID,
                    BALANCE_DXL_INITIAL_TICKS,
                    BALANCE_DXL_INITIAL_POSITION_TOL_TICKS,
                    BALANCE_DXL_INITIAL_POSITION_TIMEOUT_SEC,
                )

            if PITCH_DXL_ENABLE:
                if not dynamixel_drive_absolute_ticks(
                    dxl_ctx,
                    PITCH_DXL_MOTOR_ID,
                    PITCH_DXL_INITIAL_TICKS,
                    PITCH_DXL_GOAL_TICK_MIN,
                    PITCH_DXL_GOAL_TICK_MAX,
                ):
                    raise RuntimeError(
                        f"Failed to command initial position on motor {PITCH_DXL_MOTOR_ID}"
                    )
                wait_for_initial_position(
                    dxl_ctx,
                    PITCH_DXL_MOTOR_ID,
                    PITCH_DXL_INITIAL_TICKS,
                    PITCH_DXL_INITIAL_POSITION_TOL_TICKS,
                    BALANCE_DXL_INITIAL_POSITION_TIMEOUT_SEC,
                )

            if WRIST_HOLD_DXL_ENABLE:
                for motor_id in WRIST_HOLD_MOTOR_IDS:
                    lo, hi = WRIST_HOLD_GOAL_LIMITS[motor_id]
                    initial_ticks = WRIST_HOLD_INITIAL_TICKS[motor_id]
                    if not dynamixel_drive_absolute_ticks(
                        dxl_ctx,
                        motor_id,
                        initial_ticks,
                        lo,
                        hi,
                    ):
                        raise RuntimeError(
                            f"Failed to command initial position on motor {motor_id}"
                        )
                    wait_for_initial_position(
                        dxl_ctx,
                        motor_id,
                        initial_ticks,
                        WRIST_HOLD_INITIAL_POSITION_TOL_TICKS[motor_id],
                        BALANCE_DXL_INITIAL_POSITION_TIMEOUT_SEC,
                    )

            time.sleep(max(BALANCE_DXL_INITIAL_SETTLE_SEC, 0.0))

        roll_pos = 0.0
        pitch_pos = 0.0
        boom_pos = 0.0
        pitch_goal_ticks = PITCH_DXL_INITIAL_TICKS
        balance_goal_ticks = BALANCE_DXL_INITIAL_TICKS
        latest_oakd_roll_deg: float | None = None
        latest_oakd_pitch_deg: float | None = None
        latest_oakd_yaw_deg: float | None = None
        latest_abs_roll_deg: float | None = None
        latest_abs_roll_unwrapped_deg: float | None = None
        target_abs_roll_deg: float | None = None
        latest_axis_deg: float | None = None
        latest_axis_unwrapped_deg: float | None = None
        target_axis_deg: float | None = None
        target_axis_unwrapped_deg: float | None = None
        latest_error_deg: float | None = None
        latest_error_rate_deg_s = 0.0
        previous_error_deg: float | None = None
        latest_quat: tuple[float, float, float, float] | None = None
        target_quat: tuple[float, float, float, float] | None = None
        target_quat_samples: list[tuple[float, float, float, float]] = []
        latest_relative_axis_unwrapped_deg: float | None = None

        with dai.Device() as device:
            imu_type = str(device.getConnectedIMU())
            if imu_type in {"", "NONE", "UNKNOWN", "None"}:
                raise RuntimeError("No IMU detected on the connected OAK-D device.")

            device.startPipeline(build_imu_pipeline(imu_type))
            queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            rate = rospy.Rate(LOOP_HZ)

            while not rospy.is_shutdown():
                joystick_data = joystick_read(joystick)
                roll_pos, pitch_pos, boom_pos, pitch_goal_ticks, keep_running = teleop_step(
                    joystick_data,
                    roll_pos,
                    pitch_pos,
                    boom_pos,
                    pitch_goal_ticks,
                )
                if not keep_running:
                    break

                while True:
                    imu_data = queue.tryGet()
                    if imu_data is None:
                        break
                    for packet in imu_data.packets:
                        packet_euler = packet_euler_deg(packet)
                        if packet_euler is not None:
                            latest_oakd_roll_deg, latest_oakd_pitch_deg, latest_oakd_yaw_deg = packet_euler
                        else:
                            packet_roll = packet_roll_deg(packet)
                            if packet_roll is not None:
                                latest_oakd_roll_deg = packet_roll
                        packet_quat = packet_rotation_quaternion(packet)
                        if packet_quat is None:
                            continue
                        latest_quat = packet_quat
                        latest_abs_roll_deg = quaternion_to_roll_deg(
                            packet_quat[0],
                            packet_quat[1],
                            packet_quat[2],
                            packet_quat[3],
                        )
                        latest_abs_roll_unwrapped_deg = unwrap_angle_deg(
                            latest_abs_roll_unwrapped_deg,
                            latest_abs_roll_deg,
                        )
                        latest_axis_deg = quaternion_twist_deg(
                            packet_quat,
                            BALANCE_CONTROL_AXIS,
                        )
                        latest_axis_unwrapped_deg = unwrap_angle_deg(
                            latest_axis_unwrapped_deg,
                            latest_axis_deg,
                        )

                target_quat = maybe_lock_target_quaternion(
                    target_quat,
                    target_quat_samples,
                    latest_quat,
                )
                if target_quat is not None and target_abs_roll_deg is None:
                    target_abs_roll_deg = quaternion_to_roll_deg(
                        target_quat[0],
                        target_quat[1],
                        target_quat[2],
                        target_quat[3],
                    )
                    if latest_abs_roll_unwrapped_deg is not None:
                        target_abs_roll_deg = unwrap_angle_deg(
                            latest_abs_roll_unwrapped_deg,
                            target_abs_roll_deg,
                        )
                if target_quat is not None and target_axis_deg is None:
                    target_axis_deg = quaternion_twist_deg(target_quat, BALANCE_CONTROL_AXIS)
                    if latest_axis_unwrapped_deg is not None:
                        target_axis_unwrapped_deg = unwrap_angle_deg(
                            latest_axis_unwrapped_deg,
                            target_axis_deg,
                        )

                if target_quat is not None and latest_quat is not None:
                    latest_relative_axis_deg = relative_axis_deg(target_quat, latest_quat)
                    latest_relative_axis_unwrapped_deg = unwrap_angle_deg(
                        latest_relative_axis_unwrapped_deg,
                        latest_relative_axis_deg,
                    )
                    latest_error_deg = -latest_relative_axis_unwrapped_deg
                    if previous_error_deg is None:
                        latest_error_rate_deg_s = 0.0
                    else:
                        latest_error_rate_deg_s = (latest_error_deg - previous_error_deg) * LOOP_HZ
                    previous_error_deg = latest_error_deg

                motor_drive(md80_ctx, roll_pos, pitch_pos, boom_pos)

                if (
                    dxl_ctx is not None
                    and BALANCE_DXL_ENABLE
                    and latest_error_deg is not None
                    and target_quat is not None
                ):
                    error_deg = latest_error_deg
                    if abs(error_deg) < BALANCE_ERROR_DEADBAND_DEG:
                        error_deg = 0.0
                        latest_error_rate_deg_s = 0.0
                    motor_velocity_rad_s = BALANCE_DXL_MOTOR_SIGN * (
                        BALANCE_DXL_GAIN * math.radians(error_deg)
                        + BALANCE_DXL_DAMPING_GAIN * math.radians(latest_error_rate_deg_s)
                    )
                    goal_step_ticks = radians_to_ticks(motor_velocity_rad_s / LOOP_HZ)
                    goal_step_ticks = max(
                        -BALANCE_MAX_STEP_TICKS,
                        min(BALANCE_MAX_STEP_TICKS, goal_step_ticks),
                    )
                    balance_goal_ticks = clamp_goal_ticks(
                        balance_goal_ticks + goal_step_ticks,
                        BALANCE_DXL_GOAL_TICK_MIN,
                        BALANCE_DXL_GOAL_TICK_MAX,
                    )
                    dynamixel_drive_absolute_ticks(
                        dxl_ctx,
                        BALANCE_DXL_MOTOR_ID,
                        balance_goal_ticks,
                        BALANCE_DXL_GOAL_TICK_MIN,
                        BALANCE_DXL_GOAL_TICK_MAX,
                    )

                if dxl_ctx is not None and PITCH_DXL_ENABLE:
                    dynamixel_drive_absolute_ticks(
                        dxl_ctx,
                        PITCH_DXL_MOTOR_ID,
                        pitch_goal_ticks,
                        PITCH_DXL_GOAL_TICK_MIN,
                        PITCH_DXL_GOAL_TICK_MAX,
                    )

                if dxl_ctx is not None and WRIST_HOLD_DXL_ENABLE:
                    for motor_id in WRIST_HOLD_MOTOR_IDS:
                        lo, hi = WRIST_HOLD_GOAL_LIMITS[motor_id]
                        dynamixel_drive_absolute_ticks(
                            dxl_ctx,
                            motor_id,
                            wrist_hold_goal_ticks[motor_id],
                            lo,
                            hi,
                        )

                if (
                    latest_oakd_roll_deg is not None
                    and latest_oakd_pitch_deg is not None
                    and latest_oakd_yaw_deg is not None
                ):
                    sys.stdout.write(
                        "\r"
                        f"oakd_raw_roll: {latest_oakd_roll_deg:+.2f} deg  "
                        f"pitch: {latest_oakd_pitch_deg:+.2f} deg  "
                        f"yaw: {latest_oakd_yaw_deg:+.2f} deg"
                    )
                elif latest_oakd_roll_deg is not None:
                    sys.stdout.write(f"\roakd_raw_roll: {latest_oakd_roll_deg:+.2f} deg")
                    sys.stdout.flush()

                rate.sleep()

        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        sys.stdout.write("\n")
        sys.stdout.flush()
        if dxl_ctx is not None:
            dynamixel_disconnect(dxl_ctx)
        if md80_ctx is not None:
            motor_disconnect()
        if joystick is not None:
            joystick_disconnect(joystick)


if __name__ == "__main__":
    raise SystemExit(main())
