#!/usr/bin/env python3
"""All-in-one balance runtime.

This file owns:
    - joystick teleop for the MD80 roll / pitch / boom joints
    - OAK-D IMU roll readout to the terminal
    - optional Dynamixel camera-roll hold on motor ID 11

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

# Runtime config. Edit these values here instead of passing CLI flags.
LOOP_HZ = 200.0

IMU_RATE_ACCEL_HZ = 125
IMU_RATE_GYRO_HZ = 100
IMU_RATE_ROTATION_VECTOR_HZ = 100
IMU_BATCH_THRESHOLD = 1
IMU_MAX_BATCH_REPORTS = 10

DXL_ENABLE_HOLD = True
DXL_PORT = "/dev/ttyUSB0"
DXL_BAUDRATE = 1_000_000
DXL_MOTOR_ID = 11
DXL_INITIAL_TICKS = 1025
DXL_GOAL_TICK_MIN = -1075
DXL_GOAL_TICK_MAX = 3075
DXL_ROLL_GAIN = 10.0
DXL_MOTOR_SIGN = -1.0
DXL_INITIAL_SETTLE_SEC = 0.2
DXL_INITIAL_POSITION_TOL_TICKS = 15
DXL_INITIAL_POSITION_TIMEOUT_SEC = 5.0
DXL_TARGET_SAMPLE_COUNT = 20
CONTROL_AXIS = "z"
DXL_DAMPING_GAIN = 0
DXL_ERROR_DEADBAND_DEG = 0.75
DXL_MAX_STEP_TICKS = 20

# Legacy joint-space teleop behavior.
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


def axis_vector(axis_name: str) -> tuple[float, float, float]:
    if axis_name == "x":
        return (1.0, 0.0, 0.0)
    if axis_name == "y":
        return (0.0, 1.0, 0.0)
    if axis_name == "z":
        return (0.0, 0.0, 1.0)
    raise ValueError(f"Unsupported CONTROL_AXIS={axis_name!r}; expected 'x', 'y', or 'z'")


def axis_component(axis_name: str, x: float, y: float, z: float) -> float:
    if axis_name == "x":
        return x
    if axis_name == "y":
        return y
    if axis_name == "z":
        return z
    raise ValueError(f"Unsupported CONTROL_AXIS={axis_name!r}; expected 'x', 'y', or 'z'")


def quaternion_twist_deg(
    quat: tuple[float, float, float, float],
    axis_name: str,
) -> float:
    """Signed twist angle of a quaternion about a chosen axis."""
    axis = axis_vector(axis_name)
    qi, qj, qk, qr = quaternion_normalize(quat)
    dot = qi * axis[0] + qj * axis[1] + qk * axis[2]
    proj_i = dot * axis[0]
    proj_j = dot * axis[1]
    proj_k = dot * axis[2]
    twist = quaternion_normalize((proj_i, proj_j, proj_k, qr))
    return math.degrees(2.0 * math.atan2(dot, twist[3]))


def unwrap_angle_deg(previous_unwrapped: float | None, wrapped_deg: float) -> float:
    """Lift a wrapped [-180, 180] angle into a continuous angle trajectory."""
    if previous_unwrapped is None:
        return wrapped_deg

    delta = wrapped_deg - ((previous_unwrapped + 180.0) % 360.0 - 180.0)
    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0
    return previous_unwrapped + delta


def accel_to_roll_deg(x: float, y: float, z: float) -> float:
    return math.degrees(math.atan2(y, z))


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
    if rotation is not None:
        return quaternion_normalize(
            (
                float(rotation.i),
                float(rotation.j),
                float(rotation.k),
                float(rotation.real),
            )
        )

    return None


def packet_axis_rate_rad_s(packet) -> float | None:
    gyro = getattr(packet, "gyroscope", None)
    if gyro is None:
        return None
    return float(axis_component(CONTROL_AXIS, gyro.x, gyro.y, gyro.z))


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


def relative_axis_deg(
    target_quat: tuple[float, float, float, float],
    current_quat: tuple[float, float, float, float],
) -> float:
    relative_quat = quaternion_multiply(quaternion_conjugate(target_quat), current_quat)
    relative_quat = quaternion_normalize(relative_quat)
    return quaternion_twist_deg(relative_quat, CONTROL_AXIS)


def maybe_lock_target_quaternion(
    target_quat: tuple[float, float, float, float] | None,
    target_quat_samples: list[tuple[float, float, float, float]],
    latest_quat: tuple[float, float, float, float] | None,
) -> tuple[float, float, float, float] | None:
    if latest_quat is None or target_quat is not None:
        return target_quat

    target_quat_samples.append(latest_quat)
    if len(target_quat_samples) < max(DXL_TARGET_SAMPLE_COUNT, 1):
        return None
    return quaternion_average(target_quat_samples)


def maybe_lock_target_angle(
    target_angle_deg: float | None,
    target_angle_samples: list[float],
    latest_angle_deg: float | None,
) -> float | None:
    if latest_angle_deg is None or target_angle_deg is not None:
        return target_angle_deg

    target_angle_samples.append(latest_angle_deg)
    if len(target_angle_samples) < max(DXL_TARGET_SAMPLE_COUNT, 1):
        return None
    return sum(target_angle_samples) / len(target_angle_samples)


def validate_initial_ticks() -> None:
    if not (DXL_GOAL_TICK_MIN <= DXL_INITIAL_TICKS <= DXL_GOAL_TICK_MAX):
        raise ValueError(
            f"DXL_INITIAL_TICKS={DXL_INITIAL_TICKS} for motor {DXL_MOTOR_ID} "
            f"is outside configured limits [{DXL_GOAL_TICK_MIN}, {DXL_GOAL_TICK_MAX}]"
        )


def clamp_goal_ticks(goal_ticks: int) -> int:
    return max(DXL_GOAL_TICK_MIN, min(DXL_GOAL_TICK_MAX, int(goal_ticks)))


def dynamixel_drive_absolute_ticks(ctx, goal_ticks: int) -> bool:
    """Write one absolute goal tick directly, bypassing shared driver clamps."""
    clamped_ticks = clamp_goal_ticks(goal_ticks)
    sync_write = ctx["sync_write_pos"]
    if not sync_write.addParam(DXL_MOTOR_ID, int(clamped_ticks).to_bytes(4, "little", signed=True)):
        sync_write.clearParam()
        print(f"[dxl] sync-write addParam failed for motor {DXL_MOTOR_ID}")
        return False

    comm = sync_write.txPacket()
    sync_write.clearParam()
    if comm != COMM_SUCCESS:
        controller = ctx["controller"]
        print(
            f"[dxl] sync-write txPacket: "
            f"{controller.packet_handler.getTxRxResult(comm)}"
        )
        return False
    return True


def dynamixel_read_present_ticks(ctx) -> int | None:
    return ctx["controller"].READ(DXL_MOTOR_ID, PRESENT_POSITION)


def wait_for_initial_position(ctx, goal_ticks: int) -> int:
    """Block until motor reaches the requested startup tick within tolerance."""
    deadline = time.monotonic() + DXL_INITIAL_POSITION_TIMEOUT_SEC
    last_ticks = dynamixel_read_present_ticks(ctx)
    while time.monotonic() < deadline and not rospy.is_shutdown():
        present_ticks = dynamixel_read_present_ticks(ctx)
        if present_ticks is not None:
            last_ticks = present_ticks
            if abs(present_ticks - goal_ticks) <= DXL_INITIAL_POSITION_TOL_TICKS:
                return present_ticks
        time.sleep(0.02)

    raise RuntimeError(
        f"Motor {DXL_MOTOR_ID} failed to reach startup tick {goal_ticks} "
        f"within {DXL_INITIAL_POSITION_TIMEOUT_SEC:.1f}s; last={last_ticks}"
    )


def teleop_step(
    joystick_data: dict[str, float],
    roll_pos: float,
    pitch_pos: float,
    boom_pos: float,
) -> tuple[float, float, float, bool]:
    lx = joystick_data["LX"]
    ly = joystick_data["LY"]
    lt = joystick_data["LT"]
    rt = joystick_data["RT"]
    xb = joystick_data["XB"]
    lb = joystick_data["LB"]
    rb = joystick_data["RB"]

    if xb:
        return roll_pos, pitch_pos, boom_pos, False

    # Preserve the existing joint-space teleop feel from the legacy script.
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

    pitch_pos = max(pitch_pos, PITCH_MIN)
    boom_pos = max(min(boom_pos, BOOM_MAX), BOOM_MIN)
    return roll_pos, pitch_pos, boom_pos, True


def main() -> int:
    rospy.init_node("run_balance", anonymous=False)

    joystick = None
    md80_ctx = None
    dxl_ctx = None

    try:
        joystick = joystick_connect()
        md80_ctx = motor_connect()

        if DXL_ENABLE_HOLD:
            validate_initial_ticks()
            dxl_ctx = dynamixel_connect(
                port=DXL_PORT,
                baudrate=DXL_BAUDRATE,
                arm_ids=(DXL_MOTOR_ID,),
                gripper_ids=(),
            )
            if not dynamixel_drive_absolute_ticks(dxl_ctx, DXL_INITIAL_TICKS):
                raise RuntimeError(f"Failed to command initial position on motor {DXL_MOTOR_ID}")
            wait_for_initial_position(dxl_ctx, DXL_INITIAL_TICKS)
            time.sleep(max(DXL_INITIAL_SETTLE_SEC, 0.0))

        roll_pos = 0.0
        pitch_pos = 0.0
        boom_pos = 0.0
        latest_abs_roll_deg: float | None = None
        latest_axis_deg: float | None = None
        latest_relative_axis_deg: float | None = None
        latest_axis_unwrapped_deg: float | None = None
        latest_relative_axis_unwrapped_deg: float | None = None
        latest_axis_rate_rad_s: float | None = None
        previous_error_deg: float | None = None
        latest_error_deg: float | None = None
        latest_error_rate_deg_s = 0.0
        latest_quat: tuple[float, float, float, float] | None = None
        target_quat: tuple[float, float, float, float] | None = None
        target_quat_samples: list[tuple[float, float, float, float]] = []
        target_axis_deg: float | None = None
        target_axis_unwrapped_deg: float | None = None
        target_axis_samples: list[float] = []
        commanded_goal_ticks = DXL_INITIAL_TICKS

        with dai.Device() as device:
            imu_type = str(device.getConnectedIMU())
            if imu_type in {"", "NONE", "UNKNOWN", "None"}:
                raise RuntimeError("No IMU detected on the connected OAK-D device.")

            device.startPipeline(build_imu_pipeline(imu_type))
            queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            rate = rospy.Rate(LOOP_HZ)

            while not rospy.is_shutdown():
                joystick_data = joystick_read(joystick)
                roll_pos, pitch_pos, boom_pos, keep_running = teleop_step(
                    joystick_data,
                    roll_pos,
                    pitch_pos,
                    boom_pos,
                )
                if not keep_running:
                    break

                while True:
                    imu_data = queue.tryGet()
                    if imu_data is None:
                        break
                    for packet in imu_data.packets:
                        packet_axis_rate = packet_axis_rate_rad_s(packet)
                        if packet_axis_rate is not None:
                            latest_axis_rate_rad_s = packet_axis_rate
                        packet_quat = packet_rotation_quaternion(packet)
                        if packet_quat is not None:
                            latest_quat = packet_quat
                            latest_abs_roll_deg = quaternion_to_roll_deg(
                                packet_quat[0],
                                packet_quat[1],
                                packet_quat[2],
                                packet_quat[3],
                            )
                            latest_axis_deg = quaternion_twist_deg(packet_quat, CONTROL_AXIS)
                            latest_axis_unwrapped_deg = unwrap_angle_deg(
                                latest_axis_unwrapped_deg,
                                latest_axis_deg,
                            )
                        else:
                            packet_roll = packet_roll_deg(packet)
                            if packet_roll is not None:
                                latest_abs_roll_deg = packet_roll

                target_quat = maybe_lock_target_quaternion(
                    target_quat,
                    target_quat_samples,
                    latest_quat,
                )

                target_axis_deg = maybe_lock_target_angle(
                    target_axis_deg,
                    target_axis_samples,
                    latest_axis_unwrapped_deg,
                )

                if target_quat is not None and latest_quat is not None:
                    latest_relative_axis_deg = relative_axis_deg(target_quat, latest_quat)
                    latest_relative_axis_unwrapped_deg = unwrap_angle_deg(
                        latest_relative_axis_unwrapped_deg,
                        latest_relative_axis_deg,
                    )
                    if target_axis_unwrapped_deg is None and latest_axis_unwrapped_deg is not None:
                        target_axis_unwrapped_deg = (
                            latest_axis_unwrapped_deg - latest_relative_axis_unwrapped_deg
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
                    and latest_error_deg is not None
                    and target_quat is not None
                ):
                    error_deg = latest_error_deg
                    if abs(error_deg) < DXL_ERROR_DEADBAND_DEG:
                        error_deg = 0.0
                        latest_error_rate_deg_s = 0.0
                    motor_velocity_rad_s = DXL_MOTOR_SIGN * (
                        DXL_ROLL_GAIN * math.radians(error_deg)
                        + DXL_DAMPING_GAIN * math.radians(latest_error_rate_deg_s)
                    )
                    goal_step_ticks = radians_to_ticks(motor_velocity_rad_s / LOOP_HZ)
                    goal_step_ticks = max(
                        -DXL_MAX_STEP_TICKS,
                        min(DXL_MAX_STEP_TICKS, goal_step_ticks),
                    )
                    commanded_goal_ticks = clamp_goal_ticks(commanded_goal_ticks + goal_step_ticks)
                    dynamixel_drive_absolute_ticks(dxl_ctx, commanded_goal_ticks)

                if latest_axis_unwrapped_deg is not None:
                    target_axis_text = (
                        f"{target_axis_unwrapped_deg:+.2f} deg"
                        if target_axis_unwrapped_deg is not None
                        else "locking..."
                    )
                    relative_axis_text = (
                        f"{latest_relative_axis_unwrapped_deg:+.2f} deg"
                        if latest_relative_axis_unwrapped_deg is not None
                        else "locking..."
                    )
                    sys.stdout.write(
                        "\r"
                        f"goal_axis_{CONTROL_AXIS}: {target_axis_text}  "
                        f"current_axis_{CONTROL_AXIS}: {latest_axis_unwrapped_deg:+.2f} deg  "
                        f"rel_axis_err: {relative_axis_text}  "
                        f"err: {(latest_error_deg or 0.0):+.2f} deg  "
                        f"err_rate: {latest_error_rate_deg_s:+.2f} deg/s  "
                        f"axis_rate: {(latest_axis_rate_rad_s or 0.0):+.2f} rad/s  "
                        f"abs_roll: {latest_abs_roll_deg:+.2f} deg  "
                        f"motor_cmd: {commanded_goal_ticks:+d} ticks"
                    )
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
