#!/usr/bin/env python3
"""Launch the GIRAF wrist teleop stack with a separate balance motor.

This entrypoint mirrors run_arm_jparse_teleop.py, but also owns a dedicated
OAK-D-driven balancing Dynamixel. The wrist controller keeps owning its own
motors; this file only commands the balance motor configured below.

Edit the config block directly in code.
"""

from __future__ import annotations

import math
import sys
import time

import depthai as dai
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
from anymal_custom_control.runtime import LaunchSpec, ProcessManager, run_script_path


# Balance motor config. Edit these values for motor ID 15.
LOOP_HZ = 200.0

IMU_RATE_ACCEL_HZ = 125
IMU_RATE_GYRO_HZ = 100
IMU_RATE_ROTATION_VECTOR_HZ = 100
IMU_BATCH_THRESHOLD = 1
IMU_MAX_BATCH_REPORTS = 10

DXL_ENABLE_HOLD = True
DXL_PORT = "/dev/ttyUSB0"
DXL_BAUDRATE = 1_000_000
DXL_MOTOR_ID = 15
DXL_INITIAL_TICKS = 3075
DXL_GOAL_TICK_MIN = 2000
DXL_GOAL_TICK_MAX = 5000
DXL_ROLL_GAIN = 0
DXL_DAMPING_GAIN = 0
DXL_MOTOR_SIGN = 0
DXL_INITIAL_SETTLE_SEC = 0.2
DXL_INITIAL_POSITION_TOL_TICKS = 15
DXL_INITIAL_POSITION_TIMEOUT_SEC = 5.0
DXL_TARGET_SAMPLE_COUNT = 20
CONTROL_AXIS = "z"
DXL_ERROR_DEADBAND_DEG = 0.75
DXL_MAX_STEP_TICKS = 20
BALANCE_ENABLE_DEVICE_INDEX = 0


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
    deadline = time.monotonic() + DXL_INITIAL_POSITION_TIMEOUT_SEC
    last_ticks = dynamixel_read_present_ticks(ctx)
    while time.monotonic() < deadline:
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


def run_balance_loop(processes: dict[str, object]) -> int:
    dxl_ctx = None
    joystick = None
    commanded_goal_ticks = DXL_INITIAL_TICKS
    balance_active = False
    prev_enable_combo = 0
    latest_abs_roll_deg: float | None = None
    latest_abs_roll_unwrapped_deg: float | None = None
    latest_axis_deg: float | None = None
    latest_relative_axis_deg: float | None = None
    latest_axis_unwrapped_deg: float | None = None
    latest_relative_axis_unwrapped_deg: float | None = None
    previous_error_deg: float | None = None
    latest_error_deg: float | None = None
    latest_error_rate_deg_s = 0.0
    latest_quat: tuple[float, float, float, float] | None = None
    target_quat: tuple[float, float, float, float] | None = None
    target_abs_roll_deg: float | None = None
    target_quat_samples: list[tuple[float, float, float, float]] = []
    target_axis_deg: float | None = None
    target_axis_unwrapped_deg: float | None = None
    target_axis_samples: list[float] = []

    try:
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

        try:
            joystick = joystick_connect(device_index=BALANCE_ENABLE_DEVICE_INDEX)
        except Exception as exc:
            print(f"[balance] Could not open joystick for YB activation: {exc}")
            joystick = None

        with dai.Device() as device:
            imu_type = str(device.getConnectedIMU())
            if imu_type in {"", "NONE", "UNKNOWN", "None"}:
                raise RuntimeError("No IMU detected on the connected OAK-D device.")

            device.startPipeline(build_imu_pipeline(imu_type))
            queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            dt = 1.0 / LOOP_HZ

            while True:
                for name, proc in processes.items():
                    code = proc.poll()
                    if code is not None:
                        print(f"\n{name} exited with code {code}")
                        return code

                if joystick is not None:
                    joystick_data = joystick_read(joystick)
                    enable_combo = int(
                        joystick_data["LB"] and joystick_data["RB"] and joystick_data["YB"]
                    )
                    if not balance_active and enable_combo and not prev_enable_combo:
                        balance_active = True
                        print("\n[balance] Latched active from LB+RB+YB")
                    prev_enable_combo = enable_combo

                while True:
                    imu_data = queue.tryGet()
                    if imu_data is None:
                        break
                    for packet in imu_data.packets:
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
                        latest_axis_deg = quaternion_twist_deg(packet_quat, CONTROL_AXIS)
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
                    goal_step_ticks = radians_to_ticks(motor_velocity_rad_s * dt)
                    goal_step_ticks = max(
                        -DXL_MAX_STEP_TICKS,
                        min(DXL_MAX_STEP_TICKS, goal_step_ticks),
                    )
                    if balance_active:
                        commanded_goal_ticks = clamp_goal_ticks(commanded_goal_ticks + goal_step_ticks)
                    else:
                        commanded_goal_ticks = DXL_INITIAL_TICKS
                    dynamixel_drive_absolute_ticks(dxl_ctx, commanded_goal_ticks)

                if latest_abs_roll_unwrapped_deg is not None:
                    target_roll_text = (
                        f"{target_abs_roll_deg:+.2f} deg"
                        if target_abs_roll_deg is not None
                        else "locking..."
                    )
                    sys.stdout.write(
                        "\r"
                        f"goal_roll: {target_roll_text}  "
                        f"current_roll: {latest_abs_roll_unwrapped_deg:+.2f} deg"
                    )
                    sys.stdout.flush()

                time.sleep(dt)
    finally:
        sys.stdout.write("\n")
        sys.stdout.flush()
        if joystick is not None:
            joystick_disconnect(joystick)
        if dxl_ctx is not None:
            dynamixel_disconnect(dxl_ctx)


def main() -> int:
    specs = [
        LaunchSpec(
            name="giraf_arm_controller",
            command=[sys.executable, str(run_script_path("run_giraf_arm_controller.py"))],
        ),
        LaunchSpec(
            name="giraf_arm_teleop",
            command=[sys.executable, str(run_script_path("run_giraf_arm_teleop.py"))],
        ),
    ]

    manager = ProcessManager()
    processes: dict[str, object] = {}
    for spec in specs:
        print(f"Starting {spec.name}: {' '.join(spec.command)}")
        processes[spec.name] = manager.start(spec)

    try:
        return run_balance_loop(processes)
    except KeyboardInterrupt:
        print("\nShutting down GIRAF wrist balance stack...")
        return 0
    finally:
        manager.terminate_all()


if __name__ == "__main__":
    raise SystemExit(main())
