#!/usr/bin/env python3
"""Print the live OAK-D IMU roll angle to the terminal."""

from __future__ import annotations

import math
import sys

import depthai as dai


IMU_RATE_ACCEL_HZ = 125
IMU_RATE_GYRO_HZ = 100
IMU_RATE_ROTATION_VECTOR_HZ = 100
IMU_BATCH_THRESHOLD = 1
IMU_MAX_BATCH_REPORTS = 10
CONTROL_AXIS = "z"
TARGET_SAMPLE_COUNT = 20


def quaternion_to_roll_deg(i: float, j: float, k: float, real: float) -> float:
    sinr_cosp = 2.0 * (real * i + j * k)
    cosr_cosp = 1.0 - 2.0 * (i * i + j * j)
    return math.degrees(math.atan2(sinr_cosp, cosr_cosp))


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


def accel_to_roll_deg(x: float, y: float, z: float) -> float:
    return math.degrees(math.atan2(y, z))


def axis_vector(axis_name: str) -> tuple[float, float, float]:
    if axis_name == "x":
        return (1.0, 0.0, 0.0)
    if axis_name == "y":
        return (0.0, 1.0, 0.0)
    if axis_name == "z":
        return (0.0, 0.0, 1.0)
    raise ValueError(f"Unsupported CONTROL_AXIS={axis_name!r}")


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


def build_pipeline(imu_type: str) -> dai.Pipeline:
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
        return accel_to_roll_deg(float(accel.x), float(accel.y), float(accel.z))

    return None


def relative_axis_deg(
    target_quat: tuple[float, float, float, float],
    current_quat: tuple[float, float, float, float],
) -> float:
    relative_quat = quaternion_multiply(quaternion_conjugate(target_quat), current_quat)
    relative_quat = quaternion_normalize(relative_quat)
    return quaternion_twist_deg(relative_quat, CONTROL_AXIS)


def main() -> int:
    with dai.Device() as device:
        imu_type = str(device.getConnectedIMU())
        if imu_type in {"", "NONE", "UNKNOWN", "None"}:
            print("No IMU detected on the connected OAK-D device.", file=sys.stderr)
            return 1

        device.startPipeline(build_pipeline(imu_type))
        queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        latest_control_angle_deg: float | None = None
        latest_control_angle_unwrapped_deg: float | None = None
        latest_quat: tuple[float, float, float, float] | None = None
        target_quat: tuple[float, float, float, float] | None = None
        target_quat_samples: list[tuple[float, float, float, float]] = []

        while True:
            imu_data = queue.get()
            for packet in imu_data.packets:
                packet_quat = packet_rotation_quaternion(packet)
                if packet_quat is not None:
                    latest_quat = packet_quat
                    if target_quat is None:
                        target_quat_samples.append(packet_quat)
                        if len(target_quat_samples) >= max(TARGET_SAMPLE_COUNT, 1):
                            target_quat = quaternion_average(target_quat_samples)
                    if target_quat is not None:
                        latest_control_angle_deg = relative_axis_deg(target_quat, packet_quat)
                        latest_control_angle_unwrapped_deg = unwrap_angle_deg(
                            latest_control_angle_unwrapped_deg,
                            latest_control_angle_deg,
                        )
                if latest_control_angle_unwrapped_deg is None:
                    roll_deg = packet_roll_deg(packet)
                    if roll_deg is None:
                        continue
                    sys.stdout.write(
                        f"\rcontrol_loop_angle_{CONTROL_AXIS}: locking...  raw_roll: {roll_deg:+.2f} deg"
                    )
                    sys.stdout.flush()
                    continue

                sys.stdout.write(
                    f"\rcontrol_loop_angle_{CONTROL_AXIS}: {latest_control_angle_unwrapped_deg:+.2f} deg"
                )
                sys.stdout.flush()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        sys.stdout.write("\n")
        sys.stdout.flush()
