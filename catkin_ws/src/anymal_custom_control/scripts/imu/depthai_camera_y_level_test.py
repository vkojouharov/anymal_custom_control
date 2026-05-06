#!/usr/bin/env python3
"""Measure the OAK-D camera-Y level error from fused IMU orientation only.

This diagnostic does not command motors and does not require ROS. It requests
DepthAI's GAME_ROTATION_VECTOR by default, computes the selected camera-Y axis
in the fused gravity frame, and prints its angle above/below the horizontal
plane.

Usage:
    python3 depthai_camera_y_level_test.py
    python3 depthai_camera_y_level_test.py --rate 400 --print-rate 20
    python3 depthai_camera_y_level_test.py --camera-y-axis x --camera-y-sign -1
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional, Tuple

import depthai as dai


DEFAULT_RATE_HZ = 200
DEFAULT_PRINT_RATE_HZ = 20.0
IMU_BATCH_THRESHOLD = 1
IMU_MAX_BATCH_REPORTS = 10


Quaternion = Tuple[float, float, float, float]  # x, y, z, w
Vector3 = Tuple[float, float, float]


def quaternion_normalize(quat: Quaternion) -> Quaternion:
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("Invalid zero-length quaternion from IMU")
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_conjugate(quat: Quaternion) -> Quaternion:
    x, y, z, w = quat
    return (-x, -y, -z, w)


def quaternion_rotate_vector(quat: Quaternion, vec: Vector3) -> Vector3:
    """Rotate a local-frame vector by quaternion x/y/z/w into the fused frame."""
    x, y, z, w = quaternion_normalize(quat)
    vx, vy, vz = vec

    # Expanded q * [v, 0] * q^-1 rotation matrix form.
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz,
        2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy + 2.0 * (yz - wx) * vz,
        2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (1.0 - 2.0 * (xx + yy)) * vz,
    )


def axis_vector(axis_name: str, sign: float) -> Vector3:
    if axis_name == "x":
        return (sign, 0.0, 0.0)
    if axis_name == "y":
        return (0.0, sign, 0.0)
    if axis_name == "z":
        return (0.0, 0.0, sign)
    raise ValueError("axis_name must be x, y, or z")


def level_error_deg(quat: Quaternion, camera_y_axis: Vector3, invert_quaternion: bool) -> Tuple[float, Vector3]:
    """Return signed angle of camera-Y above the fused horizontal plane."""
    if invert_quaternion:
        quat = quaternion_conjugate(quat)
    y_fused = quaternion_rotate_vector(quat, camera_y_axis)
    z_component = max(-1.0, min(1.0, y_fused[2]))
    return math.degrees(math.asin(z_component)), y_fused


def rotation_sensor_enum(sensor_name: str):
    name_to_enum = {
        "game": "GAME_ROTATION_VECTOR",
        "arvr_game": "ARVR_STABILIZED_GAME_ROTATION_VECTOR",
        "rotation": "ROTATION_VECTOR",
        "arvr": "ARVR_STABILIZED_ROTATION_VECTOR",
    }
    enum_name = name_to_enum[sensor_name]
    try:
        return getattr(dai.IMUSensor, enum_name)
    except AttributeError as exc:
        raise RuntimeError(f"DepthAI build does not expose dai.IMUSensor.{enum_name}") from exc


def build_pipeline(args: argparse.Namespace) -> dai.Pipeline:
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("imu")

    imu.enableIMUSensor(rotation_sensor_enum(args.sensor), args.rate)
    imu.setBatchReportThreshold(args.batch_threshold)
    imu.setMaxBatchReports(args.max_batch_reports)
    imu.out.link(xout.input)
    return pipeline


def packet_quaternion(packet) -> Tuple[Optional[Quaternion], Optional[float]]:
    rotation = getattr(packet, "rotationVector", None)
    if rotation is None:
        return None, None

    quat = quaternion_normalize(
        (
            float(rotation.i),
            float(rotation.j),
            float(rotation.k),
            float(rotation.real),
        )
    )
    accuracy = getattr(rotation, "rotationVectorAccuracy", None)
    return quat, float(accuracy) if accuracy is not None else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure camera-Y angle above/below horizontal using OAK-D fused IMU orientation."
    )
    parser.add_argument(
        "--sensor",
        choices=("game", "arvr_game", "rotation", "arvr"),
        default="game",
        help="Fused rotation source. Default avoids magnetometer yaw correction.",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=DEFAULT_RATE_HZ,
        help="Requested fused rotation-vector rate in Hz.",
    )
    parser.add_argument(
        "--print-rate",
        type=float,
        default=DEFAULT_PRINT_RATE_HZ,
        help="Terminal print rate in Hz; IMU can run faster than this.",
    )
    parser.add_argument(
        "--camera-y-axis",
        choices=("x", "y", "z"),
        default="y",
        help="Which OAK IMU-frame axis corresponds to camera +Y for this mount.",
    )
    parser.add_argument(
        "--camera-y-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
        help="Sign of the selected IMU-frame camera-Y axis.",
    )
    parser.add_argument(
        "--invert-quaternion",
        action="store_true",
        help="Use quaternion inverse if the reported orientation is opposite of the expected convention.",
    )
    parser.add_argument(
        "--batch-threshold",
        type=int,
        default=IMU_BATCH_THRESHOLD,
        help="DepthAI IMU batch report threshold.",
    )
    parser.add_argument(
        "--max-batch-reports",
        type=int,
        default=IMU_MAX_BATCH_REPORTS,
        help="DepthAI maximum IMU reports per batch.",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="Print CSV rows instead of a single updating terminal line.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rate < 100:
        print("Requested rate is below 100 Hz; use 100 Hz or higher for this diagnostic.", file=sys.stderr)
        return 2
    if args.print_rate <= 0.0:
        print("--print-rate must be positive.", file=sys.stderr)
        return 2

    camera_y_axis = axis_vector(args.camera_y_axis, args.camera_y_sign)
    min_print_period = 1.0 / args.print_rate

    with dai.Device() as device:
        imu_type = str(device.getConnectedIMU())
        firmware = str(device.getIMUFirmwareVersion())
        if imu_type in {"", "NONE", "UNKNOWN", "None"}:
            print("No IMU detected on the connected OAK-D device.", file=sys.stderr)
            return 1
        if imu_type != "BNO086":
            print(f"Connected IMU is {imu_type}; fused game rotation vector usually requires BNO086.", file=sys.stderr)
            return 1

        print(f"Connected IMU: {imu_type}")
        print(f"IMU firmware: {firmware}")
        print(f"Requested sensor: {args.sensor}")
        print(f"Requested IMU rate: {args.rate} Hz")
        print(f"Camera +Y mapping: {args.camera_y_sign:+.0f} * IMU {args.camera_y_axis.upper()} axis")
        print("Level angle: asin(camera_y_axis dot fused_up). Zero means camera Y is horizontal.")
        if args.csv:
            print("t_host_sec,level_error_deg,y_fused_x,y_fused_y,y_fused_z,accuracy_rad")
        else:
            print("Press Ctrl+C to stop.")

        device.startPipeline(build_pipeline(args))
        queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

        last_print = 0.0
        while True:
            imu_data = queue.get()
            for packet in imu_data.packets:
                quat, accuracy = packet_quaternion(packet)
                if quat is None:
                    continue

                now = time.monotonic()
                if now - last_print < min_print_period:
                    continue
                last_print = now

                angle_deg, y_fused = level_error_deg(quat, camera_y_axis, args.invert_quaternion)
                accuracy_text = "" if accuracy is None else f"{accuracy:.6f}"
                if args.csv:
                    print(
                        f"{time.time():.6f},{angle_deg:.6f},"
                        f"{y_fused[0]:.6f},{y_fused[1]:.6f},{y_fused[2]:.6f},{accuracy_text}"
                    )
                else:
                    sys.stdout.write(
                        "\r"
                        f"camera_y_level_error: {angle_deg:+7.3f} deg  "
                        f"y_fused: [{y_fused[0]:+6.3f}, {y_fused[1]:+6.3f}, {y_fused[2]:+6.3f}]"
                    )
                    if accuracy is not None:
                        sys.stdout.write(f"  accuracy: {accuracy:.4f} rad")
                    sys.stdout.flush()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        sys.stdout.write("\n")
        raise SystemExit(130)
