#!/usr/bin/env python3
"""Terminal-only DepthAI IMU debugger.

Prints raw accelerometer and gyroscope values directly to stdout so the IMU can
be validated without any browser or plotting layer in the loop.

Usage:
    python3 depthai_imu_terminal_debug.py
    python3 depthai_imu_terminal_debug.py --rate-accel 125 --rate-gyro 100
    python3 depthai_imu_terminal_debug.py --with-rotation-vector
"""

import argparse
import math
import time

import depthai as dai


def fmt_triplet(vec):
    return f"x: {vec.x:+.6f} y: {vec.y:+.6f} z: {vec.z:+.6f}"


def print_packet(packet, sample_idx, with_rotation_vector):
    accel = getattr(packet, "acceleroMeter", None)
    gyro = getattr(packet, "gyroscope", None)
    rotation = getattr(packet, "rotationVector", None) if with_rotation_vector else None

    print(f"Sample {sample_idx}")

    if accel is not None:
        accel_ts = accel.getTimestamp()
        accel_mag = math.sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z)
        print(f"  Accelerometer ts: {accel_ts}")
        print(f"  Accelerometer [m/s^2]: {fmt_triplet(accel)} | |a|={accel_mag:.6f}")
    else:
        print("  Accelerometer: unavailable")

    if gyro is not None:
        gyro_ts = gyro.getTimestamp()
        gyro_mag = math.sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z)
        print(f"  Gyroscope ts: {gyro_ts}")
        print(f"  Gyroscope [rad/s]: {fmt_triplet(gyro)} | |w|={gyro_mag:.6f}")
    else:
        print("  Gyroscope: unavailable")

    if rotation is not None:
        print(
            "  Rotation vector: "
            f"i: {rotation.i:+.6f} j: {rotation.j:+.6f} "
            f"k: {rotation.k:+.6f} real: {rotation.real:+.6f}"
        )

    print()


def build_pipeline(args, imu_type):
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    xout = pipeline.create(dai.node.XLinkOut)

    xout.setStreamName("imu")

    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, args.rate_accel)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, args.rate_gyro)

    if args.with_rotation_vector and imu_type == "BNO086":
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, min(args.rate_accel, 100))

    imu.setBatchReportThreshold(args.batch_threshold)
    imu.setMaxBatchReports(args.max_batch_reports)
    imu.out.link(xout.input)
    return pipeline


def main():
    parser = argparse.ArgumentParser(description="Terminal-only DepthAI IMU debugger")
    parser.add_argument("--rate-accel", type=int, default=125, help="Requested raw accel rate in Hz")
    parser.add_argument("--rate-gyro", type=int, default=100, help="Requested raw gyro rate in Hz")
    parser.add_argument(
        "--with-rotation-vector",
        action="store_true",
        help="Also request fused rotation vector when supported by the IMU",
    )
    parser.add_argument(
        "--batch-threshold",
        type=int,
        default=1,
        help="IMU batch report threshold",
    )
    parser.add_argument(
        "--max-batch-reports",
        type=int,
        default=10,
        help="Maximum IMU batch reports",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.0,
        help="Optional sleep between prints to reduce terminal spam",
    )
    args = parser.parse_args()

    with dai.Device() as device:
        imu_type = str(device.getConnectedIMU())
        firmware = str(device.getIMUFirmwareVersion())

        print(f"Connected IMU: {imu_type}")
        print(f"IMU firmware: {firmware}")
        print(f"Requested accel rate: {args.rate_accel} Hz")
        print(f"Requested gyro rate: {args.rate_gyro} Hz")
        print(f"Rotation vector requested: {args.with_rotation_vector and imu_type == 'BNO086'}")
        print()

        if imu_type in {"", "NONE", "UNKNOWN", "None"}:
            print("No IMU detected on this device.")
            return

        pipeline = build_pipeline(args, imu_type)
        device.startPipeline(pipeline)
        queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

        sample_idx = 0
        while True:
            imu_data = queue.get()
            for packet in imu_data.packets:
                sample_idx += 1
                print_packet(packet, sample_idx, args.with_rotation_vector)
                if args.sleep > 0.0:
                    time.sleep(args.sleep)


if __name__ == "__main__":
    main()
