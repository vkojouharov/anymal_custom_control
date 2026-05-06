#!/usr/bin/env python3
"""ROS owner for OAK-D RGB/depth, AprilTag, and fused IMU data.

This node is the single DepthAI owner in the production operator station. It
publishes compressed RGB/depth visualization frames plus GAME_ROTATION_VECTOR
and the derived camera-Y angle above/below the horizontal plane.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Optional, Tuple

import cv2
import depthai as dai
import numpy as np
import rospy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, String

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None


Quaternion = Tuple[float, float, float, float]  # x, y, z, w
Vector3 = Tuple[float, float, float]

IMU_RATE_HZ = 200
IMU_BATCH_THRESHOLD = 1
IMU_MAX_BATCH_REPORTS = 10
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
RGB_FPS = 30
DEPTH_MIN_MM = 10
DEPTH_MAX_MM = 1000
JPEG_QUALITY = 75
APRILTAG_FAMILY = "tag16h5"
APRILTAG_DECISION_MARGIN = 50
APRILTAG_QUAD_DECIMATE = 1.0
APRILTAG_THREADS = 2
TAG_SIZE_M = 0.0956

CAMERA_Y_AXIS = "y"
CAMERA_Y_SIGN = 1.0
INVERT_IMU_QUATERNION = False

FRAME_ID = "oakd_imu_fused"
QUATERNION_TOPIC = "/oakd/imu/game_rotation_vector"
CAMERA_Y_FUSED_TOPIC = "/oakd/camera_y_axis_fused"
LEVEL_ERROR_TOPIC = "/oakd/camera_y_level_error"
ARM_STATE_TOPIC = "/giraf_arm/state"
RGB_COMPRESSED_TOPIC = "/oakd/rgb/image_color/compressed"
DEPTH_COMPRESSED_TOPIC = "/oakd/depth/image_colorized/compressed"
APRILTAG_STATS_TOPIC = "/oakd/apriltag/stats_json"


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
    x, y, z, w = quaternion_normalize(quat)
    vx, vy, vz = vec

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
    raise ValueError("axis name must be x, y, or z")


def packet_game_quaternion(packet) -> Optional[Quaternion]:
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


def camera_y_level_error_rad(quat: Quaternion, camera_y_axis: Vector3) -> Tuple[float, Vector3]:
    if INVERT_IMU_QUATERNION:
        quat = quaternion_conjugate(quat)
    y_fused = quaternion_rotate_vector(quat, camera_y_axis)
    z_component = max(-1.0, min(1.0, y_fused[2]))
    return math.asin(z_component), y_fused


def build_pipeline() -> dai.Pipeline:
    pipeline = dai.Pipeline()
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    imu = pipeline.create(dai.node.IMU)
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_imu = pipeline.create(dai.node.XLinkOut)

    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")
    xout_imu.setStreamName("imu")

    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(FRAME_WIDTH, FRAME_HEIGHT)
    cam_rgb.setPreviewKeepAspectRatio(False)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(RGB_FPS)

    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setFps(RGB_FPS)
    mono_right.setFps(RGB_FPS)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(FRAME_WIDTH, FRAME_HEIGHT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)

    imu.enableIMUSensor(dai.IMUSensor.GAME_ROTATION_VECTOR, IMU_RATE_HZ)
    imu.setBatchReportThreshold(IMU_BATCH_THRESHOLD)
    imu.setMaxBatchReports(IMU_MAX_BATCH_REPORTS)

    cam_rgb.preview.link(xout_rgb.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)
    imu.out.link(xout_imu.input)
    return pipeline


def publish_quaternion(pub: rospy.Publisher, stamp: rospy.Time, quat: Quaternion) -> None:
    msg = QuaternionStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ID
    msg.quaternion.x = quat[0]
    msg.quaternion.y = quat[1]
    msg.quaternion.z = quat[2]
    msg.quaternion.w = quat[3]
    pub.publish(msg)


def publish_vector(pub: rospy.Publisher, stamp: rospy.Time, vec: Vector3) -> None:
    msg = Vector3Stamped()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ID
    msg.vector.x = vec[0]
    msg.vector.y = vec[1]
    msg.vector.z = vec[2]
    pub.publish(msg)


def colorize_depth(depth_frame: np.ndarray) -> np.ndarray:
    clipped = np.clip(depth_frame, DEPTH_MIN_MM, DEPTH_MAX_MM)
    normalized = ((clipped - DEPTH_MIN_MM) * (255.0 / (DEPTH_MAX_MM - DEPTH_MIN_MM))).astype(np.uint8)
    colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    colorized[depth_frame == 0] = (0, 0, 0)
    return colorized


def detect_apriltags(detector, frame: np.ndarray, camera_params: Tuple[float, float, float, float]) -> list:
    if detector is None:
        return []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=TAG_SIZE_M,
    )
    return [det for det in detections if det.decision_margin > APRILTAG_DECISION_MARGIN]


def compute_tag_corner_depths(det) -> Optional[np.ndarray]:
    pose_R = getattr(det, "pose_R", None)
    pose_t = getattr(det, "pose_t", None)
    if pose_R is None or pose_t is None:
        return None

    half = TAG_SIZE_M / 2.0
    tag_corners = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    camera_corners = (pose_R @ tag_corners.T).T + pose_t.reshape(1, 3)
    return camera_corners[:, 2]


def compute_masked_depth_mm(depth_frame: np.ndarray, det) -> Optional[dict]:
    mask = np.zeros(depth_frame.shape, dtype=np.uint8)
    polygon = np.round(det.corners).astype(np.int32)
    cv2.fillConvexPoly(mask, polygon, 255)

    region = depth_frame[mask == 255]
    valid = region[(region > 0) & (region >= DEPTH_MIN_MM) & (region <= DEPTH_MAX_MM)]
    if valid.size == 0:
        return None

    median = float(np.median(valid))
    deviations = np.abs(valid - median)
    mad = float(np.median(deviations))
    if mad > 0.0:
        valid = valid[deviations <= 3.0 * mad]
    if valid.size == 0:
        return None

    return {
        "mean_mm": float(np.mean(valid)),
        "median_mm": float(np.median(valid)),
        "count": int(valid.size),
    }


def format_rgb_summary(detections: list) -> str:
    if not detections:
        return "No detections"

    parts = []
    for det in detections:
        corner_depths = compute_tag_corner_depths(det)
        if corner_depths is None:
            parts.append(f"ID{det.tag_id}: unavailable")
            continue
        avg_z_m = float(np.mean(corner_depths))
        parts.append(f"ID{det.tag_id}: avg Z {avg_z_m:.3f} m")
    return " | ".join(parts)


def format_depth_summary(depth_frame: Optional[np.ndarray], detections: list) -> str:
    if not detections:
        return "No detections"
    if depth_frame is None:
        return "Waiting for depth frame"

    parts = []
    for det in detections:
        stats = compute_masked_depth_mm(depth_frame, det)
        if stats is None:
            parts.append(f"ID{det.tag_id}: unavailable")
            continue
        parts.append(
            f"ID{det.tag_id}: median {stats['median_mm'] / 1000.0:.3f} m "
            f"mean {stats['mean_mm'] / 1000.0:.3f} m"
        )
    return " | ".join(parts)


def draw_apriltags(frame: np.ndarray, detections: list) -> None:
    for det in detections:
        corners = det.corners.astype(int)
        for idx in range(4):
            cv2.line(frame, tuple(corners[idx]), tuple(corners[(idx + 1) % 4]), (0, 255, 0), 2)

        cx, cy = int(det.center[0]), int(det.center[1])
        size = 15
        cv2.line(frame, (cx - size, cy), (cx + size, cy), (0, 255, 0), 2)
        cv2.line(frame, (cx, cy - size), (cx, cy + size), (0, 255, 0), 2)

        label = f"ID:{det.tag_id} ({cx},{cy}) m:{det.decision_margin:.0f}"
        cv2.putText(frame, label, (corners[0][0], corners[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def draw_apriltag_status(frame: np.ndarray, detections: list, detect_fps: float) -> None:
    if Detector is None:
        status = "AprilTag detector unavailable"
        color = (0, 0, 255)
    else:
        status = f"AprilTag {APRILTAG_FAMILY} {len(detections)} tags {detect_fps:.1f} fps"
        color = (255, 255, 255)
    cv2.putText(frame, status, (10, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def publish_apriltag_stats(pub: rospy.Publisher, detect_fps: float, detections: list, depth_frame: Optional[np.ndarray]) -> None:
    payload = {
        "enabled": Detector is not None,
        "fps": float(detect_fps),
        "detections": int(len(detections)),
        "rgb_summary": format_rgb_summary(detections),
        "depth_summary": format_depth_summary(depth_frame, detections),
        "tag_ids": [int(det.tag_id) for det in detections],
        "stamp_sec": time.time(),
    }
    pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def publish_compressed_frame(pub: rospy.Publisher, stamp: rospy.Time, frame_id: str, frame: np.ndarray) -> None:
    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        return
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = jpeg.tobytes()
    pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish OAK-D GAME_ROTATION_VECTOR and camera-Y level error to ROS.")
    parser.add_argument(
        "--wait-for-arm-state",
        action="store_true",
        help="Wait for the first /giraf_arm/state before opening the OAK-D device",
    )
    parser.add_argument(
        "--arm-state-timeout",
        type=float,
        default=30.0,
        help="Timeout in seconds when --wait-for-arm-state is set",
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("oakd_sensor_node", anonymous=False)
    quat_pub = rospy.Publisher(QUATERNION_TOPIC, QuaternionStamped, queue_size=1, tcp_nodelay=True)
    y_axis_pub = rospy.Publisher(CAMERA_Y_FUSED_TOPIC, Vector3Stamped, queue_size=1, tcp_nodelay=True)
    error_pub = rospy.Publisher(LEVEL_ERROR_TOPIC, Float64, queue_size=1, tcp_nodelay=True)
    rgb_pub = rospy.Publisher(RGB_COMPRESSED_TOPIC, CompressedImage, queue_size=1, tcp_nodelay=True)
    depth_pub = rospy.Publisher(DEPTH_COMPRESSED_TOPIC, CompressedImage, queue_size=1, tcp_nodelay=True)
    apriltag_pub = rospy.Publisher(APRILTAG_STATS_TOPIC, String, queue_size=1, tcp_nodelay=True)
    camera_y_axis = axis_vector(CAMERA_Y_AXIS, CAMERA_Y_SIGN)
    detector = (
        Detector(families=APRILTAG_FAMILY, nthreads=APRILTAG_THREADS, quad_decimate=APRILTAG_QUAD_DECIMATE)
        if Detector is not None
        else None
    )

    if args.wait_for_arm_state:
        rospy.loginfo("Waiting for %s before opening OAK-D IMU", ARM_STATE_TOPIC)
        rospy.wait_for_message(ARM_STATE_TOPIC, String, timeout=args.arm_state_timeout)
        rospy.loginfo("Received initial %s; opening OAK-D IMU", ARM_STATE_TOPIC)

    with dai.Device() as device:
        imu_type = str(device.getConnectedIMU())
        firmware = str(device.getIMUFirmwareVersion())
        if imu_type in {"", "NONE", "UNKNOWN", "None"}:
            raise RuntimeError("No IMU detected on the connected OAK-D device.")
        if imu_type != "BNO086":
            raise RuntimeError(f"Connected IMU is {imu_type}; GAME_ROTATION_VECTOR requires BNO086.")
        calib = device.readFactoryCalibration()
        intrinsics = np.array(
            calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, FRAME_WIDTH, FRAME_HEIGHT),
            dtype=np.float64,
        )
        camera_params = (
            float(intrinsics[0, 0]),
            float(intrinsics[1, 1]),
            float(intrinsics[0, 2]),
            float(intrinsics[1, 2]),
        )

        rospy.loginfo("OAK-D IMU connected: %s firmware=%s rate=%dHz", imu_type, firmware, IMU_RATE_HZ)
        if detector is None:
            rospy.logwarn("AprilTag detector unavailable; install pupil_apriltags for tag stats")
        else:
            rospy.loginfo("AprilTag detector enabled: %s tag_size=%.4fm", APRILTAG_FAMILY, TAG_SIZE_M)
        device.startPipeline(build_pipeline())
        queues = {
            "rgb": device.getOutputQueue(name="rgb", maxSize=2, blocking=False),
            "depth": device.getOutputQueue(name="depth", maxSize=2, blocking=False),
            "imu": device.getOutputQueue(name="imu", maxSize=50, blocking=False),
        }
        detect_count = 0
        detect_timer = time.perf_counter()
        detect_fps = 0.0
        latest_detections = []
        latest_depth_frame = None

        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            rgb_msg = queues["rgb"].tryGet()
            if rgb_msg is not None:
                rgb_frame = rgb_msg.getCvFrame()
                try:
                    latest_detections = detect_apriltags(detector, rgb_frame, camera_params)
                except Exception as exc:
                    rospy.logwarn_throttle(2.0, "AprilTag detection failed: %s", exc)
                    latest_detections = []
                detect_count += 1
                now = time.perf_counter()
                elapsed = now - detect_timer
                if elapsed >= 1.0:
                    detect_fps = detect_count / elapsed
                    detect_count = 0
                    detect_timer = now
                draw_apriltags(rgb_frame, latest_detections)
                draw_apriltag_status(rgb_frame, latest_detections, detect_fps)
                publish_apriltag_stats(apriltag_pub, detect_fps, latest_detections, latest_depth_frame)
                publish_compressed_frame(rgb_pub, stamp, "oakd_rgb", rgb_frame)

            depth_msg = queues["depth"].tryGet()
            if depth_msg is not None:
                latest_depth_frame = depth_msg.getFrame()
                depth_color = colorize_depth(latest_depth_frame)
                draw_apriltags(depth_color, latest_detections)
                draw_apriltag_status(depth_color, latest_detections, detect_fps)
                publish_apriltag_stats(apriltag_pub, detect_fps, latest_detections, latest_depth_frame)
                publish_compressed_frame(
                    depth_pub,
                    stamp,
                    "oakd_depth_colorized",
                    depth_color,
                )

            imu_data = queues["imu"].tryGet()
            if imu_data is not None:
                for packet in imu_data.packets:
                    quat = packet_game_quaternion(packet)
                    if quat is None:
                        continue
                    error_rad, y_fused = camera_y_level_error_rad(quat, camera_y_axis)
                    publish_quaternion(quat_pub, stamp, quat)
                    publish_vector(y_axis_pub, stamp, y_fused)
                    error_pub.publish(Float64(data=error_rad))

            if rgb_msg is None and depth_msg is None and imu_data is None:
                rospy.sleep(0.001)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
