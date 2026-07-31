#!/usr/bin/env python3
"""Stream OAK RGB with tag16h5 ID 1 pose over HTTP.

Camera capture and AprilTag detection run independently so detector latency does
not throttle the 30 FPS RGB stream. The reported tag position is the translation
column of the tag-to-camera pose, in metres.

Usage:
    python3 depthai_apriltag_rgb.py
    python3 depthai_apriltag_rgb.py --port 5003
"""

import argparse
import socket
import threading
import time

import cv2
import depthai as dai
import numpy as np
from flask import Flask, Response, jsonify, render_template_string

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None


app = Flask(__name__)

FRAME_WIDTH = 640
FRAME_HEIGHT = 360
RGB_FPS = 30
APRILTAG_FAMILY = "tag16h5"
TARGET_TAG_ID = 1
TAG_SIZE_M = 0.049
APRILTAG_DECISION_MARGIN = 35.0
APRILTAG_THREADS = 2
APRILTAG_QUAD_DECIMATE = 1.0

lock = threading.Lock()
new_frame_event = threading.Event()
detection_frame_event = threading.Event()

latest_frame = None
pending_detection_frame = None
latest_detection = None
capture_stats = {
    "fps": 0.0,
    "width": 0,
    "height": 0,
}
camera_intrinsics = None
tag_stats = {
    "enabled": Detector is not None,
    "detected": False,
    "id": TARGET_TAG_ID,
    "family": APRILTAG_FAMILY,
    "size_m": TAG_SIZE_M,
    "position_m": None,
    "decision_margin": None,
    "detection_fps": 0.0,
    "error": None,
}

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>DepthAI AprilTag RGB Stream</title>
    <style>
        body { background: #1a1a1a; color: #eee; font-family: monospace; text-align: center; margin: 20px; }
        h1 { color: #4CAF50; }
        img { max-width: 100%; border: 2px solid #333; border-radius: 4px; }
        .stats { margin-top: 12px; font-size: 18px; color: #cfcfcf; }
        .pose { color: #7CFC90; }
        .muted { color: #999; font-size: 14px; }
    </style>
</head>
<body>
    <h1>DepthAI RGB + AprilTag</h1>
    <img src="/feed" />
    <div class="stats" id="stream-stats">Waiting for frames...</div>
    <div class="stats pose" id="tag-position">Waiting for tag16h5 ID 1...</div>
    <div class="stats" id="detector-stats">Waiting for detector...</div>
    <div class="stats muted" id="intrinsics">Loading camera intrinsics...</div>
    <div class="stats muted">Camera frame: +X right, +Y down, +Z forward</div>
    <script>
        async function refreshStats() {
            try {
                const res = await fetch('/stats');
                const stats = await res.json();
                document.getElementById('stream-stats').textContent =
                    `Capture FPS: ${stats.stream.fps.toFixed(1)} | Size: ${stats.stream.width}x${stats.stream.height}`;

                if (!stats.tag.enabled) {
                    document.getElementById('tag-position').textContent =
                        'AprilTag detector unavailable (install pupil_apriltags)';
                } else if (stats.tag.detected && stats.tag.position_m) {
                    const p = stats.tag.position_m;
                    document.getElementById('tag-position').textContent =
                        `Tag ID ${stats.tag.id} position in camera frame: ` +
                        `x=${p[0].toFixed(4)} m, y=${p[1].toFixed(4)} m, z=${p[2].toFixed(4)} m`;
                } else {
                    document.getElementById('tag-position').textContent =
                        `Tag ${stats.tag.family} ID ${stats.tag.id} not detected`;
                }

                document.getElementById('detector-stats').textContent = stats.tag.error
                    ? `Detector error: ${stats.tag.error}`
                    : `Detection FPS: ${stats.tag.detection_fps.toFixed(1)} | ` +
                      `Tag size: ${(stats.tag.size_m * 1000).toFixed(0)} mm`;

                if (stats.intrinsics) {
                    document.getElementById('intrinsics').textContent =
                        `Intrinsics: fx=${stats.intrinsics.fx.toFixed(2)}, ` +
                        `fy=${stats.intrinsics.fy.toFixed(2)}, ` +
                        `cx=${stats.intrinsics.cx.toFixed(2)}, ` +
                        `cy=${stats.intrinsics.cy.toFixed(2)}`;
                }
            } catch (err) {
                document.getElementById('stream-stats').textContent = 'Stats unavailable';
            }
        }

        refreshStats();
        setInterval(refreshStats, 250);
    </script>
</body>
</html>
"""


def create_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    xout_rgb = pipeline.create(dai.node.XLinkOut)

    xout_rgb.setStreamName("rgb")
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(FRAME_WIDTH, FRAME_HEIGHT)
    cam_rgb.setPreviewKeepAspectRatio(False)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(RGB_FPS)
    cam_rgb.preview.link(xout_rgb.input)

    return pipeline


def detect_target(detector, frame, camera_params):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=TAG_SIZE_M,
    )
    matches = [
        det
        for det in detections
        if det.tag_id == TARGET_TAG_ID
        and det.decision_margin > APRILTAG_DECISION_MARGIN
    ]
    if not matches:
        return None
    return max(matches, key=lambda det: det.decision_margin)


def detection_loop(camera_params):
    global latest_detection

    detector = Detector(
        families=APRILTAG_FAMILY,
        nthreads=APRILTAG_THREADS,
        quad_decimate=APRILTAG_QUAD_DECIMATE,
    )
    detection_count = 0
    detection_timer = time.perf_counter()
    detection_fps = 0.0

    while True:
        detection_frame_event.wait()
        detection_frame_event.clear()
        with lock:
            frame = (
                pending_detection_frame.copy()
                if pending_detection_frame is not None
                else None
            )
        if frame is None:
            continue

        try:
            detection = detect_target(detector, frame, camera_params)
            error = None
        except Exception as exc:
            detection = None
            error = str(exc)

        detection_count += 1
        now = time.perf_counter()
        elapsed = now - detection_timer
        if elapsed >= 1.0:
            detection_fps = detection_count / elapsed
            detection_count = 0
            detection_timer = now

        position_m = None
        decision_margin = None
        if detection is not None:
            pose_t = np.asarray(detection.pose_t, dtype=float).reshape(-1)
            position_m = [float(value) for value in pose_t[:3]]
            decision_margin = float(detection.decision_margin)

        with lock:
            latest_detection = detection
            tag_stats["detected"] = detection is not None
            tag_stats["position_m"] = position_m
            tag_stats["decision_margin"] = decision_margin
            tag_stats["detection_fps"] = detection_fps
            tag_stats["error"] = error


def draw_detection(frame, detection, position_m):
    if detection is None:
        return

    corners = np.round(detection.corners).astype(int)
    for index in range(4):
        cv2.line(
            frame,
            tuple(corners[index]),
            tuple(corners[(index + 1) % 4]),
            (0, 255, 0),
            2,
        )

    center = tuple(np.round(detection.center).astype(int))
    cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 22, 2)
    cv2.putText(
        frame,
        f"tag16h5 ID {TARGET_TAG_ID}",
        (corners[0][0], max(20, corners[0][1] - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        2,
    )
    if position_m is not None:
        cv2.putText(
            frame,
            f"t_cam [{position_m[0]:+.3f}, {position_m[1]:+.3f}, {position_m[2]:+.3f}] m",
            (10, FRAME_HEIGHT - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
        )


def capture_loop():
    global camera_intrinsics, latest_frame, pending_detection_frame

    pipeline = create_pipeline()
    with dai.Device(pipeline) as device:
        calibration = device.readCalibration()
        intrinsic_matrix = np.asarray(
            calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A,
                FRAME_WIDTH,
                FRAME_HEIGHT,
            ),
            dtype=float,
        )
        camera_params = (
            float(intrinsic_matrix[0, 0]),
            float(intrinsic_matrix[1, 1]),
            float(intrinsic_matrix[0, 2]),
            float(intrinsic_matrix[1, 2]),
        )
        with lock:
            camera_intrinsics = {
                "fx": camera_params[0],
                "fy": camera_params[1],
                "cx": camera_params[2],
                "cy": camera_params[3],
            }

        if Detector is not None:
            threading.Thread(
                target=detection_loop,
                args=(camera_params,),
                daemon=True,
            ).start()

        queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        frame_count = 0
        fps_timer = time.perf_counter()
        fps_display = 0.0

        while True:
            frame = queue.get().getCvFrame()

            frame_count += 1
            now = time.perf_counter()
            elapsed = now - fps_timer
            if elapsed >= 1.0:
                fps_display = frame_count / elapsed
                frame_count = 0
                fps_timer = now

            with lock:
                pending_detection_frame = frame.copy()
                detection = latest_detection
                position_m = tag_stats["position_m"]
            if Detector is not None:
                detection_frame_event.set()

            draw_detection(frame, detection, position_m)
            cv2.putText(
                frame,
                f"DepthAI RGB {fps_display:.1f} fps",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            with lock:
                latest_frame = frame
                capture_stats["fps"] = fps_display
                capture_stats["height"], capture_stats["width"] = frame.shape[:2]
            new_frame_event.set()


def generate_stream():
    placeholder = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cv2.putText(
        placeholder,
        "Waiting for DepthAI frame",
        (110, FRAME_HEIGHT // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (80, 80, 80),
        2,
    )

    while True:
        new_frame_event.wait(timeout=1.0)
        new_frame_event.clear()
        with lock:
            frame = latest_frame.copy() if latest_frame is not None else placeholder

        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
        )


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/feed")
def feed():
    return Response(generate_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stats")
def stats():
    with lock:
        return jsonify(
            {
                "stream": dict(capture_stats),
                "intrinsics": dict(camera_intrinsics) if camera_intrinsics else None,
                "tag": dict(tag_stats),
            }
        )


def main():
    parser = argparse.ArgumentParser(
        description="DepthAI RGB MJPEG stream with tag16h5 ID 1 pose."
    )
    parser.add_argument("--port", type=int, default=5003, help="HTTP port")
    args = parser.parse_args()

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    threading.Thread(target=capture_loop, daemon=True).start()

    print("Stream available at:")
    print(f"  http://localhost:{args.port}")
    print(f"  http://{local_ip}:{args.port}")
    print(
        f"Target: {APRILTAG_FAMILY} ID {TARGET_TAG_ID}, "
        f"black-square edge {TAG_SIZE_M * 1000:.0f} mm"
    )
    if Detector is None:
        print("AprilTag detector disabled: install pupil_apriltags")

    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
