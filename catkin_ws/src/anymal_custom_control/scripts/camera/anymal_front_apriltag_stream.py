#!/usr/bin/env python3
"""Front ANYmal wide-angle camera stream with tag16h5 AprilTag overlays.

Uses the full-resolution front rectified compressed color stream by default:
    /wide_angle_camera_front/image_color_rect/compressed

Open the printed URL in a browser. This script does not open any local GUI.
"""

from __future__ import annotations

import argparse
import os
import socket
import threading
import time

import cv2
import numpy as np
import rospy
from flask import Flask, Response, jsonify, render_template_string
from anymal_custom_control import CameraReceiver

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None

DEFAULT_TOPIC = "/wide_angle_camera_front/image_color_rect/compressed"
RAW_TOPIC = "/wide_angle_camera_front/image_color_rect"
TAG_FAMILY = "tag16h5"
DEFAULT_PORT = 5006
DEFAULT_FPS = 8.0
DEFAULT_JPEG_QUALITY = 82
DEFAULT_TAG_DECIMATE = 2.0
DEFAULT_TAG_MARGIN = 35.0
PLACEHOLDER_WIDTH = 1440
PLACEHOLDER_HEIGHT = 1080

app = Flask(__name__)

camera = None
detector = None
detector_lock = threading.Lock()
target_fps = DEFAULT_FPS
jpeg_quality = DEFAULT_JPEG_QUALITY
max_width = 0
tag_margin = DEFAULT_TAG_MARGIN
stats_lock = threading.Lock()
stats_state = {
    "frames": 0,
    "stream_fps": 0.0,
    "detect_fps": 0.0,
    "detections": 0,
    "tag_ids": [],
    "width": 0,
    "height": 0,
    "topic": DEFAULT_TOPIC,
    "compressed": True,
}

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>ANYmal Front AprilTag Stream</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <style>
        body { background: #151515; color: #eee; font-family: monospace; margin: 18px; text-align: center; }
        h1 { margin: 0 0 14px; color: #4caf50; font-size: 24px; }
        img { width: 100%; max-width: 1440px; border: 2px solid #333; background: #080808; }
        .stats { margin-top: 12px; color: #cfcfcf; font-size: 16px; }
    </style>
</head>
<body>
    <h1>ANYmal Front Camera - tag16h5</h1>
    <img src="/feed" />
    <div class="stats" id="stats">Waiting for frames...</div>
    <script>
        async function refreshStats() {
            try {
                const stats = await fetch('/stats').then(r => r.json());
                document.getElementById('stats').textContent =
                    `stream ${stats.stream_fps.toFixed(1)} fps | detect ${stats.detect_fps.toFixed(1)} fps | ` +
                    `${stats.width}x${stats.height} | tags ${stats.detections} [${stats.tag_ids.join(',') || 'none'}]`;
            } catch (err) {
                document.getElementById('stats').textContent = 'Stats unavailable';
            }
        }
        refreshStats();
        setInterval(refreshStats, 500);
    </script>
</body>
</html>
"""


def make_placeholder() -> np.ndarray:
    frame = np.zeros((PLACEHOLDER_HEIGHT, PLACEHOLDER_WIDTH, 3), dtype=np.uint8)
    cv2.putText(
        frame,
        "Waiting for front camera frame",
        (260, PLACEHOLDER_HEIGHT // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.4,
        (90, 90, 90),
        3,
    )
    return frame


def resize_if_requested(frame: np.ndarray) -> np.ndarray:
    if max_width <= 0:
        return frame
    height, width = frame.shape[:2]
    if width <= max_width:
        return frame
    new_height = int(round(max_width * height / width))
    return cv2.resize(frame, (max_width, new_height), interpolation=cv2.INTER_AREA)


def detect_tags(frame: np.ndarray):
    if detector is None:
        return []
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    with detector_lock:
        detections = detector.detect(gray)
    return [det for det in detections if det.decision_margin >= tag_margin]


def draw_tags(frame: np.ndarray, detections) -> None:
    for det in detections:
        corners = np.asarray(det.corners, dtype=int)
        for idx in range(4):
            cv2.line(frame, tuple(corners[idx]), tuple(corners[(idx + 1) % 4]), (0, 255, 0), 3)

        cx, cy = int(det.center[0]), int(det.center[1])
        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)

        label = f"ID:{int(det.tag_id)} margin:{det.decision_margin:.0f}"
        x = int(corners[0][0])
        y = max(28, int(corners[0][1]) - 12)
        cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)


def annotate_frame(frame: np.ndarray) -> np.ndarray:
    detect_start = time.perf_counter()
    detections = detect_tags(frame)
    detect_elapsed = max(time.perf_counter() - detect_start, 1e-9)
    draw_tags(frame, detections)

    tag_ids = [int(det.tag_id) for det in detections]
    status = f"FRONT | {TAG_FAMILY}: {len(detections)} tags [{','.join(map(str, tag_ids)) or 'none'}]"
    cv2.putText(frame, status, (14, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    with stats_lock:
        stats_state["detect_fps"] = 1.0 / detect_elapsed
        stats_state["detections"] = len(detections)
        stats_state["tag_ids"] = tag_ids
        stats_state["height"], stats_state["width"] = frame.shape[:2]
    return frame


def latest_frame() -> np.ndarray:
    frame = camera.get_frame() if camera is not None else None
    if frame is None:
        return make_placeholder()
    frame = resize_if_requested(frame)
    return annotate_frame(frame)


def encode_frame(frame: np.ndarray) -> bytes:
    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    return jpeg.tobytes() if ok else b""


def generate_stream():
    interval = 1.0 / max(target_fps, 0.1)
    frame_count = 0
    fps_timer = time.perf_counter()
    while not rospy.is_shutdown():
        start = time.perf_counter()
        frame = latest_frame()
        jpeg = encode_frame(frame)
        if jpeg:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"

        frame_count += 1
        now = time.perf_counter()
        elapsed_for_fps = now - fps_timer
        if elapsed_for_fps >= 1.0:
            with stats_lock:
                stats_state["frames"] += frame_count
                stats_state["stream_fps"] = frame_count / elapsed_for_fps
            frame_count = 0
            fps_timer = now

        elapsed = time.perf_counter() - start
        if elapsed < interval:
            time.sleep(interval - elapsed)


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/feed")
def feed():
    return Response(generate_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stats")
def stats():
    with stats_lock:
        return jsonify(dict(stats_state))


def local_url(port: int) -> str:
    ros_ip = os.environ.get("ROS_IP")
    if ros_ip:
        return f"http://{ros_ip}:{port}"
    try:
        return f"http://{socket.gethostbyname(socket.gethostname())}:{port}"
    except OSError:
        return f"http://localhost:{port}"


def main() -> int:
    global camera, detector, target_fps, jpeg_quality, max_width, tag_margin

    parser = argparse.ArgumentParser(description="Front ANYmal camera stream with tag16h5 AprilTag overlays.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"HTTP port (default: {DEFAULT_PORT})")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help=f"Target MJPEG FPS (default: {DEFAULT_FPS:g})")
    parser.add_argument("--jpeg-quality", type=int, default=DEFAULT_JPEG_QUALITY, help="JPEG quality 1-100")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Front image topic")
    parser.add_argument("--raw", action="store_true", help=f"Use raw Image topic instead of compressed; default raw topic is {RAW_TOPIC}")
    parser.add_argument("--max-width", type=int, default=0, help="Optional output downscale width; 0 keeps full resolution")
    parser.add_argument("--tag-margin", type=float, default=DEFAULT_TAG_MARGIN, help="Minimum AprilTag decision margin")
    parser.add_argument("--tag-decimate", type=float, default=DEFAULT_TAG_DECIMATE, help="AprilTag quad_decimate value")
    parser.add_argument("--tag-threads", type=int, default=2, help="AprilTag detector threads")
    args = parser.parse_args()

    if Detector is None:
        print("ERROR: pupil_apriltags is not installed; cannot run AprilTag overlay.")
        return 2

    target_fps = float(args.fps)
    jpeg_quality = max(1, min(100, int(args.jpeg_quality)))
    max_width = max(0, int(args.max_width))
    tag_margin = float(args.tag_margin)
    topic = RAW_TOPIC if args.raw and args.topic == DEFAULT_TOPIC else args.topic
    compressed = not args.raw

    rospy.init_node("anymal_front_apriltag_stream", anonymous=True)
    camera = CameraReceiver(topic, compressed=compressed)
    detector = Detector(families=TAG_FAMILY, nthreads=args.tag_threads, quad_decimate=args.tag_decimate)

    with stats_lock:
        stats_state["topic"] = topic
        stats_state["compressed"] = compressed

    mode = "compressed" if compressed else "raw"
    print(f"Subscribed to front camera ({mode}): {topic}")
    print(f"AprilTag detector: {TAG_FAMILY}, margin>={tag_margin:g}, decimate={args.tag_decimate:g}, threads={args.tag_threads}")
    print(f"MJPEG: fps={target_fps:g}, jpeg_quality={jpeg_quality}, max_width={max_width or 'full'}")
    print("Open in browser:")
    print(f"  {local_url(args.port)}")
    print(f"  http://localhost:{args.port}  (from this machine/container host)\n")

    app.run(host="0.0.0.0", port=args.port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
