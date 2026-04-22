#!/usr/bin/env python3
"""Stream OAK RGB preview as MJPEG over HTTP.

No X11 or GUI required. Open the reported URL in a browser.

Usage:
    python3 depthai_rgb_stream.py
    python3 depthai_rgb_stream.py --port 8080
"""

import argparse
import socket
import threading
import time

import cv2
import depthai as dai
import numpy as np
from flask import Flask, Response, jsonify, render_template_string

app = Flask(__name__)

lock = threading.Lock()
new_frame_event = threading.Event()
latest_frame = None
capture_fps = 0.0
frame_width = 0
frame_height = 0

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>DepthAI RGB Stream</title>
    <style>
        body { background: #1a1a1a; color: #eee; font-family: monospace; text-align: center; margin: 20px; }
        h1 { color: #4CAF50; }
        img { max-width: 100%; border: 2px solid #333; border-radius: 4px; }
        .stats { margin-top: 14px; font-size: 18px; color: #cfcfcf; }
    </style>
</head>
<body>
    <h1>DepthAI RGB Stream</h1>
    <img src="/feed" />
    <div class="stats" id="stats">Waiting for frames...</div>
    <script>
        async function refreshStats() {
            try {
                const res = await fetch('/stats');
                const stats = await res.json();
                document.getElementById('stats').textContent =
                    `FPS: ${stats.fps.toFixed(1)} | Size: ${stats.width}x${stats.height}`;
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


def create_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    xout_rgb = pipeline.create(dai.node.XLinkOut)

    xout_rgb.setStreamName("rgb")
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(640, 360)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)
    cam_rgb.preview.link(xout_rgb.input)

    return pipeline


def capture_loop():
    global latest_frame, capture_fps, frame_width, frame_height

    pipeline = create_pipeline()
    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        frame_count = 0
        fps_timer = time.perf_counter()
        fps_display = 0.0

        while True:
            msg = queue.get()
            frame = msg.getCvFrame()

            frame_count += 1
            now = time.perf_counter()
            if now - fps_timer >= 1.0:
                fps_display = frame_count / (now - fps_timer)
                frame_count = 0
                fps_timer = now

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
                capture_fps = fps_display
                frame_height, frame_width = frame.shape[:2]
            new_frame_event.set()


def generate_stream():
    placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(
        placeholder,
        "Waiting for DepthAI frame",
        (110, 180),
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
                "fps": capture_fps,
                "width": frame_width,
                "height": frame_height,
            }
        )


def main():
    parser = argparse.ArgumentParser(description="DepthAI RGB MJPEG web stream.")
    parser.add_argument("--port", type=int, default=5002, help="HTTP port")
    args = parser.parse_args()

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    thread = threading.Thread(target=capture_loop, daemon=True)
    thread.start()

    print("Stream available at:")
    print(f"  http://localhost:{args.port}")
    print(f"  http://{local_ip}:{args.port}")

    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
