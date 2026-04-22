#!/usr/bin/env python3
"""Stream OAK RGB and RGB-aligned colorized depth as MJPEG over HTTP.

No X11 or GUI required. Open the reported URL in a browser.

Usage:
    python3 depthai_depth_stream.py
    python3 depthai_depth_stream.py --port 5003
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

FRAME_WIDTH = 640
FRAME_HEIGHT = 360
RGB_FPS = 30
DEPTH_MIN_MM = 10
DEPTH_MAX_MM = 1000

lock = threading.Lock()
new_frame_events = {
    "rgb": threading.Event(),
    "depth": threading.Event(),
}
latest_frames = {
    "rgb": None,
    "depth": None,
}
stream_stats = {
    "rgb": {"fps": 0.0, "width": 0, "height": 0},
    "depth": {"fps": 0.0, "width": 0, "height": 0},
}

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>DepthAI RGB + Depth Stream</title>
    <style>
        body {
            background: #161616;
            color: #f0f0f0;
            font-family: monospace;
            margin: 20px;
        }
        h1 {
            color: #4CAF50;
            text-align: center;
        }
        .layout {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(420px, 1fr));
            gap: 20px;
            align-items: start;
        }
        .panel {
            background: #202020;
            border: 1px solid #333;
            border-radius: 8px;
            padding: 14px;
            text-align: center;
        }
        .panel h2 {
            margin-top: 0;
            color: #d8d8d8;
        }
        .panel img {
            width: 100%;
            border: 2px solid #333;
            border-radius: 4px;
            background: #111;
        }
        .stats {
            margin-top: 12px;
            font-size: 18px;
            color: #cfcfcf;
        }
    </style>
</head>
<body>
    <h1>DepthAI RGB + Aligned Depth</h1>
    <div class="layout">
        <div class="panel">
            <h2>RGB</h2>
            <img src="/feed/rgb" />
            <div class="stats" id="stats-rgb">Waiting for frames...</div>
        </div>
        <div class="panel">
            <h2>Aligned Depth</h2>
            <img src="/feed/depth" />
            <div class="stats" id="stats-depth">Waiting for frames...</div>
        </div>
    </div>
    <script>
        async function refreshStats() {
            try {
                const res = await fetch('/stats');
                const stats = await res.json();
                document.getElementById('stats-rgb').textContent =
                    `FPS: ${stats.rgb.fps.toFixed(1)} | Size: ${stats.rgb.width}x${stats.rgb.height}`;
                document.getElementById('stats-depth').textContent =
                    `FPS: ${stats.depth.fps.toFixed(1)} | Size: ${stats.depth.width}x${stats.depth.height}`;
            } catch (err) {
                document.getElementById('stats-rgb').textContent = 'Stats unavailable';
                document.getElementById('stats-depth').textContent = 'Stats unavailable';
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
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)

    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")

    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(FRAME_WIDTH, FRAME_HEIGHT)
    cam_rgb.setPreviewKeepAspectRatio(False)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(RGB_FPS)

    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_left.setFps(RGB_FPS)
    mono_right.setFps(RGB_FPS)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(FRAME_WIDTH, FRAME_HEIGHT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.initialConfig.setConfidenceThreshold(220)
    config = stereo.initialConfig.get()
    config.postProcessing.speckleFilter.enable = True
    config.postProcessing.speckleFilter.speckleRange = 100
    config.postProcessing.temporalFilter.enable = True
    config.postProcessing.temporalFilter.alpha = 0.6
    config.postProcessing.temporalFilter.delta = 40
    config.postProcessing.spatialFilter.enable = True
    config.postProcessing.spatialFilter.holeFillingRadius = 4
    config.postProcessing.spatialFilter.numIterations = 2
    config.postProcessing.spatialFilter.alpha = 0.6
    config.postProcessing.spatialFilter.delta = 40
    config.postProcessing.thresholdFilter.minRange = DEPTH_MIN_MM
    config.postProcessing.thresholdFilter.maxRange = DEPTH_MAX_MM
    stereo.initialConfig.set(config)

    cam_rgb.preview.link(xout_rgb.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)

    return pipeline


def colorize_depth(depth_frame):
    clipped = np.clip(depth_frame, DEPTH_MIN_MM, DEPTH_MAX_MM)
    normalized = ((clipped - DEPTH_MIN_MM) * 255.0 / (DEPTH_MAX_MM - DEPTH_MIN_MM)).astype(np.uint8)
    normalized[depth_frame == 0] = 0
    colored = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_JET)
    colored[depth_frame == 0] = (0, 0, 0)
    return colored


def update_stream(kind, frame, fps):
    with lock:
        latest_frames[kind] = frame
        stream_stats[kind]["fps"] = fps
        stream_stats[kind]["height"], stream_stats[kind]["width"] = frame.shape[:2]
    new_frame_events[kind].set()


def capture_loop():
    pipeline = create_pipeline()
    with dai.Device(pipeline) as device:
        queues = {
            "rgb": device.getOutputQueue(name="rgb", maxSize=4, blocking=False),
            "depth": device.getOutputQueue(name="depth", maxSize=4, blocking=False),
        }
        frame_counts = {"rgb": 0, "depth": 0}
        fps_timers = {"rgb": time.perf_counter(), "depth": time.perf_counter()}
        fps_values = {"rgb": 0.0, "depth": 0.0}

        while True:
            rgb_msg = queues["rgb"].tryGet()
            if rgb_msg is not None:
                rgb_frame = rgb_msg.getCvFrame()
                frame_counts["rgb"] += 1
                now = time.perf_counter()
                elapsed = now - fps_timers["rgb"]
                if elapsed >= 1.0:
                    fps_values["rgb"] = frame_counts["rgb"] / elapsed
                    frame_counts["rgb"] = 0
                    fps_timers["rgb"] = now
                update_stream("rgb", rgb_frame, fps_values["rgb"])

            depth_msg = queues["depth"].tryGet()
            if depth_msg is not None:
                depth_frame = depth_msg.getFrame()
                depth_color = colorize_depth(depth_frame)
                frame_counts["depth"] += 1
                now = time.perf_counter()
                elapsed = now - fps_timers["depth"]
                if elapsed >= 1.0:
                    fps_values["depth"] = frame_counts["depth"] / elapsed
                    frame_counts["depth"] = 0
                    fps_timers["depth"] = now
                update_stream("depth", depth_color, fps_values["depth"])

            if rgb_msg is None and depth_msg is None:
                time.sleep(0.001)


def make_placeholder(label):
    frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cv2.putText(
        frame,
        label,
        (110, FRAME_HEIGHT // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (80, 80, 80),
        2,
    )
    return frame


def generate_stream(kind):
    placeholder = make_placeholder(f"Waiting for {kind} frame")

    while True:
        new_frame_events[kind].wait(timeout=1.0)
        new_frame_events[kind].clear()

        with lock:
            frame = latest_frames[kind].copy() if latest_frames[kind] is not None else placeholder

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


@app.route("/feed/<kind>")
def feed(kind):
    if kind not in latest_frames:
        return ("Unknown stream", 404)
    return Response(generate_stream(kind), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stats")
def stats():
    with lock:
        return jsonify(stream_stats)


def main():
    parser = argparse.ArgumentParser(description="DepthAI RGB and aligned depth MJPEG stream.")
    parser.add_argument("--port", type=int, default=5003, help="HTTP port")
    args = parser.parse_args()

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    thread = threading.Thread(target=capture_loop, daemon=True)
    thread.start()

    print("Stream available at:")
    print(f"  http://localhost:{args.port}")
    print(f"  http://{local_ip}:{args.port}")
    print(f"Depth range colorized over {DEPTH_MIN_MM}mm to {DEPTH_MAX_MM}mm")

    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
