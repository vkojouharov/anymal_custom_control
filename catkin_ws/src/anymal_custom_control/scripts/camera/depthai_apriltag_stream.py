#!/usr/bin/env python3
"""Stream OAK RGB and RGB-aligned colorized depth with AprilTag overlays over HTTP.

No X11 or GUI required. Open the reported URL in a browser.

Usage:
    python3 depthai_apriltag_stream.py
    python3 depthai_apriltag_stream.py --port 5004
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
DEPTH_MIN_MM = 10
DEPTH_MAX_MM = 1000
APRILTAG_FAMILY = "tag16h5"
APRILTAG_DECISION_MARGIN = 50
APRILTAG_QUAD_DECIMATE = 1.0
APRILTAG_THREADS = 2
DEFAULT_TAG_SIZE_M = 0.0956
TAG_SIZE_M = DEFAULT_TAG_SIZE_M

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
apriltag_stats = {
    "enabled": Detector is not None,
    "fps": 0.0,
    "detections": 0,
    "rgb_summary": "No detections",
    "depth_summary": "No detections",
}

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>DepthAI RGB + Depth + AprilTag Stream</title>
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
    <h1>DepthAI RGB + Aligned Depth + AprilTag</h1>
    <div class="panel">
        <div class="stats" id="stats-apriltag">Waiting for AprilTag stats...</div>
    </div>
    <div class="layout">
        <div class="panel">
            <h2>RGB</h2>
            <img src="/feed/rgb" />
            <div class="stats" id="stats-rgb">Waiting for frames...</div>
            <div class="stats" id="stats-rgb-depth">Waiting for AprilTag pose depth...</div>
        </div>
        <div class="panel">
            <h2>Aligned Depth</h2>
            <img src="/feed/depth" />
            <div class="stats" id="stats-depth">Waiting for frames...</div>
            <div class="stats" id="stats-depth-region">Waiting for masked depth stats...</div>
        </div>
    </div>
    <script>
        async function refreshStats() {
            try {
                const res = await fetch('/stats');
                const stats = await res.json();
                document.getElementById('stats-apriltag').textContent = stats.apriltag.enabled
                    ? `AprilTag ${stats.apriltag.detections} tags | Detect FPS: ${stats.apriltag.fps.toFixed(1)}`
                    : 'AprilTag detector unavailable (install pupil_apriltags)';
                document.getElementById('stats-rgb').textContent =
                    `FPS: ${stats.rgb.fps.toFixed(1)} | Size: ${stats.rgb.width}x${stats.rgb.height}`;
                document.getElementById('stats-depth').textContent =
                    `FPS: ${stats.depth.fps.toFixed(1)} | Size: ${stats.depth.width}x${stats.depth.height}`;
                document.getElementById('stats-rgb-depth').textContent =
                    `RGB pose depth: ${stats.apriltag.rgb_summary}`;
                document.getElementById('stats-depth-region').textContent =
                    `Depth mask: ${stats.apriltag.depth_summary}`;
            } catch (err) {
                document.getElementById('stats-apriltag').textContent = 'Stats unavailable';
                document.getElementById('stats-rgb').textContent = 'Stats unavailable';
                document.getElementById('stats-depth').textContent = 'Stats unavailable';
                document.getElementById('stats-rgb-depth').textContent = 'Stats unavailable';
                document.getElementById('stats-depth-region').textContent = 'Stats unavailable';
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


def detect_apriltags(detector, frame):
    if detector is None:
        return []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if TAG_SIZE_M is not None:
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_M,
        )
    else:
        detections = detector.detect(gray)
    return [det for det in detections if det.decision_margin > APRILTAG_DECISION_MARGIN]


def compute_tag_corner_depths(det):
    if TAG_SIZE_M is None:
        return None

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


def compute_masked_depth_mm(depth_frame, det):
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


def format_rgb_summary(detections):
    if not detections:
        return "No detections"
    if TAG_SIZE_M is None:
        return "Set --tag-size-m to enable metric pose depth"

    parts = []
    for det in detections:
        corner_depths = compute_tag_corner_depths(det)
        if corner_depths is None:
            parts.append(f"ID{det.tag_id}: unavailable")
            continue
        avg_z_m = float(np.mean(corner_depths))
        parts.append(f"ID{det.tag_id}: avg Z {avg_z_m:.3f} m")
    return " | ".join(parts)


def format_depth_summary(depth_frame, detections):
    if not detections:
        return "No detections"

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


def draw_apriltags(frame, detections):
    for det in detections:
        corners = det.corners.astype(int)

        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)

        cx, cy = int(det.center[0]), int(det.center[1])
        size = 15
        cv2.line(frame, (cx - size, cy), (cx + size, cy), (0, 255, 0), 2)
        cv2.line(frame, (cx, cy - size), (cx, cy + size), (0, 255, 0), 2)

        label = f"ID:{det.tag_id} ({cx},{cy}) m:{det.decision_margin:.0f}"
        cv2.putText(
            frame,
            label,
            (corners[0][0], corners[0][1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )


def draw_apriltag_status(frame, detections, detect_fps):
    if Detector is None:
        status = "AprilTag detector unavailable"
        color = (0, 0, 255)
    else:
        status = f"AprilTag {APRILTAG_FAMILY} {len(detections)} tags {detect_fps:.1f} fps"
        color = (255, 255, 255)

    cv2.putText(
        frame,
        status,
        (10, frame.shape[0] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        2,
    )


def update_stream(kind, frame, fps):
    with lock:
        latest_frames[kind] = frame
        stream_stats[kind]["fps"] = fps
        stream_stats[kind]["height"], stream_stats[kind]["width"] = frame.shape[:2]
    new_frame_events[kind].set()


def capture_loop():
    detector = None
    if Detector is not None:
        detector = Detector(
            families=APRILTAG_FAMILY,
            nthreads=APRILTAG_THREADS,
            quad_decimate=APRILTAG_QUAD_DECIMATE,
        )

    pipeline = create_pipeline()
    with dai.Device(pipeline) as device:
        calib = device.readFactoryCalibration()
        intrinsics = np.array(
            calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A,
                FRAME_WIDTH,
                FRAME_HEIGHT,
            ),
            dtype=np.float64,
        )
        camera_params = (
            float(intrinsics[0, 0]),
            float(intrinsics[1, 1]),
            float(intrinsics[0, 2]),
            float(intrinsics[1, 2]),
        )
        global CAMERA_PARAMS
        CAMERA_PARAMS = camera_params
        queues = {
            "rgb": device.getOutputQueue(name="rgb", maxSize=4, blocking=False),
            "depth": device.getOutputQueue(name="depth", maxSize=4, blocking=False),
        }
        frame_counts = {"rgb": 0, "depth": 0}
        fps_timers = {"rgb": time.perf_counter(), "depth": time.perf_counter()}
        fps_values = {"rgb": 0.0, "depth": 0.0}
        detect_count = 0
        detect_timer = time.perf_counter()
        detect_fps = 0.0
        latest_detections = []

        while True:
            rgb_msg = queues["rgb"].tryGet()
            if rgb_msg is not None:
                rgb_frame = rgb_msg.getCvFrame()
                latest_detections = detect_apriltags(detector, rgb_frame)
                draw_apriltags(rgb_frame, latest_detections)
                detect_count += 1
                now = time.perf_counter()
                detect_elapsed = now - detect_timer
                if detect_elapsed >= 1.0:
                    detect_fps = detect_count / detect_elapsed
                    detect_count = 0
                    detect_timer = now
                frame_counts["rgb"] += 1
                elapsed = now - fps_timers["rgb"]
                if elapsed >= 1.0:
                    fps_values["rgb"] = frame_counts["rgb"] / elapsed
                    frame_counts["rgb"] = 0
                    fps_timers["rgb"] = now
                draw_apriltag_status(rgb_frame, latest_detections, detect_fps)
                with lock:
                    apriltag_stats["fps"] = detect_fps
                    apriltag_stats["detections"] = len(latest_detections)
                    apriltag_stats["rgb_summary"] = format_rgb_summary(latest_detections)
                update_stream("rgb", rgb_frame, fps_values["rgb"])

            depth_msg = queues["depth"].tryGet()
            if depth_msg is not None:
                depth_frame = depth_msg.getFrame()
                depth_color = colorize_depth(depth_frame)
                draw_apriltags(depth_color, latest_detections)
                frame_counts["depth"] += 1
                now = time.perf_counter()
                elapsed = now - fps_timers["depth"]
                if elapsed >= 1.0:
                    fps_values["depth"] = frame_counts["depth"] / elapsed
                    frame_counts["depth"] = 0
                    fps_timers["depth"] = now
                draw_apriltag_status(depth_color, latest_detections, detect_fps)
                with lock:
                    apriltag_stats["depth_summary"] = format_depth_summary(depth_frame, latest_detections)
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
        return jsonify(
            {
                "rgb": stream_stats["rgb"],
                "depth": stream_stats["depth"],
                "apriltag": apriltag_stats,
            }
        )


def main():
    parser = argparse.ArgumentParser(description="DepthAI RGB, aligned depth, and AprilTag MJPEG stream.")
    parser.add_argument("--port", type=int, default=5004, help="HTTP port")
    parser.add_argument(
        "--tag-size-m",
        type=float,
        default=DEFAULT_TAG_SIZE_M,
        help="Physical AprilTag edge length in meters for RGB pose depth estimation",
    )
    args = parser.parse_args()
    global TAG_SIZE_M
    TAG_SIZE_M = args.tag_size_m

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    thread = threading.Thread(target=capture_loop, daemon=True)
    thread.start()

    print("Stream available at:")
    print(f"  http://localhost:{args.port}")
    print(f"  http://{local_ip}:{args.port}")
    print(f"Depth range colorized over {DEPTH_MIN_MM}mm to {DEPTH_MAX_MM}mm")
    if Detector is not None:
        print(
            f"AprilTag detector enabled: {APRILTAG_FAMILY}, "
            f"quad_decimate={APRILTAG_QUAD_DECIMATE}, nthreads={APRILTAG_THREADS}, "
            f"margin>{APRILTAG_DECISION_MARGIN}"
        )
        print(f"RGB pose depth enabled with tag size {TAG_SIZE_M:.4f} m")
    else:
        print("AprilTag detector disabled: install pupil_apriltags")

    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
