#!/usr/bin/env python3
"""Stream local webcam with AprilTag detection overlaid (no ROS required).

Detects tag16h5 AprilTags and draws bounding box + crosshair on the stream.
Open http://localhost:5001 in your browser.

Usage:
    python3 endpoint_camera_apriltag_stream.py
    python3 endpoint_camera_apriltag_stream.py --device 0 --port 5001
"""

import argparse
import threading
import time

import cv2
import numpy as np
from pupil_apriltags import Detector
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# Shared state between capture/detect thread and Flask
lock = threading.Lock()
new_frame_event = threading.Event()
latest_frame = None
latest_detections = []
detect_fps = 0.0

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>AprilTag Stream</title>
    <style>
        body { background: #1a1a1a; color: #eee; font-family: monospace; text-align: center; margin: 20px; }
        h1 { color: #4CAF50; }
        img { max-width: 100%; border: 2px solid #333; border-radius: 4px; }
    </style>
</head>
<body>
    <h1>AprilTag Detection Stream</h1>
    <img src="/feed" />
</body>
</html>
"""

GREEN = (0, 255, 0)
RED = (0, 0, 255)


def draw_detections(frame, detections):
    """Draw bounding box and crosshair for each detected tag."""
    for det in detections:
        corners = det.corners.astype(int)

        # Draw bounding box
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), GREEN, 2)

        # Center crosshair
        cx, cy = int(det.center[0]), int(det.center[1])
        size = 15
        cv2.line(frame, (cx - size, cy), (cx + size, cy), GREEN, 2)
        cv2.line(frame, (cx, cy - size), (cx, cy + size), GREEN, 2)

        # Tag ID and center coords
        label = f"ID:{det.tag_id} ({cx},{cy}) m:{det.decision_margin:.0f}"
        cv2.putText(frame, label, (corners[0][0], corners[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 2)

    return frame


def capture_loop(device):
    """Capture frames, run detection, store results."""
    global latest_frame, latest_detections, detect_fps

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"ERROR: Could not open /dev/video{device}")
        return

    cap.set(cv2.CAP_PROP_FPS, 30)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Opened /dev/video{device} ({w}x{h})")

    detector = Detector(families="tag16h5", nthreads=2, quad_decimate=2.0)

    frame_count = 0
    fps_display = 0.0
    fps_timer = time.perf_counter()

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = [d for d in detector.detect(gray) if d.decision_margin > 90]

        # Update FPS every second
        frame_count += 1
        now = time.perf_counter()
        if now - fps_timer >= 1.0:
            fps_display = frame_count / (now - fps_timer)
            frame_count = 0
            fps_timer = now

        annotated = draw_detections(frame, detections)

        # Overlay capture+detect FPS
        cv2.putText(annotated, f"{fps_display:.1f} fps capture+detect", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

        with lock:
            latest_frame = annotated
            latest_detections = detections
            detect_fps = fps_display
        new_frame_event.set()


def generate_stream():
    """Yield MJPEG frames for Flask."""
    placeholder = np.zeros((360, 480, 3), dtype=np.uint8)
    cv2.putText(placeholder, 'No Signal', (120, 180),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (80, 80, 80), 2)

    frame_count = 0
    stream_fps = 0.0
    fps_timer = time.perf_counter()

    while True:
        new_frame_event.wait()
        new_frame_event.clear()

        with lock:
            frame = latest_frame.copy() if latest_frame is not None else placeholder

        # Overlay stream FPS (capture+detect FPS already on frame)
        cv2.putText(frame, f"{stream_fps:.1f} fps stream", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)

        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

        frame_count += 1
        now = time.perf_counter()
        if now - fps_timer >= 1.0:
            stream_fps = frame_count / (now - fps_timer)
            frame_count = 0
            fps_timer = now


@app.route('/')
def index():
    return render_template_string(HTML_PAGE)


@app.route('/feed')
def feed():
    return Response(generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    parser = argparse.ArgumentParser(description="Webcam + AprilTag detection MJPEG stream.")
    parser.add_argument('--device', type=int, default=0, help="Video device index (default: 0)")
    parser.add_argument('--port', type=int, default=5001, help="HTTP port (default: 5001)")
    args = parser.parse_args()

    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    # Start capture+detect thread
    t = threading.Thread(target=capture_loop, args=(args.device,), daemon=True)
    t.start()

    print(f"AprilTag detector: tag16h5, quad_decimate=2.0, nthreads=2")
    print(f"Stream available at:")
    print(f"  http://localhost:{args.port}        (this machine only)")
    print(f"  http://{local_ip}:{args.port}  (use this from other devices)\n")

    app.run(host='0.0.0.0', port=args.port, threaded=True)


if __name__ == '__main__':
    main()
