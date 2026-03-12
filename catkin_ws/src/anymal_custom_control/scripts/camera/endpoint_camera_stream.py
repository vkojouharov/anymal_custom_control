#!/usr/bin/env python3
"""Stream local webcam as MJPEG over HTTP (no ROS required).

Opens /dev/video0 (or specified device) with OpenCV and serves it via Flask.
Open http://localhost:5000 in your browser.

Usage:
    python3 endpoint_camera_stream.py
    python3 endpoint_camera_stream.py --device 1 --port 8080
"""

import argparse
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# Shared state
lock = threading.Lock()
new_frame_event = threading.Event()
latest_frame = None
latest_jpeg = None
capture_fps = 0.0

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Local Camera</title>
    <style>
        body { background: #1a1a1a; color: #eee; font-family: monospace; text-align: center; margin: 20px; }
        h1 { color: #4CAF50; }
        img { max-width: 100%; border: 2px solid #333; border-radius: 4px; }
    </style>
</head>
<body>
    <h1>Local Camera Stream</h1>
    <img src="/feed" />
</body>
</html>
"""

GREEN = (0, 255, 0)
RED = (0, 0, 255)


def capture_loop(device):
    """Grab frames as fast as hardware allows."""
    global latest_frame, capture_fps

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"ERROR: Could not open /dev/video{device}")
        return

    cap.set(cv2.CAP_PROP_FPS, 30)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Opened /dev/video{device} ({w}x{h})")

    frame_count = 0
    fps_display = 0.0
    fps_timer = time.perf_counter()

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame_count += 1
        now = time.perf_counter()
        if now - fps_timer >= 1.0:
            fps_display = frame_count / (now - fps_timer)
            frame_count = 0
            fps_timer = now

        # Overlay capture FPS on frame, encode once
        cv2.putText(frame, f"{fps_display:.1f} fps capture", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

        with lock:
            latest_frame = frame
            capture_fps = fps_display
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

        # Overlay stream FPS (capture FPS already on frame)
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
    parser = argparse.ArgumentParser(description="Local webcam MJPEG web stream.")
    parser.add_argument('--device', type=int, default=0, help="Video device index (default: 0)")
    parser.add_argument('--port', type=int, default=5000, help="HTTP port (default: 5000)")
    args = parser.parse_args()

    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    # Start capture thread
    t = threading.Thread(target=capture_loop, args=(args.device,), daemon=True)
    t.start()

    print(f"Stream available at:")
    print(f"  http://localhost:{args.port}        (this machine only)")
    print(f"  http://{local_ip}:{args.port}  (use this from other devices)\n")

    app.run(host='0.0.0.0', port=args.port, threaded=True)


if __name__ == '__main__':
    main()
