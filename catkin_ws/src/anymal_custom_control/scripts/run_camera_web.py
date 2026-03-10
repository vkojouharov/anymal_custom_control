#!/usr/bin/env python3
"""Stream ANYmal cameras as MJPEG over HTTP (no X11/GUI required).

Serves front and rear wide-angle camera feeds as a web page using Flask.
Connect to the ANYmal WiFi and open http://10.32.78.170:5000 in your browser.

Endpoints:
    /              — web page with both camera streams
    /feed/front    — front camera MJPEG stream
    /feed/rear     — rear camera MJPEG stream
    /feed/combined — side-by-side MJPEG stream

Prerequisites:
    - ANYmal software running
    - Flask installed in the Docker container: pip install flask
    - Docker container using --net=host (or -p 5000:5000)

Usage:
    python3 run_camera_web.py
    python3 run_camera_web.py --port 8080 --fps 10
    python3 run_camera_web.py --raw   # use full-res raw images instead of compressed
"""

import argparse
import threading
import time

import cv2
import numpy as np
import rospy
from flask import Flask, Response, render_template_string
from anymal_custom_control import CameraReceiver

FRONT_TOPIC_COMPRESSED = '/wide_angle_camera_front/image_color_small/image_raw/compressed'
REAR_TOPIC_COMPRESSED = '/wide_angle_camera_rear/image_color_small/image_raw/compressed'
FRONT_TOPIC_RAW = '/wide_angle_camera_front/image_raw'
REAR_TOPIC_RAW = '/wide_angle_camera_rear/image_raw'

app = Flask(__name__)

# Globals set in main()
front_cam = None
rear_cam = None
target_fps = 10
display_width = 960

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>ANYmal Camera</title>
    <style>
        body { background: #1a1a1a; color: #eee; font-family: monospace; text-align: center; margin: 20px; }
        h1 { color: #4CAF50; }
        img { max-width: 100%; border: 2px solid #333; border-radius: 4px; }
        .streams { display: flex; justify-content: center; gap: 10px; flex-wrap: wrap; }
        .stream-box { flex: 1; min-width: 300px; max-width: 640px; }
        .stream-box h3 { margin-bottom: 5px; }
    </style>
</head>
<body>
    <h1>ANYmal Camera Streams</h1>
    <div class="streams">
        <div class="stream-box">
            <h3>FRONT</h3>
            <img src="/feed/front" />
        </div>
        <div class="stream-box">
            <h3>REAR</h3>
            <img src="/feed/rear" />
        </div>
    </div>
    <p>Combined: <a href="/feed/combined">/feed/combined</a></p>
</body>
</html>
"""


def make_placeholder(width, height, text='No Signal'):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(frame, text, (width // 4, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 80, 80), 2)
    return frame


def encode_frame(frame):
    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return jpeg.tobytes()


def generate_stream(get_frame_fn):
    interval = 1.0 / target_fps
    while True:
        t0 = time.time()
        frame = get_frame_fn()
        jpeg = encode_frame(frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
        elapsed = time.time() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)


def get_front_frame():
    frame = front_cam.get_frame()
    if frame is None:
        return make_placeholder(480, 360, 'Front — No Signal')
    return frame


def get_rear_frame():
    frame = rear_cam.get_frame()
    if frame is None:
        return make_placeholder(480, 360, 'Rear — No Signal')
    return frame


def get_combined_frame():
    half_w = display_width // 2
    front = front_cam.get_frame()
    rear = rear_cam.get_frame()

    sample = front if front is not None else rear
    if sample is not None:
        h, w = sample.shape[:2]
        display_h = int(half_w * h / w)
    else:
        display_h = int(half_w * 3 / 4)

    if front is not None:
        front = cv2.resize(front, (half_w, display_h))
    else:
        front = make_placeholder(half_w, display_h, 'Front — No Signal')

    if rear is not None:
        rear = cv2.resize(rear, (half_w, display_h))
    else:
        rear = make_placeholder(half_w, display_h, 'Rear — No Signal')

    cv2.putText(front, 'FRONT', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(rear, 'REAR', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return cv2.hconcat([front, rear])


@app.route('/')
def index():
    return render_template_string(HTML_PAGE)


@app.route('/feed/front')
def feed_front():
    return Response(generate_stream(get_front_frame),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/feed/rear')
def feed_rear():
    return Response(generate_stream(get_rear_frame),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/feed/combined')
def feed_combined():
    return Response(generate_stream(get_combined_frame),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    global front_cam, rear_cam, target_fps, display_width

    parser = argparse.ArgumentParser(description="ANYmal camera MJPEG web stream.")
    parser.add_argument('--port', type=int, default=5000, help="HTTP port (default: 5000)")
    parser.add_argument('--fps', type=int, default=10, help="Target FPS (default: 10)")
    parser.add_argument('--width', type=int, default=960, help="Combined view width (default: 960)")
    parser.add_argument('--raw', action='store_true', help="Use raw topics instead of compressed")
    args = parser.parse_args()

    target_fps = args.fps
    display_width = args.width

    if args.raw:
        front_topic, rear_topic, compressed = FRONT_TOPIC_RAW, REAR_TOPIC_RAW, False
    else:
        front_topic, rear_topic, compressed = FRONT_TOPIC_COMPRESSED, REAR_TOPIC_COMPRESSED, True

    rospy.init_node('anymal_camera_web', anonymous=True)
    front_cam = CameraReceiver(front_topic, compressed=compressed)
    rear_cam = CameraReceiver(rear_topic, compressed=compressed)

    mode = "compressed" if compressed else "raw"
    print(f"Subscribed to ({mode}):")
    print(f"  Front: {front_topic}")
    print(f"  Rear:  {rear_topic}")
    print(f"\nOpen http://0.0.0.0:{args.port} in your browser\n")

    app.run(host='0.0.0.0', port=args.port, threaded=True)


if __name__ == '__main__':
    main()
