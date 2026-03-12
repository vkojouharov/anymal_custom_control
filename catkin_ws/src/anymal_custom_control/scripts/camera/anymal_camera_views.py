#!/usr/bin/env python3
"""Display front and rear wide-angle cameras side by side.

Uses compressed + small image variants to minimize bandwidth and CPU load.

Controls:
    Q / ESC  — quit

Prerequisites:
    - ANYmal software running
    - X11 forwarding enabled (run `xhost +local:root` on host)

Usage:
    python3 run_camera_views.py
    python3 run_camera_views.py --width 960 --fps 5
    python3 run_camera_views.py --raw   # use full-res raw images instead
"""

import argparse
import time

import cv2
import numpy as np
import rospy
from anymal_custom_control import CameraReceiver

# Default topics — small + compressed for low bandwidth
FRONT_TOPIC_COMPRESSED = '/wide_angle_camera_front/image_color_small/image_raw/compressed'
REAR_TOPIC_COMPRESSED = '/wide_angle_camera_rear/image_color_small/image_raw/compressed'

# Full-res raw fallback
FRONT_TOPIC_RAW = '/wide_angle_camera_front/image_raw'
REAR_TOPIC_RAW = '/wide_angle_camera_rear/image_raw'

WINDOW_NAME = 'ANYmal Camera Views'


def make_placeholder(width, height):
    """Create a dark placeholder frame with 'No Signal' text."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(frame, 'No Signal', (width // 4, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 80, 80), 2)
    return frame


def add_label(frame, text):
    """Add a camera name label to the top-left corner."""
    cv2.putText(frame, text, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame


def main():
    parser = argparse.ArgumentParser(description="ANYmal dual camera viewer.")
    parser.add_argument('--width', type=int, default=960,
                        help="Total display width in pixels (default: 960)")
    parser.add_argument('--fps', type=int, default=10,
                        help="Display refresh rate (default: 10)")
    parser.add_argument('--raw', action='store_true',
                        help="Use full-res raw topics instead of small compressed")
    parser.add_argument('--front', type=str, default=None,
                        help="Override front camera topic")
    parser.add_argument('--rear', type=str, default=None,
                        help="Override rear camera topic")
    args = parser.parse_args()

    if args.raw:
        front_topic = args.front or FRONT_TOPIC_RAW
        rear_topic = args.rear or REAR_TOPIC_RAW
        compressed = False
    else:
        front_topic = args.front or FRONT_TOPIC_COMPRESSED
        rear_topic = args.rear or REAR_TOPIC_COMPRESSED
        compressed = not args.front  # auto-detect: custom topics may not be compressed

    half_w = args.width // 2
    display_h = int(half_w * 3 / 4)

    rospy.init_node('anymal_camera_views', anonymous=True)

    front_cam = CameraReceiver(front_topic, compressed=compressed)
    rear_cam = CameraReceiver(rear_topic, compressed=compressed)

    mode = "compressed" if compressed else "raw"
    print(f"Subscribed to ({mode}):")
    print(f"  Front: {front_topic}")
    print(f"  Rear:  {rear_topic}")
    print(f"Display: {args.width}x{display_h} @ {args.fps} fps")
    print("Press 'q' or ESC to quit.\n")

    interval = 1.0 / args.fps
    aspect_set = False

    try:
        while not rospy.is_shutdown():
            t0 = time.time()

            front_frame = front_cam.get_frame()
            rear_frame = rear_cam.get_frame()

            # Set display height from actual aspect ratio on first frame
            if not aspect_set:
                sample = front_frame if front_frame is not None else rear_frame
                if sample is not None:
                    h, w = sample.shape[:2]
                    display_h = int(half_w * h / w)
                    aspect_set = True

            if front_frame is not None:
                front_display = cv2.resize(front_frame, (half_w, display_h))
            else:
                front_display = make_placeholder(half_w, display_h)

            if rear_frame is not None:
                rear_display = cv2.resize(rear_frame, (half_w, display_h))
            else:
                rear_display = make_placeholder(half_w, display_h)

            add_label(front_display, 'FRONT')
            add_label(rear_display, 'REAR')

            combined = cv2.hconcat([front_display, rear_display])
            cv2.imshow(WINDOW_NAME, combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break

            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        front_cam.shutdown()
        rear_cam.shutdown()
        cv2.destroyAllWindows()
        print("Camera viewer closed.")


if __name__ == '__main__':
    main()
