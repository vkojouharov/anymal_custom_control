#!/usr/bin/env python3
"""Monitor ANYmal ROS topics in real-time (read-only, safe).

Subscribes to key topics and prints live data:
- Joystick input (/anyjoy/operator)
- Twist output (velocity commands)
- Motion controller state
- Camera frame info

Usage:
    python3 monitor.py                    # monitor all available topics
    python3 monitor.py --topic /anyjoy/operator  # monitor specific topic
    python3 monitor.py --help
"""

import argparse
import os
import signal
import sys
import threading
import time

import rospy
from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import TwistStamped


def _force_exit(signum, frame):
    """Force exit even when rospy is blocked (e.g. ROS master unreachable)."""
    print("\nForce quit.")
    os._exit(0)


# First Ctrl+C tries graceful shutdown via rospy.
# Second Ctrl+C force-kills (handles case where rospy blocks on lost master).
_sigint_count = 0


def _sigint_handler(signum, frame):
    global _sigint_count
    _sigint_count += 1
    if _sigint_count >= 2:
        _force_exit(signum, frame)
    rospy.signal_shutdown("user interrupt")
    print("\nShutting down... (press Ctrl+C again to force quit)")


signal.signal(signal.SIGINT, _sigint_handler)


class TopicMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._stats = {}

    def _update_stats(self, topic, info):
        with self._lock:
            if topic not in self._stats:
                self._stats[topic] = {'count': 0, 'first_time': time.time()}
            self._stats[topic]['count'] += 1
            self._stats[topic]['last_info'] = info
            self._stats[topic]['last_time'] = time.time()

    def get_stats(self):
        with self._lock:
            return dict(self._stats)


def main():
    parser = argparse.ArgumentParser(
        description="Monitor ANYmal ROS topics in real-time (read-only, safe).",
        epilog="Press Ctrl+C to stop. This script only reads topics, never publishes."
    )
    parser.add_argument('--topic', type=str, default=None,
                        help="Monitor a specific topic (default: auto-discover)")
    parser.add_argument('--rate', type=float, default=1.0,
                        help="Print update rate in Hz (default: 1)")
    args = parser.parse_args()

    rospy.init_node('anymal_monitor', anonymous=True)
    monitor = TopicMonitor()

    # Discover topics
    try:
        master = rospy.get_master()
        code, msg, topics = master.getPublishedTopics('/')
    except Exception as e:
        print(f"ERROR: Cannot connect to ROS master")
        print(f"  {e}")
        sys.exit(1)

    topic_dict = {t: mt for t, mt in topics}
    subscribers = []

    if args.topic:
        # Monitor specific topic
        if args.topic not in topic_dict:
            print(f"WARNING: Topic '{args.topic}' not found. Available topics:")
            for t in sorted(topic_dict.keys()):
                print(f"  {t}")
            print(f"\nSubscribing anyway (topic may appear later)...")
        _subscribe_generic(args.topic, monitor, subscribers)
    else:
        # Auto-discover and subscribe to key topics
        print("Auto-discovering topics...\n")

        # Joystick
        for t, mt in topics:
            if 'anyjoy' in t.lower():
                _subscribe_generic(t, monitor, subscribers)

        # Twist
        for t, mt in topics:
            if 'twist' in t.lower() and 'TwistStamped' in mt:
                _subscribe_twist(t, monitor, subscribers)

        # Motion controller
        for t, mt in topics:
            if 'active_motion' in t.lower():
                _subscribe_generic(t, monitor, subscribers)

        # Camera (just the first color topic for monitoring)
        camera_found = False
        for t, mt in topics:
            if 'color/image_raw' in t and not camera_found:
                _subscribe_image(t, monitor, subscribers)
                camera_found = True

        if not subscribers:
            print("No relevant topics found. Is the ANYmal software running?")
            print("Available topics:")
            for t in sorted(topic_dict.keys())[:20]:
                print(f"  {t}")
            sys.exit(1)

    print(f"Monitoring {len(subscribers)} topic(s). Press Ctrl+C to stop.\n")

    # Print loop — uses time.sleep instead of rospy.Rate to avoid blocking on lost master
    interval = 1.0 / args.rate
    try:
        while not rospy.is_shutdown():
            stats = monitor.get_stats()
            print("\033[2J\033[H", end="")  # clear screen
            print("=" * 70)
            print("ANYmal Topic Monitor (read-only)")
            print("=" * 70)

            for topic in sorted(stats.keys()):
                s = stats[topic]
                elapsed = s['last_time'] - s['first_time'] if s['count'] > 1 else 0
                hz = s['count'] / elapsed if elapsed > 0 else 0
                print(f"\n{topic}  ({hz:.1f} Hz, {s['count']} msgs)")
                print(f"  {s.get('last_info', '')}")

            print(f"\nPress Ctrl+C to stop")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")


def _subscribe_generic(topic, monitor, subscribers):
    from rospy import AnyMsg

    def cb(msg, t=topic):
        monitor._update_stats(t, f"received ({type(msg).__name__})")

    sub = rospy.Subscriber(topic, AnyMsg, cb, queue_size=1)
    subscribers.append(sub)
    print(f"  Subscribed: {topic}")


def _subscribe_twist(topic, monitor, subscribers):
    def cb(msg, t=topic):
        info = (f"linear=({msg.twist.linear.x:.3f}, {msg.twist.linear.y:.3f}, {msg.twist.linear.z:.3f}) "
                f"angular=({msg.twist.angular.x:.3f}, {msg.twist.angular.y:.3f}, {msg.twist.angular.z:.3f})")
        monitor._update_stats(t, info)

    sub = rospy.Subscriber(topic, TwistStamped, cb, queue_size=1)
    subscribers.append(sub)
    print(f"  Subscribed: {topic} (TwistStamped)")


def _subscribe_image(topic, monitor, subscribers):
    def cb(msg, t=topic):
        info = f"{msg.width}x{msg.height} {msg.encoding}"
        monitor._update_stats(t, info)

    sub = rospy.Subscriber(topic, Image, cb, queue_size=1)
    subscribers.append(sub)
    print(f"  Subscribed: {topic} (Image)")


if __name__ == '__main__':
    main()
