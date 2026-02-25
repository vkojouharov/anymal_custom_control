#!/usr/bin/env python3
"""Discover available ANYmal ROS topics for cameras and control.

Run this when connected to the robot to find the actual topic names
you need for CameraReceiver and MovementController.

Usage:
    python3 discover_topics.py
    python3 discover_topics.py --help
"""

import argparse
import os
import signal
import sys

import rospy


def _force_exit(signum, frame):
    """Force exit even when rospy is blocked (e.g. ROS master unreachable)."""
    print("\nForce quit.")
    os._exit(0)


_sigint_count = 0


def _sigint_handler(signum, frame):
    global _sigint_count
    _sigint_count += 1
    if _sigint_count >= 2:
        _force_exit(signum, frame)
    print("\nShutting down... (press Ctrl+C again to force quit)")
    os._exit(0)


signal.signal(signal.SIGINT, _sigint_handler)


def discover():
    rospy.init_node('anymal_discovery', anonymous=True)

    try:
        master = rospy.get_master()
        code, msg, topics = master.getPublishedTopics('/')
    except Exception as e:
        print(f"ERROR: Cannot connect to ROS master at {rospy.get_master().target}")
        print(f"  {e}")
        print()
        print("Troubleshooting:")
        print("  - Is the ANYmal launcher running?")
        print("  - Can you ping anymal-d181-lpc?")
        print(f"  - ROS_MASTER_URI = {rospy.get_param('/run_id', 'N/A')}")
        sys.exit(1)

    # Categorize topics
    camera_topics = []
    control_topics = []
    state_topics = []
    other_topics = []

    camera_keywords = ['image_raw', 'camera_info', 'depth/image', 'color/image', 'infra']
    control_keywords = ['twist', 'pose', 'joy', 'anyjoy', 'cmd_vel']
    state_keywords = ['motion_control', 'joint_state', 'anymal_state', 'active_motion']

    for topic, msg_type in topics:
        if any(kw in topic.lower() for kw in camera_keywords):
            camera_topics.append((topic, msg_type))
        elif any(kw in topic.lower() for kw in control_keywords):
            control_topics.append((topic, msg_type))
        elif any(kw in topic.lower() for kw in state_keywords):
            state_topics.append((topic, msg_type))

    # Print results
    print("=" * 60)
    print("ANYmal Topic Discovery")
    print("=" * 60)

    print(f"\nROS Master: {rospy.get_master().target}")

    print(f"\n--- CAMERA TOPICS ({len(camera_topics)} found) ---")
    if camera_topics:
        for topic, msg_type in sorted(camera_topics):
            print(f"  {topic}")
            print(f"    type: {msg_type}")
    else:
        print("  (none found)")

    print(f"\n--- CONTROL TOPICS ({len(control_topics)} found) ---")
    if control_topics:
        for topic, msg_type in sorted(control_topics):
            print(f"  {topic}")
            print(f"    type: {msg_type}")
    else:
        print("  (none found)")

    print(f"\n--- STATE TOPICS ({len(state_topics)} found) ---")
    if state_topics:
        for topic, msg_type in sorted(state_topics):
            print(f"  {topic}")
            print(f"    type: {msg_type}")
    else:
        print("  (none found)")

    print(f"\nTotal topics: {len(topics)}")

    # Print usage hints
    print()
    print("=" * 60)
    print("NEXT STEPS")
    print("=" * 60)
    if camera_topics:
        color_topics = [t for t, _ in camera_topics if 'color/image_raw' in t]
        if color_topics:
            print(f"\nCamera example:")
            print(f"  cam = CameraReceiver('{color_topics[0]}')")
    if any('anyjoy' in t.lower() for t, _ in control_topics):
        print(f"\nMovement example:")
        print(f"  mc = MovementController()  # publishes to /anyjoy/operator")


def main():
    parser = argparse.ArgumentParser(
        description="Discover available ANYmal ROS topics for cameras and control.",
        epilog="Run this when connected to the robot to find topic names."
    )
    parser.parse_args()  # just for --help support
    discover()


if __name__ == '__main__':
    main()
