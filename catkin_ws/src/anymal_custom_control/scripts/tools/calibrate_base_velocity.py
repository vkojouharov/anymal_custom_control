#!/usr/bin/env python3
"""Interactive ANYmal base velocity calibration helper.

Prompts for normalized MovementController commands, applies them briefly, and
prints the measured legged-odometry displacement. Run one axis at a time for
the cleanest command-to-velocity scale estimates.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from anymal_custom_control import MovementController
from anymal_custom_control.egocentric_servo.constants import ODOM_TOPIC
from anymal_custom_control.egocentric_servo.messages import yaw_from_quaternion


@dataclass(frozen=True)
class OdomSample:
    stamp_sec: float
    wall_sec: float
    x: float
    y: float
    yaw: float


class OdomBuffer:
    def __init__(self, topic: str) -> None:
        self._lock = threading.Lock()
        self._latest: Optional[OdomSample] = None
        self._sub = rospy.Subscriber(topic, PoseWithCovarianceStamped, self._cb, queue_size=1, tcp_nodelay=True)

    def latest(self) -> Optional[OdomSample]:
        with self._lock:
            return self._latest

    def wait_for_fresh(self, timeout_sec: float, max_age_sec: float) -> OdomSample:
        deadline = time.time() + timeout_sec
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown() and time.time() < deadline:
            sample = self.latest()
            if sample is not None and time.time() - sample.wall_sec <= max_age_sec:
                return sample
            rate.sleep()
        raise RuntimeError(f"No fresh odometry on {self._sub.resolved_name} within {timeout_sec:.1f}s")

    def _cb(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        stamp_sec = float(msg.header.stamp.to_sec()) if msg.header.stamp else rospy.get_time()
        sample = OdomSample(
            stamp_sec=stamp_sec,
            wall_sec=time.time(),
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=float(
                yaw_from_quaternion(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            ),
        )
        with self._lock:
            self._latest = sample


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Apply normalized ANYmal base commands and measure legged-odometry displacement."
    )
    parser.add_argument("--duration-sec", type=float, default=3.0, help="Command duration per trial")
    parser.add_argument("--publish-rate-hz", type=float, default=20.0, help="AnyJoy publish rate")
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--odom-timeout-sec", type=float, default=5.0)
    parser.add_argument("--fresh-age-sec", type=float, default=0.5)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("anymal_base_velocity_calibration", anonymous=True)
    odom = OdomBuffer(args.odom_topic)
    movement = MovementController(rate_hz=max(2, int(args.publish_rate_hz)))

    print("ANYmal base velocity calibration")
    print("Command format: heading,lateral,turning")
    print("  heading: +forward, -backward")
    print("  lateral: +left, -right")
    print("  turning: +CCW/left, -CW/right")
    print("Values are normalized joy-manager commands, typically in [-1, 1].")
    print("Press Enter or type q to quit.\n")

    try:
        odom.wait_for_fresh(args.odom_timeout_sec, args.fresh_age_sec)
        movement.start()
        while not rospy.is_shutdown():
            line = input("velocity command heading,lateral,turning > ").strip()
            if line == "" or line.lower() in {"q", "quit", "exit"}:
                break

            try:
                heading, lateral, turning = _parse_command(line)
            except ValueError as exc:
                print(f"Invalid command: {exc}\n")
                continue

            print(
                f"Applying command h={heading:.3f}, l={lateral:.3f}, t={turning:.3f} "
                f"for {args.duration_sec:.2f}s..."
            )
            start = odom.wait_for_fresh(args.odom_timeout_sec, args.fresh_age_sec)
            start_wall = time.time()
            try:
                movement.set_velocity(heading=heading, lateral=lateral, turning=turning)
                while time.time() - start_wall < args.duration_sec and not rospy.is_shutdown():
                    time.sleep(0.02)
            finally:
                movement.stop()
                movement.publish_once()
                time.sleep(0.25)

            end = odom.wait_for_fresh(args.odom_timeout_sec, args.fresh_age_sec)
            _print_result(start, end, heading, lateral, turning)

    except (KeyboardInterrupt, EOFError):
        print("\nStopping.")
    finally:
        movement.stop()
        movement.publish_once()
        movement.shutdown()

    return 0


def _parse_command(line: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in line.split(",")]
    if len(parts) != 3:
        raise ValueError("expected three comma-separated numbers, e.g. 1,0,0")
    try:
        values = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise ValueError("all three fields must be numeric") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("all three fields must be finite")
    if not all(-1.0 <= value <= 1.0 for value in values):
        raise ValueError("all three commands must be in [-1, 1]")
    return values


def _print_result(start: OdomSample, end: OdomSample, heading: float, lateral: float, turning: float) -> None:
    dt = max(end.stamp_sec - start.stamp_sec, 1e-9)
    dx = end.x - start.x
    dy = end.y - start.y
    dyaw = _wrap_angle(end.yaw - start.yaw)

    c0 = math.cos(start.yaw)
    s0 = math.sin(start.yaw)
    forward_m = c0 * dx + s0 * dy
    left_m = -s0 * dx + c0 * dy

    print("Measured legged odometry:")
    print(f"  duration: {dt:.3f} s")
    print(f"  start odom: x={start.x:.4f} m, y={start.y:.4f} m, yaw={math.degrees(start.yaw):.2f} deg")
    print(f"  end odom:   x={end.x:.4f} m, y={end.y:.4f} m, yaw={math.degrees(end.yaw):.2f} deg")
    print(f"  odom delta: dx={dx:.4f} m, dy={dy:.4f} m, dyaw={dyaw:.4f} rad ({math.degrees(dyaw):.2f} deg)")
    print(f"  start-frame delta: forward={forward_m:.4f} m, left={left_m:.4f} m")
    print("Average rates:")
    print(f"  forward: {forward_m / dt:.4f} m/s from command {heading:.3f}")
    print(f"  lateral: {left_m / dt:.4f} m/s from command {lateral:.3f}")
    print(f"  turning: {dyaw / dt:.4f} rad/s from command {turning:.3f}")
    print()


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


if __name__ == "__main__":
    raise SystemExit(main())
