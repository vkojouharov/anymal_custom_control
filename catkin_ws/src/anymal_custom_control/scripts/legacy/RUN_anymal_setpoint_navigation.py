#!/usr/bin/env python3
"""Autonomous setpoint navigation for ANYmal with joystick override.

Navigates to user-specified (dx, dy) displacement targets using PD control
on velocity commands, with legged odometry position feedback.

Joystick controls (always active):
    Y button  — WALK mode
    B button  — STAND mode (pauses navigation)
    A button  — REST mode  (pauses navigation)
    X button  — EMERGENCY STOP + quit

Workflow:
    1. Press Y to enter WALK mode — navigation loop begins
    2. Enter desired X,Y displacement (meters) in terminal, e.g. "0.5,0.8"
    3. Press Enter again to confirm
    4. Robot navigates autonomously to the setpoint
    5. On arrival (<10cm error), prompts for next displacement
    6. Switching to STAND/REST pauses navigation; switching back to WALK resumes prompts
    7. X button stops immediately and exits

Prerequisites:
    - ANYmal software running in WALK-ready state
    - Xbox controller plugged in
    - E-stop ready

Usage:
    python3 RUN_anymal_setpoint_navigation.py
    python3 RUN_anymal_setpoint_navigation.py --kp 1.0 --kd 0.3 --vmax 0.4
"""

import argparse
import math
import os
import signal
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from anymal_custom_control import ModeController, MovementController
from anymal_custom_control.joystick_driver import (
    joystick_connect,
    joystick_disconnect,
    joystick_read,
    joystick_rumble,
)


# ── Signal handling ──────────────────────────────────────────────────────────

_sigint_count = 0


def _sigint_handler(signum, frame):
    global _sigint_count
    _sigint_count += 1
    if _sigint_count >= 2:
        print("\nForce quit.")
        os._exit(0)
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, _sigint_handler)


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw angle from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ANYmal setpoint navigation with joystick override.")
    parser.add_argument('--kp', type=float, default=1.0,
                        help="Proportional gain (default: 1.0)")
    parser.add_argument('--kd', type=float, default=0.3,
                        help="Derivative gain (default: 0.3)")
    parser.add_argument('--kp_yaw', type=float, default=1.5,
                        help="Yaw proportional gain (default: 1.5)")
    parser.add_argument('--kd_yaw', type=float, default=0.3,
                        help="Yaw derivative gain (default: 0.3)")
    parser.add_argument('--vmax', type=float, default=0.4,
                        help="Max velocity command fraction 0-1 (default: 0.4)")
    parser.add_argument('--vmax_yaw', type=float, default=0.3,
                        help="Max yaw velocity command fraction 0-1 (default: 0.3)")
    parser.add_argument('--tolerance', type=float, default=0.01,
                        help="Arrival tolerance in meters (default: 0.01)")
    parser.add_argument('--yaw_tolerance', type=float, default=0.05,
                        help="Arrival yaw tolerance in radians (default: 0.05)")
    parser.add_argument('--rate', type=int, default=20,
                        help="Control loop rate Hz (default: 20)")
    args = parser.parse_args()

    # ── Connect joystick ─────────────────────────────────────────────────
    js = joystick_connect()
    print(f"Controller connected: {js['device'].name}")

    # ── ROS setup ────────────────────────────────────────────────────────
    rospy.init_node('anymal_setpoint_nav', anonymous=True)

    mc = MovementController()
    mc.start()
    modes = ModeController(movement_controller=mc)

    # Shared state for pose (written by ROS callback, read by control loop)
    pose_lock = threading.Lock()
    pose_state = {'x': None, 'y': None, 'yaw': None}

    def pose_cb(msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        with pose_lock:
            pose_state['x'] = p.x
            pose_state['y'] = p.y
            pose_state['yaw'] = yaw

    rospy.Subscriber('/legged_odometry/pose_in_odom', PoseWithCovarianceStamped, pose_cb)

    # ── Shared flags (joystick thread → main thread) ─────────────────────
    emergency_stop = threading.Event()
    mode_changed = threading.Event()

    # Mode button mapping
    MODE_BUTTONS = {
        'AB': ModeController.REST,
        'BB': ModeController.STAND,
        'YB': ModeController.WALK,
    }

    # ── Joystick thread ──────────────────────────────────────────────────
    def joystick_loop():
        prev_buttons = {btn: 0 for btn in list(MODE_BUTTONS.keys()) + ['XB']}

        while not rospy.is_shutdown() and not emergency_stop.is_set():
            data = joystick_read(js)

            # Emergency stop (X button)
            if data['XB'] and not prev_buttons['XB']:
                mc.stop()
                emergency_stop.set()
                print("\n\r!! EMERGENCY STOP !!                     ")
                break

            # Mode switching (rising edge)
            for btn, mode in MODE_BUTTONS.items():
                if data[btn] and not prev_buttons[btn]:
                    mode_name = modes.switch_mode(mode)
                    joystick_rumble(js)
                    mode_changed.set()
                    print(f"\n\r>> MODE: {mode_name}                     ")

            for btn in prev_buttons:
                prev_buttons[btn] = data[btn]

            time.sleep(0.02)

    js_thread = threading.Thread(target=joystick_loop, daemon=True)
    js_thread.start()

    # ── Helper: get current pose ─────────────────────────────────────────
    def get_pose():
        with pose_lock:
            if pose_state['x'] is None:
                return None
            return pose_state['x'], pose_state['y'], pose_state['yaw']

    # ── Helper: wait for pose to be available ────────────────────────────
    def wait_for_pose():
        print("Waiting for odometry...", end='', flush=True)
        while not rospy.is_shutdown() and not emergency_stop.is_set():
            if get_pose() is not None:
                print(" OK")
                return True
            time.sleep(0.1)
        return False

    # ── PD control loop to reach a setpoint ──────────────────────────────
    def navigate_to(target_x, target_y, ref_yaw):
        """Run PD controller to reach (target_x, target_y) while holding ref_yaw.

        Returns:
            'arrived'  — within tolerance
            'stopped'  — emergency stop or mode changed away from WALK
        """
        interval = 1.0 / args.rate
        prev_ex, prev_ey = None, None
        prev_eyaw = None
        prev_time = None

        print(f"  Navigating to ({target_x:.3f}, {target_y:.3f}), holding yaw {math.degrees(ref_yaw):.1f}° ...")

        while not rospy.is_shutdown() and not emergency_stop.is_set():
            now = time.time()
            pose = get_pose()
            if pose is None:
                time.sleep(interval)
                continue

            x, y, yaw = pose

            # Check if mode changed away from WALK
            if mode_changed.is_set():
                mode_changed.clear()
                if modes.current_mode != ModeController.WALK:
                    mc.stop()
                    return 'stopped'

            # Position error in odom frame
            ex_odom = target_x - x
            ey_odom = target_y - y
            dist = math.sqrt(ex_odom**2 + ey_odom**2)

            # Yaw error (wrap to [-pi, pi])
            eyaw = ref_yaw - yaw
            eyaw = math.atan2(math.sin(eyaw), math.cos(eyaw))

            # Check arrival (position + yaw)
            if dist < args.tolerance and abs(eyaw) < args.yaw_tolerance:
                mc.stop()
                return 'arrived'

            # Rotate error into body frame
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            ex_body = cos_yaw * ex_odom + sin_yaw * ey_odom
            ey_body = -sin_yaw * ex_odom + cos_yaw * ey_odom

            # Derivative term (body frame error rate)
            d_ex, d_ey, d_eyaw = 0.0, 0.0, 0.0
            if prev_ex is not None and prev_time is not None:
                dt = now - prev_time
                if dt > 0:
                    d_ex = (ex_body - prev_ex) / dt
                    d_ey = (ey_body - prev_ey) / dt
                    d_eyaw = (eyaw - prev_eyaw) / dt

            prev_ex, prev_ey = ex_body, ey_body
            prev_eyaw = eyaw
            prev_time = now

            # PD output → velocity commands (heading scaled to match robot's slower forward speed)
            cmd_heading = 0.56 * (args.kp * ex_body + args.kd * d_ex)
            cmd_lateral = args.kp * ey_body + args.kd * d_ey
            cmd_turning = args.kp_yaw * eyaw + args.kd_yaw * d_eyaw

            # Clamp
            cmd_heading = max(-args.vmax, min(args.vmax, cmd_heading))
            cmd_lateral = max(-args.vmax, min(args.vmax, cmd_lateral))
            cmd_turning = max(-args.vmax_yaw, min(args.vmax_yaw, cmd_turning))

            mc.set_velocity(heading=cmd_heading, lateral=cmd_lateral, turning=cmd_turning)

            print(f"\r  dist:{dist:.3f}m  yaw_err:{math.degrees(eyaw):+.1f}°"
                  f"  cmd: fwd={cmd_heading:+.2f} lat={cmd_lateral:+.2f} turn={cmd_turning:+.2f}"
                  f"  pos:({x:.3f},{y:.3f})   ",
                  end='', flush=True)

            time.sleep(interval)

        mc.stop()
        return 'stopped'

    # ── Main state machine ───────────────────────────────────────────────
    print()
    print("ANYmal Setpoint Navigation")
    print("==========================")
    print("  Y = WALK   B = STAND   A = REST   X = EMERGENCY STOP")
    print("  Press Y to begin, then enter displacements as 'dx,dy' (meters).")
    print()

    if not wait_for_pose():
        return

    try:
        while not rospy.is_shutdown() and not emergency_stop.is_set():
            # Wait for WALK mode
            if modes.current_mode != ModeController.WALK:
                mode_changed.clear()
                print("Waiting for WALK mode (press Y)...", end='\r', flush=True)
                while (not emergency_stop.is_set()
                       and not rospy.is_shutdown()
                       and modes.current_mode != ModeController.WALK):
                    mode_changed.wait(timeout=0.2)
                    mode_changed.clear()
                if emergency_stop.is_set():
                    break
                print("WALK mode active — ready for setpoints.        ")

            # Get current position as origin for displacement
            pose = get_pose()
            if pose is None:
                time.sleep(0.1)
                continue
            origin_x, origin_y, origin_yaw = pose
            print(f"Current position: ({origin_x:.3f}, {origin_y:.3f}), yaw: {math.degrees(origin_yaw):.1f}°")

            # Prompt for displacement
            try:
                raw = input("\nEnter displacement dx,dy (meters) [or 'q' to quit]: ").strip()
            except EOFError:
                break
            if emergency_stop.is_set():
                break
            if raw.lower() == 'q':
                break

            # Parse displacement
            try:
                parts = raw.split(',')
                dx = float(parts[0].strip())
                dy = float(parts[1].strip())
            except (ValueError, IndexError):
                print("Invalid format. Use 'dx,dy' e.g. '0.5,0.8'")
                continue

            target_x = origin_x + dx
            target_y = origin_y + dy
            print(f"Target: ({target_x:.3f}, {target_y:.3f})  [displacement: ({dx:+.3f}, {dy:+.3f})]")

            # Confirmation
            try:
                confirm = input("Press Enter to confirm (or 'c' to cancel): ").strip()
            except EOFError:
                break
            if emergency_stop.is_set():
                break
            if confirm.lower() == 'c':
                print("Cancelled.")
                continue

            # Navigate (hold starting yaw)
            result = navigate_to(target_x, target_y, origin_yaw)
            print()  # newline after \r status line

            if result == 'arrived':
                pose = get_pose()
                if pose:
                    print(f"ARRIVED at ({pose[0]:.3f}, {pose[1]:.3f})")
                else:
                    print("ARRIVED")
                joystick_rumble(js)
            elif result == 'stopped':
                print("Navigation paused (mode changed or stopped).")

    except KeyboardInterrupt:
        print("\n\rInterrupted.")
    finally:
        mc.stop()
        mc.shutdown()
        emergency_stop.set()
        js_thread.join(timeout=1.0)
        joystick_disconnect(js)
        print("Setpoint navigation stopped.")


if __name__ == '__main__':
    main()
