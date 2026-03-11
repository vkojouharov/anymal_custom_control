#!/usr/bin/env python3
"""Autonomous ANYmal + boom arm trajectory execution.

Executes a pre-planned sequence of waypoints for ANYmal (x, y, yaw) and the
boom arm (motor rad).  The state machine for each waypoint is:

    1. Navigate ANYmal to waypoint via PD control on (x, y, yaw)
    2. Ramp boom to target motor position
    3. Pause for BOOM_HOLD_TIME
    4. Retract boom to BOOM_STOW_POS
    5. Advance to next waypoint (go to 1)

All ANYmal waypoints are defined as (dx, dy, dyaw) displacements from the
starting pose.  Boom waypoints are motor positions in radians.

Controls (joystick always active):
    Y button  — WALK mode  (starts / resumes execution)
    B button  — STAND mode (pauses execution)
    A button  — REST mode  (pauses execution)
    X button  — EMERGENCY STOP + quit

Prerequisites:
    - ANYmal in WALK-ready state
    - Boom arm powered (candle_ros running)
    - Xbox controller connected
    - E-stop ready

Usage:
    python3 AUTO_anymal_arm_traj.py
    python3 AUTO_anymal_arm_traj.py --kp 1.0 --kd 0.3 --vmax 0.4 --boom_rate 1.0
"""

import argparse
import math
import os
import signal
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from anymal_custom_control import ModeController, MovementController
from anymal_custom_control.joystick_driver import (
    joystick_connect,
    joystick_disconnect,
    joystick_read,
    joystick_rumble,
)
from anymal_custom_control.motor_driver import (
    motor_connect,
    motor_drive,
    motor_disconnect,
)
from anymal_custom_control.RRP_kinematic_model import (
    num_forward_kinematics,
    get_boom_motor_rad,
    get_boom_length_d3,
)


# ═══════════════════════════════════════════════════════════════════════════════
# TRAJECTORY DEFINITION — edit these constants
# ═══════════════════════════════════════════════════════════════════════════════

# ANYmal waypoints: (dx, dy, dyaw) displacements from start pose [meters, rad]
ANYMAL_WAYPOINTS = [
    (0.5, 0.0, 0.0),     # waypoint 1: 0.5m forward
    (0.5, 0.3, 0.0),     # waypoint 2: hold x, 0.3m left
    (1.0, 0.3, 0.0),     # waypoint 3: another 0.5m forward
]

# Boom waypoints: motor position in radians (one per ANYmal waypoint)
BOOM_WAYPOINTS = [
    -8.0,                 # waypoint 1: extend boom
    -8.0,                 # waypoint 2: same extension
    -12.0,                # waypoint 3: extend further
]

# Boom stow position (retract to this after each waypoint)
BOOM_STOW_POS = -3.0     # motor rad

# Hold time at boom waypoint before retracting (seconds)
BOOM_HOLD_TIME = 1.0

# Roll and pitch held constant throughout
ROLL_POS = 0.0
PITCH_POS = 0.0

assert len(ANYMAL_WAYPOINTS) == len(BOOM_WAYPOINTS), \
    "Must have same number of ANYmal and boom waypoints"

# ═══════════════════════════════════════════════════════════════════════════════


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
    parser = argparse.ArgumentParser(description="ANYmal + boom autonomous trajectory.")
    parser.add_argument('--kp', type=float, default=1.0,
                        help="XY proportional gain (default: 1.0)")
    parser.add_argument('--kd', type=float, default=0.3,
                        help="XY derivative gain (default: 0.3)")
    parser.add_argument('--kp_yaw', type=float, default=1.5,
                        help="Yaw proportional gain (default: 1.5)")
    parser.add_argument('--kd_yaw', type=float, default=0.3,
                        help="Yaw derivative gain (default: 0.3)")
    parser.add_argument('--vmax', type=float, default=0.4,
                        help="Max ANYmal velocity fraction 0-1 (default: 0.4)")
    parser.add_argument('--vmax_yaw', type=float, default=0.3,
                        help="Max yaw velocity fraction 0-1 (default: 0.3)")
    parser.add_argument('--tolerance', type=float, default=0.01,
                        help="ANYmal arrival tolerance meters (default: 0.01)")
    parser.add_argument('--yaw_tolerance', type=float, default=0.05,
                        help="ANYmal yaw tolerance radians (default: 0.05)")
    parser.add_argument('--boom_rate', type=float, default=2.0,
                        help="Boom ramp rate rad/s (default: 2.0)")
    parser.add_argument('--boom_tolerance', type=float, default=0.1,
                        help="Boom arrival tolerance rad (default: 0.1)")
    parser.add_argument('--rate', type=int, default=20,
                        help="ANYmal control loop rate Hz (default: 20)")
    parser.add_argument('--motor_rate', type=int, default=200,
                        help="Motor command rate Hz (default: 200)")
    args = parser.parse_args()

    # ── Connect joystick ─────────────────────────────────────────────────
    js = joystick_connect()
    print(f"Controller connected: {js['device'].name}")

    # ── ROS setup ────────────────────────────────────────────────────────
    rospy.init_node('auto_anymal_arm_traj', anonymous=True)

    mc = MovementController()
    mc.start()
    modes = ModeController(movement_controller=mc)

    # ── Pose subscriber ──────────────────────────────────────────────────
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

    # ── Shared flags ─────────────────────────────────────────────────────
    emergency_stop = threading.Event()
    mode_changed = threading.Event()

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

            if data['XB'] and not prev_buttons['XB']:
                mc.stop()
                emergency_stop.set()
                print("\n\r!! EMERGENCY STOP !!")
                break

            for btn, mode in MODE_BUTTONS.items():
                if data[btn] and not prev_buttons[btn]:
                    mode_name = modes.switch_mode(mode)
                    joystick_rumble(js)
                    mode_changed.set()
                    print(f"\n\r>> MODE: {mode_name}")

            for btn in prev_buttons:
                prev_buttons[btn] = data[btn]

            time.sleep(0.02)

    js_thread = threading.Thread(target=joystick_loop, daemon=True)
    js_thread.start()

    # ── Helpers ───────────────────────────────────────────────────────────

    def get_pose():
        with pose_lock:
            if pose_state['x'] is None:
                return None
            return pose_state['x'], pose_state['y'], pose_state['yaw']

    def wait_for_pose():
        print("Waiting for odometry...", end='', flush=True)
        while not rospy.is_shutdown() and not emergency_stop.is_set():
            if get_pose() is not None:
                print(" OK")
                return True
            time.sleep(0.1)
        return False

    def wait_for_walk():
        """Block until WALK mode is active. Returns False if stopped."""
        if modes.current_mode == ModeController.WALK:
            return True
        mode_changed.clear()
        print("Waiting for WALK mode (press Y)...", end='\r', flush=True)
        while (not emergency_stop.is_set()
               and not rospy.is_shutdown()
               and modes.current_mode != ModeController.WALK):
            mode_changed.wait(timeout=0.2)
            mode_changed.clear()
        if emergency_stop.is_set():
            return False
        print("WALK mode active.                              ")
        return True

    def is_stopped():
        return emergency_stop.is_set() or rospy.is_shutdown()

    # ── ANYmal PD navigation ─────────────────────────────────────────────

    def navigate_to(target_x, target_y, target_yaw):
        """PD controller to navigate ANYmal to (target_x, target_y, target_yaw).

        Returns: 'arrived' | 'stopped'
        """
        interval = 1.0 / args.rate
        prev_ex, prev_ey, prev_eyaw = None, None, None
        prev_time = None

        print(f"  NAV → ({target_x:.3f}, {target_y:.3f}), yaw {math.degrees(target_yaw):.1f}°")

        while not is_stopped():
            now = time.time()
            pose = get_pose()
            if pose is None:
                time.sleep(interval)
                continue

            x, y, yaw = pose

            if mode_changed.is_set():
                mode_changed.clear()
                if modes.current_mode != ModeController.WALK:
                    mc.stop()
                    return 'stopped'

            # Position error in odom frame
            ex_odom = target_x - x
            ey_odom = target_y - y
            dist = math.sqrt(ex_odom**2 + ey_odom**2)

            # Yaw error
            eyaw = target_yaw - yaw
            eyaw = math.atan2(math.sin(eyaw), math.cos(eyaw))

            if dist < args.tolerance and abs(eyaw) < args.yaw_tolerance:
                mc.stop()
                return 'arrived'

            # Rotate error into body frame
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            ex_body = cos_yaw * ex_odom + sin_yaw * ey_odom
            ey_body = -sin_yaw * ex_odom + cos_yaw * ey_odom

            # Derivative
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

            # PD output
            cmd_heading = 0.56 * (args.kp * ex_body + args.kd * d_ex)
            cmd_lateral = args.kp * ey_body + args.kd * d_ey
            cmd_turning = args.kp_yaw * eyaw + args.kd_yaw * d_eyaw

            cmd_heading = max(-args.vmax, min(args.vmax, cmd_heading))
            cmd_lateral = max(-args.vmax, min(args.vmax, cmd_lateral))
            cmd_turning = max(-args.vmax_yaw, min(args.vmax_yaw, cmd_turning))

            mc.set_velocity(heading=cmd_heading, lateral=cmd_lateral, turning=cmd_turning)

            print(f"\r  NAV  dist:{dist:.3f}m  yaw_err:{math.degrees(eyaw):+.1f}°"
                  f"  cmd: fwd={cmd_heading:+.2f} lat={cmd_lateral:+.2f} turn={cmd_turning:+.2f}   ",
                  end='', flush=True)

            time.sleep(interval)

        mc.stop()
        return 'stopped'

    # ── Boom ramp control ────────────────────────────────────────────────

    def ramp_boom_to(ctx, target_boom, current_boom):
        """Smoothly ramp boom motor position from current to target.

        Returns: (final_boom_pos, 'arrived' | 'stopped')
        """
        dt = 1.0 / args.motor_rate
        boom_pos = current_boom
        d3 = get_boom_length_d3(boom_pos)

        direction = 1.0 if target_boom > boom_pos else -1.0
        step = direction * args.boom_rate * dt

        print(f"  BOOM → {target_boom:.1f} rad (from {boom_pos:.1f} rad)")

        while not is_stopped():
            # Check if arrived
            if abs(boom_pos - target_boom) < args.boom_tolerance:
                boom_pos = target_boom
                d3 = get_boom_length_d3(boom_pos)
                motor_drive(ctx, ROLL_POS, PITCH_POS, boom_pos)
                break

            boom_pos += step

            # Don't overshoot
            if direction > 0 and boom_pos > target_boom:
                boom_pos = target_boom
            elif direction < 0 and boom_pos < target_boom:
                boom_pos = target_boom

            d3 = get_boom_length_d3(boom_pos)

            # Compute FK for display
            joint_coords = [ROLL_POS, PITCH_POS + np.pi / 2, d3]
            T = num_forward_kinematics(joint_coords)
            ex, ey, ez = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])

            motor_drive(ctx, ROLL_POS, PITCH_POS, boom_pos)

            print(f"\r  BOOM  pos:{boom_pos:+.2f} rad  d3:{d3:.3f}m"
                  f"  ee:({ex:.3f}, {ey:.3f}, {ez:.3f})   ",
                  end='', flush=True)

            time.sleep(dt)

        if is_stopped():
            return boom_pos, 'stopped'
        return boom_pos, 'arrived'

    # ── Connect motors ───────────────────────────────────────────────────

    print("Connecting arm motors...")
    motor_ctx = motor_connect()
    print("Arm motors connected.")

    # Initialize boom at stow position
    current_boom = BOOM_STOW_POS
    motor_drive(motor_ctx, ROLL_POS, PITCH_POS, current_boom)
    time.sleep(0.5)

    # ── Main state machine ───────────────────────────────────────────────
    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║        AUTO ANYmal + Arm Trajectory Execution           ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print(f"║  Waypoints: {len(ANYMAL_WAYPOINTS)}                                            ║")
    for i, (aw, bw) in enumerate(zip(ANYMAL_WAYPOINTS, BOOM_WAYPOINTS)):
        print(f"║    {i+1}. ANYmal: dx={aw[0]:+.2f} dy={aw[1]:+.2f} dyaw={math.degrees(aw[2]):+.1f}°"
              f"  Boom: {bw:.1f} rad")
    print(f"║  Boom stow: {BOOM_STOW_POS:.1f} rad   Hold time: {BOOM_HOLD_TIME:.1f}s        ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print("║  Y = WALK   B = STAND   A = REST   X = EMERGENCY STOP  ║")
    print("║  Press Y to begin execution.                            ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()

    if not wait_for_pose():
        motor_disconnect()
        return

    try:
        # Wait for WALK mode to start
        if not wait_for_walk():
            return

        # Snapshot starting pose
        start_pose = get_pose()
        if start_pose is None:
            print("ERROR: Lost odometry.")
            return
        start_x, start_y, start_yaw = start_pose
        print(f"Start pose: ({start_x:.3f}, {start_y:.3f}), yaw {math.degrees(start_yaw):.1f}°")
        print()

        for wp_idx in range(len(ANYMAL_WAYPOINTS)):
            if is_stopped():
                break

            dx, dy, dyaw = ANYMAL_WAYPOINTS[wp_idx]
            boom_target = BOOM_WAYPOINTS[wp_idx]

            target_x = start_x + dx
            target_y = start_y + dy
            target_yaw = start_yaw + dyaw

            print(f"\n{'='*60}")
            print(f"  WAYPOINT {wp_idx + 1}/{len(ANYMAL_WAYPOINTS)}")
            print(f"{'='*60}")

            # ── Step 1: Navigate ANYmal ──────────────────────────────
            print(f"\n[Step 1] Navigate ANYmal to ({target_x:.3f}, {target_y:.3f})")

            # Re-enter WALK if needed
            if not wait_for_walk():
                break

            result = navigate_to(target_x, target_y, target_yaw)
            print()

            if result == 'stopped':
                print("Navigation paused. Press Y to resume...")
                if not wait_for_walk():
                    break
                # Retry this waypoint
                result = navigate_to(target_x, target_y, target_yaw)
                print()
                if result != 'arrived':
                    print("Navigation failed. Aborting trajectory.")
                    break

            pose = get_pose()
            if pose:
                print(f"  ARRIVED at ({pose[0]:.3f}, {pose[1]:.3f})")
            joystick_rumble(js)

            # ── Step 2: Drive boom to waypoint ───────────────────────
            print(f"\n[Step 2] Drive boom to {boom_target:.1f} rad")
            current_boom, result = ramp_boom_to(motor_ctx, boom_target, current_boom)
            print()

            if result == 'stopped':
                print("Boom motion interrupted. Aborting trajectory.")
                break

            print(f"  BOOM ARRIVED at {current_boom:.1f} rad")
            joystick_rumble(js)

            # ── Step 3: Hold ─────────────────────────────────────────
            print(f"\n[Step 3] Holding for {BOOM_HOLD_TIME:.1f}s...")
            hold_start = time.time()
            while time.time() - hold_start < BOOM_HOLD_TIME and not is_stopped():
                motor_drive(motor_ctx, ROLL_POS, PITCH_POS, current_boom)
                time.sleep(0.05)

            if is_stopped():
                break

            # ── Step 4: Retract boom ─────────────────────────────────
            print(f"\n[Step 4] Retracting boom to stow ({BOOM_STOW_POS:.1f} rad)")
            current_boom, result = ramp_boom_to(motor_ctx, BOOM_STOW_POS, current_boom)
            print()

            if result == 'stopped':
                print("Boom retract interrupted. Aborting trajectory.")
                break

            # FK at stowed position
            d3_stow = get_boom_length_d3(current_boom)
            joint_coords = [ROLL_POS, PITCH_POS + np.pi / 2, d3_stow]
            T = num_forward_kinematics(joint_coords)
            ex, ey, ez = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
            print(f"  BOOM STOWED — ee:({ex:.3f}, {ey:.3f}, {ez:.3f})")
            joystick_rumble(js)

        # ── Done ─────────────────────────────────────────────────────
        if not is_stopped():
            print(f"\n{'='*60}")
            print("  TRAJECTORY COMPLETE")
            print(f"{'='*60}")
            pose = get_pose()
            if pose:
                print(f"  Final pose: ({pose[0]:.3f}, {pose[1]:.3f}), yaw {math.degrees(pose[2]):.1f}°")
            print(f"  Final boom: {current_boom:.1f} rad")
            joystick_rumble(js)

    except KeyboardInterrupt:
        print("\n\rInterrupted.")
    finally:
        mc.stop()
        mc.shutdown()
        motor_disconnect()
        emergency_stop.set()
        js_thread.join(timeout=1.0)
        joystick_disconnect(js)
        print("Trajectory execution stopped.")


if __name__ == '__main__':
    main()
