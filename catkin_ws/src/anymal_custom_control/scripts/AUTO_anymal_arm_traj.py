#!/usr/bin/env python3
"""Autonomous ANYmal + boom arm trajectory execution.

Executes a pre-planned sequence of waypoints for ANYmal (x, y, yaw) and the
boom arm (task-space EE position).  The state machine for each waypoint is:

    1. Navigate ANYmal to waypoint via PD control on (x, y, yaw)
    2. Drive arm EE to target (x, y, z) via task-space P controller
       (P on EE error → clamped velocity → inverse Jacobian → joint integration)
    3. Pause for BOOM_HOLD_TIME
    4. Retract boom in joint space to BOOM_STOW_POS (motor rad)
    5. Advance to next waypoint (go to 1)

All ANYmal waypoints are defined as (dx, dy, dyaw) displacements from the
starting pose.  Arm waypoints are EE positions (x, y, z) in the arm's base
frame.  Boom retract is in motor radians (joint space).

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
    python3 AUTO_anymal_arm_traj.py --kp 1.0 --kd 0.3 --vmax 0.4
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
    num_jacobian,
    get_boom_motor_rad,
    get_boom_length_d3,
)

# Boom stow position after each waypoint (joint-space, motor rad)
BOOM_STOW_POS = -1.0

# Hold time at arm waypoint before retracting (seconds)
BOOM_HOLD_TIME = 1.0

# Task-space P controller gain and velocity clamp (m/s)
ARM_KP = 3.0
ARM_VMAX = 0.2

# EE convergence tolerance (meters)
ARM_TOLERANCE = 0.01

# Boom retract ramp rate (rad/s, joint space)
BOOM_RETRACT_RATE = 3.0


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
    parser.add_argument('traj', type=str,
                        help="Trajectory module name in trajectories/ (e.g. demo1_3x_task_r0p75)")
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
    parser.add_argument('--tolerance', type=float, default=0.02,
                        help="ANYmal arrival tolerance meters (default: 0.02)")
    parser.add_argument('--yaw_tolerance', type=float, default=0.1,
                        help="ANYmal yaw tolerance radians (default: 0.1)")
    parser.add_argument('--rate', type=int, default=20,
                        help="ANYmal control loop rate Hz (default: 20)")
    parser.add_argument('--motor_rate', type=int, default=200,
                        help="Motor command rate Hz (default: 200)")
    args = parser.parse_args()

    # ── Load trajectory ───────────────────────────────────────────────
    try:
        traj = __import__(f"trajectories.{args.traj}", fromlist=["ANYMAL_WAYPOINTS", "ARM_WAYPOINTS"])
        ANYMAL_WAYPOINTS = traj.ANYMAL_WAYPOINTS
        ARM_WAYPOINTS = traj.ARM_WAYPOINTS
    except ModuleNotFoundError:
        print(f"ERROR: Trajectory '{args.traj}' not found in trajectories/")
        available = [f.stem for f in __import__('pathlib').Path(__file__).parent.joinpath('trajectories').glob('*.py')
                     if f.name != '__init__.py']
        if available:
            print(f"Available: {', '.join(sorted(available))}")
        return
    except AttributeError as e:
        print(f"ERROR: Trajectory '{args.traj}' missing required data: {e}")
        return

    assert len(ANYMAL_WAYPOINTS) == len(ARM_WAYPOINTS), \
        "Must have same number of ANYmal and arm waypoints"

    print(f"Loaded trajectory: {args.traj} ({len(ANYMAL_WAYPOINTS)} waypoints)")

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
        print("WALK mode active. Starting in 1.0s...")
        if emergency_stop.wait(timeout=1.0):
            return False
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

    # ── Arm task-space P controller ──────────────────────────────────────

    # Mutable arm joint state — persists across calls
    arm_joints = {'roll': 0.0, 'pitch': 0.0, 'd3': get_boom_length_d3(BOOM_STOW_POS)}

    def drive_ee_to(ctx, target_xyz):
        """Task-space P controller: drive EE to (x, y, z) in arm base frame.

        Uses P gain on EE error → clamped velocity → inverse Jacobian →
        joint velocity integration (same approach as RUN_giraf_coord_teleop).

        Updates arm_joints in place.  Returns: 'arrived' | 'stopped'
        """
        dt = 1.0 / args.motor_rate
        tx, ty, tz = target_xyz

        print(f"  ARM → ee target ({tx:.3f}, {ty:.3f}, {tz:.3f})")

        while not is_stopped():
            roll = arm_joints['roll']
            pitch = arm_joints['pitch']
            d3 = arm_joints['d3']

            # Current FK (pitch offset by pi/2 for DH convention)
            joint_coords = [roll, pitch + np.pi / 2, d3]
            T = num_forward_kinematics(joint_coords)
            ex, ey, ez = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])

            # EE error
            err = np.array([tx - ex, ty - ey, tz - ez])
            dist = np.linalg.norm(err)

            if dist < ARM_TOLERANCE:
                # Hold position
                boom_pos = get_boom_motor_rad(d3)
                motor_drive(ctx, roll, pitch, boom_pos)
                return 'arrived'

            # P controller with velocity clamping
            velocity = ARM_KP * err
            speed = np.linalg.norm(velocity)
            if speed > ARM_VMAX:
                velocity = velocity * (ARM_VMAX / speed)

            # Inverse Jacobian → joint velocities
            Jv = num_jacobian(joint_coords)
            try:
                Jv_inv = np.linalg.inv(Jv)
            except np.linalg.LinAlgError:
                Jv_inv = np.zeros((3, 3))

            joint_vel = Jv_inv @ velocity.reshape(3, 1)

            # Integrate joint positions
            roll += dt * float(joint_vel[0, 0])
            pitch += dt * float(joint_vel[1, 0])
            d3 += dt * float(joint_vel[2, 0])

            # Joint limits
            roll = max(min(roll, np.radians(120)), -np.radians(120))
            pitch = max(min(pitch, np.pi / 2), 0)

            boom_pos = get_boom_motor_rad(d3)
            boom_pos = max(min(boom_pos, 0), -30)
            d3 = get_boom_length_d3(boom_pos)

            # Update shared state
            arm_joints['roll'] = roll
            arm_joints['pitch'] = pitch
            arm_joints['d3'] = d3

            motor_drive(ctx, roll, pitch, boom_pos)

            print(f"\r  ARM  err:{dist:.4f}m  ee:({ex:.3f},{ey:.3f},{ez:.3f})"
                  f"  joints: r={roll:+.3f} p={pitch:+.3f} boom={boom_pos:+.2f}   ",
                  end='', flush=True)

            time.sleep(dt)

        return 'stopped'

    # ── Boom joint-space retract ─────────────────────────────────────────

    def retract_boom(ctx):
        """Ramp boom motor position to BOOM_STOW_POS in joint space,
        then ramp roll and pitch back to 0.

        Updates arm_joints in place to keep FK consistent.
        Returns: 'arrived' | 'stopped'
        """
        dt = 1.0 / args.motor_rate
        current_boom = get_boom_motor_rad(arm_joints['d3'])
        target_boom = BOOM_STOW_POS

        direction = 1.0 if target_boom > current_boom else -1.0
        step = direction * BOOM_RETRACT_RATE * dt
        tolerance = 0.1  # rad

        roll = arm_joints['roll']
        pitch = arm_joints['pitch']

        print(f"  RETRACT → {target_boom:.1f} rad (from {current_boom:.1f} rad)")

        while not is_stopped():
            if abs(current_boom - target_boom) < tolerance:
                current_boom = target_boom
                arm_joints['d3'] = get_boom_length_d3(current_boom)
                motor_drive(ctx, roll, pitch, current_boom)
                break

            current_boom += step

            # Don't overshoot
            if direction > 0 and current_boom > target_boom:
                current_boom = target_boom
            elif direction < 0 and current_boom < target_boom:
                current_boom = target_boom

            arm_joints['d3'] = get_boom_length_d3(current_boom)

            # FK for display
            joint_coords = [roll, pitch + np.pi / 2, arm_joints['d3']]
            T = num_forward_kinematics(joint_coords)
            ex, ey, ez = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])

            motor_drive(ctx, roll, pitch, current_boom)

            print(f"\r  RETRACT  boom:{current_boom:+.2f} rad  d3:{arm_joints['d3']:.3f}m"
                  f"  ee:({ex:.3f}, {ey:.3f}, {ez:.3f})   ",
                  end='', flush=True)

            time.sleep(dt)

        if is_stopped():
            return 'stopped'

        # ── Ramp roll and pitch back to 0 ─────────────────────────────
        rp_rate = BOOM_RETRACT_RATE  # rad/s, reuse same rate
        rp_tolerance = 0.01  # rad

        print(f"\n  RESET JOINTS → roll=0, pitch=0 (from r={roll:+.3f}, p={pitch:+.3f})")

        while not is_stopped():
            at_zero = True

            # Ramp roll toward 0
            if abs(roll) > rp_tolerance:
                at_zero = False
                if roll > 0:
                    roll = max(0.0, roll - rp_rate * dt)
                else:
                    roll = min(0.0, roll + rp_rate * dt)

            # Ramp pitch toward 0
            if abs(pitch) > rp_tolerance:
                at_zero = False
                if pitch > 0:
                    pitch = max(0.0, pitch - rp_rate * dt)
                else:
                    pitch = min(0.0, pitch + rp_rate * dt)

            arm_joints['roll'] = roll
            arm_joints['pitch'] = pitch

            motor_drive(ctx, roll, pitch, current_boom)

            print(f"\r  RESET  roll:{roll:+.3f} pitch:{pitch:+.3f}   ",
                  end='', flush=True)

            if at_zero:
                arm_joints['roll'] = 0.0
                arm_joints['pitch'] = 0.0
                motor_drive(ctx, 0.0, 0.0, current_boom)
                break

            time.sleep(dt)

        if is_stopped():
            return 'stopped'
        return 'arrived'

    # ── Connect motors ───────────────────────────────────────────────────

    print("Connecting arm motors...")
    motor_ctx = motor_connect()
    print("Arm motors connected.")

    # Initialize arm at stow position
    init_boom = BOOM_STOW_POS
    motor_drive(motor_ctx, arm_joints['roll'], arm_joints['pitch'], init_boom)
    arm_joints['d3'] = get_boom_length_d3(init_boom)
    time.sleep(0.5)

    # ── Main state machine ───────────────────────────────────────────────
    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║        AUTO ANYmal + Arm Trajectory Execution           ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print(f"║  Waypoints: {len(ANYMAL_WAYPOINTS)}                                            ║")
    for i, (aw, ew) in enumerate(zip(ANYMAL_WAYPOINTS, ARM_WAYPOINTS)):
        print(f"║    {i+1}. ANYmal: dx={aw[0]:+.2f} dy={aw[1]:+.2f} dyaw={math.degrees(aw[2]):+.1f}°"
              f"  EE: ({ew[0]:.2f},{ew[1]:.2f},{ew[2]:.2f})")
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
            ee_target = ARM_WAYPOINTS[wp_idx]

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

            # ── Step 2: Drive arm EE to target ───────────────────────
            print(f"\n[Step 2] Drive arm EE to ({ee_target[0]:.3f}, {ee_target[1]:.3f}, {ee_target[2]:.3f})")
            result = drive_ee_to(motor_ctx, ee_target)
            print()

            if result == 'stopped':
                print("Arm motion interrupted. Aborting trajectory.")
                break

            # Show final EE position
            jc = [arm_joints['roll'], arm_joints['pitch'] + np.pi / 2, arm_joints['d3']]
            T = num_forward_kinematics(jc)
            print(f"  ARM ARRIVED — ee:({float(T[0,3]):.3f}, {float(T[1,3]):.3f}, {float(T[2,3]):.3f})")
            joystick_rumble(js)

            # ── Step 3: Hold ─────────────────────────────────────────
            print(f"\n[Step 3] Holding for {BOOM_HOLD_TIME:.1f}s...")
            hold_boom = get_boom_motor_rad(arm_joints['d3'])
            hold_start = time.time()
            while time.time() - hold_start < BOOM_HOLD_TIME and not is_stopped():
                motor_drive(motor_ctx, arm_joints['roll'], arm_joints['pitch'], hold_boom)
                time.sleep(0.05)

            if is_stopped():
                break

            # ── Step 4: Retract boom (joint space) ───────────────────
            print(f"\n[Step 4] Retracting boom to stow ({BOOM_STOW_POS:.1f} rad)")
            result = retract_boom(motor_ctx)
            print()

            if result == 'stopped':
                print("Boom retract interrupted. Aborting trajectory.")
                break

            # Show FK at stowed position
            jc = [arm_joints['roll'], arm_joints['pitch'] + np.pi / 2, arm_joints['d3']]
            T = num_forward_kinematics(jc)
            print(f"  BOOM STOWED — ee:({float(T[0,3]):.3f}, {float(T[1,3]):.3f}, {float(T[2,3]):.3f})")
            joystick_rumble(js)

        # ── Done ─────────────────────────────────────────────────────
        if not is_stopped():
            print(f"\n{'='*60}")
            print("  TRAJECTORY COMPLETE")
            print(f"{'='*60}")
            pose = get_pose()
            if pose:
                print(f"  Final pose: ({pose[0]:.3f}, {pose[1]:.3f}), yaw {math.degrees(pose[2]):.1f}°")
            final_boom = get_boom_motor_rad(arm_joints['d3'])
            print(f"  Final boom: {final_boom:.1f} rad")
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
