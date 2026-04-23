#!/usr/bin/env python3
"""Coordinated ANYmal + arm teleop (GIRAF).

In ARM mode: joystick drives arm in task space as usual.
In ANYMAL mode: joystick drives ANYmal locomotion, and the arm automatically
compensates to keep the end-effector stationary in the world frame via a PD
controller in the arm's task space.

Frame mapping (ANYmal body → arm base):
    ANYmal +X (forward)  → arm -Y
    ANYmal +Y (left)     → arm +X

Controls (always active):
    LB + RB        — dead man's switch (BOTH must be held)
    A button       — REST mode
    B button       — STAND mode
    Y button       — WALK mode
    X button       — emergency stop + quit
    MENULEFT       — switch to ANYMAL control (snapshots reference)
    MENURIGHT      — switch to ARM control

ANYMAL mode (LB+RB held):
    Left stick Y   — forward / backward
    Left stick X   — turn left / right
    Right stick X  — strafe left / right

ARM mode (LB+RB held):
    Left stick Y   — arm X velocity
    Left stick X   — arm Y velocity
    RT             — arm Z up
    LT             — arm Z down

Usage:
    python3 RUN_giraf_coord_teleop.py
    python3 RUN_giraf_coord_teleop.py --speed 0.3
"""

import argparse
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

# ── Shared state ────────────────────────────────────────────────────────────

joystick_data = {
    "LX": 0, "LY": 0, "RX": 0, "RY": 0,
    "LT": 0, "RT": 0,
    "AB": 0, "BB": 0, "XB": 0, "YB": 0,
    "LB": 0, "RB": 0,
    "MENULEFT": 0, "MENURIGHT": 0,
}
joystick_lock = threading.Lock()
js_handle = [None]

running = True
running_lock = threading.Lock()

# Control mode: "ANYMAL" or "ARM"
control_mode = "ANYMAL"
control_mode_lock = threading.Lock()

# ANYmal pose from odometry (updated by ROS subscriber)
anymal_pose = [None]  # (x, y) or None
anymal_pose_lock = threading.Lock()

# Reference snapshot (set when switching to ANYMAL mode)
# Stores (anymal_x0, anymal_y0, ee_x0, ee_y0, ee_z0)
coord_ref = [None]
coord_ref_lock = threading.Lock()

# Arm state for display
arm_state = {"roll": 0.0, "pitch": 0.0, "boom": 0.0,
             "ex": 0.0, "ey": 0.0, "ez": 0.0,
             "des_ex": 0.0, "des_ey": 0.0, "des_ez": 0.0}
arm_state_lock = threading.Lock()

MODE_BUTTONS = {
    'AB': ModeController.REST,
    'BB': ModeController.STAND,
    'YB': ModeController.WALK,
}

# ── Signal handling ─────────────────────────────────────────────────────────

_sigint_count = 0


def _sigint_handler(signum, frame):
    global _sigint_count, running
    _sigint_count += 1
    if _sigint_count >= 2:
        print("\nForce quit.")
        os._exit(0)
    with running_lock:
        running = False


signal.signal(signal.SIGINT, _sigint_handler)


# ── Joystick thread ────────────────────────────────────────────────────────

def joystick_monitor():
    global joystick_data, running

    js = joystick_connect()
    js_handle[0] = js
    print("\033[93mGIRAF: Joystick Connected!\033[0m")

    while running:
        with joystick_lock:
            joystick_data = joystick_read(js)
        time.sleep(0.005)

    joystick_disconnect(js)
    print("\033[93mGIRAF: Joystick Disconnected!\033[0m")


# ── Motor control thread ───────────────────────────────────────────────────

DT = 0.005  # 200 Hz
ARM_X_SPEED = 0.2
ARM_Y_SPEED = 0.2
ARM_Z_SPEED = 0.2

# P gain for coordinated mode
KP = 3.0


def motor_control():
    global joystick_data, running, control_mode

    roll_pos = 0.0
    pitch_pos = 0.0
    d3_pos = 0.310

    ctx = motor_connect()
    print("\033[93mGIRAF: Arm Motors Connected!\033[0m")

    try:
        while running:
            with joystick_lock:
                LX = joystick_data["LX"]
                LY = joystick_data["LY"]
                LT = joystick_data["LT"]
                RT = joystick_data["RT"]
                LB = joystick_data["LB"]
                RB = joystick_data["RB"]

            with control_mode_lock:
                mode = control_mode

            # Current FK
            joint_coords = [roll_pos, pitch_pos + np.pi / 2, d3_pos]
            T = num_forward_kinematics(joint_coords)
            ex, ey, ez = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])

            velocity = np.zeros((3, 1))

            if mode == "ARM" and LB and RB:
                # Direct arm taskspace control
                velocity[0] = ARM_X_SPEED * LY
                velocity[1] = -ARM_Y_SPEED * LX
                if RT and not LT:
                    velocity[2] = ARM_Z_SPEED * RT
                elif LT and not RT:
                    velocity[2] = -ARM_Z_SPEED * LT
                prev_error = np.zeros(3)

            elif mode == "ANYMAL":
                # PD compensation to hold EE in world frame
                with coord_ref_lock:
                    ref = coord_ref[0]
                with anymal_pose_lock:
                    pose = anymal_pose[0]

                if ref is not None and pose is not None:
                    ax0, ay0, ex0, ey0, ez0 = ref
                    ax, ay = pose

                    # ANYmal displacement
                    d_anymal_x = ax - ax0
                    d_anymal_y = ay - ay0

                    # Desired arm EE (compensate for base motion)
                    # ANYmal +X → arm -Y, ANYmal +Y → arm +X
                    des_ex = ex0 + d_anymal_y
                    des_ey = ey0 - d_anymal_x
                    des_ez = ez0

                    # P controller
                    error = np.array([des_ex - ex, des_ey - ey, des_ez - ez])
                    vel = KP * error
                    velocity[0] = vel[0]
                    velocity[1] = vel[1]
                    velocity[2] = vel[2]

                    with arm_state_lock:
                        arm_state["des_ex"] = des_ex
                        arm_state["des_ey"] = des_ey
                        arm_state["des_ez"] = des_ez
                else:
                    pass

            # Inverse Jacobian
            Jv = num_jacobian(joint_coords)
            try:
                Jv_inv = np.linalg.inv(Jv)
            except np.linalg.LinAlgError:
                Jv_inv = np.zeros((3, 3))

            joint_velocity = Jv_inv @ velocity

            roll_pos += DT * joint_velocity[0, 0]
            pitch_pos += DT * joint_velocity[1, 0]
            d3_pos += DT * joint_velocity[2, 0]

            # Joint limits
            roll_pos = max(min(roll_pos, np.pi / 2), -np.pi / 2)
            pitch_pos = max(min(pitch_pos, np.pi / 2), 0)

            boom_pos = get_boom_motor_rad(d3_pos)
            boom_pos = max(min(boom_pos, 0), -30)
            d3_pos = get_boom_length_d3(boom_pos)

            with arm_state_lock:
                arm_state["roll"] = roll_pos
                arm_state["pitch"] = pitch_pos
                arm_state["boom"] = boom_pos
                arm_state["ex"] = ex
                arm_state["ey"] = ey
                arm_state["ez"] = ez

            motor_drive(ctx, roll_pos, pitch_pos, boom_pos)
            time.sleep(DT)
    finally:
        motor_disconnect()
        print("\033[93mGIRAF: Arm Motors Disconnected!\033[0m")


# ── Display ─────────────────────────────────────────────────────────────────

def draw_display(ctrl_mode, anymal_mode, speed, heading, lateral, turning,
                 anymal_pos, arm):
    active_marker = lambda m: "\033[92m>>>\033[0m" if ctrl_mode == m else "   "
    anymal_xy = f"({anymal_pos[0]:.3f}, {anymal_pos[1]:.3f})" if anymal_pos else "---"
    arm_xyz = f"({arm['ex']:.3f}, {arm['ey']:.3f}, {arm['ez']:.3f})"

    lines = [
        "\033[2J\033[H",
        "╔══════════════════════════════════════════════════════════╗",
        "║            GIRAF COORDINATED TELEOP                     ║",
        "╠══════════════════════════════════════════════════════════╣",
        f"║  Control: \033[1m{ctrl_mode:6s}\033[0m        ANYmal mode: \033[1m{anymal_mode or '?':5s}\033[0m         ║",
        "╠══════════════════════════════════════════════════════════╣",
        f"║ {active_marker('ANYMAL')} ANYmal  fwd:{heading:+.2f}  lat:{lateral:+.2f}  turn:{turning:+.2f}    ║",
        f"║           pos: {anymal_xy:>22s}                ║",
        "╠══════════════════════════════════════════════════════════╣",
        f"║ {active_marker('ARM')} Arm     ee: {arm_xyz:>28s}        ║",
        f"║           roll:{arm['roll']:+.3f}  pitch:{arm['pitch']:+.3f}  boom:{arm['boom']:+.3f}   ║",
    ]

    if ctrl_mode == "ANYMAL":
        des_xyz = f"({arm['des_ex']:.3f}, {arm['des_ey']:.3f}, {arm['des_ez']:.3f})"
        lines.append(f"║     desired ee: {des_xyz:>28s}        ║")

    lines += [
        "╠══════════════════════════════════════════════════════════╣",
        f"║  Speed: {speed:.0%}   LB+RB to move   X to quit            ║",
        "╚══════════════════════════════════════════════════════════╝",
    ]
    print("\n".join(lines), end='', flush=True)


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    global running, control_mode

    parser = argparse.ArgumentParser(description="GIRAF coordinated teleop.")
    parser.add_argument('--speed', type=float, default=0.5,
                        help="ANYmal speed fraction 0.0-1.0 (default: 0.5)")
    parser.add_argument('--topic', type=str, default='/anyjoy/operator',
                        help="AnyJoy topic (default: /anyjoy/operator)")
    args = parser.parse_args()

    speed = max(0.05, min(1.0, args.speed))

    rospy.init_node('giraf_coord_teleop', anonymous=True)

    # ANYmal pose subscriber
    def _pose_cb(msg):
        p = msg.pose.pose.position
        with anymal_pose_lock:
            anymal_pose[0] = (p.x, p.y)

    rospy.Subscriber('/legged_odometry/pose_in_odom', PoseWithCovarianceStamped, _pose_cb)

    # ANYmal controllers
    mc = MovementController(topic=args.topic)
    mc.start()
    modes = ModeController(movement_controller=mc)

    # Start threads
    joystick_thread = threading.Thread(target=joystick_monitor, daemon=True)
    motor_thread = threading.Thread(target=motor_control, daemon=True)
    joystick_thread.start()
    motor_thread.start()

    while js_handle[0] is None and running:
        time.sleep(0.05)

    edge_buttons = list(MODE_BUTTONS.keys()) + ['MENULEFT', 'MENURIGHT', 'XB']
    prev_buttons = {btn: 0 for btn in edge_buttons}

    heading = 0.0
    lateral = 0.0
    turning = 0.0

    try:
        while running and not rospy.is_shutdown():
            with joystick_lock:
                data = dict(joystick_data)

            # ── Emergency stop ─────────────────────────────────────
            if data['XB'] and not prev_buttons['XB']:
                mc.stop()
                with running_lock:
                    running = False
                print("\n!! EMERGENCY STOP !!")
                break

            # ── Control mode switching ─────────────────────────────
            if data['MENULEFT'] and not prev_buttons['MENULEFT']:
                # Switch to ANYMAL: snapshot references
                with anymal_pose_lock:
                    pose = anymal_pose[0]
                with arm_state_lock:
                    arm = dict(arm_state)

                if pose is not None:
                    with coord_ref_lock:
                        coord_ref[0] = (pose[0], pose[1],
                                        arm['ex'], arm['ey'], arm['ez'])

                with control_mode_lock:
                    control_mode = "ANYMAL"
                if js_handle[0]:
                    joystick_rumble(js_handle[0])

            if data['MENURIGHT'] and not prev_buttons['MENURIGHT']:
                with control_mode_lock:
                    control_mode = "ARM"
                with coord_ref_lock:
                    coord_ref[0] = None
                mc.stop()
                if js_handle[0]:
                    joystick_rumble(js_handle[0])

            # ── ANYmal mode switching (always active) ──────────────
            for btn, mode in MODE_BUTTONS.items():
                if data[btn] and not prev_buttons[btn]:
                    modes.switch_mode(mode)
                    if js_handle[0]:
                        joystick_rumble(js_handle[0])

            # ── ANYmal movement ────────────────────────────────────
            with control_mode_lock:
                ctrl = control_mode

            bumpers_held = data['LB'] and data['RB']

            if ctrl == "ANYMAL" and bumpers_held:
                heading = data['LY'] * speed
                turning = -data['RX'] * speed
                lateral = -data['LX'] * speed
                mc.set_velocity(heading=heading, lateral=lateral, turning=turning)
            elif ctrl == "ANYMAL":
                heading = lateral = turning = 0.0
                mc.stop()
            else:
                heading = lateral = turning = 0.0
                mc.stop()

            # ── Display ────────────────────────────────────────────
            with arm_state_lock:
                arm = dict(arm_state)
            with anymal_pose_lock:
                apose = anymal_pose[0]

            draw_display(ctrl, modes.current_mode, speed,
                         heading, lateral, turning, apose, arm)

            for btn in prev_buttons:
                prev_buttons[btn] = data[btn]

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        mc.stop()
        mc.shutdown()
        with running_lock:
            running = False
        print("GIRAF coordinated teleop stopped.")


if __name__ == '__main__':
    main()
