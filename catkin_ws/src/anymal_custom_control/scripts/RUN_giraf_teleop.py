#!/usr/bin/env python3
"""Combined ANYmal + arm task-space teleop (GIRAF).

Two control modes toggled by MENULEFT / MENURIGHT:
  - ANYMAL mode: left stick drives locomotion, right stick strafes
  - ARM mode:    left stick drives arm XY, triggers drive Z

Controls (always active):
    LB + RB        — dead man's switch (BOTH must be held)
    A button       — REST mode
    B button       — STAND mode
    Y button       — WALK mode
    X button       — emergency stop + quit
    MENULEFT       — switch to ANYMAL control
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

Prerequisites:
    - ANYmal software running
    - candle_ros_node running: rosrun candle_ros candle_ros_node USB 1M
    - Xbox controller plugged in

Usage:
    python3 RUN_giraf_teleop.py
    python3 RUN_giraf_teleop.py --speed 0.3
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
js_handle = [None]  # shared so main thread can rumble

running = True
running_lock = threading.Lock()

# Control mode: "ANYMAL" or "ARM"
control_mode = "ANYMAL"
control_mode_lock = threading.Lock()

# Arm state (written by motor thread, read by main for display)
arm_state = {"roll": 0.0, "pitch": 0.0, "boom": 0.0, "ex": 0.0, "ey": 0.0, "ez": 0.0}
arm_state_lock = threading.Lock()

# ANYmal mode buttons
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

            # Only process arm commands in ARM mode with bumpers held
            velocity = np.zeros((3, 1))
            if mode == "ARM" and LB and RB:
                velocity[0] = ARM_X_SPEED * LY
                velocity[1] = -ARM_Y_SPEED * LX
                if RT and not LT:
                    velocity[2] = ARM_Z_SPEED * RT
                elif LT and not RT:
                    velocity[2] = -ARM_Z_SPEED * LT

            # Inverse Jacobian
            joint_coords = [roll_pos, pitch_pos + np.pi / 2, d3_pos]
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

            # FK
            T = num_forward_kinematics(joint_coords)

            with arm_state_lock:
                arm_state["roll"] = roll_pos
                arm_state["pitch"] = pitch_pos
                arm_state["boom"] = boom_pos
                arm_state["ex"] = T[0, 3]
                arm_state["ey"] = T[1, 3]
                arm_state["ez"] = T[2, 3]

            motor_drive(ctx, roll_pos, pitch_pos, boom_pos)
            time.sleep(DT)
    finally:
        motor_disconnect()
        print("\033[93mGIRAF: Arm Motors Disconnected!\033[0m")


# ── Display ─────────────────────────────────────────────────────────────────

DISPLAY_LINES = 8


def draw_display(ctrl_mode, anymal_mode, speed, heading, lateral, turning,
                 anymal_pos, arm):
    """Redraw the status display using ANSI escape codes."""
    anymal_xy = f"({anymal_pos[0]:.3f}, {anymal_pos[1]:.3f})" if anymal_pos else "---"
    arm_xyz = f"({arm['ex']:.3f}, {arm['ey']:.3f}, {arm['ez']:.3f})"

    active_marker = lambda m: "\033[92m>>>\033[0m" if ctrl_mode == m else "   "

    lines = [
        "\033[2J\033[H",  # clear screen + cursor home
        "╔══════════════════════════════════════════════════════╗",
        "║              GIRAF TELEOP CONTROLLER                ║",
        "╠══════════════════════════════════════════════════════╣",
        f"║  Control: \033[1m{ctrl_mode:6s}\033[0m        ANYmal mode: \033[1m{anymal_mode or '?':5s}\033[0m       ║",
        "╠══════════════════════════════════════════════════════╣",
        f"║ {active_marker('ANYMAL')} ANYmal  fwd:{heading:+.2f}  lat:{lateral:+.2f}  turn:{turning:+.2f}  ║",
        f"║           pos: {anymal_xy:>22s}              ║",
        "╠══════════════════════════════════════════════════════╣",
        f"║ {active_marker('ARM')} Arm     ee: {arm_xyz:>28s}      ║",
        f"║           roll:{arm['roll']:+.3f}  pitch:{arm['pitch']:+.3f}  boom:{arm['boom']:+.3f} ║",
        "╠══════════════════════════════════════════════════════╣",
        f"║  Speed: {speed:.0%}   LB+RB to move   X to quit          ║",
        "╚══════════════════════════════════════════════════════╝",
    ]
    print("\n".join(lines), end='', flush=True)


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    global running, control_mode

    parser = argparse.ArgumentParser(description="GIRAF combined teleop.")
    parser.add_argument('--speed', type=float, default=0.5,
                        help="ANYmal speed fraction 0.0-1.0 (default: 0.5)")
    parser.add_argument('--topic', type=str, default='/anyjoy/operator',
                        help="AnyJoy topic (default: /anyjoy/operator)")
    args = parser.parse_args()

    speed = max(0.05, min(1.0, args.speed))

    rospy.init_node('giraf_teleop', anonymous=True)

    # ANYmal pose subscriber
    anymal_pos = [None]

    def _pose_cb(msg):
        p = msg.pose.pose.position
        anymal_pos[0] = (p.x, p.y)

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

    # Wait for joystick to connect
    while js_handle[0] is None and running:
        time.sleep(0.05)

    # Rising-edge detection
    edge_buttons = list(MODE_BUTTONS.keys()) + ['MENULEFT', 'MENURIGHT', 'XB']
    prev_buttons = {btn: 0 for btn in edge_buttons}

    heading = 0.0
    lateral = 0.0
    turning = 0.0

    try:
        while running and not rospy.is_shutdown():
            with joystick_lock:
                data = dict(joystick_data)

            # ── Emergency stop (X button) ──────────────────────────
            if data['XB'] and not prev_buttons['XB']:
                mc.stop()
                with running_lock:
                    running = False
                print("\n!! EMERGENCY STOP !!")
                break

            # ── Control mode switching ─────────────────────────────
            if data['MENULEFT'] and not prev_buttons['MENULEFT']:
                with control_mode_lock:
                    control_mode = "ANYMAL"
                if js_handle[0]:
                    joystick_rumble(js_handle[0])

            if data['MENURIGHT'] and not prev_buttons['MENURIGHT']:
                with control_mode_lock:
                    control_mode = "ARM"
                mc.stop()  # zero ANYmal velocity when switching to arm
                if js_handle[0]:
                    joystick_rumble(js_handle[0])

            # ── ANYmal mode switching (always active) ──────────────
            for btn, mode in MODE_BUTTONS.items():
                if data[btn] and not prev_buttons[btn]:
                    modes.switch_mode(mode)
                    if js_handle[0]:
                        joystick_rumble(js_handle[0])

            # ── ANYmal movement (only in ANYMAL mode) ──────────────
            with control_mode_lock:
                ctrl = control_mode

            bumpers_held = data['LB'] and data['RB']

            if ctrl == "ANYMAL" and bumpers_held:
                heading = data['LY'] * speed
                turning = -data['LX'] * speed
                lateral = -data['RX'] * speed
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

            draw_display(ctrl, modes.current_mode, speed,
                         heading, lateral, turning, anymal_pos[0], arm)

            # Update edge detection
            for btn in prev_buttons:
                prev_buttons[btn] = data[btn]

            time.sleep(0.05)  # 20 Hz display

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        mc.stop()
        mc.shutdown()
        with running_lock:
            running = False
        print("GIRAF teleop stopped.")


if __name__ == '__main__':
    main()
