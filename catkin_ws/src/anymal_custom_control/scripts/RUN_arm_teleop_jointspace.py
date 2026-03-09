#!/usr/bin/env python3
"""Joint-space teleop for MD80 motors via candle_ros.

Controls:
    Left stick X   — roll
    Left stick Y   — pitch
    RT             — extend boom
    LT             — retract boom
    LB + RB        — safety interlock (BOTH must be held for movement)
    X button       — emergency stop + quit

Prerequisites:
    - candle_ros_node running: rosrun candle_ros candle_ros_node USB 1M
    - Xbox controller plugged in

Usage:
    python3 run_teleop_jointspace.py
"""

import os
import signal
import threading
import time

import rospy
from anymal_custom_control.joystick_driver import (
    joystick_connect,
    joystick_disconnect,
    joystick_read,
)
from anymal_custom_control.motor_driver import (
    motor_connect,
    motor_status,
    motor_drive,
    motor_disconnect,
)

# ── Shared state ────────────────────────────────────────────────────────────

joystick_data = {"LX": 0, "LY": 0, "LT": 0, "RT": 0, "XB": 0, "LB": 0, "RB": 0}
joystick_lock = threading.Lock()

running = True
running_lock = threading.Lock()


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
    print("\033[93mTELEOP: Joystick Connected!\033[0m")

    while running:
        with joystick_lock:
            joystick_data = joystick_read(js)
        time.sleep(0.005)

    joystick_disconnect(js)
    print("\033[93mTELEOP: Joystick Disconnected!\033[0m")


# ── Motor control thread ───────────────────────────────────────────────────

def motor_control():
    global joystick_data, running

    roll_pos = 0.0
    pitch_pos = 0.0
    boom_pos = 0.0

    ctx = motor_connect()
    print("\033[93mTELEOP: Motors Connected!\033[0m")

    try:
        while running:
            with joystick_lock:
                LX = joystick_data["LX"]
                LY = joystick_data["LY"]
                LT = joystick_data["LT"]
                RT = joystick_data["RT"]
                XB = joystick_data["XB"]
                LB = joystick_data["LB"]
                RB = joystick_data["RB"]

            if XB:
                with running_lock:
                    running = False
                break

            # dynamically adjust teleop drive ratio
            roll_drive_ratio = 0.005 / (-(boom_pos - 4) / 4)
            pitch_drive_ratio = 0.005 / (-(boom_pos - 4) / 4)
            boom_drive_ratio = 0.025

            # safety interlock
            if LB and RB:
                roll_pos = roll_pos - roll_drive_ratio * LX
                pitch_pos = pitch_pos + pitch_drive_ratio * LY
                if RT and not LT:
                    boom_pos = boom_pos - boom_drive_ratio * RT
                elif LT and not RT:
                    boom_pos = boom_pos + boom_drive_ratio * LT

            # joint limits
            pitch_pos = max(pitch_pos, 0)
            boom_pos = max(min(boom_pos, 0), -30)
            
            print(f"\r  roll:{roll_pos:+.3f}  pitch:{pitch_pos:+.3f}  boom:{boom_pos:+.3f}   ",
                  end='', flush=True)

            motor_status(ctx)
            motor_drive(ctx, roll_pos, pitch_pos, boom_pos)
            time.sleep(0.005)
    finally:
        motor_disconnect()
        print("\n\033[93mTELEOP: Motors Disconnected!\033[0m")


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('teleop_jointspace', anonymous=True)

    joystick_thread = threading.Thread(target=joystick_monitor, daemon=True)
    motor_thread = threading.Thread(target=motor_control, daemon=True)

    joystick_thread.start()
    motor_thread.start()

    while running and not rospy.is_shutdown():
        time.sleep(0.1)

    print("\nTeleop stopped.")


if __name__ == '__main__':
    main()
