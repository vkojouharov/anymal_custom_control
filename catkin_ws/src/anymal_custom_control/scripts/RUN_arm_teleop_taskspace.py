#!/usr/bin/env python3
"""Task-space teleop for MD80 motors via candle_ros.

Joystick commands Cartesian velocity (x, y, z); the inverse Jacobian
converts to joint velocities for the RRP manipulator.

Controls:
    Left stick Y   — X velocity (forward / backward)
    Left stick X   — Y velocity (left / right)
    RT             — Z velocity up
    LT             — Z velocity down
    LB + RB        — safety interlock (BOTH must be held for movement)
    X button       — emergency stop + quit

Prerequisites:
    - candle_ros_node running: rosrun candle_ros candle_ros_node USB 1M
    - Xbox controller plugged in

Usage:
    python3 RUN_arm_teleop_taskspace.py
"""

import os
import signal
import threading
import time

import numpy as np
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
from anymal_custom_control.RRP_kinematic_model import (
    num_forward_kinematics,
    num_jacobian,
    get_boom_motor_rad,
    get_boom_length_d3,
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

DT = 0.005  # control loop timestep (200 Hz)
X_SPEED = 0.2
Y_SPEED = 0.2
Z_SPEED = 0.2   # m/s max Cartesian speed (Z)


def motor_control():
    global joystick_data, running

    # Initial joint coordinates (homed positions)
    roll_pos = 0.0
    pitch_pos = 0.0
    d3_pos = 0.310  # boom fully retracted (homed)

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

            # Build Cartesian velocity from joystick
            velocity = np.zeros((3, 1))
            if LB and RB:
                velocity[0] = X_SPEED * LY  # X velocity
                velocity[1] = -Y_SPEED * LX  # Y velocity
                if RT and not LT:
                    velocity[2] = Z_SPEED * RT
                elif LT and not RT:
                    velocity[2] = -Z_SPEED * LT

            # Inverse Jacobian: Cartesian velocity -> joint velocity
            joint_coords = [roll_pos, pitch_pos + np.pi / 2, d3_pos]
            Jv = num_jacobian(joint_coords)
            try:
                Jv_inv = np.linalg.inv(Jv)
            except np.linalg.LinAlgError:
                # Near singularity — skip this step
                Jv_inv = np.zeros((3, 3))

            joint_velocity = Jv_inv @ velocity

            # Integrate joint velocities
            roll_pos += DT * joint_velocity[0, 0]
            pitch_pos += DT * joint_velocity[1, 0]
            d3_pos += DT * joint_velocity[2, 0]

            # Joint limits
            roll_pos = max(min(roll_pos, np.pi / 2), -np.pi / 2)
            pitch_pos = max(min(pitch_pos, np.pi / 2), 0)

            # Convert d3 to boom motor position
            boom_pos = get_boom_motor_rad(d3_pos)
            boom_pos = max(min(boom_pos, 0), -30)
            d3_pos = get_boom_length_d3(boom_pos)

            # FK for end-effector display
            T = num_forward_kinematics(joint_coords)
            ex, ey, ez = T[0, 3], T[1, 3], T[2, 3]

            print(f"\r  roll:{roll_pos:+.3f}  pitch:{pitch_pos:+.3f}  boom:{boom_pos:+.3f}"
                  f"  ee:({ex:.3f}, {ey:.3f}, {ez:.3f})   ",
                  end='', flush=True)

            motor_drive(ctx, roll_pos, pitch_pos, boom_pos)
            time.sleep(DT)
    finally:
        motor_disconnect()
        print(f"\n\033[93mTELEOP: Motors Disconnected!\033[0m")


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('teleop_taskspace', anonymous=True)

    joystick_thread = threading.Thread(target=joystick_monitor, daemon=True)
    motor_thread = threading.Thread(target=motor_control, daemon=True)

    joystick_thread.start()
    motor_thread.start()

    while running and not rospy.is_shutdown():
        time.sleep(0.1)

    print("\nTeleop stopped.")


if __name__ == '__main__':
    main()
