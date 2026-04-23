#!/usr/bin/env python3
"""Full GIRAF arm (wrist + gripper) task-space teleop using J-PARSE.

This mirrors RUN_arm_wrist_RMRC_teleop.py but replaces the DLS inverse
with a local Python 3.8-compatible J-PARSE implementation.
"""

import os
import signal
import threading
import time
import traceback

import numpy as np
import rospy

from anymal_custom_control.joystick_driver import (
    joystick_connect,
    joystick_disconnect,
    joystick_read,
)
from anymal_custom_control.motor_driver import (
    motor_connect,
    motor_drive,
    motor_disconnect,
)
from anymal_custom_control.dynamixel import (
    ARM_HOME,
    ARM_IDS,
    ARM_TICK_LIMITS,
    GRIPPER_IDS,
    GRIPPER_OPEN,
    GRIPPER_STROKE,
    dynamixel_connect,
    dynamixel_disconnect,
    dynamixel_drive,
    radians_to_ticks,
    ticks_to_radians,
)
from anymal_custom_control.jparse_controller import (
    JParseController,
    compute_metrics,
)
from anymal_custom_control.RRPRRR_kinematic_model import (
    num_forward_kinematics,
    num_jacobian,
)
from anymal_custom_control.RRP_kinematic_model import (
    get_boom_length_d3,
    get_boom_motor_rad,
)

# ── Tuning constants ────────────────────────────────────────────────────────

DT = 0.005            # 200 Hz control loop

ARM_X_SPEED  = 0.2    # m/s
ARM_Y_SPEED  = 0.2    # m/s
ARM_Z_SPEED  = 0.1    # m/s
ARM_WY_SPEED = 0.5    # rad/s
ARM_WZ_SPEED = 0.5    # rad/s

JPARSE_GAMMA = 0.10
JPARSE_POS_GAIN = 1.0
JPARSE_ANG_GAIN = 1.0
QDOT_LIMITS = np.array([3.0, 2.0, 0.5, 3.0, 3.0, 3.0], dtype=float)
TRANSLATION_LOCK_DAMPING = 0.05
PURE_ROTATION_LINEAR_EPS = 1e-6
PURE_ROTATION_ANGULAR_EPS = 1e-4

GRIPPER_SPEED = 2.0   # fraction per second (1.0 = open, 0.0 = closed)

ROLL_LIMIT = np.pi / 2
PITCH_MIN  = 0.0
PITCH_MAX  = np.pi / 2
D3_MIN     = 0.310
BOOM_MIN   = -30.0
BOOM_MAX   =   0.0

PITCH_KIN_OFFSET  = np.pi / 2
THETA4_KIN_OFFSET = -np.pi / 2
THETA5_KIN_OFFSET = -np.pi / 2
THETA6_KIN_OFFSET = np.pi / 2
# THETA4_KIN_OFFSET = np.pi / 2
# THETA5_KIN_OFFSET = 5 * np.pi / 6

THETA5_DXL_SIGN = -1.0  # motor 12 is mounted reversed; keep kinematic theta5 unchanged


def _joint_limit_rad(mid, sign=1.0):
    lo_tick, hi_tick = ARM_TICK_LIMITS[mid]
    home_tick = ARM_HOME[mid]
    lo_rad = sign * ticks_to_radians(lo_tick - home_tick)
    hi_rad = sign * ticks_to_radians(hi_tick - home_tick)
    return (min(lo_rad, hi_rad), max(lo_rad, hi_rad))


THETA4_MIN, THETA4_MAX = _joint_limit_rad(ARM_IDS[0])
THETA5_MIN, THETA5_MAX = _joint_limit_rad(ARM_IDS[1], THETA5_DXL_SIGN)
THETA6_MIN, THETA6_MAX = _joint_limit_rad(ARM_IDS[2])

# ── Shared state ────────────────────────────────────────────────────────────

joystick_data = {
    "LX": 0, "LY": 0, "RX": 0, "RY": 0,
    "LT": 0, "RT": 0,
    "AB": 0, "BB": 0, "XB": 0,
    "LB": 0, "RB": 0,
}
joystick_lock = threading.Lock()
js_handle = [None]

running = True
running_lock = threading.Lock()

arm_state = {
    "roll": 0.0, "pitch": 0.0, "boom": 0.0,
    "th4": 0.0, "th5": 0.0, "th6": 0.0,
    "grip": 1.0,
    "ex": 0.0, "ey": 0.0, "ez": 0.0,
    "w": 0.0, "icn": 0.0, "sigma_min": 0.0, "sigma_max": 0.0,
    "mode": "full",
}
arm_state_lock = threading.Lock()


# ── Signal handling ─────────────────────────────────────────────────────────

_sigint_count = 0


def _sigint_handler(signum, frame):
    global _sigint_count, running
    _sigint_count += 1
    if _sigint_count >= 2:
        print("\nForce quit — motor state may be unsafe.")
        os._exit(1)
    with running_lock:
        running = False


signal.signal(signal.SIGINT, _sigint_handler)


# ── Joystick thread ─────────────────────────────────────────────────────────

def joystick_monitor():
    global joystick_data, running
    js = joystick_connect()
    js_handle[0] = js
    print("\033[93mARM: Joystick Connected!\033[0m")
    prev_xb = 0
    try:
        while running:
            with joystick_lock:
                joystick_data = joystick_read(js)
                xb = joystick_data["XB"]
            if xb and not prev_xb:
                with running_lock:
                    running = False
            prev_xb = xb
            time.sleep(0.005)
    finally:
        joystick_disconnect(js)
        print("\033[93mARM: Joystick Disconnected!\033[0m")


# ── Motor control thread ────────────────────────────────────────────────────

def _dxl_ticks(th4, th5, th6, grip):
    """Combined arm + gripper tick list in ``ctx['all_ids']`` order."""
    f = max(0.0, min(1.0, float(grip)))
    g_id = GRIPPER_IDS[0]
    return [
        ARM_HOME[ARM_IDS[0]] + radians_to_ticks(th4),
        ARM_HOME[ARM_IDS[1]] + radians_to_ticks(THETA5_DXL_SIGN * th5),
        ARM_HOME[ARM_IDS[2]] + radians_to_ticks(th6),
        int(round(GRIPPER_OPEN[g_id] - GRIPPER_STROKE * (1.0 - f))),
    ]


def _is_pure_rotation_command(velocity):
    linear_norm = float(np.linalg.norm(velocity[:3]))
    angular_norm = float(np.linalg.norm(velocity[3:]))
    return linear_norm <= PURE_ROTATION_LINEAR_EPS and angular_norm > PURE_ROTATION_ANGULAR_EPS


def _compute_joint_velocity(jparse, J, velocity):
    """Use translation-priority orientation control for pure rotation commands."""
    if _is_pure_rotation_command(velocity):
        Jv = J[:3, :]
        Jw = J[3:, :]
        Jv_pinv = JParseController.damped_least_squares(Jv, damping=TRANSLATION_LOCK_DAMPING)
        nullspace_v = np.eye(J.shape[1], dtype=float) - Jv_pinv.dot(Jv)
        Jw_locked = Jw.dot(nullspace_v)
        omega_des = velocity[3:]
        Jw_locked_inv = jparse.compute(
            Jw_locked,
            singular_direction_gain_angular=JPARSE_ANG_GAIN,
            angular_dimensions=3,
        )
        joint_velocity = nullspace_v.dot(Jw_locked_inv.dot(omega_des))
        return np.asarray(joint_velocity, dtype=float).reshape(-1), "rot-lock"

    J_inv = jparse.compute(
        J,
        singular_direction_gain_position=JPARSE_POS_GAIN,
        singular_direction_gain_angular=JPARSE_ANG_GAIN,
        position_dimensions=3,
        angular_dimensions=3,
    )
    joint_velocity = J_inv.dot(velocity)
    return np.asarray(joint_velocity, dtype=float).reshape(-1), "full"


def motor_control():
    global joystick_data, running

    roll_pos = 0.0
    pitch_pos = 0.0
    d3_pos = D3_MIN
    theta4_pos = 0.0
    theta5_pos = 0.0
    theta6_pos = 0.0
    gripper_pos = 1.0

    md80_ctx = None
    dxl_ctx = None
    jparse = JParseController(gamma=JPARSE_GAMMA)

    try:
        md80_ctx = motor_connect()
        print("\033[93mARM: MD80 Motors Connected!\033[0m")
        dxl_ctx = dynamixel_connect(baudrate=1_000_000)
        print("\033[93mARM: Dynamixel Motors Connected!\033[0m")

        while running:
            with joystick_lock:
                LX = joystick_data["LX"]
                LY = joystick_data["LY"]
                RX = joystick_data["RX"]
                RY = joystick_data["RY"]
                LT = joystick_data["LT"]
                RT = joystick_data["RT"]
                AB = joystick_data["AB"]
                BB = joystick_data["BB"]
                LB = joystick_data["LB"]
                RB = joystick_data["RB"]

            velocity = np.zeros(6, dtype=float)
            gripper_velocity = 0.0

            if LB and RB:
                velocity[0] = ARM_X_SPEED * LY
                velocity[1] = -ARM_Y_SPEED * LX
                if RT and not LT:
                    velocity[2] = ARM_Z_SPEED * RT
                elif LT and not RT and pitch_pos > 0:
                    velocity[2] = -ARM_Z_SPEED * LT
                velocity[4] = ARM_WY_SPEED * RY
                velocity[5] = -ARM_WZ_SPEED * RX
                if AB and not BB:
                    gripper_velocity = -GRIPPER_SPEED
                elif BB and not AB:
                    gripper_velocity = GRIPPER_SPEED

            joint_coords = [
                roll_pos,
                pitch_pos + PITCH_KIN_OFFSET,
                d3_pos,
                theta4_pos + THETA4_KIN_OFFSET,
                theta5_pos + THETA5_KIN_OFFSET,
                theta6_pos + THETA6_KIN_OFFSET
            ]
            J = np.asarray(num_jacobian(joint_coords), dtype=float)
            joint_velocity, control_mode = _compute_joint_velocity(jparse, J, velocity)
            joint_velocity = np.clip(joint_velocity, -QDOT_LIMITS, QDOT_LIMITS)

            roll_pos += DT * joint_velocity[0]
            pitch_pos += DT * joint_velocity[1]
            d3_pos += DT * joint_velocity[2]
            theta4_pos += DT * joint_velocity[3]
            theta5_pos += DT * joint_velocity[4]
            theta6_pos += DT * joint_velocity[5]
            gripper_pos += DT * gripper_velocity

            roll_pos = max(min(roll_pos, ROLL_LIMIT), -ROLL_LIMIT)
            pitch_pos = max(min(pitch_pos, PITCH_MAX), PITCH_MIN)
            d3_pos = max(d3_pos, D3_MIN)
            theta4_pos = max(min(theta4_pos, THETA4_MAX), THETA4_MIN)
            theta5_pos = max(min(theta5_pos, THETA5_MAX), THETA5_MIN)
            theta6_pos = max(min(theta6_pos, THETA6_MAX), THETA6_MIN)
            gripper_pos = max(0.0, min(1.0, gripper_pos))

            boom_pos = get_boom_motor_rad(d3_pos)
            boom_pos = max(min(boom_pos, BOOM_MAX), BOOM_MIN)
            d3_pos = get_boom_length_d3(boom_pos)

            T = num_forward_kinematics(joint_coords)
            metrics = compute_metrics(J)
            singular_values = metrics["singular_values"]
            sigma_min = float(np.min(singular_values)) if singular_values.size else 0.0
            sigma_max = float(np.max(singular_values)) if singular_values.size else 0.0
            with arm_state_lock:
                arm_state["roll"] = roll_pos
                arm_state["pitch"] = pitch_pos
                arm_state["boom"] = boom_pos
                arm_state["th4"] = theta4_pos
                arm_state["th5"] = theta5_pos
                arm_state["th6"] = theta6_pos
                arm_state["grip"] = gripper_pos
                arm_state["ex"] = float(T[0, 0])
                arm_state["ey"] = float(T[1, 0])
                arm_state["ez"] = float(T[2, 0])
                arm_state["w"] = float(metrics["manipulability"])
                arm_state["icn"] = float(metrics["inverse_condition_number"])
                arm_state["sigma_min"] = sigma_min
                arm_state["sigma_max"] = sigma_max
                arm_state["mode"] = control_mode

            motor_drive(md80_ctx, roll_pos, pitch_pos, boom_pos)
            dynamixel_drive(
                dxl_ctx,
                _dxl_ticks(theta4_pos, theta5_pos, theta6_pos, gripper_pos),
            )
            time.sleep(DT)
    except Exception as e:
        print("\n[arm] motor_control error: %s" % (e,))
        traceback.print_exc()
        with running_lock:
            running = False
    finally:
        if md80_ctx is not None:
            try:
                motor_disconnect()
                print("\033[93mARM: MD80 Motors Disconnected!\033[0m")
            except Exception as e:
                print("[arm] MD80 disconnect error: %s" % (e,))
        if dxl_ctx is not None:
            try:
                dynamixel_disconnect(dxl_ctx)
                print("\033[93mARM: Dynamixel torque OFF, port closed!\033[0m")
            except Exception as e:
                print("[arm] Dynamixel disconnect error: %s" % (e,))


# ── Display ─────────────────────────────────────────────────────────────────

def draw_display(arm):
    arm_xyz = "({:+.3f}, {:+.3f}, {:+.3f})".format(arm["ex"], arm["ey"], arm["ez"])
    lines = [
        "\033[2J\033[H",
        "╔══════════════════════════════════════════════════════════════╗",
        "║          ARM + WRIST TELEOP CONTROLLER  (J-PARSE)           ║",
        "╠══════════════════════════════════════════════════════════════╣",
        "║  ee: {:>26s}                              ║".format(arm_xyz),
        "║  roll:{:+.3f}  pitch:{:+.3f}  boom:{:+.3f}              ║".format(
            arm["roll"], arm["pitch"], arm["boom"]
        ),
        "║  th4: {:+.3f}  th5:  {:+.3f}  th6: {:+.3f}              ║".format(
            arm["th4"], arm["th5"], arm["th6"]
        ),
        "║  gripper: {:5.1f}%                                     ║".format(arm["grip"] * 100.0),
        "╠══════════════════════════════════════════════════════════════╣",
        "║  w: {:+.3f}  icn: {:+.3f}  σmin: {:+.3f}               ║".format(
            arm["w"], arm["icn"], arm["sigma_min"]
        ),
        "║  γ: {:+.2f}  σmax: {:+.3f}                              ║".format(
            JPARSE_GAMMA, arm["sigma_max"]
        ),
        "║  mode: {:<51s}║".format(arm["mode"]),
        "╠══════════════════════════════════════════════════════════════╣",
        "║  LB+RB to move   X to quit                                  ║",
        "╚══════════════════════════════════════════════════════════════╝",
    ]
    print("\n".join(lines), end="", flush=True)


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    global running

    rospy.init_node("arm_wrist_jparse_teleop", anonymous=True)

    joystick_thread = threading.Thread(target=joystick_monitor, daemon=True)
    motor_thread = threading.Thread(target=motor_control)
    joystick_thread.start()
    motor_thread.start()

    while js_handle[0] is None and running:
        time.sleep(0.05)

    try:
        while running and not rospy.is_shutdown():
            with arm_state_lock:
                arm = dict(arm_state)
            draw_display(arm)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        with running_lock:
            running = False
        motor_thread.join(timeout=5.0)
        if motor_thread.is_alive():
            print("Warning: motor thread did not shut down within 5 s — MD80 / Dynamixel state may be unclean.")
        print("Arm-wrist J-PARSE teleop stopped.")


if __name__ == "__main__":
    main()
