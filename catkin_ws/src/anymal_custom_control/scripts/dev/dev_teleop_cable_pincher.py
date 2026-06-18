#!/usr/bin/env python3
"""Standalone GIRAF arm teleop with local cable-pincher motor constants.

This dev script intentionally keeps Dynamixel IDs, homes, limits, and pincher
open/close targets local so bench calibration can happen without editing the
shared control table.
"""

from __future__ import annotations

import math
import time
import traceback
from dataclasses import dataclass

import numpy as np
import rospy
from dynamixel_sdk import COMM_SUCCESS

from anymal_custom_control.RRP_kinematic_model import get_boom_length_d3, get_boom_motor_rad
from anymal_custom_control.RRPRRR_kinematic_model import num_forward_kinematics, num_jacobian
from anymal_custom_control.control.giraf_arm_common import (
    ARM_WY_SPEED,
    ARM_WZ_SPEED,
    ARM_X_SPEED,
    ARM_Y_SPEED,
    ARM_Z_SPEED,
    BOOM_MAX,
    BOOM_MIN,
    CONTROL_LOOP_HZ,
    D3_MIN,
    JPARSE_ANG_GAIN,
    JPARSE_GAMMA,
    JPARSE_POS_GAIN,
    PITCH_KIN_OFFSET,
    PITCH_MAX,
    PITCH_MIN,
    PURE_ROTATION_ANGULAR_EPS,
    PURE_ROTATION_LINEAR_EPS,
    QDOT_LIMITS,
    ROLL_LIMIT,
    TELEOP_PUBLISH_HZ,
    THETA4_DXL_SIGN,
    THETA4_KIN_OFFSET,
    THETA5_DXL_SIGN,
    THETA5_KIN_OFFSET,
    THETA6_KIN_OFFSET,
    TRANSLATION_LOCK_DAMPING,
)
from anymal_custom_control.dynamixel.controller import DynamixelController
from anymal_custom_control.joystick_driver import joystick_connect, joystick_disconnect, joystick_read
from anymal_custom_control.jparse_controller import JParseController, compute_metrics
from anymal_custom_control.motor_driver import motor_connect, motor_disconnect, motor_drive


# ---------------------------------------------------------------------------
# Edit these values for dev calibration.
# ---------------------------------------------------------------------------

DXL_PORT = "/dev/ttyUSB0"
DXL_BAUDRATE = 1_000_000
DXL_PROTOCOL = 2.0
DXL_REBOOT_ON_CONNECT = True
DXL_SETTLE_SEC = 2.0

# Wrist Dynamixels used by the GIRAF arm controller.
ARM_IDS = (11, 12, 13)
ARM_HOME = {
    11: 3075,
    12: 2003,
    13: 3075,
}
ARM_TICK_LIMITS = {
    11: (1030, 5000),
    12: (1003, 3003),
    13: (1000, 5000),
}

# Cable pinchers. Motor 14 is deliberately not configured or commanded here.
PINCHER_IDS = (1, 2)
PINCHER_OPEN = {
    1: 3000,
    2: 1000,
}
PINCHER_CLOSED = {
    1: 4200,
    2: -100,

}
PINCHER_TICK_LIMITS = {
    1: (0, 4095),
    2: (-1000, 4095),
}
PINCHER_PWM_LIMIT = 700

ALL_DXL_IDS = ARM_IDS + PINCHER_IDS
GOAL_TICK_LIMITS = {
    **ARM_TICK_LIMITS,
    **PINCHER_TICK_LIMITS,
}


# XH-series Protocol 2.0 register constants. Kept local to avoid importing the
# project control table in this dev runner.
OPERATING_MODE = (11, 1)
PWM_LIMIT = (36, 2)
TORQUE_ENABLE = (64, 1)
GOAL_POSITION = (116, 4)
OP_EXTENDED_POSITION = 4
TICKS_PER_REV = 4096


def radians_to_ticks(rad: float) -> int:
    return int(round(rad * TICKS_PER_REV / (2 * math.pi)))


def ticks_to_radians(ticks: float) -> float:
    return ticks * (2 * math.pi) / TICKS_PER_REV


def clamp_goal_ticks(mid: int, ticks: int) -> int:
    limits = GOAL_TICK_LIMITS.get(mid)
    if limits is None:
        return int(ticks)
    lo, hi = limits
    return int(min(max(int(ticks), lo), hi))


def dxl_connect() -> dict[str, object]:
    controller = DynamixelController(DXL_PORT, DXL_BAUDRATE, DXL_PROTOCOL)

    if DXL_REBOOT_ON_CONNECT:
        for mid in ALL_DXL_IDS:
            comm, err = controller.packet_handler.reboot(controller.port_handler, mid)
            if comm != COMM_SUCCESS:
                rospy.logwarn(
                    "[dxl] reboot ID %s: %s",
                    mid,
                    controller.packet_handler.getTxRxResult(comm),
                )
            elif err != 0:
                rospy.logwarn(
                    "[dxl] reboot ID %s packet error: %s",
                    mid,
                    controller.packet_handler.getRxPacketError(err),
                )
        time.sleep(DXL_SETTLE_SEC)

    for mid in ALL_DXL_IDS:
        controller.WRITE(mid, TORQUE_ENABLE, 0)

    for mid in ALL_DXL_IDS:
        if not controller.WRITE(mid, OPERATING_MODE, OP_EXTENDED_POSITION):
            controller.close()
            raise RuntimeError(f"Failed to set operating mode on motor {mid}")
        mode_readback = controller.READ(mid, OPERATING_MODE)
        if mode_readback != OP_EXTENDED_POSITION:
            controller.close()
            raise RuntimeError(
                f"Motor {mid} operating mode verify failed: "
                f"wrote {OP_EXTENDED_POSITION}, read {mode_readback}"
            )

    for mid in PINCHER_IDS:
        controller.WRITE(mid, PWM_LIMIT, int(PINCHER_PWM_LIMIT))

    for mid in ALL_DXL_IDS:
        if not controller.WRITE(mid, TORQUE_ENABLE, 1):
            controller.close()
            raise RuntimeError(f"Failed to enable torque on motor {mid}")

    return {
        "controller": controller,
        "sync_write_pos": controller.make_sync_write(GOAL_POSITION),
    }


def dxl_drive(ctx: dict[str, object], ticks_by_id: dict[int, int]) -> bool:
    sync_write = ctx["sync_write_pos"]
    controller = ctx["controller"]

    for mid in ALL_DXL_IDS:
        ticks = clamp_goal_ticks(mid, ticks_by_id[mid])
        if not sync_write.addParam(mid, ticks.to_bytes(4, "little", signed=True)):
            sync_write.clearParam()
            rospy.logerr("[dxl] sync-write addParam failed for motor %s", mid)
            return False

    comm = sync_write.txPacket()
    sync_write.clearParam()
    if comm != COMM_SUCCESS:
        rospy.logerr(
            "[dxl] sync-write txPacket: %s",
            controller.packet_handler.getTxRxResult(comm),
        )
        return False
    return True


def dxl_disconnect(ctx: dict[str, object]) -> None:
    controller = ctx.get("controller")
    if controller is None:
        return
    for mid in ALL_DXL_IDS:
        try:
            controller.WRITE(mid, TORQUE_ENABLE, 0)
        except Exception as exc:
            rospy.logwarn("[dxl] disconnect: failed to disable motor %s: %s", mid, exc)
    controller.close()


def joint_limit_rad(mid: int, sign: float = 1.0) -> tuple[float, float]:
    lo_tick, hi_tick = ARM_TICK_LIMITS[mid]
    home_tick = ARM_HOME[mid]
    lo_rad = sign * ticks_to_radians(lo_tick - home_tick)
    hi_rad = sign * ticks_to_radians(hi_tick - home_tick)
    return (min(lo_rad, hi_rad), max(lo_rad, hi_rad))


THETA4_MIN, THETA4_MAX = joint_limit_rad(ARM_IDS[0], THETA4_DXL_SIGN)
THETA5_MIN, THETA5_MAX = joint_limit_rad(ARM_IDS[1], THETA5_DXL_SIGN)
THETA6_MIN, THETA6_MAX = joint_limit_rad(ARM_IDS[2])


@dataclass
class PincherState:
    closed: bool = False

    def goal_ticks(self, mid: int) -> int:
        return PINCHER_CLOSED[mid] if self.closed else PINCHER_OPEN[mid]


class DevTeleopCablePincher:
    def __init__(self) -> None:
        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", CONTROL_LOOP_HZ))
        self.joystick_rate_hz = float(rospy.get_param("~joystick_rate_hz", TELEOP_PUBLISH_HZ))
        self.device_index = int(rospy.get_param("~device_index", 0))

        self.pincher_states = {mid: PincherState(closed=False) for mid in PINCHER_IDS}

    @staticmethod
    def _is_pure_rotation_command(velocity: np.ndarray) -> bool:
        linear_norm = float(np.linalg.norm(velocity[:3]))
        angular_norm = float(np.linalg.norm(velocity[3:]))
        return linear_norm <= PURE_ROTATION_LINEAR_EPS and angular_norm > PURE_ROTATION_ANGULAR_EPS

    def _compute_joint_velocity(
        self,
        jparse: JParseController,
        jacobian: np.ndarray,
        velocity: np.ndarray,
    ) -> tuple[np.ndarray, str]:
        if self._is_pure_rotation_command(velocity):
            jv = jacobian[:3, :]
            jw = jacobian[3:, :]
            jv_pinv = JParseController.damped_least_squares(jv, damping=TRANSLATION_LOCK_DAMPING)
            nullspace_v = np.eye(jacobian.shape[1], dtype=float) - jv_pinv.dot(jv)
            jw_locked = jw.dot(nullspace_v)
            jw_locked_inv = jparse.compute(
                jw_locked,
                singular_direction_gain_angular=JPARSE_ANG_GAIN,
                angular_dimensions=3,
            )
            joint_velocity = nullspace_v.dot(jw_locked_inv.dot(velocity[3:]))
            return np.asarray(joint_velocity, dtype=float).reshape(-1), "rot-lock"

        j_inv = jparse.compute(
            jacobian,
            singular_direction_gain_position=JPARSE_POS_GAIN,
            singular_direction_gain_angular=JPARSE_ANG_GAIN,
            position_dimensions=3,
            angular_dimensions=3,
        )
        joint_velocity = j_inv.dot(velocity)
        return np.asarray(joint_velocity, dtype=float).reshape(-1), "full"

    @staticmethod
    def _build_task_velocity(data: dict[str, float]) -> np.ndarray:
        velocity = np.zeros(6, dtype=float)
        if data["LB"] and data["RB"]:
            velocity[0] = ARM_X_SPEED * data["LY"]
            velocity[1] = -ARM_Y_SPEED * data["LX"]
            if data["RT"] and not data["LT"]:
                velocity[2] = ARM_Z_SPEED * data["RT"]
            elif data["LT"] and not data["RT"]:
                velocity[2] = -ARM_Z_SPEED * data["LT"]
            velocity[4] = ARM_WY_SPEED * data["RY"]
            velocity[5] = -ARM_WZ_SPEED * data["RX"]
        return velocity

    def _dxl_ticks(
        self,
        theta4: float,
        theta5: float,
        theta6: float,
    ) -> dict[int, int]:
        ticks = {
            ARM_IDS[0]: ARM_HOME[ARM_IDS[0]] + radians_to_ticks(THETA4_DXL_SIGN * theta4),
            ARM_IDS[1]: ARM_HOME[ARM_IDS[1]] + radians_to_ticks(THETA5_DXL_SIGN * theta5),
            ARM_IDS[2]: ARM_HOME[ARM_IDS[2]] + radians_to_ticks(theta6),
        }
        for mid in PINCHER_IDS:
            ticks[mid] = self.pincher_states[mid].goal_ticks(mid)
        return ticks

    def _handle_pincher_toggles(self, data: dict[str, float], prev_dpad_x: float) -> float:
        dpad_x = data["DPAD_X"]
        left_pressed = dpad_x < -0.5 and prev_dpad_x >= -0.5
        right_pressed = dpad_x > 0.5 and prev_dpad_x <= 0.5

        if left_pressed:
            mid = 1
            self.pincher_states[mid].closed = not self.pincher_states[mid].closed
            state = "closed" if self.pincher_states[mid].closed else "open"
            rospy.loginfo("D-pad left toggled pincher ID %s to %s", mid, state)

        if right_pressed:
            mid = 2
            self.pincher_states[mid].closed = not self.pincher_states[mid].closed
            state = "closed" if self.pincher_states[mid].closed else "open"
            rospy.loginfo("D-pad right toggled pincher ID %s to %s", mid, state)

        return dpad_x

    def run(self) -> int:
        roll_pos = 0.0
        pitch_pos = 0.0
        d3_pos = D3_MIN
        theta4_pos = 0.0
        theta5_pos = 0.0
        theta6_pos = 0.0

        md80_ctx = None
        dxl_ctx = None
        js = None
        jparse = JParseController(gamma=JPARSE_GAMMA)
        rate = rospy.Rate(self.loop_rate_hz)
        joystick_stride = max(1, int(round(self.loop_rate_hz / self.joystick_rate_hz)))
        loop_count = 0
        latest_data = None
        prev_dpad_x = 0.0
        prev_x_button = 0

        try:
            rospy.loginfo("Connecting joystick")
            js = joystick_connect(device_index=self.device_index)
            rospy.loginfo("Connected joystick: %s", js["device"].name)

            rospy.loginfo("Connecting MD80 and local Dynamixel dev hardware")
            md80_ctx = motor_connect()
            dxl_ctx = dxl_connect()
            rospy.loginfo("Dev teleop connected; pinchers start open")

            while not rospy.is_shutdown():
                if latest_data is None or loop_count % joystick_stride == 0:
                    latest_data = joystick_read(js)
                    prev_dpad_x = self._handle_pincher_toggles(latest_data, prev_dpad_x)

                    if latest_data["XB"] and not prev_x_button:
                        rospy.logwarn("X pressed; stopping dev teleop")
                        break
                    prev_x_button = latest_data["XB"]

                task_velocity = self._build_task_velocity(latest_data)

                joint_coords = [
                    roll_pos,
                    pitch_pos + PITCH_KIN_OFFSET,
                    d3_pos,
                    theta4_pos + THETA4_KIN_OFFSET,
                    theta5_pos + THETA5_KIN_OFFSET,
                    theta6_pos + THETA6_KIN_OFFSET,
                ]
                jacobian = np.asarray(num_jacobian(joint_coords), dtype=float)
                joint_velocity, control_mode = self._compute_joint_velocity(jparse, jacobian, task_velocity)
                joint_velocity = np.clip(joint_velocity, -QDOT_LIMITS, QDOT_LIMITS)

                roll_pos += joint_velocity[0] / self.loop_rate_hz
                pitch_pos += joint_velocity[1] / self.loop_rate_hz
                d3_pos += joint_velocity[2] / self.loop_rate_hz
                theta4_pos += joint_velocity[3] / self.loop_rate_hz
                theta5_pos += joint_velocity[4] / self.loop_rate_hz
                theta6_pos += joint_velocity[5] / self.loop_rate_hz

                roll_pos = max(min(roll_pos, ROLL_LIMIT), -ROLL_LIMIT)
                pitch_pos = max(min(pitch_pos, PITCH_MAX), PITCH_MIN)
                d3_pos = max(d3_pos, D3_MIN)
                theta4_pos = max(min(theta4_pos, THETA4_MAX), THETA4_MIN)
                theta5_pos = max(min(theta5_pos, THETA5_MAX), THETA5_MIN)
                theta6_pos = max(min(theta6_pos, THETA6_MAX), THETA6_MIN)

                boom_pos = get_boom_motor_rad(d3_pos)
                boom_pos = max(min(boom_pos, BOOM_MAX), BOOM_MIN)
                d3_pos = get_boom_length_d3(boom_pos)

                state_joint_coords = [
                    roll_pos,
                    pitch_pos + PITCH_KIN_OFFSET,
                    d3_pos,
                    theta4_pos + THETA4_KIN_OFFSET,
                    theta5_pos + THETA5_KIN_OFFSET,
                    theta6_pos + THETA6_KIN_OFFSET,
                ]
                transform = np.asarray(num_forward_kinematics(state_joint_coords), dtype=float)
                metrics = compute_metrics(np.asarray(num_jacobian(state_joint_coords), dtype=float))

                motor_drive(md80_ctx, roll_pos, pitch_pos, boom_pos)
                dxl_drive(dxl_ctx, self._dxl_ticks(theta4_pos, theta5_pos, theta6_pos))

                if loop_count % int(max(1, round(self.loop_rate_hz))) == 0:
                    rospy.logdebug(
                        "mode=%s x=%.3f y=%.3f z=%.3f manipulability=%.4f",
                        control_mode,
                        transform[0, 0],
                        transform[1, 0],
                        transform[2, 0],
                        metrics["manipulability"],
                    )

                loop_count += 1
                rate.sleep()

            return 0
        except Exception as exc:
            rospy.logerr("Dev teleop cable pincher error: %s\n%s", exc, traceback.format_exc())
            return 1
        finally:
            if md80_ctx is not None:
                try:
                    motor_drive(md80_ctx, roll_pos, pitch_pos, get_boom_motor_rad(d3_pos))
                except Exception:
                    pass
                try:
                    motor_disconnect()
                except Exception as exc:
                    rospy.logwarn("Failed to disconnect MD80 motors cleanly: %s", exc)
            if dxl_ctx is not None:
                try:
                    dxl_disconnect(dxl_ctx)
                except Exception as exc:
                    rospy.logwarn("Failed to disconnect Dynamixel motors cleanly: %s", exc)
            if js is not None:
                joystick_disconnect(js)


def main() -> int:
    rospy.init_node("dev_teleop_cable_pincher", anonymous=False)
    teleop = DevTeleopCablePincher()
    return teleop.run()


if __name__ == "__main__":
    raise SystemExit(main())
