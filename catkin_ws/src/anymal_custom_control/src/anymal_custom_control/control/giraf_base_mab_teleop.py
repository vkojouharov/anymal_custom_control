"""Direct joystick teleop for the GIRAF base MAB/MD80 joints only."""

from __future__ import annotations

import traceback

import numpy as np
import rospy

from anymal_custom_control.RRP_kinematic_model import (
    get_boom_length_d3,
    get_boom_motor_rad,
    num_forward_kinematics,
    num_jacobian,
)
from anymal_custom_control.joystick_driver import joystick_connect, joystick_disconnect, joystick_read
from anymal_custom_control.motor_driver import IDS, motor_connect, motor_disconnect, motor_drive

from .giraf_arm_common import (
    ARM_X_SPEED,
    ARM_Y_SPEED,
    ARM_Z_SPEED,
    BOOM_MAX,
    BOOM_MIN,
    CONTROL_LOOP_HZ,
    D3_MIN,
    PITCH_KIN_OFFSET,
    PITCH_MAX,
    PITCH_MIN,
    ROLL_LIMIT,
)


class GirafBaseMabTeleop:
    """Owns joystick input and directly commands the three MD80 base joints."""

    def __init__(self) -> None:
        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", CONTROL_LOOP_HZ))
        self.device_index = int(rospy.get_param("~device_index", 0))
        self.damping = float(rospy.get_param("~damping", 0.05))
        self.motor_kp = float(rospy.get_param("~motor_kp", 200.0))
        self.motor_kd = float(rospy.get_param("~motor_kd", 10.0))
        self.max_torque = float(rospy.get_param("~max_torque", 10.0))
        self.motor_gain_overrides = self._load_motor_gain_overrides()

    @staticmethod
    def _load_motor_gain_overrides() -> dict[int, dict[str, float]]:
        overrides = {}
        for drive_id in IDS.values():
            gains = {}
            for param_name, gain_name in (
                ("kp", "kp"),
                ("kd", "kd"),
                ("max_torque", "max_torque"),
            ):
                ros_param = f"~motor_{drive_id}_{param_name}"
                if rospy.has_param(ros_param):
                    gains[gain_name] = float(rospy.get_param(ros_param))
            if gains:
                overrides[drive_id] = gains
        return overrides

    def _motor_gain(self, drive_id: int, gain_name: str, default_value: float) -> float:
        return self.motor_gain_overrides.get(drive_id, {}).get(gain_name, default_value)

    def _log_motor_gains(self) -> None:
        for joint_name, drive_id in IDS.items():
            rospy.loginfo(
                "MD80 %s id=%d gains: kp=%.1f kd=%.1f max_torque=%.1f",
                joint_name.lower(),
                drive_id,
                self._motor_gain(drive_id, "kp", self.motor_kp),
                self._motor_gain(drive_id, "kd", self.motor_kd),
                self._motor_gain(drive_id, "max_torque", self.max_torque),
            )

    def _task_velocity(self, data: dict[str, float]) -> np.ndarray:
        velocity = np.zeros(3, dtype=float)
        if data["LB"] and data["RB"]:
            velocity[0] = ARM_X_SPEED * data["LY"]
            velocity[1] = -ARM_Y_SPEED * data["LX"]
            if data["RT"] and not data["LT"]:
                velocity[2] = ARM_Z_SPEED * data["RT"]
            elif data["LT"] and not data["RT"]:
                velocity[2] = -ARM_Z_SPEED * data["LT"]
        return velocity

    def _joint_velocity(self, roll: float, pitch: float, d3: float, velocity: np.ndarray) -> np.ndarray:
        jacobian = np.asarray(num_jacobian([roll, pitch + PITCH_KIN_OFFSET, d3]), dtype=float)
        damping_eye = (self.damping**2) * np.eye(jacobian.shape[0], dtype=float)
        return jacobian.T.dot(np.linalg.solve(jacobian.dot(jacobian.T) + damping_eye, velocity))

    @staticmethod
    def _publish_status(roll: float, pitch: float, boom: float, d3: float) -> None:
        transform = np.asarray(num_forward_kinematics([roll, pitch + PITCH_KIN_OFFSET, d3]), dtype=float)
        rospy.loginfo_throttle(
            0.5,
            "base MAB roll=%+.3f pitch=%+.3f boom=%+.3f ee=(%.3f, %.3f, %.3f)",
            roll,
            pitch,
            boom,
            transform[0, 3],
            transform[1, 3],
            transform[2, 3],
        )

    def run(self) -> int:
        roll_pos = 0.0
        pitch_pos = 0.0
        d3_pos = D3_MIN

        js = None
        md80_ctx = None
        rate = rospy.Rate(self.loop_rate_hz)

        try:
            js = joystick_connect(device_index=self.device_index)
            rospy.loginfo("Connected joystick: %s", js["device"].name)

            rospy.loginfo(
                "Connecting base MAB/MD80 joints with defaults: kp=%.1f kd=%.1f max_torque=%.1f",
                self.motor_kp,
                self.motor_kd,
                self.max_torque,
            )
            self._log_motor_gains()
            md80_ctx = motor_connect(
                kp=self.motor_kp,
                kd=self.motor_kd,
                max_torque=self.max_torque,
                gain_overrides=self.motor_gain_overrides,
            )
            rospy.loginfo("Base MAB/MD80 joints connected; hold LB+RB to move")

            while not rospy.is_shutdown():
                data = joystick_read(js)
                if data["XB"]:
                    rospy.logwarn("X pressed; stopping base MAB teleop")
                    break

                task_velocity = self._task_velocity(data)
                joint_velocity = self._joint_velocity(roll_pos, pitch_pos, d3_pos, task_velocity)

                roll_pos += joint_velocity[0] / self.loop_rate_hz
                pitch_pos += joint_velocity[1] / self.loop_rate_hz
                d3_pos += joint_velocity[2] / self.loop_rate_hz

                roll_pos = max(min(roll_pos, ROLL_LIMIT), -ROLL_LIMIT)
                pitch_pos = max(min(pitch_pos, PITCH_MAX), PITCH_MIN)

                boom_pos = get_boom_motor_rad(d3_pos)
                boom_pos = max(min(boom_pos, BOOM_MAX), BOOM_MIN)
                d3_pos = get_boom_length_d3(boom_pos)

                motor_drive(md80_ctx, roll_pos, pitch_pos, boom_pos)
                self._publish_status(roll_pos, pitch_pos, boom_pos, d3_pos)
                rate.sleep()

            return 0
        except Exception as exc:
            rospy.logerr("Base MAB teleop error: %s\n%s", exc, traceback.format_exc())
            return 1
        finally:
            if md80_ctx is not None:
                try:
                    motor_disconnect()
                except Exception as exc:
                    rospy.logwarn("Failed to disconnect base MAB/MD80 joints cleanly: %s", exc)
            if js is not None:
                joystick_disconnect(js)


def main() -> int:
    rospy.init_node("giraf_base_mab_teleop", anonymous=False)
    teleop = GirafBaseMabTeleop()
    return teleop.run()


if __name__ == "__main__":
    raise SystemExit(main())
