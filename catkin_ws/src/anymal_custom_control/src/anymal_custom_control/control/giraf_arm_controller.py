"""ROS controller node for the GIRAF arm J-PARSE stack."""

from __future__ import annotations

import json
import math
import traceback
from dataclasses import dataclass
from threading import Lock

import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float64, String

from anymal_custom_control.RRP_kinematic_model import get_boom_length_d3, get_boom_motor_rad
from anymal_custom_control.RRPRRR_kinematic_model import num_forward_kinematics, num_jacobian
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
from anymal_custom_control.jparse_controller import JParseController, compute_metrics
from anymal_custom_control.motor_driver import motor_connect, motor_disconnect, motor_drive

from .giraf_arm_common import (
    AUTO_GRIPPER_VELOCITY_TOPIC,
    AUTO_TASK_VELOCITY_TOPIC,
    BOOM_MAX,
    BOOM_MIN,
    COMMAND_TIMEOUT_SEC,
    CONTROL_LOOP_HZ,
    DEBUG_TOPIC,
    DEFAULT_COMMAND_SOURCE,
    D3_MIN,
    GRIPPER_SPEED,
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
    STATE_PUBLISH_HZ,
    STATE_TOPIC,
    STOP_TOPIC,
    TELEOP_GRIPPER_VELOCITY_TOPIC,
    TELEOP_TASK_VELOCITY_TOPIC,
    THETA4_DXL_SIGN,
    THETA4_KIN_OFFSET,
    THETA5_DXL_SIGN,
    THETA5_KIN_OFFSET,
    THETA6_KIN_OFFSET,
    TRANSLATION_LOCK_DAMPING,
    clamp_task_velocity,
    zero_task_velocity,
)


def _joint_limit_rad(mid: int, sign: float = 1.0) -> tuple[float, float]:
    lo_tick, hi_tick = ARM_TICK_LIMITS[mid]
    home_tick = ARM_HOME[mid]
    lo_rad = sign * ticks_to_radians(lo_tick - home_tick)
    hi_rad = sign * ticks_to_radians(hi_tick - home_tick)
    return (min(lo_rad, hi_rad), max(lo_rad, hi_rad))


THETA4_MIN, THETA4_MAX = _joint_limit_rad(ARM_IDS[0], THETA4_DXL_SIGN)
THETA5_MIN, THETA5_MAX = _joint_limit_rad(ARM_IDS[1], THETA5_DXL_SIGN)
THETA6_MIN, THETA6_MAX = _joint_limit_rad(ARM_IDS[2])


@dataclass
class _StampedCommand:
    velocity: np.ndarray
    stamp_sec: float = 0.0


class GirafArmController:
    """Owns source selection, J-PARSE control, and motor safety."""

    def __init__(self) -> None:
        self.command_source = rospy.get_param("~command_source", DEFAULT_COMMAND_SOURCE)
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", COMMAND_TIMEOUT_SEC))
        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", CONTROL_LOOP_HZ))
        self.state_publish_hz = float(rospy.get_param("~state_publish_hz", STATE_PUBLISH_HZ))

        self._lock = Lock()
        self._teleop_cmd = _StampedCommand(velocity=zero_task_velocity())
        self._auto_cmd = _StampedCommand(velocity=zero_task_velocity())
        self._teleop_gripper_cmd = _StampedCommand(velocity=np.array([0.0], dtype=float))
        self._auto_gripper_cmd = _StampedCommand(velocity=np.array([0.0], dtype=float))
        self._stop_latched = False
        self._shutdown_requested = False

        self._state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=1, latch=True)
        self._debug_pub = rospy.Publisher(DEBUG_TOPIC, String, queue_size=20)

        rospy.Subscriber(
            TELEOP_TASK_VELOCITY_TOPIC,
            TwistStamped,
            self._teleop_task_velocity_cb,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            AUTO_TASK_VELOCITY_TOPIC,
            TwistStamped,
            self._auto_task_velocity_cb,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            TELEOP_GRIPPER_VELOCITY_TOPIC,
            Float64,
            self._teleop_gripper_velocity_cb,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            AUTO_GRIPPER_VELOCITY_TOPIC,
            Float64,
            self._auto_gripper_velocity_cb,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            STOP_TOPIC,
            Bool,
            self._stop_cb,
            queue_size=1,
            tcp_nodelay=True,
        )

        self._last_status_publish_sec = 0.0

    def _publish_debug(self, level: str, message: str, **extra: object) -> None:
        payload = {
            "stamp_sec": round(rospy.get_time(), 6),
            "level": level,
            "message": message,
        }
        if extra:
            payload["details"] = extra
        encoded = json.dumps(payload, sort_keys=True)
        self._debug_pub.publish(String(data=encoded))
        log_fn = getattr(rospy, f"log{level}", rospy.loginfo)
        log_fn(message)

    @staticmethod
    def _stamp_to_sec(msg: TwistStamped) -> float:
        if msg.header.stamp == rospy.Time():
            return rospy.get_time()
        return msg.header.stamp.to_sec()

    @staticmethod
    def _twist_to_array(msg: TwistStamped) -> np.ndarray:
        return np.array(
            [
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
                msg.twist.angular.x,
                msg.twist.angular.y,
                msg.twist.angular.z,
            ],
            dtype=float,
        )

    def _teleop_task_velocity_cb(self, msg: TwistStamped) -> None:
        with self._lock:
            self._teleop_cmd = _StampedCommand(
                velocity=clamp_task_velocity(self._twist_to_array(msg)),
                stamp_sec=self._stamp_to_sec(msg),
            )

    def _auto_task_velocity_cb(self, msg: TwistStamped) -> None:
        with self._lock:
            self._auto_cmd = _StampedCommand(
                velocity=clamp_task_velocity(self._twist_to_array(msg)),
                stamp_sec=self._stamp_to_sec(msg),
            )

    def _teleop_gripper_velocity_cb(self, msg: Float64) -> None:
        velocity = float(np.clip(msg.data, -GRIPPER_SPEED, GRIPPER_SPEED))
        with self._lock:
            self._teleop_gripper_cmd = _StampedCommand(
                velocity=np.array([velocity], dtype=float),
                stamp_sec=rospy.get_time(),
            )

    def _auto_gripper_velocity_cb(self, msg: Float64) -> None:
        velocity = float(np.clip(msg.data, -GRIPPER_SPEED, GRIPPER_SPEED))
        with self._lock:
            self._auto_gripper_cmd = _StampedCommand(
                velocity=np.array([velocity], dtype=float),
                stamp_sec=rospy.get_time(),
            )

    def _stop_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            if self._stop_latched:
                return
            self._stop_latched = True
            self._shutdown_requested = True
        self._publish_debug("warn", "Stop requested; controller will shut down motors safely")

    def _command_age(self, command: _StampedCommand, now_sec: float) -> float:
        if command.stamp_sec <= 0.0:
            return float("inf")
        return max(0.0, now_sec - command.stamp_sec)

    def _current_command(self, now_sec: float) -> tuple[np.ndarray, float, str, float, float]:
        with self._lock:
            if self.command_source == "auto":
                selected_cmd = self._auto_cmd
                selected_gripper_cmd = self._auto_gripper_cmd
                active_source = "auto"
            else:
                selected_cmd = self._teleop_cmd
                selected_gripper_cmd = self._teleop_gripper_cmd
                active_source = "teleop"

            teleop_age = self._command_age(self._teleop_cmd, now_sec)
            auto_age = self._command_age(self._auto_cmd, now_sec)
            selected_age = self._command_age(selected_cmd, now_sec)
            stop_latched = self._stop_latched

        if stop_latched or selected_age > self.command_timeout_sec:
            return zero_task_velocity(), 0.0, active_source, teleop_age, auto_age

        gripper_velocity = float(selected_gripper_cmd.velocity[0])
        if self._command_age(selected_gripper_cmd, now_sec) > self.command_timeout_sec:
            gripper_velocity = 0.0

        return selected_cmd.velocity.copy(), gripper_velocity, active_source, teleop_age, auto_age

    @staticmethod
    def _dxl_ticks(th4: float, th5: float, th6: float, grip: float) -> list[int]:
        fraction = max(0.0, min(1.0, float(grip)))
        g_id = GRIPPER_IDS[0]
        return [
            ARM_HOME[ARM_IDS[0]] + radians_to_ticks(THETA4_DXL_SIGN * th4),
            ARM_HOME[ARM_IDS[1]] + radians_to_ticks(THETA5_DXL_SIGN * th5),
            ARM_HOME[ARM_IDS[2]] + radians_to_ticks(th6),
            int(round(GRIPPER_OPEN[g_id] - GRIPPER_STROKE * (1.0 - fraction))),
        ]

    @staticmethod
    def _is_pure_rotation_command(velocity: np.ndarray) -> bool:
        linear_norm = float(np.linalg.norm(velocity[:3]))
        angular_norm = float(np.linalg.norm(velocity[3:]))
        return linear_norm <= PURE_ROTATION_LINEAR_EPS and angular_norm > PURE_ROTATION_ANGULAR_EPS

    def _compute_joint_velocity(self, jparse: JParseController, jacobian: np.ndarray, velocity: np.ndarray) -> tuple[np.ndarray, str]:
        if self._is_pure_rotation_command(velocity):
            Jv = jacobian[:3, :]
            Jw = jacobian[3:, :]
            Jv_pinv = JParseController.damped_least_squares(Jv, damping=TRANSLATION_LOCK_DAMPING)
            nullspace_v = np.eye(jacobian.shape[1], dtype=float) - Jv_pinv.dot(Jv)
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
            jacobian,
            singular_direction_gain_position=JPARSE_POS_GAIN,
            singular_direction_gain_angular=JPARSE_ANG_GAIN,
            position_dimensions=3,
            angular_dimensions=3,
        )
        joint_velocity = J_inv.dot(velocity)
        return np.asarray(joint_velocity, dtype=float).reshape(-1), "full"

    def _publish_state(
        self,
        now_sec: float,
        active_source: str,
        selected_velocity: np.ndarray,
        selected_gripper_velocity: float,
        teleop_age: float,
        auto_age: float,
        control_mode: str,
        roll_pos: float,
        pitch_pos: float,
        boom_pos: float,
        theta4_pos: float,
        theta5_pos: float,
        theta6_pos: float,
        gripper_pos: float,
        metrics: dict[str, object],
        transform: np.ndarray,
    ) -> None:
        if now_sec - self._last_status_publish_sec < (1.0 / self.state_publish_hz):
            return

        singular_values = np.asarray(metrics["singular_values"], dtype=float)
        sigma_min = float(np.min(singular_values)) if singular_values.size else 0.0
        sigma_max = float(np.max(singular_values)) if singular_values.size else 0.0

        with self._lock:
            stop_latched = self._stop_latched

        def safe_float(value: float) -> float | None:
            value_f = float(value)
            return value_f if math.isfinite(value_f) else None

        payload = {
            "stamp_sec": round(now_sec, 6),
            "active_source": active_source,
            "command_source_param": self.command_source,
            "stop_latched": stop_latched,
            "teleop_cmd_age_sec": safe_float(round(float(teleop_age), 6)),
            "auto_cmd_age_sec": safe_float(round(float(auto_age), 6)),
            "selected_task_velocity": {
                "vx": float(selected_velocity[0]),
                "vy": float(selected_velocity[1]),
                "vz": float(selected_velocity[2]),
                "wx": float(selected_velocity[3]),
                "wy": float(selected_velocity[4]),
                "wz": float(selected_velocity[5]),
            },
            "selected_gripper_velocity": float(selected_gripper_velocity),
            "arm": {
                "roll": float(roll_pos),
                "pitch": float(pitch_pos),
                "boom": float(boom_pos),
                "th4": float(theta4_pos),
                "th5": float(theta5_pos),
                "th6": float(theta6_pos),
                "grip": float(gripper_pos),
            },
            "end_effector": {
                "x": float(transform[0, 0]),
                "y": float(transform[1, 0]),
                "z": float(transform[2, 0]),
            },
            "metrics": {
                "mode": control_mode,
                "manipulability": float(metrics["manipulability"]),
                "inverse_condition_number": float(metrics["inverse_condition_number"]),
                "sigma_min": sigma_min,
                "sigma_max": sigma_max,
            },
        }

        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self._last_status_publish_sec = now_sec

    def run(self) -> int:
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
        rate = rospy.Rate(self.loop_rate_hz)

        try:
            self._publish_debug("info", "Connecting GIRAF arm controller to motors")
            md80_ctx = motor_connect()
            dxl_ctx = dynamixel_connect(baudrate=1_000_000)
            self._publish_debug("info", "GIRAF arm controller connected to MD80 and Dynamixel hardware")

            while not rospy.is_shutdown():
                with self._lock:
                    if self._shutdown_requested:
                        break

                now_sec = rospy.get_time()
                task_velocity, gripper_velocity, active_source, teleop_age, auto_age = self._current_command(now_sec)

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

                roll_pos += (joint_velocity[0] / self.loop_rate_hz)
                pitch_pos += (joint_velocity[1] / self.loop_rate_hz)
                d3_pos += (joint_velocity[2] / self.loop_rate_hz)
                theta4_pos += (joint_velocity[3] / self.loop_rate_hz)
                theta5_pos += (joint_velocity[4] / self.loop_rate_hz)
                theta6_pos += (joint_velocity[5] / self.loop_rate_hz)
                gripper_pos += (gripper_velocity / self.loop_rate_hz)

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
                dynamixel_drive(
                    dxl_ctx,
                    self._dxl_ticks(theta4_pos, theta5_pos, theta6_pos, gripper_pos),
                )

                self._publish_state(
                    now_sec,
                    active_source,
                    task_velocity,
                    gripper_velocity,
                    teleop_age,
                    auto_age,
                    control_mode,
                    roll_pos,
                    pitch_pos,
                    boom_pos,
                    theta4_pos,
                    theta5_pos,
                    theta6_pos,
                    gripper_pos,
                    metrics,
                    transform,
                )
                rate.sleep()

            self._publish_debug("warn", "Controller loop stopping; shutting down motors")
            return 0
        except Exception as exc:
            self._publish_debug(
                "err",
                "GIRAF arm controller error",
                error=str(exc),
                traceback=traceback.format_exc(),
            )
            return 1
        finally:
            if md80_ctx is not None:
                try:
                    motor_disconnect()
                except Exception as exc:
                    rospy.logwarn("Failed to disconnect MD80 motors cleanly: %s", exc)
            if dxl_ctx is not None:
                try:
                    dynamixel_disconnect(dxl_ctx)
                except Exception as exc:
                    rospy.logwarn("Failed to disconnect Dynamixel motors cleanly: %s", exc)


def main() -> int:
    rospy.init_node("giraf_arm_controller", anonymous=False)
    controller = GirafArmController()
    return controller.run()


if __name__ == "__main__":
    raise SystemExit(main())
