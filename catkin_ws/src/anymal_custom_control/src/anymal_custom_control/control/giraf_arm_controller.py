"""ROS controller node for the GIRAF arm J-PARSE stack."""

from __future__ import annotations

import json
import math
import time
import traceback
from dataclasses import dataclass
from threading import Lock

import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from anymal_custom_control.dynamixel import (
    ARM_HOME,
    ARM_IDS,
    ARM_TICK_LIMITS,
    GRIPPER_IDS,
    GRIPPER_OPEN,
    GRIPPER_STROKE,
    GOAL_TICK_LIMITS,
    dynamixel_connect,
    dynamixel_disconnect,
    dynamixel_drive,
    dynamixel_read,
    radians_to_ticks,
    ticks_to_radians,
)
from anymal_custom_control.motor_driver import motor_connect, motor_disconnect, motor_drive

from .giraf_arm_core import GirafArmControlCore
from .giraf_arm_common import (
    AUTO_GRIPPER_VELOCITY_TOPIC,
    AUTO_TASK_VELOCITY_TOPIC,
    COMMAND_TIMEOUT_SEC,
    COMMAND_SOURCE_TOPIC,
    CONTROL_LOOP_HZ,
    DEBUG_TOPIC,
    DEFAULT_COMMAND_SOURCE,
    GRIPPER_SPEED,
    READINESS_TOPIC,
    STATE_PUBLISH_HZ,
    STATE_TOPIC,
    STOP_TOPIC,
    TELEOP_GRIPPER_VELOCITY_TOPIC,
    TELEOP_TASK_VELOCITY_TOPIC,
    THETA4_DXL_SIGN,
    THETA5_DXL_SIGN,
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

MD80_GAIN_OVERRIDES = {
    # Roll motor is prone to vibration at the default 1000/50/25 impedance gains.
    11: {"kp": 200.0, "kd": 10.0, "max_torque": 10.0},
}


@dataclass
class _StampedCommand:
    velocity: np.ndarray
    received_monotonic_sec: float = 0.0
    valid: bool = False


class GirafArmController:
    """Owns source selection, J-PARSE control, and motor safety."""

    def __init__(self) -> None:
        backend_explicit = rospy.has_param("~backend")
        self.backend = str(rospy.get_param("~backend", "hardware")).strip().lower()
        if self.backend not in {"dry_run", "hardware"}:
            raise ValueError("~backend must be 'dry_run' or 'hardware'")
        # Existing launchers historically invoked this node with no parameters
        # and therefore selected hardware immediately. Preserve that behavior;
        # an explicitly selected hardware backend must use the new confirmation
        # gate in run_giraf_robot_side.py.
        self._legacy_hardware_startup = self.backend == "hardware" and not backend_explicit
        self.home_confirmed = bool(
            rospy.get_param("~home_confirmed", self._legacy_hardware_startup)
        )
        self.command_source = self._normalized_command_source(
            rospy.get_param("~command_source", DEFAULT_COMMAND_SOURCE),
            DEFAULT_COMMAND_SOURCE,
        )
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", COMMAND_TIMEOUT_SEC))
        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", CONTROL_LOOP_HZ))
        self.state_publish_hz = float(rospy.get_param("~state_publish_hz", STATE_PUBLISH_HZ))
        self.home_timeout_sec = float(rospy.get_param("~home_timeout_sec", 30.0))
        self.home_dxl_rate_ticks_sec = float(rospy.get_param("~home_dxl_rate_ticks_sec", 150.0))
        self.home_dxl_tolerance_ticks = int(rospy.get_param("~home_dxl_tolerance_ticks", 25))
        self.home_stable_sec = float(rospy.get_param("~home_stable_sec", 0.5))
        self.task_velocity_limits = np.asarray(
            rospy.get_param("~task_velocity_limits", [0.2, 0.2, 0.1, 1.0, 1.0, 1.0]),
            dtype=float,
        )
        if self.task_velocity_limits.shape != (6,) or not np.all(np.isfinite(self.task_velocity_limits)):
            raise ValueError("~task_velocity_limits must contain six finite values")
        if np.any(self.task_velocity_limits <= 0.0):
            raise ValueError("~task_velocity_limits values must be positive")

        self._lock = Lock()
        self._teleop_cmd = _StampedCommand(velocity=zero_task_velocity())
        self._auto_cmd = _StampedCommand(velocity=zero_task_velocity())
        self._teleop_gripper_cmd = _StampedCommand(velocity=np.array([0.0], dtype=float))
        self._auto_gripper_cmd = _StampedCommand(velocity=np.array([0.0], dtype=float))
        self._stop_latched = False
        self._shutdown_requested = False
        self._ready = False
        self._homed = self.backend == "dry_run"
        self._hardware_connected = False
        self._md80_enabled = False
        self._dynamixel_torque_enabled = False
        self._active_faults: list[str] = []

        self._state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=1, latch=True)
        self._debug_pub = rospy.Publisher(DEBUG_TOPIC, String, queue_size=20)
        self._readiness_pub = rospy.Publisher(READINESS_TOPIC, String, queue_size=1, latch=True)
        self._dry_joint_state_pub = (
            rospy.Publisher("/md80/joint_states", JointState, queue_size=1)
            if self.backend == "dry_run"
            else None
        )

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
        rospy.Subscriber(
            COMMAND_SOURCE_TOPIC,
            String,
            self._command_source_cb,
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
    def _normalized_command_source(value: object, fallback: str | None = None) -> str:
        source = str(value).strip().lower()
        if source in {"teleop", "auto"}:
            return source
        if fallback is not None:
            return fallback
        raise ValueError(f"Unsupported command source: {value!r}")

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
        velocity = self._twist_to_array(msg)
        valid = bool(np.all(np.isfinite(velocity)))
        if not valid:
            velocity = zero_task_velocity()
        with self._lock:
            self._teleop_cmd = _StampedCommand(
                velocity=clamp_task_velocity(velocity),
                received_monotonic_sec=time.monotonic(),
                valid=valid,
            )
        if not valid:
            self._publish_debug("warn", "Rejected non-finite teleop task command; holding position")

    def _auto_task_velocity_cb(self, msg: TwistStamped) -> None:
        velocity = self._twist_to_array(msg)
        valid = bool(np.all(np.isfinite(velocity)))
        if not valid:
            velocity = zero_task_velocity()
        with self._lock:
            self._auto_cmd = _StampedCommand(
                velocity=clamp_task_velocity(velocity),
                received_monotonic_sec=time.monotonic(),
                valid=valid,
            )
        if not valid:
            self._publish_debug("warn", "Rejected non-finite autonomous task command; holding position")

    def _teleop_gripper_velocity_cb(self, msg: Float64) -> None:
        valid = math.isfinite(float(msg.data))
        velocity = float(np.clip(msg.data, -GRIPPER_SPEED, GRIPPER_SPEED)) if valid else 0.0
        with self._lock:
            self._teleop_gripper_cmd = _StampedCommand(
                velocity=np.array([velocity], dtype=float),
                received_monotonic_sec=time.monotonic(),
                valid=valid,
            )
        if not valid:
            self._publish_debug("warn", "Rejected non-finite teleop gripper command; holding position")

    def _auto_gripper_velocity_cb(self, msg: Float64) -> None:
        valid = math.isfinite(float(msg.data))
        velocity = float(np.clip(msg.data, -GRIPPER_SPEED, GRIPPER_SPEED)) if valid else 0.0
        with self._lock:
            self._auto_gripper_cmd = _StampedCommand(
                velocity=np.array([velocity], dtype=float),
                received_monotonic_sec=time.monotonic(),
                valid=valid,
            )
        if not valid:
            self._publish_debug("warn", "Rejected non-finite autonomous gripper command; holding position")

    def _stop_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            if self._stop_latched:
                return
            self._stop_latched = True
            self._shutdown_requested = True
        self._publish_debug("warn", "Stop requested; controller will shut down motors safely")

    def _command_source_cb(self, msg: String) -> None:
        try:
            source = self._normalized_command_source(msg.data)
        except ValueError:
            self._publish_debug("warn", "Ignoring unsupported command source", command_source=msg.data)
            return

        with self._lock:
            previous = self.command_source
            self.command_source = source

        if source != previous:
            self._publish_debug("info", "Command source changed", previous=previous, command_source=source)

    @staticmethod
    def _command_age(command: _StampedCommand, now_monotonic_sec: float) -> float:
        if command.received_monotonic_sec <= 0.0:
            return float("inf")
        return max(0.0, now_monotonic_sec - command.received_monotonic_sec)

    def _current_command(self, now_monotonic_sec: float) -> tuple[np.ndarray, float, str, float, float, bool]:
        with self._lock:
            if self.command_source == "auto":
                selected_cmd = self._auto_cmd
                selected_gripper_cmd = self._auto_gripper_cmd
                active_source = "auto"
            else:
                selected_cmd = self._teleop_cmd
                selected_gripper_cmd = self._teleop_gripper_cmd
                active_source = "teleop"

            teleop_age = self._command_age(self._teleop_cmd, now_monotonic_sec)
            auto_age = self._command_age(self._auto_cmd, now_monotonic_sec)
            selected_age = self._command_age(selected_cmd, now_monotonic_sec)
            stop_latched = self._stop_latched
            ready = self._ready

        watchdog_stale = selected_age > self.command_timeout_sec
        if stop_latched or watchdog_stale or not ready:
            return zero_task_velocity(), 0.0, active_source, teleop_age, auto_age, watchdog_stale

        gripper_velocity = float(selected_gripper_cmd.velocity[0])
        if self._command_age(selected_gripper_cmd, now_monotonic_sec) > self.command_timeout_sec:
            gripper_velocity = 0.0

        return selected_cmd.velocity.copy(), gripper_velocity, active_source, teleop_age, auto_age, watchdog_stale

    def _maybe_become_ready(self, now_monotonic_sec: float, feedback_valid: bool) -> None:
        with self._lock:
            if self._ready or self._stop_latched or not self._homed or not feedback_valid:
                return
            command = self._teleop_cmd if self.command_source == "teleop" else self._auto_cmd
            age = self._command_age(command, now_monotonic_sec)
            is_zero = bool(np.max(np.abs(command.velocity)) <= 1e-9)
            if command.valid and age <= self.command_timeout_sec and is_zero:
                self._ready = True
        if self._ready:
            self._publish_debug("info", "Controller ready after receiving a fresh zero command", backend=self.backend)

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
        feedback_valid: bool,
        watchdog_stale: bool,
    ) -> None:
        if now_sec - self._last_status_publish_sec < (1.0 / self.state_publish_hz):
            return

        singular_values = np.asarray(metrics["singular_values"], dtype=float)
        sigma_min = float(np.min(singular_values)) if singular_values.size else 0.0
        sigma_max = float(np.max(singular_values)) if singular_values.size else 0.0

        with self._lock:
            stop_latched = self._stop_latched
            ready = self._ready

        if ready:
            reported_source = active_source
        elif self.backend == "hardware":
            reported_source = "homing"
        else:
            reported_source = "initializing"

        def safe_float(value: float) -> float | None:
            value_f = float(value)
            return value_f if math.isfinite(value_f) else None

        payload = {
            "stamp_sec": round(now_sec, 6),
            "active_source": reported_source,
            "command_source_param": self.command_source if ready else reported_source,
            "stop_latched": stop_latched,
            "backend": self.backend,
            "ready": ready,
            "state_semantics": "integrated_command_coordinates",
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
                "x": float(transform[0, 3]),
                "y": float(transform[1, 3]),
                "z": float(transform[2, 3]),
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
        selected_age = teleop_age if active_source == "teleop" else auto_age
        readiness = {
            "stamp_sec": round(now_sec, 6),
            "backend": self.backend,
            "ready": ready,
            "hardware_connected": self._hardware_connected,
            "feedback_valid": bool(feedback_valid),
            "homed": self._homed,
            "command_source": reported_source,
            "watchdog_state": "stale" if watchdog_stale else "fresh",
            "md80_enabled": self._md80_enabled,
            "dynamixel_torque_enabled": self._dynamixel_torque_enabled,
            "stop_latched": stop_latched,
            "estop_state": "not_available_in_software",
            "active_faults": list(self._active_faults),
            "last_command_receipt_age_sec": safe_float(round(float(selected_age), 6)),
        }
        self._readiness_pub.publish(String(data=json.dumps(readiness, sort_keys=True)))
        self._last_status_publish_sec = now_sec

    def _publish_dry_joint_state(
        self,
        now_sec: float,
        roll: float,
        pitch: float,
        boom: float,
        roll_velocity: float,
        pitch_velocity: float,
        boom_velocity: float,
    ) -> None:
        if self._dry_joint_state_pub is None:
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(now_sec)
        msg.name = ["Joint 11", "Joint 12", "Joint 13"]
        msg.position = [float(roll), float(pitch), float(boom)]
        msg.velocity = [float(roll_velocity), float(pitch_velocity), float(boom_velocity)]
        msg.effort = [0.0, 0.0, 0.0]
        self._dry_joint_state_pub.publish(msg)

    def _home_hardware(self, md80_ctx: dict, dxl_ctx: dict) -> None:
        """Approach the established home convention before accepting commands."""
        target_ticks = self._dxl_ticks(0.0, 0.0, 0.0, 1.0)
        initial_state = dxl_ctx.get("initial_state") or dynamixel_read(dxl_ctx)
        initial_ticks = [initial_state[mid]["position"] for mid in dxl_ctx["all_ids"]]
        if any(value is None for value in initial_ticks):
            raise RuntimeError("Cannot home: initial Dynamixel position feedback is incomplete")

        start_sec = time.monotonic()
        last_valid_feedback_sec = start_sec
        stable_since = None
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            with self._lock:
                if self._shutdown_requested or self._stop_latched:
                    raise RuntimeError("Hardware home aborted by stop request")
            elapsed = time.monotonic() - start_sec
            if elapsed > self.home_timeout_sec:
                raise RuntimeError("Hardware home timed out before all joints reached tolerance")

            max_delta = self.home_dxl_rate_ticks_sec * elapsed
            commanded_ticks = []
            for initial, target in zip(initial_ticks, target_ticks):
                delta = float(target - initial)
                bounded_delta = float(np.clip(delta, -max_delta, max_delta))
                commanded_ticks.append(int(round(initial + bounded_delta)))

            # MAB encoders were just zeroed at their physically staged home.
            motor_drive(md80_ctx, 0.0, 0.0, 0.0)
            if not dynamixel_drive(dxl_ctx, commanded_ticks):
                raise RuntimeError("Dynamixel home command transmission failed")

            dxl_state = dynamixel_read(dxl_ctx)
            dxl_positions = [dxl_state[mid]["position"] for mid in dxl_ctx["all_ids"]]
            dxl_velocities = [dxl_state[mid]["velocity"] for mid in dxl_ctx["all_ids"]]
            complete = not any(value is None for value in dxl_positions + dxl_velocities)
            if complete:
                outside_limits = [
                    mid
                    for mid, position in zip(dxl_ctx["all_ids"], dxl_positions)
                    if mid in GOAL_TICK_LIMITS
                    and not (GOAL_TICK_LIMITS[mid][0] <= position <= GOAL_TICK_LIMITS[mid][1])
                ]
                if outside_limits:
                    raise RuntimeError(f"Dynamixel feedback crossed configured limits while homing: {outside_limits}")
            dxl_at_home = complete and all(
                abs(position - target) <= self.home_dxl_tolerance_ticks
                for position, target in zip(dxl_positions, target_ticks)
            )
            dxl_still = complete and all(abs(velocity) <= 5 for velocity in dxl_velocities)

            joint_state = md80_ctx.get("state", {}).get("joint_state")
            md80_at_home = False
            if joint_state is not None:
                positions_by_name = dict(zip(joint_state.name, joint_state.position))
                velocities_by_name = dict(zip(joint_state.name, joint_state.velocity))
                required_names = ("Joint 11", "Joint 12", "Joint 13")
                md80_at_home = all(
                    name in positions_by_name
                    and name in velocities_by_name
                    and math.isfinite(float(positions_by_name[name]))
                    and math.isfinite(float(velocities_by_name[name]))
                    and abs(float(positions_by_name[name])) <= 0.05
                    and abs(float(velocities_by_name[name])) <= 0.05
                    for name in required_names
                )

            if complete and self._hardware_feedback_valid(md80_ctx, rospy.get_time()):
                last_valid_feedback_sec = time.monotonic()
            elif elapsed > 1.0 and time.monotonic() - last_valid_feedback_sec > 0.5:
                raise RuntimeError("Hardware home aborted because motor feedback became stale or incomplete")

            commanded_home = commanded_ticks == target_ticks
            if commanded_home and dxl_at_home and dxl_still and md80_at_home:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= self.home_stable_sec:
                    self._homed = True
                    self._publish_debug("info", "Hardware home reached and stable")
                    return
            else:
                stable_since = None

            rate.sleep()

        raise RuntimeError("ROS shutdown requested during hardware home")

    @staticmethod
    def _hardware_feedback_valid(md80_ctx: object, now_sec: float) -> bool:
        if not isinstance(md80_ctx, dict):
            return False
        joint_state = md80_ctx.get("state", {}).get("joint_state")
        if joint_state is None:
            return False
        if not {"Joint 11", "Joint 12", "Joint 13"}.issubset(set(joint_state.name)):
            return False
        if len(joint_state.position) < len(joint_state.name):
            return False
        if not np.all(np.isfinite(np.asarray(joint_state.position, dtype=float))):
            return False
        stamp_sec = joint_state.header.stamp.to_sec()
        return stamp_sec > 0.0 and max(0.0, now_sec - stamp_sec) <= 0.3

    def run(self) -> int:
        md80_ctx = None
        dxl_ctx = None
        core = GirafArmControlCore(
            theta_limits=(
                (THETA4_MIN, THETA4_MAX),
                (THETA5_MIN, THETA5_MAX),
                (THETA6_MIN, THETA6_MAX),
            ),
            task_velocity_limits=self.task_velocity_limits,
        )
        rate = rospy.Rate(self.loop_rate_hz)
        previous_boom = core.coordinates.boom

        try:
            if self.backend == "hardware":
                if not self.home_confirmed:
                    raise RuntimeError(
                        "Hardware startup blocked: physically place the arm at the documented home and "
                        "start with _home_confirmed:=true"
                    )
                if self._legacy_hardware_startup:
                    self._publish_debug(
                        "warn",
                        "Legacy launcher selected hardware implicitly; assuming the established staged-home procedure",
                    )
                self._publish_debug(
                    "warn",
                    "Hardware home confirmed by operator; zeroing MAB joints and commanding checked-in wrist home",
                )
                md80_ctx = motor_connect(gain_overrides=MD80_GAIN_OVERRIDES)
                self._md80_enabled = True
                dxl_ctx = dynamixel_connect(baudrate=1_000_000)
                self._dynamixel_torque_enabled = True
                self._hardware_connected = True
                self._publish_debug("info", "GIRAF arm controller connected to MD80 and Dynamixel hardware")
                self._home_hardware(md80_ctx, dxl_ctx)
            else:
                self._publish_debug("info", "GIRAF dry-run backend active; no hardware interfaces will be opened")

            while not rospy.is_shutdown():
                with self._lock:
                    if self._shutdown_requested:
                        break

                now_monotonic_sec = time.monotonic()
                now_sec = rospy.get_time()
                # Preserve the historical controller's fixed-rate integration
                # semantics. Monotonic time is used only for safety watchdogs.
                dt_sec = 1.0 / self.loop_rate_hz

                feedback_valid = (
                    True if self.backend == "dry_run" else self._hardware_feedback_valid(md80_ctx, now_sec)
                )
                self._maybe_become_ready(now_monotonic_sec, feedback_valid)
                (
                    step.task_velocity,
                    gripper_velocity,
                    active_source,
                    teleop_age,
                    auto_age,
                    watchdog_stale,
                ) = self._current_command(now_monotonic_sec)
                if self.backend == "hardware" and not feedback_valid:
                    task_velocity = zero_task_velocity()
                    gripper_velocity = 0.0

                step = core.step(task_velocity, gripper_velocity, dt_sec)
                q = step.coordinates
                boom_pos = q.boom
                boom_velocity = (boom_pos - previous_boom) / dt_sec
                previous_boom = boom_pos

                if self.backend == "hardware":
                    motor_drive(md80_ctx, q.roll, q.pitch, boom_pos)
                    if not dynamixel_drive(dxl_ctx, self._dxl_ticks(q.th4, q.th5, q.th6, q.grip)):
                        raise RuntimeError("Dynamixel command transmission failed")

                status_due = now_sec - self._last_status_publish_sec >= (1.0 / self.state_publish_hz)
                if status_due and self.backend == "dry_run":
                    self._publish_dry_joint_state(
                        now_sec,
                        q.roll,
                        q.pitch,
                        boom_pos,
                        step.joint_velocity[0],
                        step.joint_velocity[1],
                        boom_velocity,
                    )
                    rospy.loginfo_throttle(
                        0.5,
                        "dry-run task=(%+.3f %+.3f %+.3f | %+.3f %+.3f %+.3f) "
                        "q=(%+.3f %+.3f %+.3f %+.3f %+.3f %+.3f)",
                        *step.task_velocity,
                        q.roll,
                        q.pitch,
                        boom_pos,
                        q.th4,
                        q.th5,
                        q.th6,
                    )

                self._publish_state(
                    now_sec,
                    active_source,
                    step.task_velocity,
                    gripper_velocity,
                    teleop_age,
                    auto_age,
                    step.control_mode,
                    q.roll,
                    q.pitch,
                    boom_pos,
                    q.th4,
                    q.th5,
                    q.th6,
                    q.grip,
                    step.metrics,
                    step.transform,
                    feedback_valid,
                    watchdog_stale,
                )
                rate.sleep()

            self._publish_debug("warn", "Controller loop stopping", backend=self.backend)
            return 0
        except Exception as exc:
            self._active_faults.append(str(exc))
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
                self._md80_enabled = False
            if dxl_ctx is not None:
                try:
                    dynamixel_disconnect(dxl_ctx)
                except Exception as exc:
                    rospy.logwarn("Failed to disconnect Dynamixel motors cleanly: %s", exc)
                self._dynamixel_torque_enabled = False
            self._hardware_connected = False


def main() -> int:
    rospy.init_node("giraf_arm_controller", anonymous=False)
    controller = GirafArmController()
    return controller.run()


if __name__ == "__main__":
    raise SystemExit(main())
