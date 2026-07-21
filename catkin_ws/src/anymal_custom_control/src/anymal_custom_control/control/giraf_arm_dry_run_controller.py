"""Hardware-free ROS backend for the GIRAF OptiTrack interface."""

from __future__ import annotations

import json
import math
import time
from threading import Lock

import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .giraf_arm_common import (
    COMMAND_TIMEOUT_SEC,
    CONTROL_LOOP_HZ,
    DEBUG_TOPIC,
    STATE_PUBLISH_HZ,
    STATE_TOPIC,
    STOP_TOPIC,
    TELEOP_TASK_VELOCITY_TOPIC,
    zero_task_velocity,
)
from .giraf_arm_core import GirafArmControlCore


READINESS_TOPIC = "/giraf_arm/readiness"
REMOTE_TASK_VELOCITY_LIMITS = np.array([0.050, 0.050, 0.025, 0.125, 0.125, 0.125], dtype=float)
DRY_WRIST_LIMITS = (
    (-1.5339807879, 2.5463306200),
    (-1.5339807879, 1.5339807879),
    (-3.0679615758, 3.0679615758),
)


class GirafArmDryRunController:
    def __init__(self) -> None:
        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", CONTROL_LOOP_HZ))
        self.state_publish_hz = float(rospy.get_param("~state_publish_hz", STATE_PUBLISH_HZ))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", COMMAND_TIMEOUT_SEC))
        self.core = GirafArmControlCore(
            theta_limits=DRY_WRIST_LIMITS,
            task_velocity_limits=REMOTE_TASK_VELOCITY_LIMITS,
        )

        self._lock = Lock()
        self._command = zero_task_velocity()
        self._command_received_sec = 0.0
        self._command_valid = False
        self._ready = False
        self._stop_latched = False
        self._shutdown_requested = False
        self._last_publish_sec = 0.0
        self._previous_boom = self.core.coordinates.boom

        self._state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=1, latch=True)
        self._readiness_pub = rospy.Publisher(READINESS_TOPIC, String, queue_size=1, latch=True)
        self._debug_pub = rospy.Publisher(DEBUG_TOPIC, String, queue_size=20)
        self._joint_pub = rospy.Publisher("/md80/joint_states", JointState, queue_size=1)
        rospy.Subscriber(
            TELEOP_TASK_VELOCITY_TOPIC,
            TwistStamped,
            self._command_cb,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(STOP_TOPIC, Bool, self._stop_cb, queue_size=1, tcp_nodelay=True)

    @staticmethod
    def _twist_array(msg: TwistStamped) -> np.ndarray:
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

    def _command_cb(self, msg: TwistStamped) -> None:
        command = self._twist_array(msg)
        valid = bool(np.all(np.isfinite(command)))
        with self._lock:
            self._command = command if valid else zero_task_velocity()
            self._command_received_sec = time.monotonic()
            self._command_valid = valid

    def _stop_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            self._stop_latched = True
            self._shutdown_requested = True

    def _selected_command(self, now_sec: float) -> tuple[np.ndarray, float, bool]:
        with self._lock:
            command = self._command.copy()
            received_sec = self._command_received_sec
            valid = self._command_valid
            stop_latched = self._stop_latched
        age = float("inf") if received_sec <= 0.0 else max(0.0, now_sec - received_sec)
        stale = age > self.command_timeout_sec
        if not self._ready and valid and not stale and np.max(np.abs(command)) <= 1e-9:
            self._ready = True
        if stop_latched or stale or not valid or not self._ready:
            return zero_task_velocity(), age, stale
        return command, age, stale

    @staticmethod
    def _safe_float(value: float) -> float | None:
        return float(value) if math.isfinite(float(value)) else None

    def _publish(self, now_ros_sec: float, command: np.ndarray, age: float, stale: bool, step) -> None:
        if now_ros_sec - self._last_publish_sec < 1.0 / self.state_publish_hz:
            return
        q = step.coordinates
        source = "teleop" if self._ready else "initializing"
        state = {
            "stamp_sec": round(now_ros_sec, 6),
            "active_source": source,
            "command_source_param": source,
            "stop_latched": self._stop_latched,
            "backend": "dry_run",
            "ready": self._ready,
            "state_semantics": "integrated_command_coordinates",
            "teleop_cmd_age_sec": self._safe_float(round(age, 6)),
            "selected_task_velocity": {
                "vx": float(command[0]),
                "vy": float(command[1]),
                "vz": float(command[2]),
                "wx": float(command[3]),
                "wy": float(command[4]),
                "wz": float(command[5]),
            },
            "selected_gripper_velocity": 0.0,
            "arm": {
                "roll": float(q.roll),
                "pitch": float(q.pitch),
                "boom": float(q.boom),
                "th4": float(q.th4),
                "th5": float(q.th5),
                "th6": float(q.th6),
                "grip": float(q.grip),
            },
            "end_effector": {
                "x": float(step.transform[0, 0]),
                "y": float(step.transform[1, 0]),
                "z": float(step.transform[2, 0]),
            },
        }
        self._state_pub.publish(String(data=json.dumps(state, sort_keys=True)))

        readiness = {
            "stamp_sec": round(now_ros_sec, 6),
            "backend": "dry_run",
            "ready": self._ready,
            "hardware_connected": False,
            "feedback_valid": True,
            "homed": True,
            "command_source": source,
            "watchdog_state": "stale" if stale else "fresh",
            "md80_enabled": False,
            "dynamixel_torque_enabled": False,
            "stop_latched": self._stop_latched,
            "estop_state": "not_applicable",
            "active_faults": [],
            "last_command_receipt_age_sec": self._safe_float(round(age, 6)),
        }
        self._readiness_pub.publish(String(data=json.dumps(readiness, sort_keys=True)))

        boom_velocity = (q.boom - self._previous_boom) * self.state_publish_hz
        self._previous_boom = q.boom
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.from_sec(now_ros_sec)
        joint_state.name = ["Joint 11", "Joint 12", "Joint 13"]
        joint_state.position = [float(q.roll), float(q.pitch), float(q.boom)]
        joint_state.velocity = [float(step.joint_velocity[0]), float(step.joint_velocity[1]), float(boom_velocity)]
        joint_state.effort = [0.0, 0.0, 0.0]
        self._joint_pub.publish(joint_state)
        self._last_publish_sec = now_ros_sec

    def run(self) -> int:
        rospy.loginfo("GIRAF dry-run controller active; no hardware interfaces are imported or opened")
        rate = rospy.Rate(self.loop_rate_hz)
        while not rospy.is_shutdown():
            with self._lock:
                if self._shutdown_requested:
                    break
            command, age, stale = self._selected_command(time.monotonic())
            step = self.core.step(command, 0.0, 1.0 / self.loop_rate_hz)
            self._publish(rospy.get_time(), step.task_velocity, age, stale, step)
            rate.sleep()
        return 0


def main() -> int:
    rospy.init_node("giraf_arm_controller", anonymous=False)
    return GirafArmDryRunController().run()


if __name__ == "__main__":
    raise SystemExit(main())
