#!/usr/bin/env python3
"""Run GIRAF arm teleop with OAK-D camera-Y level stabilization.

This launcher starts the existing GIRAF arm controller, then runs a replacement
teleop publisher that adds a conservative angular-velocity correction to the
joystick task-space command. The correction is based on the shared OAK-D
camera-Y level-error topic and tries to drive the camera +Y axis back to
horizontal.

No separate motor is commanded here. The correction is sent as task-space
angular velocity to the normal Jacobian-based arm controller.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float64, String

from anymal_custom_control.RRP_kinematic_model import get_boom_length_d3
from anymal_custom_control.RRPRRR_kinematic_model import num_forward_transform
from anymal_custom_control.control.giraf_arm_common import (
    ARM_WY_SPEED,
    ARM_WZ_SPEED,
    ARM_X_SPEED,
    ARM_Y_SPEED,
    ARM_Z_SPEED,
    GRIPPER_SPEED,
    PITCH_KIN_OFFSET,
    STOP_TOPIC,
    TELEOP_GRIPPER_VELOCITY_TOPIC,
    TELEOP_TASK_VELOCITY_TOPIC,
    THETA4_KIN_OFFSET,
    THETA5_KIN_OFFSET,
    THETA6_KIN_OFFSET,
)
from anymal_custom_control.joystick_driver import joystick_connect, joystick_disconnect, joystick_read
from anymal_custom_control.runtime import LaunchSpec, ProcessManager, run_script_path


Vector3 = Tuple[float, float, float]

STATE_TOPIC = "/giraf_arm/state"

LOOP_HZ = 100.0

CAMERA_Z_AXIS = "z"
CAMERA_Z_SIGN = 1.0

OAKD_LEVEL_ERROR_TOPIC = "/oakd/camera_y_level_error"
OAKD_ERROR_TIMEOUT_SEC = 0.25
CONTROLLER_STOP_TIMEOUT_SEC = 10.0
STABILIZE_ALWAYS_ON = True
STABILIZE_ENABLE_BUTTON = "YB"
STABILIZE_REQUIRES_BUMPERS = False
STABILIZE_KP = 5.0
STABILIZE_KI = 0.0
STABILIZE_KD = 0.5
STABILIZE_DEADBAND_DEG = 0.75
STABILIZE_MAX_ANGULAR_SPEED = 1.0  # rad/s
STABILIZE_INTEGRAL_LIMIT_RAD = 0.25
STABILIZE_SIGN = -1.0


@dataclass
class ArmState:
    stamp_sec: float
    roll: float
    pitch: float
    boom: float
    th4: float
    th5: float
    th6: float


def axis_vector(axis_name: str, sign: float) -> Vector3:
    if axis_name == "x":
        return (sign, 0.0, 0.0)
    if axis_name == "y":
        return (0.0, sign, 0.0)
    if axis_name == "z":
        return (0.0, 0.0, sign)
    raise ValueError("axis name must be x, y, or z")


def joystick_twist(data: Dict[str, float], stamp: rospy.Time) -> TwistStamped:
    msg = TwistStamped()
    msg.header.stamp = stamp

    if data["LB"] and data["RB"]:
        msg.twist.linear.x = ARM_X_SPEED * data["LY"]
        msg.twist.linear.y = -ARM_Y_SPEED * data["LX"]
        if data["RT"] and not data["LT"]:
            msg.twist.linear.z = ARM_Z_SPEED * data["RT"]
        elif data["LT"] and not data["RT"]:
            msg.twist.linear.z = -ARM_Z_SPEED * data["LT"]
        msg.twist.angular.y = ARM_WY_SPEED * data["RY"]
        msg.twist.angular.z = -ARM_WZ_SPEED * data["RX"]

    return msg


def joystick_gripper_velocity(data: Dict[str, float]) -> float:
    if not (data["LB"] and data["RB"]):
        return 0.0
    if data["AB"] and not data["BB"]:
        return -GRIPPER_SPEED
    if data["BB"] and not data["AB"]:
        return GRIPPER_SPEED
    return 0.0


def state_from_json(msg: String) -> Optional[ArmState]:
    try:
        payload = json.loads(msg.data)
        arm = payload["arm"]
        return ArmState(
            stamp_sec=float(payload.get("stamp_sec", rospy.get_time())),
            roll=float(arm["roll"]),
            pitch=float(arm["pitch"]),
            boom=float(arm["boom"]),
            th4=float(arm["th4"]),
            th5=float(arm["th5"]),
            th6=float(arm["th6"]),
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        rospy.logwarn_throttle(2.0, "Failed to parse /giraf_arm/state: %s", exc)
        return None


def joint_coords_from_state(state: ArmState) -> List[float]:
    return [
        state.roll,
        state.pitch + PITCH_KIN_OFFSET,
        get_boom_length_d3(state.boom),
        state.th4 + THETA4_KIN_OFFSET,
        state.th5 + THETA5_KIN_OFFSET,
        state.th6 + THETA6_KIN_OFFSET,
    ]


def corrected_axis_global(state: ArmState, camera_z_axis: Vector3) -> Optional[np.ndarray]:
    try:
        transform = np.asarray(num_forward_transform(joint_coords_from_state(state)), dtype=float)
    except Exception as exc:
        rospy.logwarn_throttle(2.0, "Failed to compute FK transform for stabilization: %s", exc)
        return None

    axis = transform[:3, :3].dot(np.asarray(camera_z_axis, dtype=float))
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-9:
        return None
    return axis / norm


def add_angular_correction(msg: TwistStamped, axis_global: np.ndarray, speed_rad_s: float) -> None:
    omega = axis_global * speed_rad_s
    msg.twist.angular.x += float(omega[0])
    msg.twist.angular.y += float(omega[1])
    msg.twist.angular.z += float(omega[2])


class StabilizedTeleop:
    def __init__(self) -> None:
        self.task_pub = rospy.Publisher(TELEOP_TASK_VELOCITY_TOPIC, TwistStamped, queue_size=1)
        self.gripper_pub = rospy.Publisher(TELEOP_GRIPPER_VELOCITY_TOPIC, Float64, queue_size=1)
        self.stop_pub = rospy.Publisher(STOP_TOPIC, Bool, queue_size=1)
        self.latest_state: Optional[ArmState] = None
        self.latest_error_rad: Optional[float] = None
        self.latest_error_time = 0.0
        self.integral_rad = 0.0
        self.previous_error_rad: Optional[float] = None
        self.previous_time = time.monotonic()
        rospy.Subscriber(STATE_TOPIC, String, self._state_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(
            OAKD_LEVEL_ERROR_TOPIC,
            Float64,
            self._oakd_error_cb,
            queue_size=1,
            tcp_nodelay=True,
        )

    def _state_cb(self, msg: String) -> None:
        parsed = state_from_json(msg)
        if parsed is not None:
            self.latest_state = parsed

    def _oakd_error_cb(self, msg: Float64) -> None:
        error_rad = float(msg.data)
        if not math.isfinite(error_rad):
            rospy.logwarn_throttle(2.0, "Ignoring non-finite OAK-D level error: %r", msg.data)
            return
        if self.latest_error_rad is None:
            rospy.loginfo("Receiving OAK-D stabilization data on %s", OAKD_LEVEL_ERROR_TOPIC)
        self.latest_error_rad = error_rad
        self.latest_error_time = time.monotonic()

    def oakd_error_fresh(self) -> bool:
        return (
            self.latest_error_rad is not None
            and (time.monotonic() - self.latest_error_time) <= OAKD_ERROR_TIMEOUT_SEC
        )

    @staticmethod
    def stabilization_enabled(data: Dict[str, float]) -> bool:
        if STABILIZE_ALWAYS_ON:
            return True
        if STABILIZE_REQUIRES_BUMPERS and not (data["LB"] and data["RB"]):
            return False
        return bool(data.get(STABILIZE_ENABLE_BUTTON, 0))

    def correction_speed(self, enabled: bool) -> Tuple[float, float]:
        now = time.monotonic()
        dt = max(1e-3, now - self.previous_time)
        self.previous_time = now

        if not enabled or self.latest_error_rad is None:
            self.integral_rad = 0.0
            self.previous_error_rad = self.latest_error_rad
            return 0.0, 0.0

        error = self.latest_error_rad
        if abs(math.degrees(error)) < STABILIZE_DEADBAND_DEG:
            error = 0.0

        self.integral_rad += error * dt
        self.integral_rad = max(
            -STABILIZE_INTEGRAL_LIMIT_RAD,
            min(STABILIZE_INTEGRAL_LIMIT_RAD, self.integral_rad),
        )

        if self.previous_error_rad is None:
            error_rate = 0.0
        else:
            error_rate = (error - self.previous_error_rad) / dt
        self.previous_error_rad = error

        speed = -STABILIZE_SIGN * (
            STABILIZE_KP * error
            + STABILIZE_KI * self.integral_rad
            + STABILIZE_KD * error_rate
        )
        speed = max(-STABILIZE_MAX_ANGULAR_SPEED, min(STABILIZE_MAX_ANGULAR_SPEED, speed))
        return speed, error_rate

    def publish_zero(self) -> None:
        self.task_pub.publish(TwistStamped())
        self.gripper_pub.publish(Float64(data=0.0))

    def wait_for_controller_stop(self, processes: Dict[str, object]) -> bool:
        """Let the controller finish its motor-disable sequence before SIGINT."""
        controller = processes.get("giraf_arm_controller")
        if controller is None:
            return True

        deadline = time.monotonic() + CONTROLLER_STOP_TIMEOUT_SEC
        while controller.poll() is None and time.monotonic() < deadline and not rospy.is_shutdown():
            self.publish_zero()
            # Repeat the stop request in case the first message raced subscriber setup.
            self.stop_pub.publish(Bool(data=True))
            time.sleep(0.05)

        if controller.poll() is None:
            rospy.logerr(
                "Arm controller did not complete shutdown within %.1fs; launcher will now interrupt it",
                CONTROLLER_STOP_TIMEOUT_SEC,
            )
            return False

        rospy.loginfo("Arm controller completed its motor shutdown sequence")
        return True

    def run(self, processes: Dict[str, object]) -> int:
        joystick = None
        prev_x_button = 0
        camera_z_axis = axis_vector(CAMERA_Z_AXIS, CAMERA_Z_SIGN)

        try:
            joystick = joystick_connect()
            rospy.loginfo("Connected joystick: %s", joystick["device"].name)
            rate = rospy.Rate(LOOP_HZ)

            while not rospy.is_shutdown():
                for name, proc in processes.items():
                    code = proc.poll()
                    if code is not None:
                        print(f"\n{name} exited with code {code}")
                        return code

                data = joystick_read(joystick)
                stamp = rospy.Time.now()

                if data["XB"] and not prev_x_button:
                    self.publish_zero()
                    self.stop_pub.publish(Bool(data=True))
                    rospy.logwarn("X pressed; stop requested")
                    self.wait_for_controller_stop(processes)
                    return 0
                prev_x_button = data["XB"]

                msg = joystick_twist(data, stamp)
                stabilization_requested = self.stabilization_enabled(data)
                oakd_fresh = self.oakd_error_fresh()
                enabled = stabilization_requested and oakd_fresh
                if stabilization_requested and not oakd_fresh:
                    rospy.logwarn_throttle(
                        5.0,
                        "Stabilization inactive: no fresh %s data within %.2fs; "
                        "verify the OAK-D node reports imu=BNO086 and fused_imu=on",
                        OAKD_LEVEL_ERROR_TOPIC,
                        OAKD_ERROR_TIMEOUT_SEC,
                    )
                speed_rad_s, _error_rate_rad_s = self.correction_speed(enabled)
                axis_global = (
                    corrected_axis_global(self.latest_state, camera_z_axis)
                    if self.latest_state is not None
                    else None
                )
                if enabled and axis_global is not None:
                    add_angular_correction(msg, axis_global, speed_rad_s)

                self.task_pub.publish(msg)
                self.gripper_pub.publish(Float64(data=joystick_gripper_velocity(data)))

                rate.sleep()
        finally:
            self.publish_zero()
            if joystick is not None:
                joystick_disconnect(joystick)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--no-oakd",
        action="store_true",
        help="Do not start the OAK-D owner; use when another launcher already owns the camera",
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("giraf_arm_stabilized_teleop", anonymous=False)
    specs = [
        LaunchSpec(
            name="giraf_arm_controller",
            command=[sys.executable, str(run_script_path("run_giraf_arm_controller.py"))],
        ),
    ]
    if not args.no_oakd:
        specs.append(
            LaunchSpec(
                name="oakd_sensor",
                command=[
                    sys.executable,
                    str(run_script_path("run_oakd_sensor_node.py")),
                    "--wait-for-arm-state",
                ],
            )
        )

    manager = ProcessManager()
    processes: Dict[str, object] = {}
    for spec in specs:
        print(f"Starting {spec.name}: {' '.join(spec.command)}")
        processes[spec.name] = manager.start(spec)

    teleop = StabilizedTeleop()
    try:
        return teleop.run(processes)
    except KeyboardInterrupt:
        print("\nShutting down stabilized teleop stack...")
        return 0
    finally:
        manager.terminate_all()


if __name__ == "__main__":
    raise SystemExit(main())
