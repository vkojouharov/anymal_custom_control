#!/usr/bin/env python3
"""Prototype teleop wrapper with YB autonomous-mode toggling."""

from __future__ import annotations

import json
import time
from typing import Dict

import rospy
from std_msgs.msg import Bool, Float64, String

from anymal_custom_control.control.giraf_arm_common import (
    COMMAND_SOURCE_TOPIC,
    STOP_TOPIC,
    TELEOP_GRIPPER_VELOCITY_TOPIC,
    TELEOP_TASK_VELOCITY_TOPIC,
)
from anymal_custom_control.joystick_driver import joystick_connect, joystick_disconnect, joystick_read

from run_teleop_stabilized import (
    CAMERA_Z_AXIS,
    CAMERA_Z_SIGN,
    LOOP_HZ,
    StabilizedTeleop,
    add_angular_correction,
    axis_vector,
    corrected_axis_global,
    joystick_gripper_velocity,
    joystick_twist,
)


VISUAL_SERVO_STATUS_TOPIC = "/giraf_arm/visual_servo_status_json"
READY_TIMEOUT_SEC = 0.5


class VisualServoTeleop(StabilizedTeleop):
    def __init__(self) -> None:
        super().__init__()
        self.command_source_pub = rospy.Publisher(COMMAND_SOURCE_TOPIC, String, queue_size=1, latch=True)
        self.current_source = "teleop"
        self.latest_ready = False
        self.latest_status_time = 0.0
        rospy.Subscriber(
            VISUAL_SERVO_STATUS_TOPIC,
            String,
            self._visual_servo_status_cb,
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

    def _visual_servo_status_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse visual-servo status: %s", exc)
            return
        self.latest_ready = bool(payload.get("ready", False))
        self.latest_status_time = time.monotonic()

    def _command_source_cb(self, msg: String) -> None:
        source = msg.data.strip().lower()
        if source in {"teleop", "auto"}:
            self.current_source = source

    def ready_for_auto(self) -> bool:
        return self.latest_ready and (time.monotonic() - self.latest_status_time) <= READY_TIMEOUT_SEC

    def publish_source(self, source: str) -> None:
        self.current_source = source
        self.command_source_pub.publish(String(data=source))
        rospy.loginfo("Command source: %s", source)

    def run(self, processes: Dict[str, object]) -> int:
        joystick = None
        prev_x_button = 0
        prev_y_button = 0
        camera_z_axis = axis_vector(CAMERA_Z_AXIS, CAMERA_Z_SIGN)
        self.publish_source("teleop")

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
                    self.publish_source("teleop")
                    self.publish_zero()
                    self.stop_pub.publish(Bool(data=True))
                    rospy.logwarn("X pressed; stop requested")
                    return 0
                prev_x_button = data["XB"]

                if data["YB"] and not prev_y_button:
                    if self.current_source == "auto":
                        self.publish_source("teleop")
                        self.publish_zero()
                    elif self.ready_for_auto():
                        self.publish_zero()
                        self.publish_source("auto")
                    else:
                        rospy.logwarn("YB ignored; ID1 is not currently ready for autonomous mode")
                prev_y_button = data["YB"]

                msg = joystick_twist(data, stamp)
                if self.current_source == "teleop":
                    enabled = self.stabilization_enabled(data) and self.oakd_error_fresh()
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
                else:
                    self.task_pub.publish(type(msg)())
                    self.gripper_pub.publish(Float64(data=0.0))

                rate.sleep()
        finally:
            self.publish_zero()
            self.publish_source("teleop")
            if joystick is not None:
                joystick_disconnect(joystick)


def main() -> int:
    rospy.init_node("giraf_arm_visual_servo_teleop", anonymous=False)
    teleop = VisualServoTeleop()
    try:
        return teleop.run({})
    except KeyboardInterrupt:
        print("\nShutting down visual-servo prototype teleop...")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
