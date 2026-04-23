"""ROS joystick publisher for GIRAF arm teleoperation."""

from __future__ import annotations

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float64

from anymal_custom_control.joystick_driver import joystick_connect, joystick_disconnect, joystick_read

from .giraf_arm_common import (
    ARM_WY_SPEED,
    ARM_WZ_SPEED,
    ARM_X_SPEED,
    ARM_Y_SPEED,
    ARM_Z_SPEED,
    GRIPPER_SPEED,
    STOP_TOPIC,
    TELEOP_GRIPPER_VELOCITY_TOPIC,
    TELEOP_PUBLISH_HZ,
    TELEOP_TASK_VELOCITY_TOPIC,
)


class GirafArmTeleop:
    """Reads the joystick and publishes teleop task-space commands."""

    def __init__(self) -> None:
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", TELEOP_PUBLISH_HZ))
        self.device_index = int(rospy.get_param("~device_index", 0))

        self._task_pub = rospy.Publisher(TELEOP_TASK_VELOCITY_TOPIC, TwistStamped, queue_size=1)
        self._gripper_pub = rospy.Publisher(TELEOP_GRIPPER_VELOCITY_TOPIC, Float64, queue_size=1)
        self._stop_pub = rospy.Publisher(STOP_TOPIC, Bool, queue_size=1)

    @staticmethod
    def _zero_twist(stamp: rospy.Time) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = stamp
        return msg

    def _build_twist(self, data: dict[str, float], stamp: rospy.Time) -> TwistStamped:
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

    @staticmethod
    def _build_gripper_velocity(data: dict[str, float]) -> float:
        if not (data["LB"] and data["RB"]):
            return 0.0
        if data["AB"] and not data["BB"]:
            return -GRIPPER_SPEED
        if data["BB"] and not data["AB"]:
            return GRIPPER_SPEED
        return 0.0

    def _publish_zero(self) -> None:
        stamp = rospy.Time.now()
        self._task_pub.publish(self._zero_twist(stamp))
        self._gripper_pub.publish(Float64(data=0.0))

    def run(self) -> int:
        js = None
        prev_x_button = 0
        rate = rospy.Rate(self.publish_rate_hz)

        try:
            js = joystick_connect(device_index=self.device_index)
            rospy.loginfo("Connected joystick: %s", js["device"].name)

            while not rospy.is_shutdown():
                data = joystick_read(js)
                stamp = rospy.Time.now()

                if data["XB"] and not prev_x_button:
                    self._publish_zero()
                    self._stop_pub.publish(Bool(data=True))
                    rospy.logwarn("X pressed; stop requested")
                    return 0

                self._task_pub.publish(self._build_twist(data, stamp))
                self._gripper_pub.publish(Float64(data=self._build_gripper_velocity(data)))
                prev_x_button = data["XB"]
                rate.sleep()

            return 0
        except Exception as exc:
            rospy.logerr("Teleop error: %s", exc)
            return 1
        finally:
            self._publish_zero()
            if js is not None:
                joystick_disconnect(js)


def main() -> int:
    rospy.init_node("giraf_arm_teleop", anonymous=False)
    teleop = GirafArmTeleop()
    return teleop.run()


if __name__ == "__main__":
    raise SystemExit(main())
