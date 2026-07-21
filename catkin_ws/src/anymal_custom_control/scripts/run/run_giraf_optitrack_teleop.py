#!/usr/bin/env python3
"""Run the physical GIRAF arm from remote OptiTrack teleop commands.

This is intentionally a thin hardware entrypoint around the proven production
arm controller. The normal ROS master and CANdle/MD80 node must already be
running. Startup uses the same established home behavior as the joystick
teleop launchers, then listens for geometry_msgs/TwistStamped commands on
/giraf_arm/teleop_task_velocity_cmd.

The controller holds its integrated position target when commands are absent
or older than its 150 ms watchdog. Stop this process with Ctrl-C to run the
normal motor-disconnect path.
"""

import rospy

from anymal_custom_control.control.giraf_arm_common import (
    COMMAND_TIMEOUT_SEC,
    TELEOP_TASK_VELOCITY_TOPIC,
)
from anymal_custom_control.control.giraf_arm_controller import GirafArmController


def main() -> int:
    rospy.init_node("giraf_arm_controller", anonymous=False)
    rospy.set_param("~command_source", "teleop")
    rospy.logwarn("Starting physical GIRAF arm for remote OptiTrack teleoperation")
    rospy.loginfo(
        "Waiting for %s (watchdog %.0f ms); stale or missing commands hold position",
        TELEOP_TASK_VELOCITY_TOPIC,
        1000.0 * COMMAND_TIMEOUT_SEC,
    )
    return GirafArmController().run()


if __name__ == "__main__":
    raise SystemExit(main())
