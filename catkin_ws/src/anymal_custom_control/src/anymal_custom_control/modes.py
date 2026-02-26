"""ModeController — switch ANYmal operational modes (Rest / Stand / Walk).

Publishes SwitchOperationalModeActionGoal to the operational_mode_manager.
Mode switching is a one-shot publish (not continuous like velocity commands).

Safety:
    If a MovementController is provided, velocity is zeroed before every
    mode switch to prevent the robot from continuing to move during a
    mode transition.
"""

import threading

import rospy
from operational_mode_manager_msgs.msg import SwitchOperationalModeActionGoal


# Default topic for the operational_mode_manager action goal
DEFAULT_TOPIC = '/operational_mode_manager/switch_operational_mode/goal'


class ModeController:
    """Switch ANYmal between operational modes.

    Usage:
        mc = MovementController()
        mc.start()
        modes = ModeController(movement_controller=mc)
        modes.switch_mode(ModeController.WALK)
        # ... do movement ...
        modes.switch_mode(ModeController.REST)

    Standalone (no velocity control):
        modes = ModeController()
        modes.switch_mode(ModeController.REST)
    """

    # Operational mode names (must match ANYmal software, case-sensitive)
    REST = 'Rest'
    STAND = 'Stand'
    WALK = 'Walk'
    VALID_MODES = (REST, STAND, WALK)

    def __init__(self, movement_controller=None, topic=DEFAULT_TOPIC):
        """
        Args:
            movement_controller: Optional MovementController instance.
                If provided, velocity is zeroed before each mode switch.
            topic: ROS topic for SwitchOperationalModeActionGoal messages.
        """
        self._movement_controller = movement_controller
        self._topic = topic
        self._pub = rospy.Publisher(
            topic, SwitchOperationalModeActionGoal, queue_size=1,
        )
        self._lock = threading.Lock()
        self._current_mode = None

    def switch_mode(self, mode_name):
        """Switch to the given operational mode.

        Zeroes velocity (if a MovementController was provided) and publishes
        a SwitchOperationalModeActionGoal.

        Args:
            mode_name: One of ModeController.REST, .STAND, .WALK.

        Returns:
            str: The mode name that was requested.

        Raises:
            ValueError: If mode_name is not a valid mode.
        """
        if mode_name not in self.VALID_MODES:
            raise ValueError(
                "Invalid mode '{}'. Valid modes: {}".format(
                    mode_name, ', '.join(self.VALID_MODES))
            )

        # Safety: zero velocity before switching
        if self._movement_controller is not None:
            self._movement_controller.stop()

        msg = SwitchOperationalModeActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.goal.target.name = mode_name
        self._pub.publish(msg)

        with self._lock:
            self._current_mode = mode_name

        rospy.loginfo("ModeController: switch requested -> %s", mode_name)
        return mode_name

    @property
    def current_mode(self):
        """The last mode requested, or None if no switch has been made."""
        with self._lock:
            return self._current_mode

    @property
    def topic(self):
        """The ROS topic this controller publishes to."""
        return self._topic
