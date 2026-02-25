"""MovementController — publish movement commands to ANYmal via joy_manager.

Publishes joy_manager_msgs/AnyJoy messages to /anyjoy/operator at 10 Hz.
The joy_manager node processes these and outputs geometry_msgs/TwistStamped
scaled by the active motion controller's velocity limits.

Axis mapping (from Output.hpp):
    axes[0] = LATERAL   (-1.0 = right, +1.0 = left)
    axes[1] = HEADING   (-1.0 = backward, +1.0 = forward)
    axes[2] = ROLL
    axes[3] = TURNING   (-1.0 = CW, +1.0 = CCW)
    axes[4] = VERTICAL
    axes[5] = PITCH

Values are [-1.0, 1.0] scale factors. The joy_manager multiplies them by
the active controller's max velocity.

Safety:
    - Joy_manager has a 0.5s timeout. If we stop publishing, the robot stops.
    - When stop() is called, we publish zero axes (not silence).
    - On shutdown/crash, the timeout auto-stops the robot.
"""

import threading
import time

import rospy
from joy_manager_msgs.msg import AnyJoy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


# Axis indices (from Output.hpp:10)
LATERAL = 0
HEADING = 1
ROLL = 2
TURNING = 3
VERTICAL = 4
PITCH = 5
NUM_AXES = 6

# Default button count (matching typical joystick)
NUM_BUTTONS = 12


class MovementController:
    """Programmatic control of ANYmal movement via the joy_manager pipeline.

    Usage:
        mc = MovementController()
        mc.start()             # starts 10 Hz publish thread
        mc.forward(0.3)        # 30% of max forward velocity
        rospy.sleep(2.0)
        mc.stop()              # zero all velocities
        mc.shutdown()          # stop publish thread

    Context manager:
        with MovementController() as mc:
            mc.forward(0.3)
            rospy.sleep(2.0)
    """

    def __init__(self, topic='/anyjoy/operator', rate_hz=10):
        """
        Args:
            topic: ROS topic to publish AnyJoy messages to.
            rate_hz: Publish rate in Hz. Must be >= 2 Hz (joy_manager timeout is 0.5s).
                     Default 10 Hz matches the joy_manager worker loop.
        """
        self._topic = topic
        self._rate_hz = rate_hz
        self._pub = rospy.Publisher(topic, AnyJoy, queue_size=10)

        self._lock = threading.Lock()
        self._axes = [0.0] * NUM_AXES
        self._buttons = [0] * NUM_BUTTONS

        self._running = False
        self._thread = None

    def start(self):
        """Start the background publish thread. Publishes zeros initially."""
        if self._running:
            rospy.logwarn("MovementController: already running")
            return
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        rospy.loginfo("MovementController: publishing to %s at %d Hz", self._topic, self._rate_hz)

    def shutdown(self):
        """Stop the publish thread. Robot will halt after 0.5s timeout."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        rospy.loginfo("MovementController: stopped (robot will halt after 0.5s timeout)")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        self.shutdown()
        return False

    # --- Movement commands ---

    def forward(self, speed=0.5):
        """Move forward. Speed in [0, 1.0] (fraction of max velocity)."""
        self.set_velocity(heading=abs(speed))

    def backward(self, speed=0.5):
        """Move backward. Speed in [0, 1.0]."""
        self.set_velocity(heading=-abs(speed))

    def strafe_left(self, speed=0.5):
        """Strafe left. Speed in [0, 1.0]."""
        self.set_velocity(lateral=abs(speed))

    def strafe_right(self, speed=0.5):
        """Strafe right. Speed in [0, 1.0]."""
        self.set_velocity(lateral=-abs(speed))

    def turn_left(self, speed=0.5):
        """Turn left (counter-clockwise). Speed in [0, 1.0]."""
        self.set_velocity(turning=abs(speed))

    def turn_right(self, speed=0.5):
        """Turn right (clockwise). Speed in [0, 1.0]."""
        self.set_velocity(turning=-abs(speed))

    def set_velocity(self, heading=0.0, lateral=0.0, turning=0.0,
                     vertical=0.0, roll=0.0, pitch=0.0):
        """Set velocity on all axes. Values clamped to [-1.0, 1.0].

        Args:
            heading: Forward/backward (-1 to 1). Positive = forward.
            lateral: Left/right strafe (-1 to 1). Positive = left.
            turning: Yaw rotation (-1 to 1). Positive = counter-clockwise.
            vertical: Body height (-1 to 1).
            roll: Body roll (-1 to 1).
            pitch: Body pitch (-1 to 1).
        """
        with self._lock:
            self._axes[HEADING] = _clamp(heading)
            self._axes[LATERAL] = _clamp(lateral)
            self._axes[TURNING] = _clamp(turning)
            self._axes[VERTICAL] = _clamp(vertical)
            self._axes[ROLL] = _clamp(roll)
            self._axes[PITCH] = _clamp(pitch)

    def stop(self):
        """Zero all velocity axes. The robot halts but the publish thread continues."""
        with self._lock:
            self._axes = [0.0] * NUM_AXES

    def get_current_velocity(self):
        """Return the currently commanded axes as a dict."""
        with self._lock:
            return {
                'heading': self._axes[HEADING],
                'lateral': self._axes[LATERAL],
                'turning': self._axes[TURNING],
                'vertical': self._axes[VERTICAL],
                'roll': self._axes[ROLL],
                'pitch': self._axes[PITCH],
            }

    # --- Internal ---

    def _publish_loop(self):
        interval = 1.0 / self._rate_hz
        while self._running and not rospy.is_shutdown():
            msg = AnyJoy()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()

            msg.joy = Joy()
            msg.joy.header = msg.header
            with self._lock:
                msg.joy.axes = list(self._axes)
                msg.joy.buttons = list(self._buttons)

            msg.modules = []
            msg.commands = []

            self._pub.publish(msg)
            time.sleep(interval)


def _clamp(value, min_val=-1.0, max_val=1.0):
    return max(min_val, min(max_val, float(value)))
