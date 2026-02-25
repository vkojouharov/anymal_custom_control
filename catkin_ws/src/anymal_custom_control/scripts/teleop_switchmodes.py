#!/usr/bin/env python3
"""WASD teleop with mode switching (sit/stand/walk) for ANYmal robot.

Controls:
    W / S   — forward / backward
    A / D   — strafe left / strafe right
    Q / E   — turn left / turn right
    +/-     — increase / decrease speed
    1       — switch to FREEZE mode (sit)
    2       — switch to STAND mode
    3       — switch to WALK mode (locomotion — enables WASD movement)
    X       — quit

Mode Switching:
    Publishes SwitchOperationalModeActionGoal to the operational_mode_manager.
    Velocity commands are published separately via AnyJoy on /anyjoy/operator.

Prerequisites:
    - ANYmal software running
    - E-stop ready

Usage:
    python3 teleop_switchmodes.py
    python3 teleop_switchmodes.py --speed 0.3
    python3 teleop_switchmodes.py --topic /anyjoy/operator
"""

import argparse
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import rospy
from joy_manager_msgs.msg import AnyJoy
from operational_mode_manager_msgs.msg import SwitchOperationalModeActionGoal
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


# ── Axis indices (from Output.hpp) ──────────────────────────────────────────
LATERAL = 0
HEADING = 1
ROLL = 2
TURNING = 3
VERTICAL = 4
PITCH = 5
NUM_AXES = 6
NUM_BUTTONS = 12

# ── Mode definitions ────────────────────────────────────────────────────────
# These are operational mode names recognised by the operational_mode_manager.
# The names must match what the ANYmal software expects (case-sensitive).
MODES = {
    '1': {'name': 'REST', 'op_mode': 'Rest'},
    '2': {'name': 'STAND',  'op_mode': 'Stand'},
    '3': {'name': 'WALK',   'op_mode': 'Walk'},
}

# ── WASD key bindings ───────────────────────────────────────────────────────
# Key -> (heading, lateral, turning)
BINDINGS = {
    'w': ( 1.0,  0.0,  0.0),   # forward
    's': (-1.0,  0.0,  0.0),   # backward
    'a': ( 0.0,  1.0,  0.0),   # strafe left
    'd': ( 0.0, -1.0,  0.0),   # strafe right
    'q': ( 0.0,  0.0,  1.0),   # turn left  (CCW)
    'e': ( 0.0,  0.0, -1.0),   # turn right (CW)
}

HELP_TEXT = """
ANYmal WASD Teleop + Mode Switching
====================================
  W/S  — forward / backward
  A/D  — strafe left / right
  Q/E  — turn left / right
  +/-  — increase / decrease speed (current: {speed:.0%})
  1    — FREEZE (sit)
  2    — STAND
  3    — WALK   (locomotion — enables WASD movement)
  X    — quit

Robot moves ONLY while key is held.  Releasing stops immediately.
WASD controls only take effect in WALK mode (press 3 first).
"""


# ── Signal handling ─────────────────────────────────────────────────────────

def _force_exit(signum, frame):
    print("\nForce quit.")
    os._exit(0)


_sigint_count = 0


def _sigint_handler(signum, frame):
    global _sigint_count
    _sigint_count += 1
    if _sigint_count >= 2:
        _force_exit(signum, frame)
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, _sigint_handler)


def _clamp(value, min_val=-1.0, max_val=1.0):
    return max(min_val, min(max_val, float(value)))


# ── Controller ──────────────────────────────────────────────────────────────

class TeleopController:
    """Publish AnyJoy at 10 Hz for velocity, and publish mode-switch goals
    to the operational_mode_manager on demand.
    """

    def __init__(self, topic='/anyjoy/operator', rate_hz=10):
        self._topic = topic
        self._rate_hz = rate_hz

        # Velocity publisher (AnyJoy at 10 Hz)
        self._joy_pub = rospy.Publisher(topic, AnyJoy, queue_size=10)

        # Mode-switch publisher (one-shot goals)
        self._mode_pub = rospy.Publisher(
            '/operational_mode_manager/switch_operational_mode/goal',
            SwitchOperationalModeActionGoal,
            queue_size=1,
        )

        self._lock = threading.Lock()
        self._axes = [0.0] * NUM_AXES
        self._buttons = [0] * NUM_BUTTONS

        self._running = False
        self._thread = None
        self._current_mode = 'UNKNOWN'

    def start(self):
        """Start the 10 Hz background publish thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        rospy.loginfo("TeleopController: velocity on %s at %d Hz",
                      self._topic, self._rate_hz)
        rospy.loginfo("TeleopController: mode switch on "
                      "/operational_mode_manager/switch_operational_mode/goal")

    def shutdown(self):
        """Stop the publish thread.  Robot halts after 0.5 s timeout."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # ── Velocity ────────────────────────────────────────────────────────

    def set_velocity(self, heading=0.0, lateral=0.0, turning=0.0):
        with self._lock:
            self._axes[HEADING] = _clamp(heading)
            self._axes[LATERAL] = _clamp(lateral)
            self._axes[TURNING] = _clamp(turning)

    def stop(self):
        """Zero all velocity axes (publish thread keeps running)."""
        with self._lock:
            self._axes = [0.0] * NUM_AXES

    # ── Mode switching ──────────────────────────────────────────────────

    def switch_mode(self, mode_key):
        """Send a SwitchOperationalMode goal to the operational_mode_manager.

        Returns the mode name on success, None if the key is invalid.
        """
        mode = MODES.get(mode_key)
        if mode is None:
            return None

        # Publish the mode-switch goal
        msg = SwitchOperationalModeActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.goal.target.name = mode['op_mode']
        self._mode_pub.publish(msg)

        with self._lock:
            self._current_mode = mode['name']
            # Zero velocity on mode switch for safety
            self._axes = [0.0] * NUM_AXES

        rospy.loginfo("Mode switch requested: %s", mode['op_mode'])
        return mode['name']

    @property
    def current_mode(self):
        return self._current_mode

    # ── Publish loop (velocity only) ─────────────────────────────────

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

            self._joy_pub.publish(msg)
            time.sleep(interval)


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="WASD teleop with sit/stand/walk mode switching for ANYmal.",
        epilog="Press X to quit.",
    )
    parser.add_argument('--speed', type=float, default=0.3,
                        help="Initial speed as fraction of max velocity, "
                             "0.0-1.0 (default: 0.3)")
    parser.add_argument('--topic', type=str, default='/anyjoy/operator',
                        help="AnyJoy topic (default: /anyjoy/operator)")
    args = parser.parse_args()

    speed = max(0.05, min(1.0, args.speed))

    rospy.init_node('anymal_teleop_switchmodes', anonymous=True)
    ctrl = TeleopController(topic=args.topic)
    ctrl.start()

    # Save terminal settings and switch to raw / cbreak mode
    old_settings = termios.tcgetattr(sys.stdin)

    print(HELP_TEXT.format(speed=speed))
    print("Waiting for key input...\r")

    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            # Poll stdin with a short timeout.  When nothing is pressed
            # within the timeout window we send zero velocities (stop).
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()

                if key == 'x':
                    print("\rQuitting...                    ")
                    break

                elif key in ('+', '='):
                    speed = min(1.0, speed + 0.05)
                    print(f"\rSpeed: {speed:.0%}                   ",
                          end='', flush=True)
                    continue

                elif key == '-':
                    speed = max(0.05, speed - 0.05)
                    print(f"\rSpeed: {speed:.0%}                   ",
                          end='', flush=True)
                    continue

                elif key in MODES:
                    mode_name = ctrl.switch_mode(key)
                    print(f"\r>> MODE: {mode_name}                ",
                          end='', flush=True)
                    continue

                elif key in BINDINGS:
                    heading, lateral, turning = BINDINGS[key]
                    ctrl.set_velocity(
                        heading=heading * speed,
                        lateral=lateral * speed,
                        turning=turning * speed,
                    )
                    direction = {
                        'w': 'FWD', 's': 'BWD', 'a': 'LEFT',
                        'd': 'RIGHT', 'q': 'TURN_L', 'e': 'TURN_R',
                    }[key]
                    print(f"\r{direction} @ {speed:.0%} [{ctrl.current_mode}]   ",
                          end='', flush=True)
                    continue

            # No key pressed (or unrecognized key) — stop
            ctrl.stop()

    except KeyboardInterrupt:
        print("\r\nInterrupted.")
    finally:
        ctrl.stop()
        ctrl.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Teleop stopped. Robot should halt within 0.5s.")


if __name__ == '__main__':
    main()
