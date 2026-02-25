#!/usr/bin/env python3
"""WASD keyboard teleop for ANYmal robot.

Moves only while a key is held down. Releases to zero automatically.

Controls:
    W / S   — forward / backward
    A / D   — strafe left / strafe right
    Q / E   — turn left / turn right
    +/-     — increase / decrease speed
    X       — quit

Prerequisites:
    - ANYmal software running
    - Robot in locomotion mode (activate trotting via GUI)
    - E-stop ready

Usage:
    python3 teleop.py
    python3 teleop.py --speed 0.3
    python3 teleop.py --help
"""

import argparse
import os
import select
import signal
import sys
import termios
import tty

import rospy

# Resolve import path before we mess with terminal settings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from anymal_custom_control.movement import MovementController


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


# Key -> (heading, lateral, turning)
BINDINGS = {
    'w': (1.0,  0.0,  0.0),   # forward
    's': (-1.0, 0.0,  0.0),   # backward
    'a': (0.0,  1.0,  0.0),   # strafe left
    'd': (0.0, -1.0,  0.0),   # strafe right
    'q': (0.0,  0.0,  1.0),   # turn left (CCW)
    'e': (0.0,  0.0, -1.0),   # turn right (CW)
}

HELP_TEXT = """
ANYmal WASD Teleop
==================
  W/S  — forward/backward
  A/D  — strafe left/right
  Q/E  — turn left/right
  +/-  — increase/decrease speed (current: {speed:.0%})
  X    — quit

Robot moves ONLY while key is held. Releasing stops immediately.
"""


def main():
    parser = argparse.ArgumentParser(
        description="WASD keyboard teleop for ANYmal. Moves only while key is held.",
        epilog="Press X to quit."
    )
    parser.add_argument('--speed', type=float, default=0.3,
                        help="Initial speed as fraction of max velocity, 0.0-1.0 (default: 0.3)")
    parser.add_argument('--topic', type=str, default='/anyjoy/operator',
                        help="AnyJoy topic (default: /anyjoy/operator)")
    args = parser.parse_args()

    speed = max(0.05, min(1.0, args.speed))

    rospy.init_node('anymal_teleop', anonymous=True)
    mc = MovementController(topic=args.topic)
    mc.start()

    # Save and set terminal to raw mode (character-by-character input)
    old_settings = termios.tcgetattr(sys.stdin)

    print(HELP_TEXT.format(speed=speed))
    print("Waiting for key input...\r")

    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            # Check for keypress with short timeout
            # When no key is pressed within timeout, we send zero (stop)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()

                if key == 'x':
                    print("\rQuitting...                    ")
                    break
                elif key == '+' or key == '=':
                    speed = min(1.0, speed + 0.05)
                    print(f"\rSpeed: {speed:.0%}                   ", end='', flush=True)
                    continue
                elif key == '-':
                    speed = max(0.05, speed - 0.05)
                    print(f"\rSpeed: {speed:.0%}                   ", end='', flush=True)
                    continue
                elif key in BINDINGS:
                    heading, lateral, turning = BINDINGS[key]
                    mc.set_velocity(
                        heading=heading * speed,
                        lateral=lateral * speed,
                        turning=turning * speed,
                    )
                    direction = {
                        'w': 'FWD', 's': 'BWD', 'a': 'LEFT',
                        'd': 'RIGHT', 'q': 'TURN_L', 'e': 'TURN_R'
                    }[key]
                    print(f"\r{direction} @ {speed:.0%}              ", end='', flush=True)
                    continue

            # No key pressed (or unrecognized key) — stop
            mc.stop()

    except KeyboardInterrupt:
        print("\r\nInterrupted.")
    finally:
        mc.stop()
        mc.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Teleop stopped. Robot should halt within 0.5s.")


if __name__ == '__main__':
    main()
