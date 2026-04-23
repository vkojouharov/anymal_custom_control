#!/usr/bin/env python3
"""WASD teleop with mode switching (Rest/Stand/Walk) for ANYmal robot.

Controls:
    W / S   — forward / backward
    A / D   — strafe left / strafe right
    Q / E   — turn left / turn right
    +/-     — increase / decrease speed
    1       — switch to REST mode
    2       — switch to STAND mode
    3       — switch to WALK mode (locomotion — enables WASD movement)
    X       — quit

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
import tty

import rospy
from anymal_custom_control import ModeController, MovementController


# ── Key bindings ────────────────────────────────────────────────────────────
# Key -> (heading, lateral, turning)
BINDINGS = {
    'w': ( 1.0,  0.0,  0.0),   # forward
    's': (-1.0,  0.0,  0.0),   # backward
    'a': ( 0.0,  1.0,  0.0),   # strafe left
    'd': ( 0.0, -1.0,  0.0),   # strafe right
    'q': ( 0.0,  0.0,  1.0),   # turn left  (CCW)
    'e': ( 0.0,  0.0, -1.0),   # turn right (CW)
}

MODE_KEYS = {
    '1': ModeController.REST,
    '2': ModeController.STAND,
    '3': ModeController.WALK,
}

DIRECTION_NAMES = {
    'w': 'FWD', 's': 'BWD', 'a': 'LEFT',
    'd': 'RIGHT', 'q': 'TURN_L', 'e': 'TURN_R',
}

HELP_TEXT = """
ANYmal WASD Teleop + Mode Switching
====================================
  W/S  — forward / backward
  A/D  — strafe left / right
  Q/E  — turn left / right
  +/-  — increase / decrease speed (current: {speed:.0%})
  1    — REST
  2    — STAND
  3    — WALK   (locomotion — enables WASD movement)
  X    — quit

Robot moves ONLY while key is held.  Releasing stops immediately.
WASD controls only take effect in WALK mode (press 3 first).
"""


# ── Signal handling ─────────────────────────────────────────────────────────

_sigint_count = 0


def _sigint_handler(signum, frame):
    global _sigint_count
    _sigint_count += 1
    if _sigint_count >= 2:
        print("\nForce quit.")
        os._exit(0)
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, _sigint_handler)


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="WASD teleop with mode switching for ANYmal.",
        epilog="Press X to quit.",
    )
    parser.add_argument('--speed', type=float, default=0.3,
                        help="Initial speed fraction 0.0-1.0 (default: 0.3)")
    parser.add_argument('--topic', type=str, default='/anyjoy/operator',
                        help="AnyJoy topic (default: /anyjoy/operator)")
    args = parser.parse_args()

    speed = max(0.05, min(1.0, args.speed))

    rospy.init_node('anymal_teleop_switchmodes', anonymous=True)

    mc = MovementController(topic=args.topic)
    mc.start()
    modes = ModeController(movement_controller=mc)

    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP_TEXT.format(speed=speed))
    print("Waiting for key input...\r")

    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
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

                elif key in MODE_KEYS:
                    mode_name = modes.switch_mode(MODE_KEYS[key])
                    print(f"\r>> MODE: {mode_name}                ",
                          end='', flush=True)
                    continue

                elif key in BINDINGS:
                    heading, lateral, turning = BINDINGS[key]
                    mc.set_velocity(
                        heading=heading * speed,
                        lateral=lateral * speed,
                        turning=turning * speed,
                    )
                    print(f"\r{DIRECTION_NAMES[key]} @ {speed:.0%} "
                          f"[{modes.current_mode or 'UNKNOWN'}]   ",
                          end='', flush=True)
                    continue

            # No key pressed — stop
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
