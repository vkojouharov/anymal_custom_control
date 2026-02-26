#!/usr/bin/env python3
"""Joystick teleop with mode switching for ANYmal robot.

Controls:
    Left stick Y   — forward / backward
    Left stick X   — turn left / turn right
    Right stick X  — strafe left / strafe right
    LB + RB        — dead man's switch (BOTH must be held for movement)
    A button       — REST mode
    B button       — STAND mode
    Y button       — WALK mode
    MENULEFT       — decrease speed 10%
    MENURIGHT      — increase speed 10%
    X button       — emergency stop + quit

Prerequisites:
    - ANYmal software running
    - Xbox controller plugged in
    - E-stop ready

Usage:
    python3 run_teleop_joystick.py
    python3 run_teleop_joystick.py --speed 0.3
    python3 run_teleop_joystick.py --topic /anyjoy/operator
"""

import argparse
import os
import signal
import sys
import time

import rospy
from anymal_custom_control import ModeController, MovementController
from anymal_custom_control.joystick_driver import (
    joystick_connect,
    joystick_disconnect,
    joystick_read,
)

# Mode button mapping
MODE_BUTTONS = {
    'AB': ModeController.REST,
    'BB': ModeController.STAND,
    'YB': ModeController.WALK,
}

HELP_TEXT = """
ANYmal Joystick Teleop + Mode Switching
========================================
  Left stick Y    — forward / backward
  Left stick X    — turn left / right
  Right stick X   — strafe left / right
  LB + RB         — dead man's switch (hold BOTH for movement)
  A button        — REST
  B button        — STAND
  Y button        — WALK   (enables movement)
  MENULEFT        — decrease speed 10%
  MENURIGHT       — increase speed 10%
  X button        — EMERGENCY STOP + quit

Max speed: {speed:.0%}
Movement only works when BOTH bumpers are held AND in WALK mode.
"""


# ── Signal handling ──────────────────────────────────────────────────────────

_sigint_count = 0


def _sigint_handler(signum, frame):
    global _sigint_count
    _sigint_count += 1
    if _sigint_count >= 2:
        print("\nForce quit.")
        os._exit(0)
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, _sigint_handler)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Joystick teleop with mode switching for ANYmal.",
    )
    parser.add_argument('--speed', type=float, default=0.5,
                        help="Max speed fraction 0.0-1.0 (default: 0.5)")
    parser.add_argument('--topic', type=str, default='/anyjoy/operator',
                        help="AnyJoy topic (default: /anyjoy/operator)")
    args = parser.parse_args()

    speed = max(0.05, min(1.0, args.speed))

    # Connect joystick first (fail fast if not plugged in)
    js = joystick_connect()
    print(f"Controller connected: {js['device'].name}")

    rospy.init_node('anymal_teleop_joystick', anonymous=True)

    mc = MovementController(topic=args.topic)
    mc.start()
    modes = ModeController(movement_controller=mc)

    print(HELP_TEXT.format(speed=speed))

    # Track previous button states for rising-edge detection
    edge_buttons = list(MODE_BUTTONS.keys()) + ['MENULEFT', 'MENURIGHT', 'XB']
    prev_buttons = {btn: 0 for btn in edge_buttons}

    try:
        while not rospy.is_shutdown():
            data = joystick_read(js)

            # ── Emergency stop + quit (X button) ─────────────────────
            if data['XB'] and not prev_buttons['XB']:
                mc.stop()
                print("\r!! EMERGENCY STOP !!                     ")
                break

            # ── Speed adjustment (rising edge) ───────────────────────
            if data['MENULEFT'] and not prev_buttons['MENULEFT']:
                speed = max(0.05, speed - 0.10)
                rospy.loginfo("Speed decreased to %.0f%%", speed * 100)

            if data['MENURIGHT'] and not prev_buttons['MENURIGHT']:
                speed = min(1.0, speed + 0.10)
                rospy.loginfo("Speed increased to %.0f%%", speed * 100)

            # ── Mode switching (rising edge only) ────────────────────
            for btn, mode in MODE_BUTTONS.items():
                if data[btn] and not prev_buttons[btn]:
                    mode_name = modes.switch_mode(mode)
                    print(f"\r>> MODE: {mode_name}                     ",
                          end='', flush=True)

            # ── Dead man's switch: both bumpers must be held ─────────
            bumpers_held = data['LB'] and data['RB']

            if bumpers_held:
                heading = data['LY'] * speed
                turning = -data['LX'] * speed
                lateral = -data['RX'] * speed
                mc.set_velocity(heading=heading, lateral=lateral, turning=turning)

                print(f"\r  H:{heading:+.2f}  L:{lateral:+.2f}  T:{turning:+.2f}"
                      f"  spd:{speed:.0%}  [{modes.current_mode or '?'}]   ",
                      end='', flush=True)
            else:
                mc.stop()

            # Update previous button states
            for btn in prev_buttons:
                prev_buttons[btn] = data[btn]

            time.sleep(0.02)  # ~50 Hz

    except KeyboardInterrupt:
        print("\r\nInterrupted.")
    finally:
        mc.stop()
        mc.shutdown()
        joystick_disconnect(js)
        print("Teleop stopped. Robot should halt within 0.5s.")


if __name__ == '__main__':
    main()
