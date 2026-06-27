#!/usr/bin/env python3
"""Thin launcher for the dedicated egocentric servo web console."""

import os
import sys

from anymal_custom_control.runtime.paths import scripts_root


def main() -> None:
    target = scripts_root() / "egocentric_servo_console" / "egocentric_servo_console.py"
    os.execv(sys.executable, [sys.executable, str(target), *sys.argv[1:]])


if __name__ == "__main__":
    main()
