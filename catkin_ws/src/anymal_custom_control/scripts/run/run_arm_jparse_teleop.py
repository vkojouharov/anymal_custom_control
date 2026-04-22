#!/usr/bin/env python3
"""Preferred launcher for J-PARSE arm wrist teleop."""

import os
import sys

from anymal_custom_control.runtime.paths import legacy_script_path


def main() -> None:
    target = legacy_script_path("RUN_arm_wrist_JPARSE_teleop.py")
    os.execv(sys.executable, [sys.executable, str(target), *sys.argv[1:]])


if __name__ == "__main__":
    main()
