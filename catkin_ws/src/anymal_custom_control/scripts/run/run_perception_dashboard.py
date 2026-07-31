#!/usr/bin/env python3
"""Preferred launcher for the current OAK-D perception dashboard."""

import os
import sys

from anymal_custom_control.runtime.paths import camera_script_path


def main() -> None:
    target = camera_script_path("depthai_apriltag_rgb_n_depth.py")
    os.execv(sys.executable, [sys.executable, str(target), *sys.argv[1:]])


if __name__ == "__main__":
    main()
