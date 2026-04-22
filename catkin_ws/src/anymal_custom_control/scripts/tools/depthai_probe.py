#!/usr/bin/env python3
"""Preferred tool wrapper for the OAK-D probe."""

import os
import sys

from anymal_custom_control.runtime.paths import camera_script_path


def main() -> None:
    target = camera_script_path("depthai_probe.py")
    os.execv(sys.executable, [sys.executable, str(target), *sys.argv[1:]])


if __name__ == "__main__":
    main()
