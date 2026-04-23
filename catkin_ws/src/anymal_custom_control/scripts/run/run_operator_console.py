#!/usr/bin/env python3
"""Preferred launcher for the unified operator console."""

import os
import sys

from anymal_custom_control.runtime.paths import operator_console_script_path


def main() -> None:
    target = operator_console_script_path("operator_console.py")
    os.execv(sys.executable, [sys.executable, str(target), *sys.argv[1:]])


if __name__ == "__main__":
    main()
