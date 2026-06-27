#!/usr/bin/env python3
"""Thin launcher for the ANYmal egocentric visual-servo ROS node."""

from anymal_custom_control.egocentric_servo.node import main


if __name__ == "__main__":
    raise SystemExit(main())
