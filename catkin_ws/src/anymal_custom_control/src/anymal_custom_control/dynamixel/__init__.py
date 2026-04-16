"""Dynamixel (XH-430-W250-T) control for ANYmal GIRAF arm + grippers.

High-level functional API mirrors ``motor_driver.py`` (MD80): ``_connect``,
``_drive``, ``_read``/``_status``, ``_disconnect``.  See ``README.md`` for
the hardware-to-software map, wiring assumptions, and end-to-end examples.
"""
from .controller import DynamixelController
from .driver import (
    dynamixel_connect,
    dynamixel_disconnect,
    dynamixel_drive,
    dynamixel_drive_arm,
    dynamixel_drive_gripper,
    dynamixel_home,
    dynamixel_read,
    dynamixel_status,
    radians_to_ticks,
    ticks_to_radians,
)
from .control_table import (
    ALL_IDS,
    ARM_HOME,
    ARM_IDS,
    GRIPPER_IDS,
    GRIPPER_OPEN,
    GRIPPER_STROKE,
    TICKS_PER_REV,
)

__all__ = [
    # lifecycle
    'dynamixel_connect', 'dynamixel_disconnect',
    # drive
    'dynamixel_drive', 'dynamixel_drive_arm', 'dynamixel_drive_gripper',
    'dynamixel_home',
    # feedback
    'dynamixel_read', 'dynamixel_status',
    # conversions
    'radians_to_ticks', 'ticks_to_radians',
    # constants
    'ARM_IDS', 'GRIPPER_IDS', 'ALL_IDS', 'ARM_HOME', 'GRIPPER_OPEN',
    'GRIPPER_STROKE', 'TICKS_PER_REV',
    # low-level
    'DynamixelController',
]
