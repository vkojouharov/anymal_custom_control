"""Dynamixel XH-430-W250-T control table + project-specific motor defaults.

Address tuples are (register_address, byte_length) — consumed by
DynamixelController.READ / .WRITE and by GroupSyncRead/Write handles.
"""

# ── Control table (XH-430-W250-T, Protocol 2.0) ─────────────────────────────
MODEL_NUMBER       = (0, 2)
ID_ADDR            = (7, 1)
BAUD_RATE          = (8, 1)
RETURN_DELAY_TIME  = (9, 1)
DRIVE_MODE         = (10, 1)
OPERATING_MODE     = (11, 1)
HOMING_OFFSET      = (20, 4)
MIN_POSITION_LIMIT = (52, 4)
MAX_POSITION_LIMIT = (48, 4)
PWM_LIMIT          = (36, 2)
CURRENT_LIMIT      = (38, 2)
VELOCITY_LIMIT     = (44, 4)

TORQUE_ENABLE         = (64, 1)
LED_ENABLE            = (65, 1)
POSITION_D_GAIN       = (80, 2)
POSITION_I_GAIN       = (82, 2)
POSITION_P_GAIN       = (84, 2)
GOAL_PWM              = (100, 2)
GOAL_CURRENT          = (102, 2)
GOAL_VELOCITY         = (104, 4)
PROFILE_ACCELERATION  = (108, 4)
PROFILE_VELOCITY      = (112, 4)
GOAL_POSITION         = (116, 4)
MOVING                = (122, 1)
PRESENT_PWM           = (124, 2)
PRESENT_LOAD          = (126, 2)
PRESENT_VELOCITY      = (128, 4)
PRESENT_POSITION      = (132, 4)
PRESENT_INPUT_VOLTAGE = (144, 2)
PRESENT_TEMPERATURE   = (146, 1)

# Operating-mode values (written to OPERATING_MODE)
OP_CURRENT           = 0
OP_VELOCITY          = 1
OP_POSITION          = 3
OP_EXTENDED_POSITION = 4
OP_CURRENT_POSITION  = 5
OP_PWM               = 16

TICKS_PER_REV = 4096  # XH-series single-turn resolution

# ── Project motor assignments ───────────────────────────────────────────────
# Arm joints (3 × XH-430-W250-T) + grippers (3 × XH-430-W250-T).
ARM_IDS     = (11, 12, 13)       # JOINT1, JOINT2, JOINT3
GRIPPER_IDS = (100, 101, 102)    # GRIPPER1, GRIPPER2, GRIPPER3
ALL_IDS     = ARM_IDS + GRIPPER_IDS

# Home (reference kinematic configuration) — ticks
ARM_HOME = {
    11: 2044,
    12: 3860,
    13: 4160,
}

# Gripper fully-open positions — ticks.  Fully-closed is OPEN - GRIPPER_STROKE.
GRIPPER_OPEN = {
    100: 1700,
    101:  100,
    102: 2100,
}
GRIPPER_STROKE = 4000  # tick delta from open to closed (extended-position mode)
