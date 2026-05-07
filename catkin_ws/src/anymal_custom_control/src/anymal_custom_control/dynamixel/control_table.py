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

# ── motor assignments ───────────────────────────────────────────────
# Arm joints (3 × XH-430-W250-T) + gripper (1 × XH-430-W250-T).
# EXPERIMENTAL ROLL-PITCH-PITCH wrist
# ARM_IDS     = (11, 12, 13)       # JOINT1 (th4), JOINT2 (th5), JOINT3 (th6)
# GRIPPER_IDS = (14,)              # single gripper
# ALL_IDS     = ARM_IDS + GRIPPER_IDS

# # Home (reference kinematic configuration) — ticks
# ARM_HOME = {
#     11: 1850,
#     12: 3075,
#     13: 1025,
# }

# # Absolute tick limits for each wrist joint on the new hardware.
# ARM_TICK_LIMITS = {
#     11: (0, 3690),
#     12: (1003, 6000),    # asymmetric range from bench calibration
#     13: (-1500, 4068),    # home 1060, approx +/- 2000 ticks
# }

# GRIPPER_OPEN = {
#     14: 680,
# }
# GRIPPER_CLOSED = {
#     14: -1685,
# }

# below is stanley metal wrist

# # ── motor assignments ───────────────────────────────────────────────
# Arm joints (3 × XH-430-W250-T) + gripper (1 × XH-430-W250-T).
ARM_IDS     = (11, 12, 13)       # JOINT1 (th4), JOINT2 (th5), JOINT3 (th6)
GRIPPER_IDS = (14,)              # single gripper
ALL_IDS     = ARM_IDS + GRIPPER_IDS

# Home (reference kinematic configuration) — ticks
ARM_HOME = {
    11: 2030,
    12: 2003,
    13: 2068,
}

# Absolute tick limits for each wrist joint on the new hardware.
ARM_TICK_LIMITS = {
    11: (1030, 3690),
    12: (1003, 3003),    # asymmetric range from bench calibration
    13: (68, 4068),    # home 1060, approx +/- 2000 ticks
}

GRIPPER_OPEN = {
    14: 680,
}
GRIPPER_CLOSED = {
    14: -1685,
}

# ---------- OLD GRIPPER ----------
# # Arm joints (3 × XH-430-W250-T) + gripper (1 × XH-430-W250-T).
# ARM_IDS     = (11, 12, 13)       # JOINT1 (th4), JOINT2 (th5), JOINT3 (th6)
# GRIPPER_IDS = (14,)              # single gripper
# ALL_IDS     = ARM_IDS + GRIPPER_IDS

# # Home (reference kinematic configuration) — ticks
# ARM_HOME = {
#     11: 3075,
#     12: 2050,
#     13: 1075,
# }

# # Absolute tick limits for each wrist joint on the new hardware.
# ARM_TICK_LIMITS = {
#     11: (1900, 3850),      # home 2057, approx +/- 2000 ticks
#     12: (950, 3090),    # asymmetric range from bench calibration
#     13: (-1075, 3075),    # home 1060, approx +/- 2000 ticks
# }

# GRIPPER_OPEN = {
#     14: 1190,
# }
# GRIPPER_CLOSED = {
#     14: 4925,
# }

GRIPPER_STROKE = GRIPPER_OPEN[14] - GRIPPER_CLOSED[14]
GRIPPER_TICK_LIMITS = {
    14: (min(GRIPPER_OPEN[14], GRIPPER_CLOSED[14]), max(GRIPPER_OPEN[14], GRIPPER_CLOSED[14])),
}

# Final safety clamp used by the driver before every sync-write.
GOAL_TICK_LIMITS = {
    **ARM_TICK_LIMITS,
    **GRIPPER_TICK_LIMITS,
}
