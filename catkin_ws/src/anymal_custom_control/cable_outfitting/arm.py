from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np
import rospy
from dynamixel_sdk import COMM_SUCCESS

from anymal_custom_control.RRP_kinematic_model import get_boom_length_d3, get_boom_motor_rad
from anymal_custom_control.control.giraf_arm_common import (
    BOOM_MAX,
    BOOM_MIN,
    COMMAND_TIMEOUT_SEC,
    CONTROL_LOOP_HZ,
    D3_MIN,
    PITCH_KIN_OFFSET,
    PITCH_MAX,
    PITCH_MIN,
    QDOT_LIMITS,
    ROLL_LIMIT,
    TASK_VELOCITY_LIMITS,
    THETA4_DXL_SIGN,
    THETA4_KIN_OFFSET,
    THETA5_DXL_SIGN,
    THETA5_KIN_OFFSET,
    THETA6_KIN_OFFSET,
)
from anymal_custom_control.dynamixel import (
    ARM_HOME,
    ARM_IDS,
    ARM_TICK_LIMITS,
    GRIPPER_IDS,
    dynamixel_connect,
    dynamixel_disconnect,
    radians_to_ticks,
    ticks_to_radians,
)
from anymal_custom_control.dynamixel.control_table import OPERATING_MODE, OP_POSITION, TORQUE_ENABLE
from anymal_custom_control.motor_driver import motor_connect, motor_disconnect, motor_drive

from .console import Rate
from .kinematics import num_forward_transform, num_jacobian


DLS_DAMPING = 0.05
CABLE_GRIPPER_ID = 14
CABLE_GRIPPER_OPEN_TICKS = 2900
CABLE_GRIPPER_CLOSED_TICKS = 3340
MD80_GAIN_OVERRIDES = {11: {"kp": 200.0, "kd": 10.0, "max_torque": 10.0}}


def _joint_limit(mid: int, sign: float = 1.0) -> tuple[float, float]:
    lo, hi = ARM_TICK_LIMITS[mid]
    home = ARM_HOME[mid]
    values = sign * ticks_to_radians(lo - home), sign * ticks_to_radians(hi - home)
    return min(values), max(values)


THETA_LIMITS = (
    _joint_limit(ARM_IDS[0], THETA4_DXL_SIGN),
    _joint_limit(ARM_IDS[1], THETA5_DXL_SIGN),
    _joint_limit(ARM_IDS[2]),
)


@dataclass(frozen=True)
class ArmSnapshot:
    joints: np.ndarray
    T_base_tool: np.ndarray
    gripper_closed: bool
    task_velocity: np.ndarray
    control_hz: float


def _kinematic_joints(joints: np.ndarray) -> np.ndarray:
    return np.array(
        [
            joints[0],
            joints[1] + PITCH_KIN_OFFSET,
            joints[2],
            joints[3] + THETA4_KIN_OFFSET,
            joints[4] + THETA5_KIN_OFFSET,
            joints[5] + THETA6_KIN_OFFSET,
        ],
        dtype=float,
    )


def _damped_velocity(jacobian: np.ndarray, task_velocity: np.ndarray) -> np.ndarray:
    regularized = jacobian @ jacobian.T + DLS_DAMPING**2 * np.eye(6)
    return jacobian.T @ np.linalg.solve(regularized, task_velocity)


def _configure_gripper(dxl) -> None:
    if tuple(GRIPPER_IDS) != (CABLE_GRIPPER_ID,):
        raise RuntimeError(f"expected cable gripper motor {CABLE_GRIPPER_ID}")
    controller = dxl["controller"]
    if not controller.WRITE(CABLE_GRIPPER_ID, TORQUE_ENABLE, 0):
        raise RuntimeError("failed to disable gripper torque")
    if not controller.WRITE(CABLE_GRIPPER_ID, OPERATING_MODE, OP_POSITION):
        raise RuntimeError("failed to set gripper position mode")
    if controller.READ(CABLE_GRIPPER_ID, OPERATING_MODE) != OP_POSITION:
        raise RuntimeError("gripper position mode verification failed")
    if not controller.WRITE(CABLE_GRIPPER_ID, TORQUE_ENABLE, 1):
        raise RuntimeError("failed to enable gripper torque")


def _ticks(joints: np.ndarray, gripper_closed: bool) -> list[int]:
    return [
        ARM_HOME[ARM_IDS[0]] + radians_to_ticks(THETA4_DXL_SIGN * joints[3]),
        ARM_HOME[ARM_IDS[1]] + radians_to_ticks(THETA5_DXL_SIGN * joints[4]),
        ARM_HOME[ARM_IDS[2]] + radians_to_ticks(joints[5]),
        CABLE_GRIPPER_CLOSED_TICKS if gripper_closed else CABLE_GRIPPER_OPEN_TICKS,
    ]


def _drive_dynamixels(dxl, ticks: list[int]) -> bool:
    limits = dict(ARM_TICK_LIMITS)
    limits[CABLE_GRIPPER_ID] = tuple(sorted((CABLE_GRIPPER_OPEN_TICKS, CABLE_GRIPPER_CLOSED_TICKS)))
    if len(ticks) != len(dxl["all_ids"]):
        raise ValueError("Dynamixel target count does not match connected motors")
    writer = dxl["sync_write_pos"]
    for motor_id, target in zip(dxl["all_ids"], ticks):
        target = int(np.clip(target, *limits[motor_id]))
        if not writer.addParam(motor_id, target.to_bytes(4, "little", signed=True)):
            writer.clearParam()
            return False
    result = writer.txPacket()
    writer.clearParam()
    return result == COMM_SUCCESS


class CableArm:
    def __init__(self, stop: threading.Event):
        self.stop = stop
        self.ready = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="cable_arm")
        self._task = np.zeros(6, dtype=float)
        self._home_mab_position = None
        self._command_time = time.monotonic()
        self._gripper_closed = False
        self._joints = np.array([0.0, 0.0, D3_MIN, 0.0, 0.0, 0.0], dtype=float)
        self._transform = np.eye(4, dtype=float)
        self._control_hz = 0.0
        self._failure: str | None = None

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        self._thread.join(timeout)

    @property
    def failure(self) -> str | None:
        with self._lock:
            return self._failure

    def command(self, task_velocity, gripper_closed: bool | None = None) -> None:
        task = np.asarray(task_velocity, dtype=float).reshape(-1)
        if task.shape != (6,) or not np.all(np.isfinite(task)):
            raise ValueError("arm task velocity must contain six finite values")
        if gripper_closed is not None and not isinstance(gripper_closed, (bool, np.bool_)):
            raise ValueError("gripper command must be true, false, or null")
        with self._lock:
            self._task = np.clip(task, -TASK_VELOCITY_LIMITS, TASK_VELOCITY_LIMITS)
            self._home_mab_position = None
            if gripper_closed is not None:
                self._gripper_closed = bool(gripper_closed)
            self._command_time = time.monotonic()

    def command_home(self, mab_position) -> None:
        target = np.asarray(mab_position, dtype=float).reshape(-1)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("MAB home position must contain three finite values")
        with self._lock:
            self._task = np.zeros(6, dtype=float)
            self._home_mab_position = target.copy()
            self._gripper_closed = False
            self._command_time = time.monotonic()

    def stop_motion(self) -> None:
        self.command(np.zeros(6, dtype=float))

    def snapshot(self) -> ArmSnapshot:
        with self._lock:
            return ArmSnapshot(
                self._joints.copy(),
                self._transform.copy(),
                self._gripper_closed,
                self._task.copy(),
                self._control_hz,
            )

    def _run(self) -> None:
        md80 = dxl = None
        joints = self._joints.copy()
        dt = 1.0 / CONTROL_LOOP_HZ
        rate = Rate()
        try:
            print("connecting cable arm hardware")
            md80 = motor_connect(gain_overrides=MD80_GAIN_OVERRIDES)
            dxl = dynamixel_connect(baudrate=1_000_000)
            _configure_gripper(dxl)
            self.ready.set()
            print("cable arm ready")
            rate.reset()
            while not self.stop.is_set() and not rospy.is_shutdown():
                tick = time.monotonic()
                with self._lock:
                    task = self._task.copy()
                    home_mab_position = (
                        None
                        if self._home_mab_position is None
                        else self._home_mab_position.copy()
                    )
                    command_age = tick - self._command_time
                    gripper_closed = self._gripper_closed
                if command_age > COMMAND_TIMEOUT_SEC:
                    task[:] = 0.0
                if home_mab_position is None:
                    jacobian = np.asarray(num_jacobian(_kinematic_joints(joints)), dtype=float)
                    velocity = np.clip(_damped_velocity(jacobian, task), -QDOT_LIMITS, QDOT_LIMITS)
                    if not np.all(np.isfinite(velocity)):
                        raise ValueError("DLS produced a non-finite joint velocity")
                    joints += dt * velocity
                    joints[0] = np.clip(joints[0], -ROLL_LIMIT, ROLL_LIMIT)
                    joints[1] = np.clip(joints[1], PITCH_MIN, PITCH_MAX)
                    joints[2] = max(joints[2], D3_MIN)
                    for index, limits in enumerate(THETA_LIMITS, start=3):
                        joints[index] = np.clip(joints[index], *limits)
                    boom = float(np.clip(get_boom_motor_rad(joints[2]), BOOM_MIN, BOOM_MAX))
                else:
                    joints[0] = np.clip(home_mab_position[0], -ROLL_LIMIT, ROLL_LIMIT)
                    joints[1] = np.clip(home_mab_position[1], PITCH_MIN, PITCH_MAX)
                    boom = float(np.clip(home_mab_position[2], BOOM_MIN, BOOM_MAX))
                    joints[3:] = 0.0
                joints[2] = get_boom_length_d3(boom)
                transform = np.asarray(num_forward_transform(_kinematic_joints(joints)), dtype=float)
                motor_drive(md80, joints[0], joints[1], boom)
                if not _drive_dynamixels(dxl, _ticks(joints, gripper_closed)):
                    raise RuntimeError("Dynamixel command failed")
                with self._lock:
                    self._joints = joints.copy()
                    self._transform = transform
                    self._control_hz = rate.tick(time.monotonic())
                self.stop.wait(max(0.0, dt - (time.monotonic() - tick)))
        except Exception as exc:
            with self._lock:
                self._failure = f"cable arm failed: {exc}"
            print(self._failure)
            self.ready.set()
            self.stop.set()
        finally:
            if md80 is not None:
                try:
                    motor_disconnect()
                except Exception as exc:
                    print(f"MD80 shutdown warning: {exc}")
            if dxl is not None:
                try:
                    dynamixel_disconnect(dxl)
                except Exception as exc:
                    print(f"Dynamixel shutdown warning: {exc}")
            print("cable arm stopped")
