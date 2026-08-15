from __future__ import annotations

import argparse
import threading
import time
from collections import deque

import numpy as np
import rospy

from anymal_custom_control import MovementController
from anymal_custom_control.control.giraf_arm_common import TASK_VELOCITY_LIMITS
from anymal_custom_control.joystick_driver import joystick_connect, joystick_disconnect, joystick_read

from .arm import CableArm
from .camera import CameraWorker
from .mpc import BaseMpc, CONTROL_HZ, DT, goal_reached, median_state, movement_axes, state_from_tag
from .policies import POLICY_REGISTRY
from .trajectory import HardwareConfig, TaskPoint, Trajectory, load_hardware, load_trajectory


LOOP_HZ = 100.0
TAG_TIMEOUT_S = 1.0
TAG_FRESH_S = 0.25
TAG_STABLE_S = 0.25
GOAL_STABLE_S = 0.5


class Operator:
    def __init__(self, stop: threading.Event):
        self.stop = stop
        self.ready = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="joystick")
        self._events = {"start": False, "pause": False, "estop": False}
        self._failure: str | None = None

    def start(self):
        self._thread.start()

    def join(self, timeout=None):
        self._thread.join(timeout)

    @property
    def failure(self):
        with self._lock:
            return self._failure

    def consume(self):
        with self._lock:
            events = self._events.copy()
            self._events = {key: False for key in self._events}
        return events

    def _run(self):
        joystick = None
        previous = {"YB": 0, "BB": 0, "XB": 0}
        try:
            joystick = joystick_connect()
            print(f"joystick ready: {joystick['device'].name}")
            self.ready.set()
            while not self.stop.is_set() and not rospy.is_shutdown():
                data = joystick_read(joystick)
                pressed = {
                    "start": data["YB"] and not previous["YB"],
                    "pause": data["BB"] and not previous["BB"],
                    "estop": data["XB"] and not previous["XB"],
                }
                with self._lock:
                    for key, value in pressed.items():
                        self._events[key] |= bool(value)
                previous = {key: data[key] for key in previous}
                self.stop.wait(0.01)
        except Exception as exc:
            with self._lock:
                self._failure = f"joystick failed: {exc}"
            print(self._failure)
            self.ready.set()
            self.stop.set()
        finally:
            if joystick is not None:
                joystick_disconnect(joystick)


class CableExecutor:
    def __init__(self, hardware: HardwareConfig, trajectory: Trajectory):
        self.trajectory = trajectory
        self.stop = threading.Event()
        self.navigation_camera = CameraWorker("navigation", hardware.navigation_camera, self.stop, require_usb3=True)
        self.arm_camera = CameraWorker("arm", hardware.arm_camera, self.stop)
        self.arm = CableArm(self.stop)
        self.operator = Operator(self.stop)
        self.movement = MovementController(rate_hz=int(CONTROL_HZ))
        self.mpc = BaseMpc()
        self.phase = "WAITING"
        self.resume_phase: str | None = None
        self.point_index = 0
        self.camera_role: str | None = None
        self.failure: str | None = None
        self.policy = self.policy_observation = self.policy_command = None
        self._started_threads = []
        self._nav_samples = deque(maxlen=3)
        self._last_nav_tag_stamp = -1.0
        self._last_solve_tag_stamp = -1.0
        self._nav_grace_until = 0.0
        self._nav_state = None
        self._control_trajectory = None
        self._control_time = 0.0
        self._previous_control = np.zeros(3)
        self._next_solve = 0.0
        self._next_base_publish = 0.0
        self._solve_failures = 0
        self._goal_stable = 0.0
        self._deploy_elapsed = 0.0
        self._deploy_stable = 0.0
        self._last_step = time.monotonic()

    @property
    def point(self) -> TaskPoint:
        return self.trajectory.task_points[self.point_index]

    def _start_thread(self, component) -> None:
        component.start()
        self._started_threads.append(component)

    def _wait_ready(self, component, label: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while not component.ready.wait(0.1):
            if time.monotonic() >= deadline or self.stop.is_set():
                break
        failure = component.failure
        if failure:
            raise RuntimeError(failure)
        if not component.ready.is_set():
            raise RuntimeError(f"timed out waiting for {label}")

    def _select_camera(self, role: str | None) -> None:
        if role == self.camera_role:
            return
        self.navigation_camera.set_active(role == "navigation")
        self.arm_camera.set_active(role == "arm")
        self.camera_role = role

    @staticmethod
    def _tag(camera: CameraWorker, tag_id: int, now: float):
        snapshot = camera.snapshot()
        tag = snapshot.latest_tags.get(tag_id)
        visible = tag_id in snapshot.visible_tags
        age = float("inf") if tag is None else max(0.0, now - tag.stamp_sec)
        return tag, visible, age

    def _base(self, heading=0.0, lateral=0.0, turning=0.0, force=False) -> None:
        self.movement.set_velocity(heading=heading, lateral=lateral, turning=turning)
        now = time.monotonic()
        if force or now >= self._next_base_publish:
            self.movement.publish_once()
            self._next_base_publish = now + 1.0 / CONTROL_HZ

    def _reset_navigation(self, now: float) -> None:
        self._nav_samples.clear()
        self._last_nav_tag_stamp = -1.0
        self._last_solve_tag_stamp = -1.0
        self._nav_state = None
        self._control_trajectory = None
        self._previous_control[:] = 0.0
        self._next_solve = now
        self._solve_failures = 0
        self._goal_stable = 0.0

    def _begin_navigation(self, now: float) -> None:
        self.phase = "NAVIGATING"
        self._select_camera("navigation")
        self.arm.stop_motion()
        self._reset_navigation(now)
        self._nav_grace_until = now + TAG_TIMEOUT_S
        print(f"point {self.point_index + 1}/{len(self.trajectory.task_points)} {self.point.name}: navigating to tag {self.point.navigation_tag_id}")

    def _begin_deployment(self, now: float) -> None:
        self.phase = "DEPLOYING"
        self._select_camera("arm")
        self._base(force=True)
        self._deploy_elapsed = self._deploy_stable = 0.0
        self._last_step = now
        print(f"{self.point.name}: deploying until arm tag {self.point.manipulation_tag_id} is acquired")

    def _begin_policy(self, now: float) -> None:
        policy_class, self.policy_observation, self.policy_command = POLICY_REGISTRY[self.point.policy]
        self.policy = policy_class()
        self.phase = "MANIPULATING"
        self._last_step = now
        self.arm.stop_motion()
        print(f"{self.point.name}: starting {self.point.policy} policy on tag {self.point.manipulation_tag_id}")

    def _pause(self, reason: str) -> None:
        if self.phase not in {"NAVIGATING", "DEPLOYING", "MANIPULATING"}:
            return
        self.resume_phase = self.phase
        self.phase = "PAUSED"
        self.arm.stop_motion()
        self._base(force=True)
        print(f"paused ({reason}); press Y to resume")

    def _resume(self, now: float) -> None:
        if self.resume_phase == "NAVIGATING":
            _, visible, age = self._tag(self.navigation_camera, self.point.navigation_tag_id, now)
            if not visible or age > TAG_FRESH_S:
                print("resume ignored: navigation tag is not fresh")
                return
            self.phase = "NAVIGATING"
            self._reset_navigation(now)
        elif self.resume_phase == "DEPLOYING":
            self.phase = "DEPLOYING"
            self._deploy_stable = 0.0
            self._last_step = now
        elif self.resume_phase == "MANIPULATING":
            _, visible, age = self._tag(self.arm_camera, self.point.manipulation_tag_id, now)
            if not visible or age > TAG_FRESH_S:
                print("resume ignored: manipulation tag is not fresh")
                return
            self.phase = "MANIPULATING"
            self._last_step = now
        else:
            return
        print(f"resumed {self.phase.lower()}")

    def _step_navigation(self, now: float) -> None:
        self.arm.stop_motion()
        tag, _, age = self._tag(self.navigation_camera, self.point.navigation_tag_id, now)
        if age > TAG_TIMEOUT_S and now >= self._nav_grace_until:
            self._pause("navigation tag lost")
            return
        if tag is not None and tag.stamp_sec != self._last_nav_tag_stamp:
            state = state_from_tag(tag.T_camera_tag)
            if np.all(np.isfinite(state)):
                self._nav_samples.append((tag.stamp_sec, state))
                self._last_nav_tag_stamp = tag.stamp_sec
        newest_stamp = self._nav_samples[-1][0] if len(self._nav_samples) == 3 else -1.0
        if len(self._nav_samples) == 3 and newest_stamp != self._last_solve_tag_stamp and now >= self._next_solve:
            self._next_solve = now + DT
            self._last_solve_tag_stamp = newest_stamp
            state = median_state([sample[1] for sample in self._nav_samples])
            result = self.mpc.solve(state, self.point.navigation_goal, self._previous_control)
            if result is None:
                self._solve_failures += 1
                self._control_trajectory = None
                self._base(force=True)
                if self._solve_failures >= 3:
                    raise RuntimeError("MPC failed three consecutive solves")
            else:
                self._solve_failures = 0
                self._nav_state = state
                self._control_trajectory = result.trajectory
                self._control_time = now
                self._previous_control = result.trajectory[0].copy()
                self._goal_stable = self._goal_stable + DT if goal_reached(state, self.point.navigation_goal) else 0.0
                print(f"nav state={np.round(state, 3)} u0={np.round(result.trajectory[0], 3)} cost={result.state_cost:.4f} solve={result.solve_ms:.1f}ms")
                if self._goal_stable >= GOAL_STABLE_S:
                    self._begin_deployment(now)
                    return
        if self._control_trajectory is None or self._nav_state is None:
            self._base()
            return
        index = min(int(max(0.0, now - self._control_time) / DT), len(self._control_trajectory) - 1)
        heading, lateral, turning = movement_axes(self._nav_state, self._control_trajectory[index])
        self._base(heading, lateral, turning)

    def _step_deployment(self, now: float) -> None:
        self._base()
        dt = max(0.0, now - self._last_step)
        self._last_step = now
        tag, visible, age = self._tag(self.arm_camera, self.point.manipulation_tag_id, now)
        self._deploy_stable = self._deploy_stable + dt if tag is not None and visible and age <= TAG_FRESH_S else 0.0
        if self._deploy_stable >= TAG_STABLE_S:
            self._begin_policy(now)
            return
        self._deploy_elapsed += dt
        if self._deploy_elapsed >= self.point.deployment_timeout_s:
            raise RuntimeError(f"deployment timed out after {self.point.deployment_timeout_s:.2f}s")
        self.arm.command(self.point.deployment_twist)

    def _step_policy(self, now: float) -> None:
        self._base()
        tag, visible, age = self._tag(self.arm_camera, self.point.manipulation_tag_id, now)
        if age > TAG_TIMEOUT_S:
            self._pause("manipulation tag lost")
            return
        arm = self.arm.snapshot()
        observation = self.policy_observation(
            T_camera_tag=None if tag is None else tag.T_camera_tag.copy(),
            tag_age_sec=age,
            tag_visible=visible,
            tag_decision_margin=None if tag is None else tag.decision_margin,
            camera_available=self.arm_camera.failure is None,
            joint_position=arm.joints,
            T_base_tool=arm.T_base_tool,
            gripper_closed=arm.gripper_closed,
            dt_sec=max(1e-6, now - self._last_step),
        )
        self._last_step = now
        command = self.policy.step(observation)
        if not isinstance(command, self.policy_command):
            raise TypeError("policy returned the wrong command type")
        task = np.asarray(command.task_velocity_base, dtype=float).reshape(-1)
        if task.shape != (6,) or not np.all(np.isfinite(task)):
            raise ValueError("policy returned an invalid task velocity")
        self.arm.command(np.clip(task, -TASK_VELOCITY_LIMITS, TASK_VELOCITY_LIMITS), command.gripper_closed)
        if self.policy.finished:
            self.arm.stop_motion()
            if self.point_index + 1 == len(self.trajectory.task_points):
                self.phase = "COMPLETE"
                self._select_camera(None)
                self._base(force=True)
                print("trajectory complete; holding final arm pose (X or Ctrl-C to stop)")
            else:
                self.point_index += 1
                self.policy = self.policy_observation = self.policy_command = None
                self._begin_navigation(now)

    def _component_failure(self) -> str | None:
        return self.navigation_camera.failure or self.arm_camera.failure or self.arm.failure or self.operator.failure

    def run(self) -> int:
        user_stop = False
        try:
            self._select_camera("navigation")
            self._start_thread(self.navigation_camera)
            self._start_thread(self.arm_camera)
            self._wait_ready(self.navigation_camera, "navigation camera", 10.0)
            self._wait_ready(self.arm_camera, "arm camera", 10.0)
            self._start_thread(self.operator)
            self._wait_ready(self.operator, "joystick", 10.0)
            self._start_thread(self.arm)
            self._wait_ready(self.arm, "cable arm", 20.0)
            print(f"trajectory {self.trajectory.name}: {len(self.trajectory.task_points)} points")
            print("Y start/resume | B pause | X stop | ANYmal must already be in Walk")
            period = 1.0 / LOOP_HZ
            while not rospy.is_shutdown():
                tick = time.monotonic()
                failure = self._component_failure()
                if failure:
                    raise RuntimeError(failure)
                if self.stop.is_set():
                    break
                events = self.operator.consume()
                if events["estop"]:
                    print("X pressed: stopping")
                    user_stop = True
                    break
                if events["pause"]:
                    self._pause("operator")
                if events["start"]:
                    if self.phase == "WAITING":
                        _, visible, age = self._tag(self.navigation_camera, self.point.navigation_tag_id, tick)
                        if visible and age <= TAG_FRESH_S:
                            self._begin_navigation(tick)
                        else:
                            print("start ignored: navigation tag is not fresh")
                    elif self.phase == "PAUSED":
                        self._resume(tick)
                if self.phase == "NAVIGATING":
                    self._step_navigation(tick)
                elif self.phase == "DEPLOYING":
                    self._step_deployment(tick)
                elif self.phase == "MANIPULATING":
                    self._step_policy(tick)
                elif self.phase in {"WAITING", "PAUSED", "COMPLETE"}:
                    self.arm.stop_motion()
                    self._base()
                self.stop.wait(max(0.0, period - (time.monotonic() - tick)))
            failure = self._component_failure()
            if failure:
                raise RuntimeError(failure)
            return 0 if user_stop or rospy.is_shutdown() else 1
        except Exception as exc:
            self.failure = str(exc)
            print(f"trajectory fault: {exc}")
            return 1
        finally:
            try:
                self._base(force=True)
            except Exception as exc:
                print(f"base shutdown warning: {exc}")
            self.stop.set()
            for component in reversed(self._started_threads):
                component.join(3.0)
            alive = [component._thread.name for component in self._started_threads if component._thread.is_alive()]
            if alive:
                print(f"shutdown warning: threads still alive: {alive}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run an integrated cable-outfitting trajectory")
    parser.add_argument("hardware", help="dual-camera hardware YAML")
    parser.add_argument("trajectory", help="task-point trajectory YAML")
    args = parser.parse_args(rospy.myargv()[1:] if argv is None else argv)
    try:
        hardware = load_hardware(args.hardware)
        trajectory = load_trajectory(args.trajectory)
    except ValueError as exc:
        parser.error(str(exc))
    rospy.init_node("cable_outfitting_trajectory", anonymous=False)
    return CableExecutor(hardware, trajectory).run()


if __name__ == "__main__":
    raise SystemExit(main())
