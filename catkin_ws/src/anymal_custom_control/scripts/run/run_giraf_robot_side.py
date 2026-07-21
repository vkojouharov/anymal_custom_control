#!/usr/bin/env python3
"""Start the single robot-side GIRAF ROS stack.

Dry-run is the default and opens no hardware devices.  Hardware mode requires
an explicit home confirmation and is intended only for supervised operation.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from urllib.parse import urlparse
import xmlrpc.client

from anymal_custom_control.control.giraf_arm_common import REMOTE_TASK_VELOCITY_LIMITS
from anymal_custom_control.runtime import LaunchSpec, ProcessManager, run_script_path


HARDWARE_CONFIRMATION = "ARM_PHYSICALLY_AT_HOME"
CONTROLLER_NODE = "/giraf_arm_controller"
CANDLE_NODE = "/candle_ros_node"


def master_uri() -> str:
    return os.environ.get("ROS_MASTER_URI", "http://localhost:11311")


def master_call(method: str, *args: object) -> object:
    proxy = xmlrpc.client.ServerProxy(master_uri())
    code, message, value = getattr(proxy, method)("/giraf_robot_side_launcher", *args)
    if code != 1:
        raise RuntimeError(f"ROS master {method} failed: {message}")
    return value


def master_available() -> bool:
    try:
        master_call("getPid")
        return True
    except (OSError, RuntimeError, xmlrpc.client.Error):
        return False


def master_address_is_local() -> bool:
    parsed = urlparse(master_uri())
    if not parsed.hostname:
        return False
    try:
        test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            test_socket.bind((parsed.hostname, 0))
            return True
        finally:
            test_socket.close()
    except OSError:
        return parsed.hostname in {"localhost", "127.0.0.1", socket.gethostname(), socket.getfqdn()}


def wait_for_master(timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if master_available():
            return
        time.sleep(0.1)
    raise RuntimeError(f"ROS master did not become available at {master_uri()} within {timeout_sec:.1f}s")


def registered_nodes() -> set[str]:
    publishers, subscribers, services = master_call("getSystemState")
    nodes: set[str] = set()
    for entries in (publishers, subscribers, services):
        for _resource, callers in entries:
            nodes.update(callers)
    return nodes


def main() -> int:
    parser = argparse.ArgumentParser(description="Start the robot-side GIRAF controller stack")
    parser.add_argument(
        "--backend",
        choices=("dry-run", "hardware"),
        default="dry-run",
        help="dry-run is the safe default and opens no actuator interfaces",
    )
    parser.add_argument(
        "--start-master",
        action="store_true",
        help="start roscore if ROS_MASTER_URI is local and no master is running",
    )
    parser.add_argument(
        "--confirm-home",
        default="",
        metavar=HARDWARE_CONFIRMATION,
        help=f"hardware-only acknowledgement; value must be exactly {HARDWARE_CONFIRMATION}",
    )
    parser.add_argument("--master-timeout", type=float, default=10.0)
    args = parser.parse_args()

    if args.backend == "hardware" and args.confirm_home != HARDWARE_CONFIRMATION:
        parser.error(
            "hardware mode requires physical home placement and "
            f"--confirm-home {HARDWARE_CONFIRMATION}"
        )

    manager = ProcessManager()
    try:
        if not master_available():
            if not args.start_master:
                raise RuntimeError(
                    f"No ROS master at {master_uri()}; pass --start-master only when this URI belongs to the NUC"
                )
            if not master_address_is_local():
                raise RuntimeError(
                    f"Refusing to start a local roscore for non-local ROS_MASTER_URI {master_uri()}"
                )
            port = urlparse(master_uri()).port or 11311
            print(f"Starting ROS master at {master_uri()}")
            manager.start(LaunchSpec(name="roscore", command=["roscore", "-p", str(port)]))
            wait_for_master(args.master_timeout)
        else:
            print(f"Using existing ROS master at {master_uri()}")

        conflicts = registered_nodes().intersection({CONTROLLER_NODE, CANDLE_NODE})
        if conflicts:
            raise RuntimeError(f"Refusing to start competing robot-side owners: {sorted(conflicts)}")

        controller_command = [
            sys.executable,
            str(run_script_path("run_giraf_arm_controller.py")),
            f"_backend:={'hardware' if args.backend == 'hardware' else 'dry_run'}",
            "_command_source:=teleop",
            "_task_velocity_limits:=[" + ",".join(str(value) for value in REMOTE_TASK_VELOCITY_LIMITS) + "]",
        ]

        if args.backend == "hardware":
            print("Starting CANdle/MD80 ROS node (hardware mode)")
            manager.start(
                LaunchSpec(
                    name="candle_ros_node",
                    command=["rosrun", "candle_ros", "candle_ros_node", "USB", "1M"],
                )
            )
            controller_command.append("_home_confirmed:=true")
        else:
            print("Starting dry-run controller; CAN, USB, serial, and motor services remain unopened")

        manager.start(LaunchSpec(name="giraf_arm_controller", command=controller_command))
        name, code = manager.wait_until_any_exit()
        print(f"{name} exited with code {code}")
        return code
    except KeyboardInterrupt:
        print("\nShutting down GIRAF robot-side stack...")
        return 0
    except Exception as exc:
        print(f"Robot-side startup failed: {exc}", file=sys.stderr)
        return 1
    finally:
        manager.terminate_all()


if __name__ == "__main__":
    raise SystemExit(main())
