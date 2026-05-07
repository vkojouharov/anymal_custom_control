#!/usr/bin/env python3
"""Launch the prototype AprilTag visual-servo demo stack."""

from __future__ import annotations

import argparse
import sys

from anymal_custom_control.runtime import LaunchSpec, ProcessManager, dashboard_url, run_script_path


def build_visual_servo_demo_specs(python_executable: str, port: int, diagnostics: bool) -> list[LaunchSpec]:
    autonomy_command = [
        python_executable,
        str(run_script_path("run_visual_servo_autonomy.py")),
        "--port",
        str(port),
    ]
    if diagnostics:
        autonomy_command.append("--diagnostics")

    return [
        LaunchSpec(
            name="giraf_arm_controller",
            command=[python_executable, str(run_script_path("run_giraf_arm_controller.py"))],
        ),
        LaunchSpec(
            name="oakd_sensor",
            command=[
                python_executable,
                str(run_script_path("run_oakd_sensor_node.py")),
                "--wait-for-arm-state",
            ],
        ),
        LaunchSpec(
            name="visual_servo_autonomy",
            command=autonomy_command,
        ),
        LaunchSpec(
            name="visual_servo_teleop",
            command=[python_executable, str(run_script_path("run_visual_servo_teleop.py"))],
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description="Launch the GIRAF AprilTag visual-servo prototype")
    parser.add_argument("--port", type=int, default=5010, help="Visual-servo Flask dashboard port")
    parser.add_argument(
        "--diagnostics",
        action="store_true",
        help="Print compact autonomy diagnostics JSON lines at 10 Hz while auto is active",
    )
    args = parser.parse_args()

    manager = ProcessManager()
    for spec in build_visual_servo_demo_specs(sys.executable, args.port, args.diagnostics):
        print(f"Starting {spec.name}: {' '.join(spec.command)}")
        manager.start(spec)

    print(f"Visual-servo dashboard: {dashboard_url(args.port)}")
    try:
        name, code = manager.wait_until_any_exit()
        print(f"{name} exited with code {code}")
        return code
    except KeyboardInterrupt:
        print("\nShutting down visual-servo demo...")
        return 0
    finally:
        manager.terminate_all()


if __name__ == "__main__":
    raise SystemExit(main())
