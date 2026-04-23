#!/usr/bin/env python3
"""Launch the new GIRAF arm controller + teleop stack."""

import sys

from anymal_custom_control.runtime import LaunchSpec, ProcessManager, run_script_path


def main() -> int:
    specs = [
        LaunchSpec(
            name="giraf_arm_controller",
            command=[sys.executable, str(run_script_path("run_giraf_arm_controller.py"))],
        ),
        LaunchSpec(
            name="giraf_arm_teleop",
            command=[sys.executable, str(run_script_path("run_giraf_arm_teleop.py"))],
        ),
    ]

    manager = ProcessManager()
    for spec in specs:
        print(f"Starting {spec.name}: {' '.join(spec.command)}")
        manager.start(spec)

    try:
        name, code = manager.wait_until_any_exit()
        print(f"{name} exited with code {code}")
        return code
    except KeyboardInterrupt:
        print("\nShutting down GIRAF arm teleop stack...")
        return 0
    finally:
        manager.terminate_all()


if __name__ == "__main__":
    raise SystemExit(main())
