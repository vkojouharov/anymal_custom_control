#!/usr/bin/env python3
"""Launch the preferred operator-station stack."""

import argparse
import sys

from anymal_custom_control.runtime import (
    ProcessManager,
    build_operator_station_specs,
    dashboard_url,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Launch the ANYmal operator station")
    parser.add_argument(
        "--mode",
        choices=("full", "teleop", "perception"),
        default="full",
        help="Which stack to launch",
    )
    parser.add_argument(
        "--perception-port",
        type=int,
        default=5004,
        help="HTTP port for the perception dashboard",
    )
    args = parser.parse_args()

    specs = build_operator_station_specs(
        sys.executable,
        mode=args.mode,
        perception_port=args.perception_port,
    )
    if not specs:
        print("No launch specs selected.")
        return 1

    manager = ProcessManager()
    for spec in specs:
        print(f"Starting {spec.name}: {' '.join(spec.command)}")
        manager.start(spec)

    if args.mode in {"full", "perception"}:
        print(f"Dashboard: {dashboard_url(args.perception_port)}")

    try:
        name, code = manager.wait_until_any_exit()
        print(f"{name} exited with code {code}")
        return code
    except KeyboardInterrupt:
        print("\nShutting down operator station...")
        return 0
    finally:
        manager.terminate_all()


if __name__ == "__main__":
    raise SystemExit(main())
