"""Operator-station launch specifications."""

from __future__ import annotations

from .paths import run_script_path
from .process_manager import LaunchSpec


def dashboard_url(port: int) -> str:
    return f"http://localhost:{port}"


def build_operator_station_specs(
    python_executable: str,
    *,
    mode: str,
    perception_port: int,
) -> list[LaunchSpec]:
    specs: list[LaunchSpec] = []

    if mode in {"full", "teleop"}:
        specs.append(
            LaunchSpec(
                name="oakd_sensor",
                command=[
                    python_executable,
                    str(run_script_path("run_oakd_sensor_node.py")),
                    "--wait-for-arm-state",
                ],
            )
        )
        specs.append(
            LaunchSpec(
                name="teleop",
                command=[
                    python_executable,
                    str(run_script_path("run_teleop_stabilized.py")),
                    "--no-oakd",
                ],
            )
        )

    if mode in {"full", "perception"}:
        command = [
            python_executable,
            str(run_script_path("run_operator_console.py")),
            "--port",
            str(perception_port),
        ]
        if mode == "full":
            command.append("--no-camera")
        specs.append(
            LaunchSpec(
                name="perception",
                command=command,
            )
        )

    return specs
