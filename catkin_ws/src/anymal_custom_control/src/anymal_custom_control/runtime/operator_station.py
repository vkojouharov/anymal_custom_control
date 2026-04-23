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
                name="teleop",
                command=[
                    python_executable,
                    str(run_script_path("run_arm_jparse_teleop.py")),
                ],
            )
        )

    if mode in {"full", "perception"}:
        specs.append(
            LaunchSpec(
                name="perception",
                command=[
                    python_executable,
                    str(run_script_path("run_operator_console.py")),
                    "--port",
                    str(perception_port),
                ],
            )
        )

    return specs
