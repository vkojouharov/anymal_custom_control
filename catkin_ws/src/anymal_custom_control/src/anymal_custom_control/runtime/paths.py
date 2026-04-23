"""Filesystem helpers for stable launcher scripts."""

from pathlib import Path


def package_root() -> Path:
    return Path(__file__).resolve().parents[3]


def scripts_root() -> Path:
    return package_root() / "scripts"


def camera_script_path(name: str) -> Path:
    return scripts_root() / "camera" / name


def legacy_script_path(name: str) -> Path:
    return scripts_root() / "legacy" / name


def operator_console_script_path(name: str) -> Path:
    return scripts_root() / "operator_console" / name


def run_script_path(name: str) -> Path:
    return scripts_root() / "run" / name
