#!/usr/bin/env python3
"""Shared paths and trial naming for BOTA boom compliance scripts."""

from __future__ import annotations

import re
from decimal import Decimal
from pathlib import Path


EXPERIMENT_DIR = Path(__file__).resolve().parents[2]
DATA_DIR = EXPERIMENT_DIR / "data"
RAW_DATA_DIR = DATA_DIR / "raw_data"
SHEAR_CENTER_DATA_DIR = DATA_DIR / "shear_center_data"
CONVERTED_AXES_DATA_DIR = DATA_DIR / "converted_axes_data"
COMPLIANCE_MATRIX_DIR = DATA_DIR / "compliance_matrices_about_shear_center"

TRIAL_RE = re.compile(r"^bota_(?P<length>\d+(?:p\d+)?)m(?:_|$)")


def format_length_m(length_m: float) -> str:
    text = format(Decimal(str(length_m)).normalize(), "f")
    text = text.rstrip("0").rstrip(".") if "." in text else text
    return text.replace(".", "p")


def trial_stem(length_m: float) -> str:
    return f"bota_{format_length_m(length_m)}m"


def length_from_stem(stem: str) -> float:
    match = TRIAL_RE.match(stem)
    if match is None:
        raise ValueError(f"cannot parse boom length from file stem: {stem}")
    return float(match.group("length").replace("p", "."))


def newest_csv(directory: Path, pattern: str) -> Path:
    candidates = sorted(directory.glob(pattern), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"no files matching {directory / pattern}")
    return candidates[-1]


def select_input_csv(cli_input: Path | None, manual_input_csv: str, default_directory: Path, pattern: str) -> Path:
    if cli_input is not None:
        return cli_input.expanduser()

    if manual_input_csv:
        path = Path(manual_input_csv).expanduser()
        if not path.is_absolute():
            if path.parent == Path("."):
                path = default_directory / path
            else:
                path = EXPERIMENT_DIR / path
                if not path.exists():
                    data_path = DATA_DIR / Path(manual_input_csv).expanduser()
                    if data_path.exists():
                        path = data_path
        if not path.exists():
            raise FileNotFoundError(f"manual input CSV does not exist: {path}")
        return path

    return newest_csv(default_directory, pattern)


def raw_data_path(length_m: float) -> Path:
    return RAW_DATA_DIR / f"{trial_stem(length_m)}.csv"


def shear_center_path(raw_csv: Path) -> Path:
    return SHEAR_CENTER_DATA_DIR / f"{raw_csv.stem}_shear_center.csv"


def converted_axes_path(shear_center_csv: Path) -> Path:
    return CONVERTED_AXES_DATA_DIR / f"{shear_center_csv.stem}_converted_axes.csv"


def compliance_matrix_path(converted_axes_csv: Path) -> Path:
    suffix = "_shear_center_converted_axes"
    stem = converted_axes_csv.stem
    if stem.endswith(suffix):
        stem = stem[: -len(suffix)]
    return COMPLIANCE_MATRIX_DIR / f"{stem}_compliance_matrix.csv"
