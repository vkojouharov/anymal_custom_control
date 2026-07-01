#!/usr/bin/env python3
"""Fit a length-parametric compliance matrix model from fitted matrices."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

import numpy as np

from src.pipeline_paths import COMPLIANCE_MATRIX_DIR, DATA_DIR, length_from_stem, trial_stem


DEFAULT_OUTPUT_DIR = DATA_DIR / "compliance_length_scaling_model"
DEFAULT_POWERS = (3, 2, 1)
DEFAULT_PSD_GRID_COUNT = 41
COMPONENT_NAMES = ("ux", "uy", "uz", "theta_x", "theta_y", "theta_z")


@dataclass(frozen=True)
class MatrixSample:
    path: Path
    length_m: float
    matrix: np.ndarray


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def sample_length(path: Path) -> float:
    suffix = "_compliance_matrix"
    stem = path.stem
    if stem.endswith(suffix):
        stem = stem[: -len(suffix)]
    return length_from_stem(stem)


def load_samples(matrix_dir: Path, pattern: str) -> list[MatrixSample]:
    paths = sorted(matrix_dir.glob(pattern), key=sample_length)
    if not paths:
        raise FileNotFoundError(f"no compliance matrices matching {matrix_dir / pattern}")
    return [MatrixSample(path=path, length_m=sample_length(path), matrix=load_compliance(path)) for path in paths]


def parse_powers(text: str) -> tuple[int, ...]:
    powers = tuple(int(part.strip()) for part in text.split(",") if part.strip())
    if not powers:
        raise argparse.ArgumentTypeError("at least one power is required")
    if len(set(powers)) != len(powers):
        raise argparse.ArgumentTypeError(f"duplicate powers are not allowed: {text}")
    if any(power < 0 for power in powers):
        raise argparse.ArgumentTypeError(f"powers must be nonnegative: {text}")
    return powers


def feature_row(length_m: float, powers: tuple[int, ...], normalize_by: float) -> np.ndarray:
    normalized_length = length_m / normalize_by
    return np.array([normalized_length**power for power in powers], dtype=float)


def reconstruct(coefficients: dict[int, np.ndarray], length_m: float, normalize_by: float) -> np.ndarray:
    matrix = np.zeros((6, 6), dtype=float)
    for power, coefficient in coefficients.items():
        matrix += coefficient * (length_m / normalize_by) ** power
    return matrix


def fit_length_model(
    *,
    samples: list[MatrixSample],
    powers: tuple[int, ...],
    normalize_by: float,
    psd_grid_count: int,
    ridge: float,
    solver: str | None,
    verbose: bool,
) -> tuple[dict[int, np.ndarray], str, float]:
    try:
        import cvxpy as cp
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "cvxpy is required for this optimizer. Install it in this Python environment "
            "or run this script in the same environment used for scripts/STEP_4_fit_compliance_cvx.py."
        ) from exc

    coefficient_vars = {power: cp.Variable((6, 6), symmetric=True) for power in powers}
    objective_terms = []

    for sample in samples:
        predicted = sum(
            coefficient_vars[power] * (sample.length_m / normalize_by) ** power
            for power in powers
        )
        objective_terms.append(cp.sum_squares(predicted - sample.matrix))

    if ridge > 0.0:
        for power in powers:
            objective_terms.append(ridge * cp.sum_squares(coefficient_vars[power]))

    constraints = []
    if psd_grid_count > 0:
        lengths = np.linspace(samples[0].length_m, samples[-1].length_m, psd_grid_count)
        for length_m in lengths:
            predicted = sum(
                coefficient_vars[power] * (length_m / normalize_by) ** power
                for power in powers
            )
            constraints.append(predicted >> 0)

    problem = cp.Problem(cp.Minimize(sum(objective_terms)), constraints)
    solve_kwargs = {"verbose": verbose}
    if solver is not None:
        solve_kwargs["solver"] = solver
    problem.solve(**solve_kwargs)

    if problem.status not in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE}:
        raise RuntimeError(f"CVXPY solve failed with status {problem.status}")

    coefficients = {power: np.asarray(coefficient_vars[power].value, dtype=float) for power in powers}
    for matrix in coefficients.values():
        matrix[np.abs(matrix) < 1e-12] = 0.0
    return coefficients, str(problem.status), float(problem.value)


def save_outputs(
    *,
    output_dir: Path,
    samples: list[MatrixSample],
    powers: tuple[int, ...],
    normalize_by: float,
    coefficients: dict[int, np.ndarray],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    header = ",".join(COMPONENT_NAMES)

    for power in powers:
        path = output_dir / f"coefficient_L_power_{power}.csv"
        np.savetxt(path, coefficients[power], delimiter=",", header=header, comments="")
        print(f"saved coefficient L^{power}: {path}")

    npz_path = output_dir / "compliance_length_scaling_coefficients.npz"
    np.savez(
        npz_path,
        powers=np.array(powers, dtype=int),
        normalize_by=np.array(normalize_by, dtype=float),
        **{f"coefficient_power_{power}": coefficients[power] for power in powers},
    )
    print(f"saved model archive: {npz_path}")

    predicted_dir = output_dir / "predicted_matrices_at_measured_lengths"
    predicted_dir.mkdir(parents=True, exist_ok=True)
    for sample in samples:
        predicted = reconstruct(coefficients, sample.length_m, normalize_by)
        path = predicted_dir / f"{trial_stem(sample.length_m)}_predicted_compliance_matrix.csv"
        np.savetxt(path, predicted, delimiter=",", header=header, comments="")
    print(f"saved predicted matrices: {predicted_dir}")


def print_summary(
    *,
    samples: list[MatrixSample],
    powers: tuple[int, ...],
    normalize_by: float,
    coefficients: dict[int, np.ndarray],
    status: str,
    objective: float,
) -> None:
    print(f"status: {status}")
    print(f"objective: {objective:.6e}")
    print(f"powers: {', '.join(f'L^{power}' for power in powers)}")
    print(f"length normalization: L / {normalize_by:g}")
    print("fit residuals at measured lengths:")
    for sample in samples:
        predicted = reconstruct(coefficients, sample.length_m, normalize_by)
        residual = predicted - sample.matrix
        fro_norm = float(np.linalg.norm(residual, ord="fro"))
        max_abs = float(np.max(np.abs(residual)))
        print(f"  {sample.length_m:g} m: fro={fro_norm:.6e}, max_abs={max_abs:.6e}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix-dir", type=Path, default=COMPLIANCE_MATRIX_DIR)
    parser.add_argument("--pattern", default="bota_*_compliance_matrix.csv")
    parser.add_argument(
        "--powers",
        type=parse_powers,
        default=DEFAULT_POWERS,
        help="Comma-separated powers in the model, e.g. 3,2,1 or 3,2,1,0",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--psd-grid-count",
        type=int,
        default=DEFAULT_PSD_GRID_COUNT,
        help="Number of lengths where C(L) is constrained positive semidefinite; 0 disables PSD constraints",
    )
    parser.add_argument(
        "--normalize-by",
        type=float,
        default=None,
        help="Length scale used in powers. Defaults to max measured length.",
    )
    parser.add_argument("--ridge", type=float, default=0.0, help="Optional L2 regularization on coefficient matrices")
    parser.add_argument("--solver", default=None, help="Optional CVXPY solver name, such as CLARABEL or SCS")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    if args.psd_grid_count < 0:
        raise ValueError("--psd-grid-count must be nonnegative")
    if args.ridge < 0.0:
        raise ValueError("--ridge must be nonnegative")

    samples = load_samples(args.matrix_dir.expanduser(), args.pattern)
    normalize_by = float(args.normalize_by) if args.normalize_by is not None else samples[-1].length_m
    if normalize_by <= 0.0:
        raise ValueError("--normalize-by must be positive")

    print("loaded matrices:")
    for sample in samples:
        print(f"  {sample.length_m:g} m: {sample.path}")

    coefficients, status, objective = fit_length_model(
        samples=samples,
        powers=args.powers,
        normalize_by=normalize_by,
        psd_grid_count=args.psd_grid_count,
        ridge=args.ridge,
        solver=args.solver,
        verbose=args.verbose,
    )
    print_summary(
        samples=samples,
        powers=args.powers,
        normalize_by=normalize_by,
        coefficients=coefficients,
        status=status,
        objective=objective,
    )
    save_outputs(
        output_dir=args.output_dir.expanduser(),
        samples=samples,
        powers=args.powers,
        normalize_by=normalize_by,
        coefficients=coefficients,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
