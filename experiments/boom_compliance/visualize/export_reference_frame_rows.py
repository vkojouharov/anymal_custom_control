#!/usr/bin/env python3
"""Export reference-frame visualizations for every row in a raw capture CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from visualize_reference_frame import (
    DEFAULT_COMPLIANCE_CSV,
    DEFAULT_DIAMETER_MM,
    DEFAULT_FORCE_N,
    DEFAULT_INPUT_CSV,
    DEFAULT_SUBTENDED_ANGLE_DEG,
    DEFAULT_THICKNESS_MM,
    load_compliance,
    plot_reference_frame_row,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--compliance", type=Path, default=DEFAULT_COMPLIANCE_CSV)
    parser.add_argument("--force-n", type=float, default=DEFAULT_FORCE_N)
    parser.add_argument("--diameter-mm", type=float, default=DEFAULT_DIAMETER_MM)
    parser.add_argument("--subtended-angle-deg", type=float, default=DEFAULT_SUBTENDED_ANGLE_DEG)
    parser.add_argument("--thickness-mm", type=float, default=DEFAULT_THICKNESS_MM)
    parser.add_argument("--y-sc-mm", type=float, default=None)
    parser.add_argument("--no-predicted", action="store_true")
    parser.add_argument("--dpi", type=int, default=160)
    args = parser.parse_args()

    output_dir = Path(__file__).resolve().parent / args.input.stem
    output_dir.mkdir(parents=True, exist_ok=True)

    with args.input.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    compliance_matrix = None if args.no_predicted else load_compliance(args.compliance)

    for row_number, row in enumerate(rows, start=1):
        fig, _ax = plot_reference_frame_row(
            row,
            row_number,
            force_n=args.force_n,
            diameter_mm=args.diameter_mm,
            subtended_angle_deg=args.subtended_angle_deg,
            thickness_mm=args.thickness_mm,
            y_sc_mm=args.y_sc_mm,
            compliance_matrix=compliance_matrix,
            print_debug=False,
        )
        output_path = output_dir / f"row{row_number}.png"
        fig.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
        plt.close(fig)

    print(f"Wrote {len(rows)} PNGs to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
