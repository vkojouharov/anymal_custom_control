#!/usr/bin/env python3
"""Minimal starter for fitting boom compliance data
"""

from pathlib import Path

import cvxpy as cp
import numpy as np

# Load wrench/displacement data (about shear center)
EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
CSV_PATH = EXPERIMENT_DIR / "data_1p5m_10N_shear_center.csv"
OUTPUT_PATH = EXPERIMENT_DIR / "data_1p5m_10N_compliance.csv"
print(f"Loading {CSV_PATH}")
data = np.loadtxt(CSV_PATH, delimiter=",", skiprows=1)
W = data[:, 0:6].T  # [Fx, Fy, Fz, Mx, My, Mz], 6 x N
X = data[:, 6:12].T  # [ux, uy, uz, theta_x, theta_y, theta_z], 6 x N

# Characteristic length for scaling rotational displacements to translational displacements
BOOM_LENGTH_M = 1.5
H_sqrt = np.diag([1.0, 1.0, 1.0, BOOM_LENGTH_M, BOOM_LENGTH_M, BOOM_LENGTH_M])

# Fit compliance in CVXPY
C = cp.Variable((6, 6), symmetric=True) # positive semidefinite compliance matrix
residual = X - C @ W
objective = cp.sum_squares(H_sqrt @ residual)  # sum of squared errors weighted by H
constraints = [C >> 0]  # C is positive semidefinite
problem = cp.Problem(cp.Minimize(objective), constraints)

problem.solve()
print("status:", problem.status)
print("objective:", problem.value)
print(C.value)
if C.value is not None:
    np.savetxt(OUTPUT_PATH, C.value, delimiter=",", header="ux,uy,uz,theta_x,theta_y,theta_z", comments="")
    print(f"saved: {OUTPUT_PATH}")
