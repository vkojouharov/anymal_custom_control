"""Local J-PARSE implementation compatible with Python 3.8.

This mirrors the upstream numpy-only J-PARSE core while keeping the code
self-contained for the current control stack and container environment.
"""

from __future__ import annotations

from typing import List, Optional, Tuple, Union

import numpy as np


Array = np.ndarray


class JParseController(object):
    """Singularity-aware Jacobian inverse based on J-PARSE."""

    def __init__(self, gamma=0.1):
        gamma = float(gamma)
        if not 0.0 < gamma < 1.0:
            raise ValueError("gamma must be in (0, 1), got %r" % (gamma,))
        self.gamma = gamma

    @staticmethod
    def _svd_compose(U, singular_values, Vt):
        """Recompose a matrix from SVD factors, allowing truncated U."""
        U = np.asarray(U, dtype=float)
        Vt = np.asarray(Vt, dtype=float)
        Sfull = np.zeros((U.shape[1], Vt.shape[0]), dtype=float)
        for i, value in enumerate(singular_values):
            if i >= Sfull.shape[0] or i >= Sfull.shape[1]:
                break
            Sfull[i, i] = float(value)
        return np.asarray(U.dot(Sfull).dot(Vt), dtype=float)

    def compute(
        self,
        jacobian,
        singular_direction_gain_position=1.0,
        singular_direction_gain_angular=1.0,
        position_dimensions=None,
        angular_dimensions=None,
        return_nullspace=False,
    ):
        """Compute the J-PARSE pseudo-inverse."""
        J = np.asarray(jacobian, dtype=float)
        U, singular_values, Vt = np.linalg.svd(J)

        if singular_values.size == 0:
            J_parse = np.zeros((J.shape[1], J.shape[0]), dtype=float)
            if return_nullspace:
                return J_parse, np.eye(J.shape[1], dtype=float)
            return J_parse

        sigma_max = float(np.max(singular_values))
        if sigma_max <= 1e-12:
            J_parse = np.zeros((J.shape[1], J.shape[0]), dtype=float)
            if return_nullspace:
                return J_parse, np.eye(J.shape[1], dtype=float)
            return J_parse

        adjusted_condition_numbers = singular_values / sigma_max

        U_new_proj = []
        S_new_proj = []
        for idx, sigma in enumerate(singular_values):
            if sigma > self.gamma * sigma_max:
                U_new_proj.append(U[:, idx : idx + 1])
                S_new_proj.append(float(sigma))

        if U_new_proj:
            U_new_proj_arr = np.concatenate(U_new_proj, axis=1)
            J_proj = self._svd_compose(U_new_proj_arr, S_new_proj, Vt)
        else:
            J_proj = J.copy()

        S_new_safety = [
            float(sigma) if (sigma / sigma_max) > self.gamma else self.gamma * sigma_max
            for sigma in singular_values
        ]
        J_safety = self._svd_compose(U, S_new_safety, Vt)

        U_new_sing = []
        phi = []
        has_singular_directions = False
        for idx, condition_number in enumerate(adjusted_condition_numbers):
            if condition_number <= self.gamma:
                has_singular_directions = True
                U_new_sing.append(U[:, idx : idx + 1])
                phi.append(float(condition_number / self.gamma))

        phi_singular = np.zeros((J.shape[0], J.shape[0]), dtype=float)
        if has_singular_directions:
            U_new_sing_arr = np.concatenate(U_new_sing, axis=1)
            phi_mat = np.diag(phi)

            if position_dimensions is None and angular_dimensions is None:
                gains = np.full(J.shape[0], float(singular_direction_gain_position), dtype=float)
            elif angular_dimensions is None and position_dimensions is not None:
                gains = np.full(int(position_dimensions), float(singular_direction_gain_position), dtype=float)
            elif position_dimensions is None and angular_dimensions is not None:
                gains = np.full(int(angular_dimensions), float(singular_direction_gain_angular), dtype=float)
            else:
                gains = np.array(
                    [float(singular_direction_gain_position)] * int(position_dimensions)
                    + [float(singular_direction_gain_angular)] * int(angular_dimensions),
                    dtype=float,
                )

            Kp_singular = np.diag(gains)
            phi_singular = U_new_sing_arr.dot(phi_mat).dot(U_new_sing_arr.T).dot(Kp_singular)

        J_safety_pinv = np.linalg.pinv(J_safety)
        J_proj_pinv = np.linalg.pinv(J_proj)

        J_parse = J_safety_pinv.dot(J_proj).dot(J_proj_pinv)
        if has_singular_directions:
            J_parse = J_parse + J_safety_pinv.dot(phi_singular)
        J_parse = np.asarray(J_parse, dtype=float)

        if return_nullspace:
            J_safety_nullspace = np.eye(J_safety.shape[1], dtype=float) - J_safety_pinv.dot(J_safety)
            return J_parse, np.asarray(J_safety_nullspace, dtype=float)

        return J_parse

    @staticmethod
    def pinv(jacobian):
        return np.linalg.pinv(np.asarray(jacobian, dtype=float))

    @staticmethod
    def damped_least_squares(jacobian, damping=0.01, return_nullspace=False):
        J = np.asarray(jacobian, dtype=float)
        J_dls = np.linalg.inv(J.T.dot(J) + float(damping) ** 2 * np.eye(J.shape[1])).dot(J.T)
        J_dls = np.asarray(J_dls, dtype=float)
        if return_nullspace:
            nullspace = np.eye(J.shape[1], dtype=float) - J_dls.dot(J)
            return J_dls, nullspace
        return J_dls


def manipulability_measure(jacobian):
    """Compute Yoshikawa manipulability."""
    J = np.asarray(jacobian, dtype=float)
    gram = J.dot(J.T)
    det_gram = float(np.linalg.det(gram))
    return float(np.sqrt(max(det_gram, 0.0)))


def inverse_condition_number(jacobian):
    """Compute sigma_min / sigma_max."""
    J = np.asarray(jacobian, dtype=float)
    singular_values = np.linalg.svd(J, compute_uv=False)
    if singular_values.size == 0:
        return 0.0
    sigma_max = float(np.max(singular_values))
    if sigma_max <= 1e-12:
        return 0.0
    return float(np.min(singular_values) / sigma_max)


def compute_metrics(jacobian):
    """Convenience helper for display and debugging."""
    J = np.asarray(jacobian, dtype=float)
    singular_values = np.linalg.svd(J, compute_uv=False)
    return {
        "manipulability": manipulability_measure(J),
        "inverse_condition_number": inverse_condition_number(J),
        "singular_values": singular_values,
    }
