"""Smoke test for the local J-PARSE controller.

Run:
    docker compose run --rm anymal-control \
        python3 /catkin_ws/src/anymal_custom_control/scripts/test_jparse_controller.py
"""

import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(SCRIPT_DIR, "..", "src", "anymal_custom_control")
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from RRPRRR_kinematic_model import num_jacobian
from jparse_controller import JParseController, inverse_condition_number, manipulability_measure


q_kin = (0.2, np.pi / 2 + 0.4, 0.9, np.pi / 2 + 0.1, 5 * np.pi / 6 + 0.2, -0.1)
J = np.asarray(num_jacobian(q_kin), dtype=float)
v_des = np.array([0.1, 0.05, 0.0, 0.0, 0.02, 0.0], dtype=float)

jparse = JParseController(gamma=0.1)
J_jparse = jparse.compute(
    J,
    singular_direction_gain_position=1.0,
    singular_direction_gain_angular=1.0,
    position_dimensions=3,
    angular_dimensions=3,
)
qdot = np.asarray(J_jparse.dot(v_des), dtype=float).reshape(-1)
residual = np.linalg.norm(J.dot(qdot) - v_des)

print("q_kin = {}".format(q_kin))
print("manipulability = {:.6f}".format(manipulability_measure(J)))
print("inverse condition number = {:.6f}".format(inverse_condition_number(J)))
print("qdot = {}".format(qdot))
print("residual = {:.6e}".format(residual))

assert J_jparse.shape == (6, 6)
assert np.all(np.isfinite(J_jparse))
assert np.all(np.isfinite(qdot))
assert residual < 5e-2, "J-PARSE residual unexpectedly large: %r" % (residual,)

print("J-PARSE smoke test passed.")
