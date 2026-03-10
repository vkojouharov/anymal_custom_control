import numpy as np
import sympy as sp

## -------------------- CONSTANTS (physical parameters) --------------------
# Link Lengths
L1_CONST = 0.21  # m
L2_CONST = 0.055  # m
L3_CONST = 0.133 # m
rho = 0.188 # [kg/m]
m_e = 0.5 # [kg]
g = 9.81 # m/s^2

# Flexural rigidity of boom
EI_CONST = 91.24628715 # Nm^2

## -------------------- KINEMATIC MODEL (dh_parameters) --------------------
th1, th2, d3 = sp.symbols('th1 th2 d3', real=True)
MDH_sym = {
    1: {'a': 0,         'al': 0,       'd': L1_CONST, 'th': th1},
    2: {'a': 0,         'al': sp.pi/2, 'd': 0,        'th': th2},
    3: {'a': -L2_CONST, 'al': sp.pi/2, 'd': d3,       'th': sp.pi/2},
}

## Forward Kinematics for a link as defined by modified DH parameters
def sym_MDH_forward(dh_param):
    a = dh_param['a'] # a (i-1)
    al = dh_param['al'] # alpha (i-1)
    d = dh_param['d'] # d (i)
    th = dh_param['th'] # theta (i)
    return sp.Matrix([
        [sp.cos(th), -sp.sin(th), 0, a],
        [sp.sin(th)*sp.cos(al), sp.cos(th)*sp.cos(al), -sp.sin(al), -sp.sin(al)*d],
        [sp.sin(th)*sp.sin(al), sp.cos(th)*sp.sin(al), sp.cos(al), sp.cos(al)*d],
        [0, 0, 0, 1]
    ])

## SYMBOLIC Forward Kinematics for Modified DH Parameter Link Description
def sym_forward_kinematics(MDH):
    T = sp.eye(4)  # Initialize transform as identity matrix
    for i in sorted(MDH.keys()): # apply transforms for each link described by DH
        Ti = sym_MDH_forward(MDH[i])
        T = T @ Ti
    return T

def sym_forward_kinematics_corrected(MDH):
    # Rigid link transforms
    T01 = sym_MDH_forward(MDH[1])
    T12 = sym_MDH_forward(MDH[2])
    T23 = sym_MDH_forward(MDH[3])

    # deflection compensation
    L = d3 - 255/1000
    delta = sp.cos(th2 - sp.pi/2) / EI_CONST * (rho * g * L**4 / 8 + m_e * g * L**3 / 3)
    phi = sp.cos(th2 - sp.pi/2) / EI_CONST * (rho * g * L**3 / 6 + m_e * g * L**2 / 2)
    T3d = sp.Matrix([[1, 0,             0,              0],
                     [0, sp.cos(-phi),  -sp.sin(-phi),  delta],
                     [0, sp.sin(-phi),  sp.cos(-phi),   0],
                     [0, 0,             0,              1]])

    T = T01 @ T12 @ T23 @ T3d
    return T

## SYMBOLIC Linear Velocity Jacobian
def sym_jacobian_linear(T):
    x = T[0,3]
    y = T[1,3]
    z = T[2,3]
    Jv = sp.Matrix([[x.diff(th1), x.diff(th2), x.diff(d3)],
                    [y.diff(th1), y.diff(th2), y.diff(d3)],
                    [z.diff(th1), z.diff(th2), z.diff(d3)]])
    return Jv
    # return sp.simplify(Jv) # linear velocity Jacobian

## -------------------- BOOM SPOOL MAPPING --------------------
# Cubic: d3 (meters) -> boom motor position (radians)
_BOOM_P1 = -0.0508
_BOOM_P2 = -0.4122
_BOOM_P3 = -15.2992
_BOOM_P4 = 4.7840

def get_boom_motor_rad(d3):
    """Convert boom extension length d3 (m) to spool motor position (rad)."""
    return _BOOM_P1 * d3**3 + _BOOM_P2 * d3**2 + _BOOM_P3 * d3 + _BOOM_P4

def get_boom_length_d3(boom_pos, tol=1e-10, max_iter=20):
    """Convert spool motor position (rad) to boom extension d3 (m).
    Uses Newton's method on get_boom_motor_rad(d3) - boom_pos = 0."""
    # Initial guess from linear term
    d3 = (_BOOM_P4 - boom_pos) / _BOOM_P3
    for _ in range(max_iter):
        f = _BOOM_P1 * d3**3 + _BOOM_P2 * d3**2 + _BOOM_P3 * d3 + _BOOM_P4 - boom_pos
        fp = 3 * _BOOM_P1 * d3**2 + 2 * _BOOM_P2 * d3 + _BOOM_P3
        d3 -= f / fp
        if abs(f) < tol:
            break
    return d3

## NUMERICAL XYZ forward kinematics
def num_forward_kinematics(joint_coords):
    return np.array(FK_num(*joint_coords))
## NUMERICAL Basic Jacobian (6x6)
def num_jacobian(joint_coords):
    return np.array(Jv_num(*joint_coords))

print("Starting symbolic kinematic derivations")
T = sym_forward_kinematics(MDH_sym)
print("Computed FK")
Jv = sym_jacobian_linear(T)
print("Computed linear velocity jacobian")
FK_num = sp.lambdify((th1, th2, d3), T, modules='numpy')
Jv_num = sp.lambdify((th1, th2, d3), Jv, modules='numpy')
