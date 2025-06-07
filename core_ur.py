# core_ur.py

from .robot_state import get_active_robot
from ur_analytic_ik_ext import ur16e, ur5e
import numpy as np

ARMATURE_BONES = ["j1", "j2", "j3", "j4", "j5", "j6"]
AXES = ['z', 'x', 'x', 'z', 'x', 'z']

def _get_solver():
    if get_active_robot() == "UR16e":
        return ur16e
    elif get_active_robot() == "UR5e":
        return ur5e
    else:
        raise ValueError(f"Unknown UR robot: {ACTIVE_ROBOT}")

def inverse_kinematics(T):
    return _get_solver().inverse_kinematics(T)

def forward_kinematics(q):
    return _get_solver().forward_kinematics(*q)

def inverse_kinematics_fixed_q3(T: np.ndarray, q3: float):
    return inverse_kinematics(T)