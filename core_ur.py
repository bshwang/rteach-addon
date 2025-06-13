import bpy

from .robot_state import get_active_robot
from ur_analytic_ik_ext import ur16e, ur5e
import numpy as np

ARMATURE_BONES = ["j1", "j2", "j3", "j4", "j5", "j6"]
AXES = ['z', 'x', 'x', 'z', 'x', 'z']

def _get_solver():
    robot = get_active_robot()
    if robot == "UR5E":
        print("[DEBUG] Using UR5E solver")
        return ur5e
    elif robot == "UR16E":
        print("[DEBUG] Using UR16E solver")
        return ur16e
    else:
        raise ValueError(f"[core_ur] Unknown UR variant: '{robot}'")

def inverse_kinematics(T):
    return _get_solver().inverse_kinematics(T)

def forward_kinematics(q):
    return _get_solver().forward_kinematics(*q)

def inverse_kinematics_fixed_q3(T: np.ndarray, q3: float):
    return inverse_kinematics(T)
