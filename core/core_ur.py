import numpy as np
from rteach.ext.ur_analytic_ik_ext import ur16e, ur5e
from rteach.core.robot_state import get_active_robot

def _get_solver():
    robot = get_active_robot().upper()
    if "UR5" in robot:
        return ur5e
    elif "UR16" in robot:
        return ur16e
    else:
        raise ValueError(f"[core_ur] Unknown UR variant: '{robot}'")

def inverse_kinematics(T: np.ndarray):
    return _get_solver().inverse_kinematics(T)

def forward_kinematics(q: np.ndarray):
    return _get_solver().forward_kinematics(*q)

def inverse_kinematics_fixed_q3(T: np.ndarray, q3: float):
    return inverse_kinematics(T)  
