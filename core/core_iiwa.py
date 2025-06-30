import numpy as np
from rteach.core.linear_motion_iiwa import plan_linear_path

KUKA_SOLVER_TYPE = "PYD"

try:
    import rteach.ext.kuka_iiwa_ik as kuka_iiwa_ik
    has_solver = hasattr(kuka_iiwa_ik, 'solve')
    print("[core_iiwa] Using .pyd analytic IK")
    KUKA_SOLVER_TYPE = "PYD"
except Exception as e:
    print(f"[core_iiwa] Failed to load .pyd → fallback to .py: {e}")
    import rteach.ext.kuka_iiwa_ik_py as kuka_iiwa_ik
    has_solver = True
    KUKA_SOLVER_TYPE = "PY"

try:
    from rteach.ext.kuka_iiwa_ik_generated import kuka_iiwa_fk as fk_py
except Exception as e:
    print(f"[core_iiwa] Failed to load kuka_iiwa_ik_generated.py: {e}")
    fk_py = None

def inverse_kinematics(T: np.ndarray, fixed_q3: float = 0.0):
    if kuka_iiwa_ik is None or not has_solver:
        raise ImportError("kuka_iiwa_ik.pyd is not loaded or solve() is missing.")
    return kuka_iiwa_ik.solve(T, fixed_q3)

def forward_kinematics(joint_angles: np.ndarray):
    if fk_py is None:
        raise ImportError("kuka_iiwa_ik_generated.py is not loaded.")
    return fk_py(joint_angles)

def inverse_kinematics_fixed_q3(T: np.ndarray, q3: float):
    return inverse_kinematics(T, fixed_q3=q3)

def linear_move(q_init: np.ndarray, T_goal: np.ndarray, props):
    return plan_linear_path(q_init, T_goal, props)
