import bpy
import numpy as np
from rteach.core.linear_motion_iiwa import plan_linear_path

def get_user_kuka_solver_choice():
    try:
        prefs = bpy.context.preferences.addons["rteach"].preferences
        return getattr(prefs, "kuka_solver_choice", "PYD")
    except:
        return "PYD"

KUKA_SOLVER_TYPE = get_user_kuka_solver_choice()

try:
    if KUKA_SOLVER_TYPE == "PYD":
        import rteach.ext.kuka_iiwa_ik as kuka_iiwa_ik
        has_solver = hasattr(kuka_iiwa_ik, 'solve')
        print("[core_iiwa] Using .pyd analytic IK (User Preference)")
    else:
        raise ImportError("Force fallback to .py")
except Exception as e:
    print(f"[core_iiwa] Fallback to .py IK solver: {e}")
    import rteach.ext.kuka_iiwa_ik_py as kuka_iiwa_ik
    has_solver = True
    KUKA_SOLVER_TYPE = "PY"

try:
    from rteach.ext.kuka_iiwa_ik_generated import kuka_iiwa_fk as fk_py
except Exception as e:
    print(f"[core_iiwa] Failed to load FK module: {e}")
    fk_py = None

def inverse_kinematics(T: np.ndarray, fixed_q3: float = 0.0):
    if kuka_iiwa_ik is None or not has_solver:
        raise ImportError("KUKA IK solver not loaded.")
    return kuka_iiwa_ik.solve(T, fixed_q3)

def forward_kinematics(joint_angles: np.ndarray):
    if fk_py is None:
        raise ImportError("KUKA FK not available.")
    return fk_py(joint_angles)

def inverse_kinematics_fixed_q3(T: np.ndarray, q3: float):
    return inverse_kinematics(T, fixed_q3=q3)

def linear_move(q_init: np.ndarray, T_goal: np.ndarray, props):
    return plan_linear_path(q_init, T_goal, props)
