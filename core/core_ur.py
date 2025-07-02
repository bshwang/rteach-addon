import numpy as np
from rteach.core.robot_presets import ROBOT_CONFIGS

UR_SOLVER_TYPE = "PYD"

try:
    from rteach.ext.ur_analytic_ik_ext import ur16e, ur5e, ur3e, ur10e, ur15, ur20, ur30
    print("[core_ur] Using .pyd analytic IK module")
    UR_SOLVER_TYPE = "PYD"
except ImportError:
    from rteach.ext.ur_analytic_ik_py import ur_inverse_kinematics
    print("[core_ur] Using pure Python fallback IK solver")
    UR_SOLVER_TYPE = "PY"

# Hardcoded DH parameters used only for fallback .py solver
UR_DH_PARAMS = {
    "ur3e":  dict(d1=0.15185, a2=-0.24355, a3=-0.2132,  d4=0.13105, d5=0.08535, d6=0.0921),
    "ur5e":  dict(d1=0.1625,  a2=-0.425,   a3=-0.3922,  d4=0.1333,  d5=0.0997,  d6=0.0996),
    "ur10e": dict(d1=0.1807,  a2=-0.6127,  a3=-0.57155, d4=0.17415, d5=0.11985, d6=0.11655),
    "ur16e": dict(d1=0.1807,  a2=-0.4784,  a3=-0.36,    d4=0.17415, d5=0.11985, d6=0.11655),
    "ur15":  dict(d1=0.1807,  a2=-0.4784,  a3=-0.36,    d4=0.17415, d5=0.11985, d6=0.11655),
    "ur20":  dict(d1=0.245,   a2=-0.795,   a3=-0.302,   d4=0.175,   d5=0.175,   d6=0.1),
    "ur30":  dict(d1=0.245,   a2=-0.795,   a3=-0.302,   d4=0.175,   d5=0.175,   d6=0.1),
}

def _get_solver(robot_type: str, armature_name: str):
    config = ROBOT_CONFIGS.get(robot_type.lower(), {})
    map_dict = config.get("armature_solver_map", {})

    solver_key = map_dict.get(armature_name)
    if solver_key == "ur3e":
        return ur3e if UR_SOLVER_TYPE == "PYD" else ("ur3e", UR_DH_PARAMS["ur3e"])
    elif solver_key == "ur5e":
        return ur5e if UR_SOLVER_TYPE == "PYD" else ("ur5e", UR_DH_PARAMS["ur5e"])
    elif solver_key == "ur10e":
        return ur10e if UR_SOLVER_TYPE == "PYD" else ("ur10e", UR_DH_PARAMS["ur10e"])
    elif solver_key == "ur15":
        return ur15 if UR_SOLVER_TYPE == "PYD" else ("ur15", UR_DH_PARAMS["ur15"])
    elif solver_key == "ur16e":
        return ur16e if UR_SOLVER_TYPE == "PYD" else ("ur16e", UR_DH_PARAMS["ur16e"])
    elif solver_key == "ur20":
        return ur20 if UR_SOLVER_TYPE == "PYD" else ("ur20", UR_DH_PARAMS["ur20"])
    elif solver_key == "ur30":
        return ur30 if UR_SOLVER_TYPE == "PYD" else ("ur30", UR_DH_PARAMS["ur30"])
    else:
        raise ValueError(f"[core_ur] Solver not defined for armature '{armature_name}' in robot_type '{robot_type}'")

def inverse_kinematics(T: np.ndarray):
    from bpy import context
    p = context.scene.ik_motion_props
    solver = _get_solver(p.robot_type, p.armature)
    if UR_SOLVER_TYPE == "PYD":
        return solver.inverse_kinematics(T)
    else:
        _, params = solver
        return ur_inverse_kinematics(T, params["d1"], params["d4"], params["d5"], params["d6"], params["a2"], params["a3"])

def forward_kinematics(q: np.ndarray):
    from bpy import context
    p = context.scene.ik_motion_props
    solver = _get_solver(p.robot_type, p.armature)
    if UR_SOLVER_TYPE == "PYD":
        return solver.forward_kinematics(*q)
    else:
        # .py fallback currently does not implement FK
        raise NotImplementedError("forward_kinematics not available in fallback solver")

def inverse_kinematics_fixed_q3(T: np.ndarray, q3: float):
    return inverse_kinematics(T)
