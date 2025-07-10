# rteach/core/core_ur.py

import numpy as np
from rteach.core.robot_presets import ROBOT_CONFIGS

# Try to load the compiled (.pyd) solver
UR_SOLVER_TYPE = "PYD"
try:
    from rteach.ext.ur_analytic_ik_ext import (
        ur3e as _ur3e_ext,
        ur5e as _ur5e_ext,
        ur10e as _ur10e_ext,
        ur15 as _ur15_ext,
        ur16e as _ur16e_ext,
        ur20 as _ur20_ext,
        ur30 as _ur30_ext,
    )
    UR_SOLVERS_EXT = {
        "ur3e": _ur3e_ext,
        "ur5e": _ur5e_ext,
        "ur10e": _ur10e_ext,
        "ur15": _ur15_ext,
        "ur16e": _ur16e_ext,
        "ur20": _ur20_ext,
        "ur30": _ur30_ext,
    }
    print("[core_ur] Using .pyd analytic IK module")
except ImportError:
    UR_SOLVER_TYPE = "PY"
    from rteach.ext.ur_analytic_ik_py import (
        ur3e as _ur3e_py,
        ur5e as _ur5e_py,
        ur10e as _ur10e_py,
        ur15 as _ur15_py,
        ur16e as _ur16e_py,
        ur20 as _ur20_py,
        ur30 as _ur30_py,
    )
    UR_SOLVERS_PY = {
        "ur3e": _ur3e_py,
        "ur5e": _ur5e_py,
        "ur10e": _ur10e_py,
        "ur15": _ur15_py,
        "ur16e": _ur16e_py,
        "ur20": _ur20_py,
        "ur30": _ur30_py,
    }
    print("[core_ur] Using pure‐Python fallback IK solver")

def _get_solver(robot_type: str, armature_name: str):
    cfg = ROBOT_CONFIGS.get(robot_type.lower(), {})
    key = cfg.get("armature_solver_map", {}).get(armature_name)
    if not key:
        raise ValueError(f"[core_ur] No solver mapping for armature '{armature_name}' in '{robot_type}'")
    if UR_SOLVER_TYPE == "PYD":
        return UR_SOLVERS_EXT[key]
    else:
        return UR_SOLVERS_PY[key]

def inverse_kinematics(T: np.ndarray):
    from bpy import context
    p = context.scene.ik_motion_props
    solver = _get_solver(p.robot_type, p.armature)
    return solver.inverse_kinematics(T)

def forward_kinematics(q: np.ndarray):
    from bpy import context
    p = context.scene.ik_motion_props
    solver = _get_solver(p.robot_type, p.armature)
    return solver.forward_kinematics(*q)

def inverse_kinematics_fixed_q3(T: np.ndarray, q3: float):
    # UR e-series does not use fixed-q3
    return inverse_kinematics(T)
