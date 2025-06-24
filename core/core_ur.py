import numpy as np
from rteach.ext.ur_analytic_ik_ext import ur16e, ur5e
from rteach.core.robot_presets import ROBOT_CONFIGS

def _get_solver(robot_type: str, armature_name: str):
    config = ROBOT_CONFIGS.get(robot_type.lower(), {})
    map_dict = config.get("armature_solver_map", {})

    solver_key = map_dict.get(armature_name)
    if solver_key == "ur5e":
        return ur5e
    elif solver_key == "ur16e":
        return ur16e
    else:
        raise ValueError(f"[core_ur] Solver not defined for armature '{armature_name}' in robot_type '{robot_type}'")

def inverse_kinematics(T: np.ndarray):
    from bpy import context
    p = context.scene.ik_motion_props
    return _get_solver(p.robot_type, p.armature).inverse_kinematics(T)

def forward_kinematics(q: np.ndarray):
    from bpy import context
    p = context.scene.ik_motion_props
    return _get_solver(p.robot_type, p.armature).forward_kinematics(*q)

def inverse_kinematics_fixed_q3(T: np.ndarray, q3: float):
    return inverse_kinematics(T)
