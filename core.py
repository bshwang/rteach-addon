# core.py (0604)

import math
import bpy
import numpy as np
from math import pi, degrees
from scipy.spatial.transform import Rotation as R
from .robot_state import get_active_robot, set_active_robot

# UR
from .core_ur import (
    forward_kinematics as forward_kinematics_ur,
    inverse_kinematics as ik_ur,
)

# KUKA
from .core_iiwa import (
    forward_kinematics as forward_kinematics_iiwa,
    inverse_kinematics_fixed_q3 as ik_kuka,
)

def get_BONES():
    from .core_ur import ARMATURE_BONES as UR_BONES
    from .core_iiwa import ARMATURE_BONES as IIWA_BONES
    return IIWA_BONES if get_active_robot() == "KUKA" else UR_BONES

def get_joint_limits():
    if get_active_robot().startswith("UR"):
        return [(-360, 360)] * 6  # 6축
    else:
        return [
            (-170, 170),  # j1
            (-120, 120),  # j2
            (-170, 170),  # j3
            (-120, 120),  # j4
            (-170, 170),  # j5
            (-120, 120),  # j6
            (-175, 175),  # j7
        ]

def get_AXES():
    from .core_ur import AXES as UR_AXES
    from .core_iiwa import AXES as IIWA_AXES
    return IIWA_AXES if get_active_robot() == "KUKA" else UR_AXES

def get_forward_kinematics():
    from .core_iiwa import forward_kinematics as fk_kuka
    from .core_ur import forward_kinematics as fk_ur

    if get_active_robot() == "KUKA":
        return fk_kuka
    else:
        return fk_ur

def get_inverse_kinematics(p):
    from .core_iiwa import inverse_kinematics_fixed_q3
    from .core_ur import inverse_kinematics  

    if get_active_robot() == "KUKA":
        return lambda T: inverse_kinematics_fixed_q3(T, p.fixed_q3)
    else:
        solver = inverse_kinematics
        return solver

def compute_base_matrix(p):
    return np.array(p.base_object.matrix_world) if p.base_object else np.eye(4)

def compute_tcp_offset_matrix(p):
    T_offset = np.eye(4)
    
    if p.ee_object and p.tcp_object:
        M_ee = np.array(p.ee_object.matrix_world)
        M_tcp = np.array(p.tcp_object.matrix_world)

        T_offset = np.linalg.inv(M_ee) @ M_tcp
        axis_order = (2, 1, 0)        
        axis_signs = (1, 1, -1)       
        pos = T_offset[:3, 3].copy()
        corrected_pos = np.zeros(3)
        for i in range(3):
            corrected_pos[i] = pos[axis_order[i]] * axis_signs[i]
        T_offset[:3, 3] = corrected_pos

    if get_active_robot() == "KUKA":
        R_corr = R.from_euler('y', 90, degrees=True).as_matrix()
        T_offset[:3, :3] = T_offset[:3, :3] @ R_corr

    return T_offset

def apply_solution(arm, q, frame, insert_keyframe=True):
    AXES = get_AXES()
    BONES = get_BONES()
    
    if bpy.context.scene.frame_current != frame:
        bpy.context.scene.frame_set(frame)

    prev_act = bpy.context.view_layer.objects.active
    prev_mode = bpy.context.mode
    bpy.context.view_layer.objects.active = arm
    if prev_mode != 'POSE':
        bpy.ops.object.mode_set(mode='POSE')

    for i, bn in enumerate(BONES):
        pb = arm.pose.bones.get(bn)
        if not pb:
            print(f"[WARN] Bone {bn} not found!")
            continue

        axis = AXES[i]
        pb.rotation_mode = 'XYZ'
        q_target = q[i]

        if axis == 'x':
            rot = (q_target, 0, 0)
        elif axis == 'y':
            rot = (0, q_target, 0)
        else:
            rot = (0, 0, q_target)

        pb.rotation_euler = rot

    bpy.context.view_layer.update()
    bpy.context.evaluated_depsgraph_get().update()

    if insert_keyframe:
        for i, bn in enumerate(BONES):
            pb = arm.pose.bones.get(bn)
            if not pb:
                continue

            axis = AXES[i]
            q_target = q[i]

            pb.keyframe_insert("rotation_euler", frame=frame)

            curve_path = f'pose.bones["{bn}"].rotation_euler'
            curve_index = {'x': 0, 'y': 1, 'z': 2}[axis]
            fc = arm.animation_data.action.fcurves.find(curve_path, index=curve_index)
            if fc:
                to_delete = [kp for kp in fc.keyframe_points if int(kp.co[0]) == frame]
                for kp in to_delete:
                    fc.keyframe_points.remove(kp)
                fc.keyframe_points.insert(frame, q_target)
                for kp in fc.keyframe_points:
                    if int(kp.co[0]) == frame:
                        kp.interpolation = 'LINEAR'
                fc.update()

    if prev_mode != 'POSE':
        bpy.ops.object.mode_set(mode=prev_mode)
        bpy.context.view_layer.objects.active = prev_act

def get_armature_bones():
    if get_active_robot() == "KUKA":
        return ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
    else:
        return ["j1", "j2", "j3", "j4", "j5", "j6"]
    return []

def sort_solutions(sols):
    def score(q):
        s = 1 if q[0] > 0 else 0  # Shoulder
        e = 1 if q[2] > 0 else 0  # Elbow
        w = 1 if q[4] > 0 else 0  # Wrist
        return s * 4 + e * 2 + w
    return sorted(sols, key=score)

def solve_and_apply(ctx, p, T_goal, frame, insert_keyframe=True):

    fk_func = get_forward_kinematics()
    ik_solver = get_inverse_kinematics(p)
    
    T_base   = compute_base_matrix(p)
    T_offset = compute_tcp_offset_matrix(p)
    T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)
    
    ik_solver = get_inverse_kinematics(p)
    sols = ik_solver(T_flange)

    if not sols:
        print("❌ IK failed")
        return False

    def ang_diff(a, b):
        return ((a - b + pi) % (2 * pi)) - pi

    if p.solutions and len(p.solutions) > p.current_index:
        q_prev = p.solutions[p.current_index]
        best = min(
            range(len(sols)),
            key=lambda i: sum(abs(ang_diff(sols[i][j], q_prev[j])) for j in range(len(q_prev)))
        )
    elif get_active_robot().startswith("UR"):
        ss = -1 if p.shoulder == 'L' else 1
        es = -1 if p.elbow   == 'U' else 1
        ws = -1 if p.wrist   == 'I' else 1

        def score(q):
            return ((math.copysign(1, q[0]) == ss) +
                    (math.copysign(1, q[2]) == es) +
                    (math.copysign(1, q[4]) == ws))
        best = max(range(len(sols)), key=lambda i: score(sols[i]))
    else:
        best = 0

    arm = bpy.data.objects.get(p.armature)
    if not arm:
        print("[DEBUG] Armature not found:", p.armature)
        return False

    ctx.scene.frame_set(frame)

    apply_solution(arm, sols[best], frame, insert_keyframe=insert_keyframe)

    q_sel = sols[best]
    fk_func = get_forward_kinematics()
    T_fk = fk_func(q_sel)

    return True