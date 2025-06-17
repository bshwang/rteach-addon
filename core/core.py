import math
import bpy
import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R

from .robot_presets import ROBOT_CONFIGS
from .robot_state import get_active_robot

from .core_ur import forward_kinematics as fk_ur, inverse_kinematics as ik_ur
from .core_iiwa import forward_kinematics as fk_kuka, inverse_kinematics_fixed_q3 as ik_kuka

# ──────────────────────────────────────────────
def get_robot_config():
    p = bpy.context.scene.ik_motion_props
    return ROBOT_CONFIGS.get(getattr(p, "robot_type", ""), {})

# ──────────────────────────────────────────────
def get_BONES():
    config = get_robot_config()
    axes = config.get("axes")
    if axes:
        return [f"j{i+1}" for i in range(len(axes))]
    raise ValueError(f"[get_BONES] Unknown or misconfigured robot_type: {getattr(bpy.context.scene.ik_motion_props, 'robot_type', '')}")

def get_AXES():
    config = get_robot_config()
    return config.get("axes", [])

def get_joint_limits():
    config = get_robot_config()
    limits = config.get("joint_limits_deg")
    if limits:
        return limits
    raise ValueError(f"[get_joint_limits] joint_limits_deg missing for robot_type: {getattr(bpy.context.scene.ik_motion_props, 'robot_type', '')}")

def get_forward_kinematics():
    p = bpy.context.scene.ik_motion_props
    if p.robot_type.lower().startswith("ur"):
        return fk_ur
    else:
        return fk_kuka

def get_inverse_kinematics(p):
    if p.robot_type.lower().startswith("ur"):
        return ik_ur
    else:
        return lambda T: ik_kuka(T, p.fixed_q3)

# ──────────────────────────────────────────────
def compute_base_matrix(p):
    return np.array(p.base_object.matrix_world) if p.base_object else np.eye(4)

def compute_tcp_offset_matrix(p):
    try:
        if p.ee_object and p.tcp_object:
            M_ee = np.array(p.ee_object.matrix_world)
            M_tcp = np.array(p.tcp_object.matrix_world)
            if np.allclose(M_ee, M_tcp, atol=1e-6):
                return np.eye(4)
            return np.linalg.inv(M_ee) @ M_tcp
    except Exception as e:
        print(f"[ERROR] compute_tcp_offset_matrix(): {e}")
    return np.eye(4)

# ──────────────────────────────────────────────
def apply_solution(arm, q, frame, insert_keyframe=True):
    AXES  = get_AXES()
    BONES = get_BONES()

    if bpy.context.scene.frame_current != frame:
        bpy.context.scene.frame_set(frame)

    prev_act = bpy.context.view_layer.objects.active
    prev_mode = arm.mode
    prev_selected = [o for o in bpy.context.selected_objects]

    try:
        for o in bpy.context.selected_objects:
            o.select_set(False)
        arm.select_set(True)
        bpy.context.view_layer.objects.active = arm
        bpy.context.view_layer.update()
        if arm.mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
    except RuntimeError as e:
        print(f"[ERROR] Failed to enter POSE mode: {e}")
        return

    for i, bn in enumerate(BONES):
        pb = arm.pose.bones.get(bn)
        if not pb:
            print(f"[WARN] Bone {bn} not found!")
            continue
        axis = AXES[i]
        pb.rotation_mode = 'XYZ'
        q_target = q[i]
        rot = (q_target, 0, 0) if axis == 'x' else (0, q_target, 0) if axis == 'y' else (0, 0, q_target)
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

    if prev_act:
        bpy.context.view_layer.objects.active = prev_act
    for o in bpy.data.objects:
        o.select_set(False)
    for o in prev_selected:
        o.select_set(True)
    if prev_mode != 'POSE':
        try:
            bpy.ops.object.mode_set(mode=prev_mode)
        except:
            pass

# ──────────────────────────────────────────────
def sort_solutions(sols):
    def score(q):
        s = 1 if q[0] > 0 else 0
        e = 1 if q[2] > 0 else 0
        w = 1 if q[4] > 0 else 0
        return s * 4 + e * 2 + w
    return sorted(sols, key=score)

def get_best_ik_solution(p, T_goal, q_ref=None):
    T_base = compute_base_matrix(p)
    T_offset = compute_tcp_offset_matrix(p)
    T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)

    ik_solver = get_inverse_kinematics(p)
    robot = p.robot_type
    sols = ik_solver(T_flange)
    if not sols:
        return None, []

    joint_limits = get_joint_limits()
    ll = np.radians([lim[0] for lim in joint_limits])
    ul = np.radians([lim[1] for lim in joint_limits])

    filtered = []
    for i, q in enumerate(sols):
        if len(q) != len(ll):
            continue
        if not np.all(q >= ll) or not np.all(q <= ul):
            continue
        filtered.append(q)
    sols = filtered
    if not sols:
        return None, []

    def ang_diff(a, b):
        return ((a - b + pi) % (2 * pi)) - pi

    if q_ref is not None:
        best = min(
            range(len(sols)),
            key=lambda i: sum(
                abs(ang_diff(sols[i][j], q_ref[j]))
                for j in range(min(len(sols[i]), len(q_ref)))
            )
        )
    elif robot.lower().startswith("ur"):
        ss = -1 if p.shoulder == 'L' else 1
        es = -1 if p.elbow == 'U' else 1
        ws = -1 if p.wrist == 'I' else 1
        def score(q):
            return ((math.copysign(1, q[0]) == ss) +
                    (math.copysign(1, q[2]) == es) +
                    (math.copysign(1, q[4]) == ws))
        best = max(range(len(sols)), key=lambda i: score(sols[i]))
    else:
        best = 0

    return sols[best], sols
