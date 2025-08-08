import math
import bpy
import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R

from rteach.core.robot_presets import ROBOT_CONFIGS
from rteach.core.robot_state import get_armature_type
from rteach.core.core_ur import forward_kinematics as fk_ur, inverse_kinematics as ik_ur
from rteach.core.core_iiwa import forward_kinematics as fk_kuka, inverse_kinematics_fixed_q3 as ik_kuka

def get_robot_config():
    p = bpy.context.scene.ik_motion_props
    return ROBOT_CONFIGS.get(getattr(p, "robot_type", ""), {})

def get_BONES():
    config = get_robot_config()
    axes = config.get("axes")
    if axes:
        return [f"j{i+1}" for i in range(len(axes))]
    return []

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
    arm_type = get_armature_type(p.robot_type)
    if arm_type == "UR":
        return fk_ur 
    elif arm_type == "KUKA":
        return fk_kuka
    else:
        raise ValueError(f"Unknown armature_type: {arm_type}")

def get_inverse_kinematics(p):
    arm_type = get_armature_type(p.robot_type)
    if arm_type == "UR":
        return ik_ur
    elif arm_type == "KUKA":
        return lambda T: ik_kuka(T, p.fixed_q3)
    else:
        raise ValueError(f"Unknown armature_type: {arm_type}")

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

def apply_solution(arm, q, frame, insert_keyframe=True):
    AXES = get_AXES()
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

def sort_solutions(sols):
    def score(q):
        s = 1 if q[0] > 0 else 0
        e = 1 if q[2] > 0 else 0
        w = 1 if q[4] > 0 else 0
        return s * 4 + e * 2 + w
    return sorted(sols, key=score)

def solve_and_apply(ctx, p, T_goal, frame, insert_keyframe=True):
    print("[solve_and_apply] Start")
    print("→ frame:", frame)
    print("→ insert_keyframe:", insert_keyframe)

    fk_func = get_forward_kinematics()
    ik_solver = get_inverse_kinematics(p)

    T_base = compute_base_matrix(p)
    T_offset = compute_tcp_offset_matrix(p)
    T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)

    sols = ik_solver(T_flange)

    joint_limits = get_joint_limits()
    ll = np.radians([lim[0] for lim in joint_limits])
    ul = np.radians([lim[1] for lim in joint_limits])

    sols = [q for q in sols if np.all(q >= ll) and np.all(q <= ul)]
    
    print(f"[IK] Solutions found: {len(sols)}")
    for i, q in enumerate(sols):
        q_deg = [round(math.degrees(a), 2) for a in q]
        print(f"  ▷ sol[{i}] = {q_deg}")

    if not sols:
        print("IK failed")
        return False

    def ang_diff(a, b):
        return ((a - b + pi) % (2 * pi)) - pi

    if p.solutions and len(p.solutions) > p.current_index:
        q_prev = p.solutions[p.current_index]
        print("[DEBUG] Selecting best match to previous q:", q_prev)
        best = min(
            range(len(sols)),
            key=lambda i: sum(abs(ang_diff(sols[i][j], q_prev[j])) for j in range(min(len(sols[i]), len(q_prev))))
        )
    elif get_armature_type(p.robot_type) == "UR":
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

    print(f"Selected solution index: {best}")

    arm = bpy.data.objects.get(p.armature)
    if not arm:
        print("[DEBUG] Armature not found:", p.armature)
        return False

    ctx.scene.frame_set(frame)
    apply_solution(arm, sols[best], frame, insert_keyframe=insert_keyframe)

    q_sel = sols[best]
    fk_func = get_forward_kinematics()
    T_fk = fk_func(q_sel)
    print("[DEBUG] FK result of selected solution:\n", T_fk)

    print("[solve_and_apply] Done")
    return True

def get_best_ik_solution(p, T_goal, q_ref=None):
    T_base = compute_base_matrix(p)
    T_offset = compute_tcp_offset_matrix(p)
    T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)

    print("[IK] Solving IK for T_goal:")
    for row in T_goal:
        print("  ", [round(v, 4) for v in row])

    ik_solver = get_inverse_kinematics(p)
    sols = ik_solver(T_flange)
    if not sols:
        print("[IK] No IK solution found.")
        return None, []

    joint_limits = get_joint_limits()
    ll = np.radians([lim[0] for lim in joint_limits])
    ul = np.radians([lim[1] for lim in joint_limits])

    sols = [q for q in sols if np.all(q >= ll) and np.all(q <= ul)]

    print(f"[IK] Valid IK solutions: {len(sols)}")
    for i, q in enumerate(sols):
        q_deg = [round(math.degrees(a), 2) for a in q]
        print(f"  sol[{i}] = {q_deg}")

    if not sols:
        print("[IK] All solutions out of joint limits.")
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
        print(f"[IK] q_ref-based match: best = {best}")
    elif get_armature_type(p.robot_type) == "UR":
        ss = -1 if p.shoulder == 'L' else 1
        es = -1 if p.elbow == 'U' else 1
        ws = -1 if p.wrist == 'I' else 1

        def score(q):
            return ((math.copysign(1, q[0]) == ss) +
                    (math.copysign(1, q[2]) == es) +
                    (math.copysign(1, q[4]) == ws))
        best = max(range(len(sols)), key=lambda i: score(sols[i]))
        print(f"[IK] Shoulder/Elbow/Wrist preference match: best = {best}")
    else:
        best = 0
        print(f"[IK] Defaulting to first solution: best = {best}")

    q_best = sols[best]
    q_best_deg = [round(math.degrees(a), 2) for a in q_best]
    print(f"[IK] Selected solution: {q_best_deg}")

    return q_best, sols

def get_tcp_object():
    p = bpy.context.scene.ik_motion_props
    return p.tcp_object if p and p.tcp_object else None

def is_stage_robot_key(rob_key: str) -> bool:
    config = ROBOT_CONFIGS.get(rob_key.lower())
    return bool(config and config.get("stage_joints"))

def get_stage_joint_names(rob_key, rob_key_to_config_key):
    preset_key = rob_key_to_config_key.get(rob_key)
    config = ROBOT_CONFIGS.get(preset_key, {})
    return [joint[0] for joint in config.get("stage_joints", [])] 

