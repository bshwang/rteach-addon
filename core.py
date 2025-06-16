import math
import bpy
import numpy as np
from math import pi, degrees
from scipy.spatial.transform import Rotation as R
from .robot_state import get_active_robot, set_active_robot

# UR
from .core_ur import (
    forward_kinematics as fk_ur,
    inverse_kinematics as ik_ur,
    ARMATURE_BONES as UR_BONES,
    AXES as UR_AXES,
)

# KUKA
from .core_iiwa import (
    forward_kinematics as fk_kuka,
    inverse_kinematics_fixed_q3 as ik_kuka,
    ARMATURE_BONES as IIWA_BONES,
    AXES as IIWA_AXES,
)

CUSTOM_JOINT_LIMITS = None
ACTIVE_SYSTEM_NAME = None  # <- system 이름 보존용

def set_active_robot(name: str):
    global ACTIVE_SYSTEM_NAME
    ACTIVE_SYSTEM_NAME = name


def get_BONES():
    robot = get_active_robot(safe=True)
    if robot.startswith("UR"):
        return UR_BONES
    elif robot == "KUKA":
        return IIWA_BONES
    else:
        return ValueError(f"[get_BONES] Unknown robot_type '{robot}'")

def get_joint_limits():
    global CUSTOM_JOINT_LIMITS
    robot = get_active_robot(safe=True)

    if CUSTOM_JOINT_LIMITS:
        return CUSTOM_JOINT_LIMITS

    if robot.startswith("UR"):
        return [(-360, 360)] * 6
    elif robot == "KUKA":
        return [
            (-170, 170),
            (-120, 120),
            (-170, 170),
            (-120, 120),
            (-170, 170),
            (-120, 120),
            (-175, 175)
        ]
    else:
        return [(-360, 360)] * 6 

def get_AXES():
    robot = get_active_robot(safe=True)
    if robot.startswith("UR"):
        return UR_AXES
    elif robot == "KUKA":
        return IIWA_AXES
    else:
        return []

def get_forward_kinematics():
    robot = get_active_robot(safe=True)
    if robot.startswith("UR"):
        return fk_ur
    elif robot == "KUKA":
        return fk_kuka
    else:
        return []

def get_inverse_kinematics(p):
    robot = get_active_robot(safe=True)
    if robot.startswith("UR"):
        return ik_ur
    elif robot == "KUKA":
        return lambda T: ik_kuka(T, p.fixed_q3)
    else:
        return []

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
    AXES  = get_AXES()
    BONES = get_BONES()

    if bpy.context.scene.frame_current != frame:
        bpy.context.scene.frame_set(frame)

    prev_act     = bpy.context.view_layer.objects.active
    prev_mode    = arm.mode
    prev_selected= [o for o in bpy.context.selected_objects]

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

            axis     = AXES[i]
            q_target = q[i]
            pb.keyframe_insert("rotation_euler", frame=frame)

            curve_path  = f'pose.bones["{bn}"].rotation_euler'
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

def get_armature_bones():
    if get_active_robot(safe=True) == "KUKA":
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

    T_base = compute_base_matrix(p)
    T_offset = compute_tcp_offset_matrix(p)
    T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)

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
    elif get_active_robot(safe=True).startswith("UR"):
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

    arm = bpy.data.objects.get(p.armature)
    if not arm:
        print("[DEBUG] Armature not found:", p.armature)
        return False

    ctx.scene.frame_set(frame)
    apply_solution(arm, sols[best], frame, insert_keyframe=insert_keyframe)

    return True

def get_best_ik_solution(p, T_goal, q_ref=None):
    T_base   = compute_base_matrix(p)
    T_offset = compute_tcp_offset_matrix(p)
    T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)


    ik_solver = get_inverse_kinematics(p)
    robot = get_active_robot(safe=True)
    print(f"[DEBUG][IK] Active robot: {robot}")
    print("[DEBUG][IK] IK Solver function:", ik_solver)
    print(f"[DEBUG][BLENDER] T_flange:\n{T_flange}")
    sols = ik_solver(T_flange)
    print(f"[DEBUG][BLENDER] IK solutions: {len(sols)}")
    for i, q in enumerate(sols):
        print(f"  Sol {i}: {np.round(q, 4)}")
    
    joint_limits = get_joint_limits()
    ll = np.radians([lim[0] for lim in joint_limits])
    ul = np.radians([lim[1] for lim in joint_limits])

    for i, q in enumerate(sols):
        print(f"  Sol {i}: {np.round(q, 4)}")

    print(f"[DEBUG] Total IK solutions before filtering: {len(sols)}")
    print(f"[DEBUG] Joint limits: LL={ll}, UL={ul}")

    filtered = []
    for i, q in enumerate(sols):
        if len(q) != len(ll):
            print(f"[WARN] Sol {i} skipped due to length mismatch: {len(q)} vs {len(ll)}")
            continue
        if not np.all(q >= ll) or not np.all(q <= ul):
            print(f"[WARN] Sol {i} filtered out by joint limit check")
            continue
        filtered.append(q)

    print(f"[DEBUG] {len(filtered)} solutions remained after filtering")
    sols = filtered

    if not sols:
        return None, []

    def ang_diff(a, b):
        return ((a - b + math.pi) % (2 * math.pi)) - math.pi

    if q_ref is not None:
        best = min(
            range(len(sols)),
            key=lambda i: sum(
                abs(ang_diff(sols[i][j], q_ref[j]))
                for j in range(min(len(sols[i]), len(q_ref)))  # ✅ 길이 보정
            )
        )
    elif get_active_robot(safe=True).startswith("UR"):
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

    return sols[best], sols
