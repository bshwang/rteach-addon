import bpy 
import math
import numpy as np, math

from mathutils import Matrix
from scipy.spatial.transform import Rotation as R, Slerp
from rteach.core.robot_state import get_armature_type
from rteach.core.core import (
    apply_solution, get_inverse_kinematics, compute_base_matrix, 
    compute_tcp_offset_matrix, get_forward_kinematics, 
    get_BONES, get_AXES, get_best_ik_solution, get_joint_limits
)
from rteach.core.core_iiwa import linear_move
from rteach.ops.ops_teach_util import update_tcp_sorted_list, find_object_by_prefix
from rteach.ops.ops_teach_main import get_best_ik_solution, apply_solution
from rteach.core.robot_presets import ROBOT_CONFIGS

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_teach_pose(bpy.types.Operator):
    bl_idname = "object.teach_pose"
    bl_label = "Teach"

    def execute(self, ctx):
        print("[DEBUG] IK operator started")

        p = ctx.scene.ik_motion_props
        tgt = p.goal_object
        print("[DEBUG] Goal object:", tgt)
        print("[DEBUG] Armature:", p.armature)
        print(f"[DEBUG] UI fixed_q3 (rad): {round(p.fixed_q3, 6)} ({round(math.degrees(p.fixed_q3), 2)}°)")

        q = tgt.matrix_world.to_quaternion()
        T_goal = np.eye(4)
        T_goal[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_goal[:3, 3] = np.array(tgt.matrix_world.to_translation())

        print("[DEBUG] T_goal matrix:")
        for row in T_goal:
            print("  ", [round(v, 4) for v in row])
        print("[DEBUG] T_goal position:", [round(v, 4) for v in tgt.location])
        print("[DEBUG] T_goal rotation (deg):", [round(math.degrees(a), 2) for a in tgt.rotation_euler])

        ik_solver = get_inverse_kinematics(p)
        T_base = compute_base_matrix(p)
        T_offset = compute_tcp_offset_matrix(p)
        T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)

        print("[DEBUG] T_flange matrix (IK input):")
        for row in T_flange:
            print("  ", [round(v, 4) for v in row])

        sols = ik_solver(T_flange)

        print(f"[DEBUG] Raw IK solutions: {len(sols)}")

        joint_limits = get_joint_limits()
        ll = np.radians([lim[0] for lim in joint_limits])
        ul = np.radians([lim[1] for lim in joint_limits])
        print("[DEBUG] Joint limits (deg):")
        for i, (a, b) in enumerate(joint_limits):
            print(f"  j{i+1}: {a}° ~ {b}°")

        valid_sols = []
        for i, q in enumerate(sols):
            out_of_bounds = [(j+1, math.degrees(q[j])) for j in range(len(q)) if q[j] < ll[j] or q[j] > ul[j]]
            if not out_of_bounds:
                valid_sols.append(q)
            else:
                print(f"[WARN] sol[{i}] exceeds limits: {[f'j{j}={round(val,1)}°' for j, val in out_of_bounds]}")

        print(f"[DEBUG] Valid IK solutions after joint-limit filtering: {len(valid_sols)}")

        if not valid_sols:
            print("[RESULT] No valid IK solutions found.")
            p.status_text = "IK failed: no valid solutions"
            return {'FINISHED'}

        # 기존 가장 가까운 해 선택 로직 유지
        q_prev = p.solutions[p.current_index] if p.solutions and len(p.solutions) > p.current_index else None
        if not q_prev:
            arm = bpy.data.objects.get(p.armature)
            if arm:
                q_prev = []
                bones = get_BONES()
                axes = get_AXES()
                for i, bn in enumerate(bones):
                    pb = arm.pose.bones.get(bn)
                    if pb:
                        q_prev.append(getattr(pb.rotation_euler, axes[i]))

        if q_prev:
            def ang_diff(a, b):
                return ((a - b + math.pi) % (2 * math.pi)) - math.pi
            idx = min(range(len(valid_sols)),
                      key=lambda i: sum(abs(ang_diff(valid_sols[i][j], q_prev[j]))
                                        for j in range(min(len(valid_sols[i]), len(q_prev)))))
        else:
            idx = 0

        p.solutions = [list(map(float, s)) for s in valid_sols]
        p.max_solutions = len(valid_sols)
        p.current_index = idx
        p.solution_index_ui = idx + 1
        q_sel = p.solutions[idx]

        frame = ctx.scene.frame_current
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            p.status_text = "IK failed: Armature not found"
            return {'FINISHED'}

        apply_solution(arm, q_sel, frame, insert_keyframe=False)

        fk_func = get_forward_kinematics()
        T_fk = fk_func(q_sel)
        T_fk_world = compute_base_matrix(p) @ T_fk @ compute_tcp_offset_matrix(p)

        pos_fk = T_fk_world[:3, 3] * 1000
        rot_fk = Matrix(T_fk_world[:3, :3]).to_euler('XYZ')
        rot_fk_deg = [math.degrees(a) for a in rot_fk]

        pos_goal = T_goal[:3, 3] * 1000
        rot_goal = Matrix(T_goal[:3, :3]).to_euler('XYZ')
        rot_goal_deg = [math.degrees(a) for a in rot_goal]

        print("[DEBUG] FK vs Target comparison")
        print(f"  Target Pos (mm): {np.round(pos_goal,2)}")
        print(f"  FK Pos     (mm): {np.round(pos_fk,2)}")
        print(f"  Target Rot (°):  {[round(v,2) for v in rot_goal_deg]}")
        print(f"  FK Rot     (°):  {[round(v,2) for v in rot_fk_deg]}")

        p.status_text = f"Applied {idx+1}/{len(valid_sols)}"
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_execute_linear_motion(bpy.types.Operator):
    bl_idname = "object.execute_linear_motion"
    bl_label  = "Move_L"

    DEBUG_MOVE_L = False

    def execute(self, ctx):

        p = ctx.scene.ik_motion_props
        src, dst = p.goal_object, p.linear_target
        if not (src and dst):
            p.status_text = "Linear source/target not set"
            return {'FINISHED'}

        arm = bpy.data.objects.get(p.armature)
        if not arm:
            self.report({'ERROR'}, "Armature not found")
            return {'CANCELLED'}

        dof     = len(get_BONES())
        frame0  = ctx.scene.frame_current
        steps   = max(1, int(p.linear_frames))

        # Precise LIN
        if get_armature_type(p.robot_type) == "KUKA" and p.precise_linear and src.get("joint_pose"):
            q_init = np.array(sanitize_q(src["joint_pose"], dof), float)
            T_goal = (
                np.linalg.inv(compute_base_matrix(p))
                @ np.array(dst.matrix_world)
                @ np.linalg.inv(compute_tcp_offset_matrix(p))
            )

            path = linear_move(q_init, T_goal, p)
            if path:
                for i, q in enumerate(path):
                    ctx.scene.frame_set(frame0 + i)
                    apply_solution(arm, q, frame0 + i, insert_keyframe=True)
                p.status_text = f"Precise LIN done ({len(path)} steps)"
                return {'FINISHED'}

            self.report({'WARNING'}, "Precise LIN failed – fallback to fast mode")

        # Fast LIN
        M_start, M_end = map(np.array, (src.matrix_world, dst.matrix_world))
        pos_series = np.linspace(M_start[:3, 3], M_end[:3, 3], steps)

        q_start = src.matrix_world.to_quaternion()
        q_end   = dst.matrix_world.to_quaternion()
        R_key   = R.from_quat([[q_start.x, q_start.y, q_start.z, q_start.w],
                               [q_end.x,   q_end.y,   q_end.z,   q_end.w]])
        slerp   = Slerp([0, 1], R_key)

        ik_solver = get_inverse_kinematics(p)

        preferred_idx = int(src.get("solution_index", p.current_index))
        master_q      = sanitize_q(src.get("joint_pose", []), dof)
        print(f"[Bake] Source TCP: {src.name}, preferred_idx = {preferred_idx}, q_start = {[round(math.degrees(v),1) for v in master_q]}")

        def ang_diff(a, b):
            return ((a - b + math.pi) % (2 * math.pi)) - math.pi

        def _pattern(q):
            return (q[0] >= 0, q[2] >= 0, q[4] >= 0)

        preferred_pat = _pattern(master_q)

        for i in range(steps):
            f = frame0 + i
            T_goal = np.eye(4)
            T_goal[:3, 3] = pos_series[i]
            t = i / (steps - 1) if steps > 1 else 0.0
            T_goal[:3, :3] = slerp([t])[0].as_matrix()

            T_flange = (
                np.linalg.inv(compute_base_matrix(p))
                @ T_goal
                @ np.linalg.inv(compute_tcp_offset_matrix(p))
            )
            U, _, Vt = np.linalg.svd(T_flange[:3, :3])
            T_flange[:3, :3] = U @ Vt

            sols_raw = ik_solver(T_flange)
            sols = [sanitize_q(s, dof) for s in sols_raw]
            
            if not sols:
                if self.DEBUG_MOVE_L:
                    print(f"[Move_L] f{f}: IK fail")
                continue

            q_sel = sols[preferred_idx] if preferred_idx < len(sols) else None

            if q_sel is None:
                same_pat = [q for q in sols if _pattern(q) == preferred_pat]
                if same_pat:
                    q_sel = min(same_pat, key=lambda q: sum(abs(ang_diff(q[j], master_q[j])) for j in range(dof)))

            if q_sel is None:
                q_sel = min(sols, key=lambda q: sum(abs(ang_diff(q[j], master_q[j])) for j in range(dof)))

            chosen_idx = sols.index(q_sel)

            if i == steps - 1 and dst.get("joint_pose"):
                q_sel = sanitize_q(dst["joint_pose"], dof)
                chosen_idx = dst.get("solution_index", chosen_idx)

            if self.DEBUG_MOVE_L:
                print(f"[Move_L] f{f}: picked idx={chosen_idx}, pat={_pattern(q_sel)}, q={[round(math.degrees(v),1) for v in q_sel]}")

            master_q = q_sel

            if self.DEBUG_MOVE_L:
                print(f"[Move_L] f{frame0+i}: idx={chosen_idx}, q={[round(math.degrees(v),1) for v in q_sel]}")

            ctx.scene.frame_set(frame0 + i)
            apply_solution(arm, q_sel, frame0 + i, insert_keyframe=True)

        fk_func = get_forward_kinematics()
        T_last = (
            compute_base_matrix(p)
            @ fk_func(master_q)
            @ compute_tcp_offset_matrix(p)
        )
        p.goal_object.matrix_world = Matrix(T_last)

        p.status_text = f"Fast LIN done ({steps} steps)"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────
def sanitize_q(q_raw, dof):
    q = list(q_raw)
    return (q + [0.0] * dof)[:dof]

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_record_tcp_point(bpy.types.Operator):
    bl_idname = "object.record_tcp_point"
    bl_label  = "Record Goal Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        tgt = p.goal_object
        if not tgt:
            self.report({'ERROR'}, "Goal not set")
            return {'CANCELLED'}

        coll = bpy.data.collections.get("Teach data") or bpy.data.collections.new("Teach data")
        if coll.name not in {c.name for c in ctx.scene.collection.children}:
            ctx.scene.collection.children.link(coll)

        # goal label
        goal_label = tgt.get("goal", "").strip()
        if not goal_label:
            goal_label = "GOAL"

        # count seq for this goal
        same_goal_objs = [o for o in coll.objects if o.get("goal", "").strip() == goal_label]
        goal_seq = len(same_goal_objs) + 1

        # suffix: Left or Right
        rob_key = p.robot_type.lower()
        if "left" in rob_key:
            suffix = "L"
        elif "right" in rob_key:
            suffix = "R"
        else:
            suffix = "X"

        name = f"{goal_label}_{goal_seq:02d}_{suffix}"

        dup = bpy.data.objects.new(name=name, object_data=tgt.data)

        if "_RNA_UI" not in dup:
            dup["_RNA_UI"] = {}
        dup["_RNA_UI"]["speed"] = {"min": 0.0, "max": 500.0, "soft_min": 0.0, "soft_max": 500.0}
        dup["_RNA_UI"]["wait_time_sec"] = {"min": 0.0, "max": 10.0, "soft_min": 0.0, "soft_max": 10.0}

        dup.matrix_world = tgt.matrix_world
        dup["index"] = len([o for o in bpy.data.objects if o.name.startswith("P.")])
        dup["motion_type"] = "LINEAR"
        dup["fixed_q3"] = p.fixed_q3
        dup["speed"] = p.motion_speed
        dup["wait_time_sec"] = p.wait_time_sec
        dup["bake_enabled"] = True
        dup["robot"] = p.robot_type
        dup.show_name = True
        dup["goal"] = goal_label

        dup["_RNA_UI"]["goal"] = {"description": "Teach point goal name"}

        if p.solutions and len(p.solutions) > p.current_index:
            q_safe = [float(v) for v in p.solutions[p.current_index]]
            dup["solution_index"] = p.current_index
            dup["joint_pose"] = q_safe

        coll.objects.link(dup)

        prev_solutions = p.solutions[:]
        prev_idx       = p.current_index
        update_tcp_sorted_list()
        p.solutions     = prev_solutions
        p.current_index = prev_idx

        p.selected_teach_point = dup
        p.tcp_list_index       = len(p.tcp_sorted_list) - 1
        p.solution_index_ui    = p.current_index + 1

        self.report({'INFO'}, f"Recorded: {name}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_apply_preview_pose(bpy.types.Operator):
    bl_idname = "object.apply_preview_pose"
    bl_label = "Apply Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        tgt = p.goal_object
        obj = p.selected_teach_point

        if not arm or not tgt or not obj:
            self.report({'ERROR'}, "Target or armature or TCP not set")
            return {'CANCELLED'}

        q = tgt.matrix_world.to_quaternion()
        T_goal = np.eye(4)
        T_goal[:3, :3] = q.to_matrix()
        T_goal[:3, 3] = np.array(tgt.matrix_world.to_translation())

        if not p.solutions:
            q_sel, sols = get_best_ik_solution(p, T_goal)
            if not sols:
                self.report({'ERROR'}, "IK failed")
                return {'CANCELLED'}

            p.solutions = [list(map(float, s)) for s in sols]
            p.current_index = 0

        if p.current_index >= len(p.solutions):
            self.report({'ERROR'}, "Invalid solution index")
            return {'CANCELLED'}

        q_sel = p.solutions[p.current_index]
        frame = ctx.scene.frame_current
        ctx.scene.frame_set(frame)
        apply_solution(arm, q_sel, frame, insert_keyframe=False)

        obj["joint_pose"] = list(map(float, q_sel))
        obj["solution_index"] = p.current_index

        if get_armature_type(p.robot_type) == "KUKA":
            obj["fixed_q3"] = p.fixed_q3

        p.status_text = f"Keyframed {p.current_index+1}/{len(p.solutions)}"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────
class OBJECT_OT_bake_teach_sequence(bpy.types.Operator):
    bl_idname = "object.bake_teach_sequence"
    bl_label  = "Bake Sequence"

    def execute(self, ctx):

        p   = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        giz = p.goal_object
        coll = bpy.data.collections.get("Teach data")
        if not (arm and giz and coll):
            self.report({'ERROR'}, "Armature / Goal / Teach data not set")
            return {'CANCELLED'}

        tps = sorted(
            (o for o in coll.objects if o.name.startswith("P.") and o.get("bake_enabled", False)),
            key=lambda o: o.get("index", 9999)
        )
        if not tps:
            self.report({'WARNING'}, "No TCPs selected for baking")
            return {'CANCELLED'}

        fps           = ctx.scene.render.fps
        default_speed = p.motion_speed
        default_wait  = p.wait_time_sec
        f             = p.bake_start_frame

        fk_func = get_forward_kinematics()
        T_base  = compute_base_matrix(p)
        T_off   = compute_tcp_offset_matrix(p)

        for i, tp in enumerate(tps):

            q_tp = np.asarray(tp.get("joint_pose"), float)
            if q_tp.size == 0:
                self.report({'WARNING'}, f"{tp.name} missing joint_pose")
                return {'CANCELLED'}
            q_tp_deg = [round(math.degrees(a),1) for a in q_tp]

            giz.matrix_world = tp.matrix_world.copy()
            ctx.scene.frame_set(f)
            apply_solution(arm, q_tp, f, insert_keyframe=True)

            fk_world = T_base @ fk_func(q_tp) @ T_off
            tcp_pos  = np.array(tp.matrix_world.translation) * 1000
            fk_pos   = fk_world[:3,3] * 1000
            print(f"[CHK] TCP {tp.name} FK = {np.round(fk_pos,1)} mm | TCP = {np.round(tcp_pos,1)} mm")
            print(f"[CHK] TCP {tp.name} Q_saved = {q_tp_deg}")

            wait_s     = tp.get("wait_time_sec", default_wait)
            wait_frames = round(wait_s * fps)
            if wait_frames > 1:
                f_wait_end = f + wait_frames - 1
                ctx.scene.frame_set(f_wait_end)
                apply_solution(arm, q_tp, f_wait_end, insert_keyframe=True)
                fk_wait = T_base @ fk_func(q_tp) @ T_off
                fk_wpos = fk_wait[:3,3] * 1000
                print(f"[BAKE] {tp.name} | Wait {wait_s}s → {wait_frames} frames (f{f} ~ f{f_wait_end})")
                print(f"[CHK] WAIT {tp.name} FK = {np.round(fk_wpos,1)} mm | TCP = {np.round(tcp_pos,1)} mm | Q_wait = {q_tp_deg}")
                f = f_wait_end

            if i == len(tps) - 1:
                continue

            next_tp = tps[i + 1]
            motion  = tp.get("motion_type", "LINEAR").upper()

            A         = np.array(tp.matrix_world.translation)
            B         = np.array(next_tp.matrix_world.translation)
            dist_mm   = np.linalg.norm(B - A) * 1000.0
            speed_mm  = tp.get("speed", default_speed)
            motion_s  = dist_mm / max(speed_mm, 1e-3)
            span      = max(1, round(motion_s * fps))

            print(f"[BAKE] Segment: {tp.name} → {next_tp.name}")
            print(f"  Distance: {round(dist_mm,1)} mm  |  Speed: {speed_mm} mm/s")
            print(f"  Motion time: {round(motion_s,3)} s | Frame span: {span}")
            print(f"  Start frame: {f} → End frame: {f+span-1}")

            if motion == "LINEAR":
                precise_done = False
                if get_armature_type(p.robot_type) == "KUKA" and p.precise_linear:
                    q_start = np.asarray(q_tp, float)
                    T_start = T_base @ fk_func(q_start) @ T_off
                    T_goal  = np.array(next_tp.matrix_world)
                    T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_off)

                    p.step_mm = dist_mm / max(span, 1)
                    path = linear_move(q_start, T_flange, p)
                    if path:
                        print(f"  [IK] Precise LIN solved, path length = {len(path)}")
                        for k, q in enumerate(path):
                            fk_k  = T_base @ fk_func(q) @ T_off
                            pos_k = fk_k[:3,3] * 1000
                            print(f"    Step {k}: FK_pos = {np.round(pos_k,1)} mm | Q = {[round(math.degrees(a),1) for a in q]}")
                            ctx.scene.frame_set(f + k)
                            apply_solution(arm, q, f + k, insert_keyframe=True)

                        fk_end  = T_base @ fk_func(path[-1]) @ T_off
                        tcp_end = np.array(next_tp.matrix_world.translation) * 1000
                        print(f"[CHK] LIN start FK = {np.round((T_start[:3,3]*1000),1)} mm | TCP {tp.name} = {np.round(tcp_pos,1)} mm")
                        print(f"[CHK] LIN start Q_fk = {[round(math.degrees(a),1) for a in path[0]]} | Q_tcp = {q_tp_deg}")
                        print(f"[CHK] LIN end   FK = {np.round((fk_end[:3,3]*1000),1)} mm | TCP {next_tp.name} = {np.round(tcp_end,1)} mm")
                        print(f"[CHK] LIN end   Q_fk = {[round(math.degrees(a),1) for a in path[-1]]} | Q_tcp = {[round(math.degrees(a),1) for a in next_tp.get('joint_pose',[])]}")

                        next_tp["joint_pose"] = list(map(float, path[-1]))
                        f += len(path) - 1
                        precise_done = True

                if not precise_done:
                    p.goal_object   = giz
                    p.linear_target = next_tp
                    p.linear_frames = span
                    ctx.scene.frame_set(f + 1)
                    bpy.ops.object.execute_linear_motion('EXEC_DEFAULT')
                    f += span - 1

            elif motion == "JOINT":
                q_next = next_tp.get("joint_pose")
                if q_next is None:
                    self.report({'WARNING'}, f"{next_tp.name} missing joint_pose")
                    return {'CANCELLED'}
                ctx.scene.frame_set(f + span - 1)
                apply_solution(arm, q_next, f + span - 1, insert_keyframe=True)
                giz.matrix_world = next_tp.matrix_world.copy()
                f += span - 1

            else:
                self.report({'WARNING'}, f"{tp.name} unsupported motion_type")
                return {'CANCELLED'}

            f += 1

        self.report({'INFO'}, "Bake finished")
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_update_tcp_pose(bpy.types.Operator):
    bl_idname = "object.update_tcp_pose"
    bl_label  = "Update TCP Pose"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        print("▶ [UPDATE TCP] Operator invoked")

        p   = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        obj = p.selected_teach_point

        if not obj:
            self.report({'ERROR'}, "No TCP selected in UI panel")
            return {'CANCELLED'}

        if "Path" not in obj:
            obj["Path"] = ""

        tgt = p.goal_object
        if not (arm and tgt):
            self.report({'ERROR'}, "Goal / Armature not set")
            return {'CANCELLED'}

        if ctx.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        ctx.view_layer.objects.active = obj

        import numpy as np
        from rteach.core.robot_state import get_armature_type

        tgt.matrix_world = obj.matrix_world.copy()

        q = tgt.matrix_world.to_quaternion()
        T_goal = np.eye(4)
        T_goal[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_goal[:3, 3] = np.array(tgt.matrix_world.to_translation())

        q_ref = np.asarray(obj.get("joint_pose", []), float)

        if get_armature_type(p.robot_type) == "KUKA" and q_ref.size >= 3:
            p.fixed_q3 = float(q_ref[2])
            print(f"→ Pre-set fixed_q3 from joint_pose: {math.degrees(p.fixed_q3):.2f}°")

        moved = not np.allclose(
            np.array(obj.matrix_world),
            np.array(tgt.matrix_world),
            atol=1e-6
        )
        fixed_q3_changed = False
        if get_armature_type(p.robot_type) == "KUKA":
            prev_q3 = float(obj.get("fixed_q3", 0.0))
            cur_q3  = float(p.fixed_q3)
            fixed_q3_changed = abs(prev_q3 - cur_q3) > 1e-6

        print(f"→ Moved: {moved}, fixed_q3 changed: {fixed_q3_changed}")

        if moved or fixed_q3_changed:
            print("→ Mode: IK update from TCP → Gizmo")

            if q_ref.size > 0:
                q_sel, sols = get_best_ik_solution(p, T_goal, q_ref=q_ref)
            else:
                q_sel, sols = get_best_ik_solution(p, T_goal)

            print(f"→ IK solutions found: {len(sols)}")
            if not sols:
                self.report({'ERROR'}, "IK failed")
                return {'CANCELLED'}

            print(f"→ Applying q_sel: {[round(math.degrees(a),1) for a in q_sel]}")
            apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)

            idx = next((i for i, s in enumerate(sols)
                        if np.allclose(s, q_sel, atol=1e-6)), 0)
            obj["solution_index"] = idx
            obj["joint_pose"]     = q_sel.tolist()

            if get_armature_type(p.robot_type) == "KUKA":
                p.fixed_q3      = q_sel[2]
                obj["fixed_q3"] = p.fixed_q3
                print(f"→ Updated fixed_q3 from IK: {math.degrees(p.fixed_q3):.2f}°")

            p.solutions         = [list(map(float, s)) for s in sols]
            p.max_solutions     = len(sols)
            p.current_index     = idx
            p.solution_index_ui = idx + 1
            p.status_text       = f"{obj.name} updated via IK"

        else:
            print("→ Mode: Updating Gizmo from TCP (no IK)")
            tgt.matrix_world = obj.matrix_world.copy()

            if "joint_pose" in obj:
                q_stored = obj["joint_pose"]
                apply_solution(arm, q_stored, ctx.scene.frame_current, insert_keyframe=False)
                if get_armature_type(p.robot_type) == "KUKA":
                    p.fixed_q3 = q_stored[2]
                    print(f"→ Restored fixed_q3 from TCP: {math.degrees(p.fixed_q3):.2f}°")
                p.solutions         = [q_stored]
                p.max_solutions     = 1
                p.current_index     = int(obj.get("solution_index", 0))
                p.solution_index_ui = p.current_index + 1
                p.status_text       = f"Preview {obj.name}"
            else:
                tgt.matrix_world = obj.matrix_world.copy()
                q = tgt.matrix_world.to_quaternion()
                T_goal = np.eye(4)
                T_goal[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                T_goal[:3, 3] = np.array(tgt.matrix_world.to_translation())

                q_sel, sols = get_best_ik_solution(p, T_goal)
                if not sols:
                    self.report({'ERROR'}, "IK failed")
                    return {'CANCELLED'}
                apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)
                obj["solution_index"] = 0
                obj["joint_pose"]     = q_sel.tolist()
                p.solutions           = [list(map(float, s)) for s in sols]
                p.max_solutions       = len(sols)
                p.current_index       = 0
                p.solution_index_ui   = 1
                p.status_text         = f"{obj.name} previewed"

        obj["last_matrix_world"] = list(obj.matrix_world)

        print("✔ [UPDATE TCP] Done")
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
def get_robot_mapping(ctx=None) -> dict:
    if ctx is None:
        ctx = bpy.context
    return ctx.scene.get("robot_mapping", {})

class OBJECT_OT_update_all_tcp_poses(bpy.types.Operator):
    bl_idname = "object.update_all_tcp_poses"
    bl_label  = "Update All TCPs"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        current_armature_name = p.armature
        robot_mapping = get_robot_mapping(ctx)

        matched_robot_key = None
        for rob_key, arm_name in robot_mapping.items():
            if arm_name == current_armature_name:
                matched_robot_key = rob_key
                break

        if not matched_robot_key:
            self.report({'ERROR'}, f"Current armature '{current_armature_name}' not mapped in robot_mapping")
            return {'CANCELLED'}

        print(f"[RECOMPUTE] Active armature: {current_armature_name} → matched robot_key: {matched_robot_key}")

        for idx, tp in enumerate(p.tcp_sorted_list):
            obj = bpy.data.objects.get(tp.name)
            if not obj:
                continue

            if not obj.get("bake_enabled", True):
                continue

            if obj.get("robot_key", "") != matched_robot_key:
                continue

            print(f"[RECOMPUTE] {idx+1:02}: {obj.name}")
            p.selected_teach_point = obj
            p.tcp_list_index = idx

            bpy.ops.object.update_tcp_pose(name=obj.name)

        self.report({'INFO'}, f"Updated TCPs for robot_key: {matched_robot_key}")
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
def preview_obj_pose(ctx, obj, forward=True):

    p = ctx.scene.ik_motion_props
    p.selected_teach_point = obj

    if p.goal_object:
        p.goal_object.matrix_world = obj.matrix_world.copy()
        p.goal_object.scale = (1, 1, 1)

    arm = bpy.data.objects.get(p.armature)
    if not arm:
        print("[PREVIEW] Armature not found")
        return

    if "joint_pose" in obj:
        q = obj["joint_pose"]
        apply_solution(arm, q, ctx.scene.frame_current, insert_keyframe=False)
        print(f"[PREVIEW] Previewing TCP: {obj.name}")
        pos = obj.matrix_world.to_translation()
        rot = obj.matrix_world.to_euler('XYZ')
        print(f"[PREVIEW] Position: {np.round(pos[:], 4)}")
        print(f"[PREVIEW] Rotation (deg): {np.round(np.degrees(rot[:]), 1)}")
        print(f"[PREVIEW] Using saved joint_pose: {[round(math.degrees(a), 2) for a in q]}")
        return

    # Fallback: solve IK
    T_goal = np.array(obj.matrix_world)
    q_sel, sols = get_best_ik_solution(p, T_goal)
    if not sols:
        print("[PREVIEW] No IK solution found")
        return

    apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)
    pos = obj.matrix_world.to_translation()
    rot = obj.matrix_world.to_euler('XYZ')

    print(f"[PREVIEW] Previewing TCP: {obj.name}")
    print(f"[PREVIEW] Position: {np.round(pos[:], 4)}")
    print(f"[PREVIEW] Rotation (deg): {np.round(np.degrees(rot[:]), 1)}")
    print(f"[IK] Solving IK for T_goal:")
    print(np.round(T_goal, 4))
    print(f"[IK] Valid IK solutions: {len(sols)}")
    for i, q in enumerate(sols):
        print(f"  sol[{i}] = {[round(math.degrees(a), 2) for a in q]}")
    print(f"[IK] Selected solution: {[round(math.degrees(a), 2) for a in q_sel]}")

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_toggle_motion_type(bpy.types.Operator):
    bl_idname = "object.toggle_motion_type"
    bl_label = "Toggle Motion Type"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        obj = p.selected_teach_point

        if not obj:
            self.report({'ERROR'}, "No waypoint selected")
            return {'CANCELLED'}

        if "motion_type" in obj:
            obj["motion_type"] = "LINEAR" if obj["motion_type"] == "JOINT" else "JOINT"
            self.report({'INFO'}, f"{obj.name} motion type set to {obj['motion_type']}")
        else:
            self.report({'WARNING'}, "Waypoint missing 'motion_type' key")

        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_clear_bake_keys(bpy.types.Operator):

    """Delete all animation keys created by Bake"""
    bl_idname = "object.clear_bake_keys"
    bl_label  = "Delete Bake Keys"
    bl_options = {'UNDO'}

    def execute(self, ctx):
        p    = ctx.scene.ik_motion_props
        arm  = bpy.data.objects.get(p.armature)
        goal = p.goal_object

        if arm and arm.animation_data and arm.animation_data.action:
            for fc in list(arm.animation_data.action.fcurves):
                arm.animation_data.action.fcurves.remove(fc)

        if goal and goal.animation_data and goal.animation_data.action:
            for fc in list(goal.animation_data.action.fcurves):
                goal.animation_data.action.fcurves.remove(fc)

        self.report({'INFO'}, "Bake keys deleted")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_snap_gizmo_on_path(bpy.types.Operator):
    bl_idname = "object.snap_gizmo_on_path"
    bl_label = "Snap Gizmo on Path"

    def execute(self, ctx):

        p = ctx.scene.ik_motion_props
        giz = p.goal_object
        tp  = p.selected_teach_point
        coll = bpy.data.collections.get("Teach data")

        if not tp or not giz or not coll:
            self.report({'ERROR'}, "Gizmo / TCP / Teach data missing")
            return {'CANCELLED'}

        tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))
        idx = tps.index(tp) if tp in tps else -1
        if idx == -1 or idx >= len(tps) - 1:
            self.report({'WARNING'}, "No next TCP after selection")
            return {'CANCELLED'}

        tp_a = tps[idx]
        tp_b = tps[idx + 1]
        t = max(0.0, min(1.0, p.path_percent))
        A = np.array(tp_a.matrix_world.translation)
        B = np.array(tp_b.matrix_world.translation)
        Q = tp_a.matrix_world.to_quaternion().slerp(tp_b.matrix_world.to_quaternion(), t)

        giz.location = (1 - t) * A + t * B
        giz.rotation_euler = Q.to_euler()

        T_goal = np.eye(4)
        T_goal[:3, :3] = Q.to_matrix()
        T_goal[:3, 3] = (1 - t) * A + t * B

        q_sel, sols = get_best_ik_solution(p, T_goal)
        if sols:
            arm = bpy.data.objects.get(p.armature)
            if arm:
                apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)

                found_index = None
                for i, s in enumerate(sols):
                    if np.allclose(s, q_sel, atol=1e-6):
                        found_index = i
                        break

                if found_index is not None:
                    p.current_index = found_index
                    p.solution_index_ui = found_index + 1

        p.status_text = f"Gizmo moved to {round(t*100)}% between {tp_a.name} and {tp_b.name}"
        return {'FINISHED'}
        
# ──────────────────────────────────────────────────────────────       
class OBJECT_OT_apply_global_speed(bpy.types.Operator):
    bl_idname = "object.apply_global_speed"
    bl_label = "Apply Speed to All"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        obj = p.selected_teach_point
        if not obj or "speed" not in obj:
            self.report({'ERROR'}, "Selected TCP does not have 'speed'")
            return {'CANCELLED'}

        value = obj["speed"]
        count = 0
        for tp in bpy.data.collections.get("Teach data", {}).objects:
            tp["speed"] = value
            count += 1

        self.report({'INFO'}, f"Speed {value} mm/s applied to {count} TCPs")
        return {'FINISHED'}
        
# ──────────────────────────────────────────────────────────────      
class OBJECT_OT_apply_global_wait(bpy.types.Operator):
    bl_idname = "object.apply_global_wait"
    bl_label = "Apply Wait Time to All"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        obj = p.selected_teach_point
        if not obj or "wait_time_sec" not in obj:
            self.report({'ERROR'}, "Selected TCP does not have 'wait_time_sec'")
            return {'CANCELLED'}

        value = obj["wait_time_sec"]
        count = 0
        for tp in bpy.data.collections.get("Teach data", {}).objects:
            tp["wait_time_sec"] = value
            count += 1

        self.report({'INFO'}, f"Wait time {value}s applied to {count} TCPs")
        return {'FINISHED'}
        
# ──────────────────────────────────────────────────────────────   
class OBJECT_OT_tcp_list_select(bpy.types.Operator):
    bl_idname = "object.tcp_list_select"
    bl_label  = "Snap + Preview"

    index: bpy.props.IntProperty()

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        if not (0 <= self.index < len(p.tcp_sorted_list)):
            return {'CANCELLED'}
        name = p.tcp_sorted_list[self.index].name
        obj  = bpy.data.objects.get(name)
        if not obj:
            return {'CANCELLED'}

        p.selected_teach_point = obj
        p.tcp_list_index       = self.index

        if get_armature_type(p.robot_type) == "KUKA":
            source_fixed_q3 = obj.get("fixed_q3", 0.0)  # stored in radians
            p.fixed_q3_deg  = source_fixed_q3          # assign radian to ANGLE-type property :contentReference[oaicite:2]{index=2}

            ui_deg = math.degrees(p.fixed_q3_deg)
            print(f"[DEBUG] UI R angle (deg): {ui_deg:.2f}°")
            print(f"[DEBUG] → obj['fixed_q3']: {source_fixed_q3:.6f} rad  →  {math.degrees(source_fixed_q3):.2f}°")

        preview_obj_pose(ctx, obj, forward=False)
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_record_tcp_from_jog(bpy.types.Operator):
    bl_idname = "object.record_tcp_from_jog"
    bl_label = "Record TCP from Jog"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        jog = ctx.scene.jog_props
        bones = get_BONES()
        axes = get_AXES()
        q = []

        for i, bn in enumerate(bones):
            val = getattr(jog, f"joint_{i}", 0.0)
            q.append(val)

        p.solutions = [q]
        p.current_index = 0

        fk_func = get_forward_kinematics()
        T_fk = fk_func(q)
        T_world = (
            compute_base_matrix(p)
            @ T_fk
            @ compute_tcp_offset_matrix(p)
        )

        giz = p.goal_object
        if giz:
            giz.matrix_world = Matrix(T_world)

        bpy.ops.object.record_tcp_point('INVOKE_DEFAULT')
        p.status_text = "TCP recorded from Jog (FK pose)"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_preview_goal_pose(bpy.types.Operator):
    bl_idname = "object.preview_goal_pose"
    bl_label  = "Preview Goal Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        bpy.ops.object.teach_pose('EXEC_DEFAULT')
        p.selected_teach_point = None            
        p.tcp_list_index = -1                    
        return {'FINISHED'}
    
class OBJECT_OT_record_goal_pose(bpy.types.Operator):
    bl_idname = "object.record_goal_pose"
    bl_label  = "Record Goal Pose"

    def execute(self, ctx):
        result = bpy.ops.object.teach_pose('EXEC_DEFAULT')  # apply IK
        if result == {'FINISHED'}:
            return bpy.ops.object.record_tcp_point('EXEC_DEFAULT')  # create TCP
        return result
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_preview_tcp_prev_pose(bpy.types.Operator):
    bl_idname = "object.preview_tcp_prev_pose"
    bl_label = "Prev TCP + Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        lst = p.tcp_sorted_list
        if not lst:
            return {'CANCELLED'}

        if p.tcp_list_index > 0:
            p.tcp_list_index -= 1
            tcp_item = lst[p.tcp_list_index]
            obj = bpy.data.objects.get(tcp_item.name)
            if obj:
                p.selected_teach_point = obj
                apply_selected_tcp_pose(ctx)

                if p.goal_object:
                    p.goal_object.matrix_world = obj.matrix_world.copy()
        return {'FINISHED'}

class OBJECT_OT_preview_tcp_next_pose(bpy.types.Operator):
    bl_idname = "object.preview_tcp_next_pose"
    bl_label = "Next TCP + Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        lst = p.tcp_sorted_list
        if not lst:
            return {'CANCELLED'}

        if p.tcp_list_index < len(lst) - 1:
            p.tcp_list_index += 1
            tcp_item = lst[p.tcp_list_index]
            obj = bpy.data.objects.get(tcp_item.name)
            if obj:
                p.selected_teach_point = obj
                apply_selected_tcp_pose(ctx)

                if p.goal_object:
                    p.goal_object.matrix_world = obj.matrix_world.copy()
        return {'FINISHED'}

    
def apply_selected_tcp_pose(ctx):
    p   = ctx.scene.ik_motion_props
    tcp = p.selected_teach_point
    if not tcp or "joint_pose" not in tcp:
        return
    arm = bpy.data.objects.get(p.armature)
    if arm:
        apply_solution(arm, list(tcp["joint_pose"]),
                       ctx.scene.frame_current,
                       insert_keyframe=False)

# ──────────────────────────────────────────────────────────────    
classes = (
    OBJECT_OT_teach_pose,
    OBJECT_OT_execute_linear_motion,
    OBJECT_OT_record_tcp_point,
    OBJECT_OT_apply_preview_pose,
    OBJECT_OT_bake_teach_sequence,
    OBJECT_OT_update_tcp_pose,
    OBJECT_OT_update_all_tcp_poses,
    OBJECT_OT_toggle_motion_type,
    OBJECT_OT_clear_bake_keys,
    OBJECT_OT_snap_gizmo_on_path,
    OBJECT_OT_apply_global_wait,
    OBJECT_OT_apply_global_speed,
    OBJECT_OT_tcp_list_select,
    OBJECT_OT_record_tcp_from_jog,
    OBJECT_OT_preview_goal_pose,
    OBJECT_OT_record_goal_pose,
    OBJECT_OT_preview_tcp_prev_pose,
    OBJECT_OT_preview_tcp_next_pose,
)
