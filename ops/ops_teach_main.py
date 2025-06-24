import bpy
import math

import numpy as np, math

from mathutils import Matrix, Euler
from scipy.spatial.transform import Rotation as R, Slerp
from rteach.core.robot_state import get_armature_type
from rteach.core.core import (
    apply_solution, get_inverse_kinematics, compute_base_matrix, 
    compute_tcp_offset_matrix, get_forward_kinematics, 
    get_BONES, get_AXES, get_best_ik_solution
)
from rteach.core.core_iiwa import linear_move
from rteach.ops.ops_teach_util import update_tcp_sorted_list, find_object_by_prefix
from rteach.config import settings_static as ss
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

        q = tgt.matrix_world.to_quaternion()
        T_goal = np.eye(4)
        T_goal[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_goal[:3, 3] = np.array(tgt.matrix_world.to_translation())

        ik_solver = get_inverse_kinematics(p)
        T_base = compute_base_matrix(p)
        T_offset = compute_tcp_offset_matrix(p)
        T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)
        sols = ik_solver(T_flange)

        print(f"[DEBUG] Direct IK returned {len(sols)} solutions:")
        for i, s in enumerate(sols):
            print(f"  sol[{i}] = {[round(a, 3) for a in s]}")

        if not sols:
            p.status_text = "IK failed: no solutions found"
            return {'FINISHED'}

        q_prev = p.solutions[p.current_index] if p.solutions and len(p.solutions) > p.current_index else None
        if q_prev:
            def ang_diff(a, b):
                return ((a - b + math.pi) % (2 * math.pi)) - math.pi
            idx = min(range(len(sols)),key=lambda i: sum(abs(ang_diff(sols[i][j], q_prev[j])) for j in range(min(len(sols[i]), len(q_prev)))))
        else:
            idx = 0

        p.solutions = [list(map(float, s)) for s in sols]
        p.max_solutions = len(sols)
        p.current_index = idx
        p.solution_index_ui = idx + 1
        q_sel = p.solutions[idx]

        frame = ctx.scene.frame_current
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            p.status_text = "IK failed: Armature not found"
            return {'FINISHED'}

        apply_solution(arm, q_sel, frame, insert_keyframe=False)

        if p.auto_record:
            bpy.ops.object.record_tcp_point()

        p.status_text = f"Applied {idx+1}/{len(sols)}"

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
    bl_label  = "Record Goal"

    def execute(self, ctx):
        p   = ctx.scene.ik_motion_props
        tgt = p.goal_object
        if not tgt:
            self.report({'ERROR'}, "Goal not set")
            return {'CANCELLED'}

        coll = bpy.data.collections.get("Teach data") or bpy.data.collections.new("Teach data")
        if coll.name not in {c.name for c in ctx.scene.collection.children}:
            ctx.scene.collection.children.link(coll)

        new_idx = len([o for o in bpy.data.objects if o.name.startswith("P.")])
        dup = bpy.data.objects.new(name=f"P.{new_idx:03d}", object_data=tgt.data)
        if "_RNA_UI" not in dup:
            dup["_RNA_UI"] = {}
        dup["_RNA_UI"]["speed"] = {
            "min": 0.0, "max": 500.0,
            "soft_min": 0.0, "soft_max": 500.0
        }
        dup["_RNA_UI"]["wait_time_sec"] = {
            "min": 0.0, "max": 10.0,
            "soft_min": 0.0, "soft_max": 10.0
        }

        dup.matrix_world = tgt.matrix_world
        dup.scale        = tgt.scale * 0.5
        dup["index"]     = new_idx
        dup["motion_type"] = "LINEAR"
        dup["fixed_q3"]  = p.fixed_q3
        dup["speed"]     = p.motion_speed
        dup["wait_time_sec"] = p.wait_time_sec

        if "_RNA_UI" not in dup:
            dup["_RNA_UI"] = {}
        dup["_RNA_UI"]["speed"] = {"min": 0.0, "max": 500.0, "soft_min": 0.0, "soft_max": 500.0}
        dup["_RNA_UI"]["wait_time_sec"] = {"min": 0.0, "max": 10.0, "soft_min": 0.0, "soft_max": 10.0}

        dup.show_name = True
        p.selected_teach_point = dup

        dof = len(get_BONES())
        if p.solutions and len(p.solutions) > p.current_index:
            q_safe = sanitize_q(p.solutions[p.current_index], dof)
            dup["solution_index"] = p.current_index
            dup["joint_pose"] = q_safe

        coll.objects.link(dup)
        update_tcp_sorted_list()
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────
class OBJECT_OT_cycle_pose_preview(bpy.types.Operator):
    bl_idname = "object.cycle_pose_preview"
    bl_label  = "Cycle IK"

    direction: bpy.props.EnumProperty(
        items=[('NEXT','Next',''),('PREV','Prev','')]
    )

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        tgt = p.goal_object
        arm = bpy.data.objects.get(p.armature)

        if not tgt:
            self.report({'ERROR'}, "Target not set")
            return {'CANCELLED'}

        bpy.context.view_layer.update()
        bpy.context.evaluated_depsgraph_get().update()

        q = tgt.matrix_world.to_quaternion()
        T_goal = np.eye(4)
        T_goal[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_goal[:3, 3]  = np.array(tgt.matrix_world.to_translation())

        _, sols = get_best_ik_solution(p, T_goal)
        if not sols:
            self.report({'WARNING'}, "No IK solutions")
            return {'CANCELLED'}

        p.solutions = [list(map(float, s)) for s in sols]
        count = len(p.solutions)

        idx = p.current_index
        if self.direction == 'NEXT':
            idx = (idx + 1) % count
        else:
            idx = (idx - 1 + count) % count

        p.current_index = idx
        p.solution_index_ui = idx + 1
        q_sel = p.solutions[idx]
        p.ik_solution_str = str([round(math.degrees(x), 1) for x in q_sel])

        if arm:
            frame = ctx.scene.frame_current
            apply_solution(arm, q_sel, frame, insert_keyframe=False)

        p.status_text = f"Pose {idx+1}/{count} previewed"
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
    bl_label = "Bake Sequence"

    def execute(self, ctx):

        p   = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        giz = p.goal_object
        coll = bpy.data.collections.get("Teach data")
        if not (arm and giz and coll):
            self.report({'ERROR'}, "Armature / Goal / Teach data not set")
            return {'CANCELLED'}

        tps_all = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        if p.bake_all_tcp:
            start_idx = 0
            end_idx   = len(tps_all) - 1
        else:
            start_idx = p.bake_start_idx
            end_idx   = p.bake_end_idx

        start_idx, end_idx = sorted((start_idx, end_idx))
        tps = [tp for tp in tps_all if start_idx <= tp.get("index", 9999) <= end_idx]

        if not tps:
            self.report({'WARNING'}, f"No TCPs between index {start_idx} and {end_idx}")
            return {'CANCELLED'}

        fps            = ctx.scene.render.fps
        default_speed  = p.motion_speed
        default_wait_s = p.wait_time_sec
        f0          = p.bake_start_frame
        giz_scale   = giz.scale[:]
        f           = f0

        for i, tp in enumerate(tps):
            q_tp = tp.get("joint_pose")
            if q_tp is None:
                self.report({'WARNING'}, f"{tp.name} missing joint_pose")
                return {'CANCELLED'}

            giz.matrix_world = tp.matrix_world.copy()
            giz.scale        = giz_scale
            ctx.scene.frame_set(f)
            apply_solution(arm, q_tp, f, insert_keyframe=True)

            wait_s     = tp.get("wait_time_sec", default_wait_s)
            wait_frames = round(wait_s * fps)
            if wait_frames > 0:
                f_wait_end = f + wait_frames - 1
                ctx.scene.frame_set(f_wait_end)
                apply_solution(arm, q_tp, f_wait_end, insert_keyframe=True)
                f = f_wait_end

            if i == len(tps) - 1:
                break

            next_tp   = tps[i + 1]
            motion    = tp.get("motion_type", "LINEAR").upper()
            A        = np.array(tp.matrix_world.translation)
            B        = np.array(next_tp.matrix_world.translation)
            dist_mm  = np.linalg.norm(B - A) * 1000.0
            speed_mm = tp.get("speed", default_speed)
            motion_s = dist_mm / max(speed_mm, 1e-3)
            span     = max(1, round(motion_s * fps))

            if motion == "LINEAR":

                if get_armature_type(p.robot_type) == "KUKA" and p.precise_linear:
                    q_start_raw = tp.get("joint_pose")
                    q_start = np.asarray(q_start_raw, float) if q_start_raw else None

                    if q_start is not None:
                        fk_func = get_forward_kinematics()
                        T_base  = compute_base_matrix(p)
                        T_off   = compute_tcp_offset_matrix(p)

                        T_start = T_base @ fk_func(q_start) @ T_off
                        T_goal  = T_start.copy()
                        T_goal[:3, 3] = np.array(next_tp.matrix_world)[:3, 3]
                        T_goal_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_off)

                        start_pos = T_start[:3, 3] * 1000
                        goal_pos  = T_goal[:3, 3] * 1000
                        dist_mm   = np.linalg.norm(goal_pos - start_pos)
                        fps = ctx.scene.render.fps
                        speed = tp.get("speed", p.motion_speed)

                        motion_time_sec = dist_mm / max(speed, 1e-3)
                        n_steps = round(motion_time_sec * fps)
                        auto_step_mm = dist_mm / max(n_steps, 1)

                        p.step_mm = auto_step_mm
                        path = linear_move(q_start, T_goal_flange, p)
                        if path:
                            for i, q in enumerate(path):
                                ctx.scene.frame_set(f + i + 1)
                                apply_solution(arm, q, f + i + 1, insert_keyframe=True)

                            next_tp["joint_pose"] = list(map(float, path[-1]))
                            f += len(path)
                            continue

                    if get_armature_type(p.robot_type) == "KUKA":
                        p.fixed_q3 = float(tp.get("fixed_q3", p.fixed_q3))

                p.goal_object   = giz
                p.linear_target = next_tp
                p.linear_frames = span

                ctx.scene.frame_set(f + 1)
                bpy.ops.object.execute_linear_motion('EXEC_DEFAULT')
                f += span

            elif motion == "JOINT":
                q_next = next_tp.get("joint_pose")
                if q_next is None:
                    self.report({'WARNING'}, f"{next_tp.name} missing joint_pose")
                    return {'CANCELLED'}

                end_frame = f + span
                ctx.scene.frame_set(end_frame)
                apply_solution(arm, q_next, end_frame, insert_keyframe=True)

                giz.matrix_world = next_tp.matrix_world.copy()
                giz.scale        = giz_scale

                f = end_frame

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
        p   = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        obj = bpy.data.objects.get(self.name or (p.selected_teach_point.name if p.selected_teach_point else None))
        tgt = p.goal_object
        if not (arm and obj and tgt):
            self.report({'ERROR'}, "Goal / Waypoint / Armature not set")
            return {'CANCELLED'}

        moved = not np.allclose(np.array(tgt.matrix_world), np.array(obj.matrix_world), atol=1e-6)

        if moved:
            obj.matrix_world = tgt.matrix_world.copy()
            obj.scale *= 0.5

            T_goal = np.array(tgt.matrix_world)
            q_sel, sols = get_best_ik_solution(p, T_goal)
            if not sols:
                self.report({'ERROR'}, "IK failed")
                return {'CANCELLED'}

            q_ref = np.asarray(obj["joint_pose"], float) if "joint_pose" in obj else None
            if q_ref is not None:
                q_sel, sols = get_best_ik_solution(p, T_goal, q_ref=q_ref)

            apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)
            found_index = None
            for i, sol in enumerate(sols):
                if np.allclose(sol, q_sel, atol=1e-6):
                    found_index = i
                    break
            if found_index is None:
                found_index = 0

            obj["solution_index"] = found_index
            obj["joint_pose"]     = q_sel.tolist()

            if get_armature_type(p.robot_type) == "KUKA":
                p.fixed_q3 = q_sel[2]
                obj["fixed_q3"] = p.fixed_q3

            p.current_index     = found_index
            p.solution_index_ui = found_index + 1
            p.status_text = f"{obj.name} updated from Gizmo"

        else:
            tgt.matrix_world = obj.matrix_world.copy()
            tgt.scale        = (1, 1, 1)
            if "joint_pose" in obj:
                q_stored = obj["joint_pose"]
                apply_solution(arm, q_stored, ctx.scene.frame_current, insert_keyframe=False)
                if get_armature_type(p.robot_type) == "KUKA":
                    p.fixed_q3 = q_stored[2]
                p.current_index     = int(obj.get("solution_index", 0))
                p.solution_index_ui = p.current_index + 1
                p.status_text = f"Preview {obj.name}"
            else:
                T_goal = np.array(tgt.matrix_world)
                q_sel, sols = get_best_ik_solution(p, T_goal)
                if not sols:
                    self.report({'ERROR'}, "IK failed")
                    return {'CANCELLED'}
                apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)
                obj["solution_index"] = 0
                obj["joint_pose"]     = q_sel.tolist()
                p.current_index = 0
                p.solution_index_ui = 1
                p.status_text = f"{obj.name} previewed"

        bpy.ops.object.draw_teach_path()
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
def preview_obj_pose(ctx, obj, forward: bool):
    p = ctx.scene.ik_motion_props
    arm = bpy.data.objects.get(p.armature)

    for o in ctx.selected_objects:
        o.select_set(False)
    obj.select_set(True)
    ctx.view_layer.objects.active = obj

    if "joint_pose" in obj and arm:
        apply_solution(arm, obj["joint_pose"], ctx.scene.frame_current, insert_keyframe=False)
        if p.goal_object:
            p.goal_object.matrix_world = obj.matrix_world
            p.goal_object.scale = (1, 1, 1)
        if obj != p.selected_teach_point:
            p.selected_teach_point = obj

        if len(obj["joint_pose"]) >= 3:
            p.fixed_q3 = obj.get("fixed_q3", obj["joint_pose"][2])
            idx = int(obj.get("solution_index", 0))
            p.solution_index_ui = idx + 1
            p.current_index = idx
            p.fixed_q3_deg = p.fixed_q3

        p.status_text = f"Preview {obj.name} (stored)"
        return {'FINISHED'}

    T_goal = np.array(obj.matrix_world)
    q_sel, sols = get_best_ik_solution(p, T_goal)
    if not sols:
        p.status_text = "IK failed"
        return {'CANCELLED'}

    idx_saved = int(obj.get("solution_index", 0))
    idx = max(0, min(idx_saved, len(sols) - 1))

    if arm:
        apply_solution(arm, sols[idx], ctx.scene.frame_current, insert_keyframe=False)

    if p.goal_object:
        p.goal_object.matrix_world = obj.matrix_world
        p.goal_object.scale = (1, 1, 1)

    if ctx.area:
        ctx.area.tag_redraw()

    p.status_text = f"Preview {obj.name} (IK pose {idx+1})"
    return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_preview_tcp_next(bpy.types.Operator):
    bl_idname = "object.preview_tcp_next"
    bl_label = "Next TCP Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll or not coll.objects:
            self.report({'INFO'}, "No Waypoints")
            return {'CANCELLED'}

        lst = sorted(coll.objects, key=lambda o: o.get("index", 9999))
        p.preview_tcp_index = (p.preview_tcp_index + 1) % len(lst)
        obj = lst[p.preview_tcp_index]

        for o in ctx.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        ctx.view_layer.objects.active = obj

        return preview_obj_pose(ctx, obj, forward=True)
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_preview_tcp_prev(bpy.types.Operator):
    bl_idname = "object.preview_tcp_prev"
    bl_label = "Prev TCP Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll or not coll.objects:
            self.report({'INFO'}, "No Waypoints")
            return {'CANCELLED'}

        lst = sorted(coll.objects, key=lambda o: o.get("index", 9999))
        p.preview_tcp_index = (p.preview_tcp_index - 1 + len(lst)) % len(lst)
        obj = lst[p.preview_tcp_index]

        for o in ctx.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        ctx.view_layer.objects.active = obj

        return preview_obj_pose(ctx, obj, forward=False)

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
        giz.scale = (1, 1, 1)

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
    bl_label = "Snap + Preview"

    index: bpy.props.IntProperty()

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props

        if not (0 <= self.index < len(p.tcp_sorted_list)):
            return {'CANCELLED'}

        name = p.tcp_sorted_list[self.index].name
        obj = bpy.data.objects.get(name)
        if not obj:
            return {'CANCELLED'}

        p.selected_teach_point = obj
        p.tcp_list_index = self.index

        from rteach.ops.ops_teach_main import preview_obj_pose
        return preview_obj_pose(ctx, obj, forward=False)
    
# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_keyframe_joint_pose(bpy.types.Operator):
    bl_idname = "object.keyframe_joint_pose"
    bl_label = "Keyframe Pose"
    bl_description = "Insert keyframes for the current Jog slider pose at current frame"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        jog = ctx.scene.jog_props
        arm = bpy.data.objects.get(p.armature)

        if not arm:
            self.report({'ERROR'}, "Armature not found")
            return {'CANCELLED'}

        bones = get_BONES()
        axes  = get_AXES()

        frame = ctx.scene.frame_current
        for i, bn in enumerate(bones):
            pb = arm.pose.bones.get(bn)
            if not pb:
                continue

            axis = axes[i]
            angle = getattr(jog, f"joint_{i}", 0.0)
            pb.rotation_mode = 'XYZ'
            rot = [0, 0, 0]
            idx = {'x': 0, 'y': 1, 'z': 2}[axis]
            rot[idx] = angle
            pb.rotation_euler = rot
            pb.keyframe_insert(data_path="rotation_euler", frame=frame, index=idx)

        self.report({'INFO'}, f"Keyframes inserted at frame {frame}")
        p.status_text = f"Jog pose keyframed at frame {frame}"
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
            giz.scale = (1, 1, 1)

        bpy.ops.object.record_tcp_point('INVOKE_DEFAULT')
        p.status_text = "TCP recorded from Jog (FK pose)"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────   
class OBJECT_OT_recompute_selected_tcp(bpy.types.Operator):
    bl_idname = "object.recompute_selected_tcp"
    bl_label = "Update IK from Location"
    bl_description = "Recompute joint_pose/fixed_q3 for selected TCPs based on current transform"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            self.report({'ERROR'}, "Armature not found")
            return {'CANCELLED'}

        count = 0
        for obj in ctx.selected_objects:
            if not obj.name.startswith("P."):
                continue

            T_goal = np.array(obj.matrix_world)
            q_sel, sols = get_best_ik_solution(p, T_goal)
            if not sols:
                self.report({'WARNING'}, f"IK failed: {obj.name}")
                continue

            obj["joint_pose"] = q_sel.tolist()
            obj["solution_index"] = sols.index(q_sel) if q_sel in sols else 0

            if get_armature_type(p.robot_type) == "KUKA":
                obj["fixed_q3"] = q_sel[2]

            count += 1

        self.report({'INFO'}, f"Updated {count} TCPs from current position")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_keyframe_stage_joint(bpy.types.Operator):
    bl_idname = "object.keyframe_stage_joint"
    bl_label = "Insert Stage Keyframe"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        props = ctx.scene.stage_props
        value = getattr(props, self.name, None)
        obj = bpy.data.objects.get(self.name)

        if obj is None:
            self.report({'ERROR'}, f"Object '{self.name}' not found")
            return {'CANCELLED'}

        robot = ctx.scene.ik_motion_props.robot_type
        config = ROBOT_CONFIGS.get(robot, {})
        stage_joints = config.get("stage_joints", [])

        axis = None
        joint_type = None

        for sj in stage_joints:
            if sj[0] == self.name:
                axis = sj[5]  
                joint_type = sj[6] 
                break

        if axis is None or joint_type is None:
            self.report({'ERROR'}, f"Joint definition not found for '{self.name}'")
            return {'CANCELLED'}

        idx = {"x": 0, "y": 1, "z": 2}.get(axis.lower(), 0)
        frame = ctx.scene.frame_current

        if joint_type == "location":
            obj.location[idx] = value
            obj.keyframe_insert(data_path="location", frame=frame, index=idx)
        elif joint_type == "rotation":
            obj.rotation_mode = 'XYZ'
            obj.rotation_euler[idx] = value
            obj.keyframe_insert(data_path="rotation_euler", frame=frame, index=idx)

        self.report({'INFO'}, f"Keyframed {self.name} at frame {frame} ({joint_type}.{axis})")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_focus_stage_joint(bpy.types.Operator):
    bl_idname = "object.focus_stage_joint"
    bl_label = "Select Joint in Viewport"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        obj = bpy.data.objects.get(self.name)
        if not obj:
            self.report({'ERROR'}, f"Object '{self.name}' not found")
            return {'CANCELLED'}

        for o in ctx.selected_objects:
            o.select_set(False)

        obj.select_set(True)
        ctx.view_layer.objects.active = obj

        self.report({'INFO'}, f"Selected {obj.name}")
        return {'FINISHED'}
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_snap_target_to_fk(bpy.types.Operator):
    bl_idname = "object.snap_target_to_fk"
    bl_label = "Snap Target to FK"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        goal = p.goal_object
        if not arm or not goal:
            self.report({'ERROR'}, "Armature or Goal object not set")
            return {'CANCELLED'}

        bones = get_BONES()
        axes = get_AXES()
        q = []

        for i, bn in enumerate(bones):
            pb = arm.pose.bones.get(bn)
            if not pb:
                q.append(0.0)
                continue
            axis = axes[i]
            angle = getattr(pb.rotation_euler, axis)
            q.append(angle)

        fk_func = get_forward_kinematics()
        T = fk_func(q)

        T_full = compute_base_matrix(p) @ T @ compute_tcp_offset_matrix(p)
        goal.matrix_world = Matrix(T_full)
        goal.scale = (1, 1, 1)

        p.status_text = "Target snapped to FK"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────   
class OBJECT_OT_go_home_pose(bpy.types.Operator):
    bl_idname = "object.go_home_pose"
    bl_label = "Go Home Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            self.report({'ERROR'}, "Armature not found")
            return {'CANCELLED'}

        bones = get_BONES()
        axes  = get_AXES()

        for i, bn in enumerate(bones):
            pb = arm.pose.bones.get(bn)
            if not pb:
                continue
            axis = axes[i]
            pb.rotation_mode = 'XYZ'
            rot = [0.0, 0.0, 0.0]
            idx = {'x': 0, 'y': 1, 'z': 2}[axis]
            rot[idx] = 0.0
            pb.rotation_euler = rot
            pb.keyframe_insert(data_path="rotation_euler", frame=ctx.scene.frame_current, index=idx)

        p.status_text = "Home pose applied"
        return {'FINISHED'}
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_setup_tcp_from_gizmo(bpy.types.Operator):
    bl_idname = "object.setup_tcp_from_gizmo"
    bl_label  = "Create TCP from Gizmo"

    def execute(self, ctx):
        p, ee, tgt = ctx.scene.ik_motion_props, ctx.scene.ik_motion_props.ee_object, ctx.scene.ik_motion_props.goal_object
        if not ee or not tgt:
            self.report({'ERROR'}, "EE or Target Gizmo not set")
            return {'CANCELLED'}

        vis_name, real_name = "TCP_visible", "TCP"

        for nm in (vis_name, real_name):
            o = bpy.data.objects.get(nm)
            if o:
                bpy.data.objects.remove(o, do_unlink=True)

        setup = bpy.data.collections.get("Setup") or ctx.scene.collection

        ee_mat, tgt_mat = ee.matrix_world.copy(), tgt.matrix_world.copy()
        ee_loc, ee_rot3 = ee_mat.to_translation(), ee_mat.to_3x3()
        tgt_loc, tgt_rot3 = tgt_mat.to_translation(), tgt_mat.to_3x3()

        v_local = ee_mat.inverted() @ tgt_loc   

        if get_armature_type(p.robot_type) == "KUKA":
            R_corr3    = Euler((0, math.radians(90), 0), 'XYZ').to_matrix()
            v_corr     = R_corr3 @ v_local
            final_rot3 = tgt_rot3 @ R_corr3
        else:                                   
            v_corr     = v_local
            final_rot3 = ee_rot3                

        final_loc = ee_loc + ee_rot3 @ v_corr

        tcp_vis = bpy.data.objects.new(vis_name, None)
        tcp_vis.empty_display_type, tcp_vis.empty_display_size = 'SPHERE', 0.02
        setup.objects.link(tcp_vis)
        tcp_vis.matrix_world = tgt_mat
        tcp_vis.parent, tcp_vis.matrix_parent_inverse = ee, ee.matrix_world.inverted()

        tcp_real = bpy.data.objects.new(real_name, None)
        tcp_real.empty_display_type, tcp_real.empty_display_size = 'ARROWS', 0.015
        setup.objects.link(tcp_real)
        tcp_real.parent, tcp_real.matrix_parent_inverse = tcp_vis, tcp_vis.matrix_world.inverted()
        tcp_real.matrix_world = Matrix.Translation(final_loc) @ final_rot3.to_4x4()

        for o in (tcp_vis, tcp_real):
            o.hide_viewport = o.hide_render = o.hide_select = False

        p.tcp_object = tcp_real
        self.report({'INFO'}, f"TCP pair created (rotation offset disabled for UR)")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_sync_robot_type(bpy.types.Operator):
    bl_idname = "object.sync_robot_type"
    bl_label = "Sync Robot Type"

    def execute(self, ctx):

        p = ctx.scene.ik_motion_props
        robot = p.robot_type.lower()
        print(f"[DEBUG] Syncing JogProperties for robot: {robot}")

        ss.unregister_static_properties()
        ss.register_static_properties(robot)
        bpy.utils.register_class(ss.JogProperties)
        bpy.types.Scene.jog_props = bpy.props.PointerProperty(type=ss.JogProperties)

        config = ROBOT_CONFIGS.get(robot, {})
        dof = len(config.get("axes", []))
        n_stage = len(config.get("stage_joints", []))
        total = dof + n_stage
        print(f"[DEBUG] Robot DOF={dof}, stage joints={n_stage}, total plot joints={total}")

        default_plot = [True] * total
        if len(default_plot) < 14:
            default_plot += [False] * (14 - len(default_plot))
        p.show_plot_joints = default_plot[:14]

        setup = config.get("setup_objects", {})
        for key in ["goal", "base", "tcp", "ee"]:
            name = setup.get(key)
            if not isinstance(name, str) or not name.strip():
                print(f"[SYNC] Skipping {key}_object (name missing)")
                continue
            obj = find_object_by_prefix(name)
            if obj:
                setattr(p, f"{key}_object", obj)
                print(f"[SYNC] {key}_object → {obj.name}")
            else:
                print(f"[SYNC] {key}_object not found (name='{name}')")

        arm_sets = config.get("armature_sets", {})
        arm_name = getattr(p, "armature", "")
        if arm_name in arm_sets:
            print(f"[SYNC] Applying armature_set: {arm_name}")
            arm_cfg = arm_sets[arm_name]
            p.base_object = find_object_by_prefix(arm_cfg.get("base", ""))
            p.ee_object   = find_object_by_prefix(arm_cfg.get("ee", ""))
            p.tcp_object  = find_object_by_prefix(arm_cfg.get("tcp", ""))
        else:
            print(f"[SYNC] No matching armature_set found for: {arm_name}")

        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────    
classes = (
    OBJECT_OT_teach_pose,
    OBJECT_OT_execute_linear_motion,
    OBJECT_OT_record_tcp_point,
    OBJECT_OT_apply_preview_pose,
    OBJECT_OT_cycle_pose_preview,
    OBJECT_OT_bake_teach_sequence,
    OBJECT_OT_update_tcp_pose,
    OBJECT_OT_preview_tcp_next,
    OBJECT_OT_preview_tcp_prev,
    OBJECT_OT_toggle_motion_type,
    OBJECT_OT_clear_bake_keys,
    OBJECT_OT_snap_gizmo_on_path,
    OBJECT_OT_apply_global_wait,
    OBJECT_OT_apply_global_speed,
    OBJECT_OT_tcp_list_select,
    OBJECT_OT_keyframe_joint_pose,
    OBJECT_OT_record_tcp_from_jog,
    OBJECT_OT_recompute_selected_tcp,
    OBJECT_OT_keyframe_stage_joint,
    OBJECT_OT_focus_stage_joint,
    OBJECT_OT_snap_target_to_fk,
    OBJECT_OT_go_home_pose,
    OBJECT_OT_setup_tcp_from_gizmo,
    OBJECT_OT_sync_robot_type,
)
