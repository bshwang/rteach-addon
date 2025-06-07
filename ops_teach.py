import bpy
import math
import bmesh
import json

import numpy as np, math
from math import degrees
from mathutils import Matrix
from scipy.spatial.transform import Rotation as R, Slerp
from .settings import IKMotionProperties
from .robot_state import get_active_robot, set_active_robot
from .core import (
    solve_and_apply, apply_solution, get_inverse_kinematics, compute_base_matrix, 
    compute_tcp_offset_matrix, get_forward_kinematics, 
    sort_solutions, get_BONES, 
)
from .core_iiwa import linear_move
        
# ──────────────────────────────────────────────────────────────
class OBJECT_OT_compute_ik_ur(bpy.types.Operator):
    bl_idname = "object.compute_ik_ur"
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

        T_base   = compute_base_matrix(p)
        T_offset = compute_tcp_offset_matrix(p)

        T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)

        ik_solver = get_inverse_kinematics(p)
        sols = ik_solver(T_flange)

        if get_active_robot().startswith("UR"):
            sols = sort_solutions(sols)

        p.solutions = [list(map(float, s)) for s in sols]
        p.max_solutions = len(p.solutions)

        if not p.solutions or len(p.solutions) == 0:
            p.status_text = "IK failed: no solutions found"
            return {'FINISHED'}

        idx = p.solution_index_ui - 1 if 1 <= p.solution_index_ui <= len(p.solutions) else 0
        if p.use_last_pose:
            idx = p.current_index if 0 <= p.current_index < len(p.solutions) else idx

        if not (0 <= idx < len(p.solutions)):
            p.status_text = f"IK failed: invalid solution index {idx}"
            return {'FINISHED'}

        p.current_index = idx
        p.solution_index_ui = idx + 1
        q_sel = p.solutions[idx]

        frame = ctx.scene.frame_current
        success = solve_and_apply(ctx, p, T_goal, frame, insert_keyframe=False)

        if success:
            if p.auto_record:
                bpy.ops.object.record_goal_as_empty()
            p.status_text = f"Applied 1/{len(p.solutions)}"
        else:
            p.status_text = "IK failed"

        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_move_l(bpy.types.Operator):
    bl_idname = "object.move_l_ik"
    bl_label  = "Move_L"

    DEBUG_MOVE_L = True

    def execute(self, ctx):

        p = ctx.scene.ik_motion_props
        src, dst = p.goal_object, p.linear_target
        if not (src and dst):
            p.status_text = "Linear source/target not set"
            return {'FINISHED'}

        M_start, M_end = map(np.array, (src.matrix_world, dst.matrix_world))
        steps      = max(1, int(p.linear_frames))
        pos_series = np.linspace(M_start[:3, 3], M_end[:3, 3], steps)

        robot      = get_active_robot()
        arm        = bpy.data.objects.get(p.armature)
        if not arm:
            self.report({'ERROR'}, "Armature not found")
            return {'CANCELLED'}

        frame0   = ctx.scene.frame_current

        # ① KUKA + High-Precision LIN 
        if robot == "KUKA" and p.precise_linear and src.get("joint_pose"):
            q_init   = np.array(src["joint_pose"], float)
            T_goal   = np.linalg.inv(compute_base_matrix(p)) @ \
                       np.array(dst.matrix_world) @ \
                       np.linalg.inv(compute_tcp_offset_matrix(p))
                       
            path = linear_move(q_init, T_goal, p)
            if path is not None:
                for i, q in enumerate(path):
                    ctx.scene.frame_set(frame0 + i)
                    apply_solution(arm, q, frame0 + i, insert_keyframe=True)
                p.status_text = f"Precise LIN done ({len(path)} steps)"
                return {'FINISHED'}

            self.report({'WARNING'}, "Precise LIN failed – fallback to fast mode")

        # ② Fast LIN  
        q_start, q_end = src.matrix_world.to_quaternion(), dst.matrix_world.to_quaternion()
        R_key = R.from_quat([[q_start.x, q_start.y, q_start.z, q_start.w],
                             [q_end.x,   q_end.y,   q_end.z,   q_end.w]])
        slerp = Slerp([0, 1], R_key)

        if robot == "KUKA":
            q3_start = src.get("joint_pose", [0.0]*3)[2]
            q3_end   = dst.get("joint_pose", [0.0]*3)[2]

        ik_solver = get_inverse_kinematics(p)

        master_q = None
        success  = 0

        def stable_ang_diff(a, b):
            return ((a - b + math.pi) % (2 * math.pi)) - math.pi

        for i in range(steps):
            T_goal = np.eye(4)
            T_goal[:3, 3] = pos_series[i]
            t_val = i / (steps - 1) if steps > 1 else 0.0
            T_goal[:3, :3] = slerp([t_val])[0].as_matrix()

            if robot == "KUKA":
                p.fixed_q3 = (1.0 - t_val) * q3_start + t_val * q3_end

            T_flange = (
                np.linalg.inv(compute_base_matrix(p)) @
                T_goal @
                np.linalg.inv(compute_tcp_offset_matrix(p))
            )
            U, _, Vt = np.linalg.svd(T_flange[:3, :3])
            T_flange[:3, :3] = U @ Vt

            sols = ik_solver(T_flange)
            if not sols:
                if self.DEBUG_MOVE_L:
                    print(f"[Move_L] f{frame0+i}: IK failed")
                continue

            if master_q is None:
                if "joint_pose" in src:
                    master_q = np.array(src["joint_pose"], float)
                elif p.solutions and len(p.solutions) > p.current_index:
                    master_q = np.array(p.solutions[p.current_index], float)
                else:
                    master_q = np.array(sols[0], float)

            def _score(q):
                q = np.asarray(q)
                return sum(abs(stable_ang_diff(q[j], master_q[j]))
                           for j in range(len(master_q)))

            q_sel = min(sols, key=_score)
            if i == steps - 1 and dst.get("joint_pose"):
                q_sel = np.array(dst["joint_pose"], float)
            master_q = q_sel

            ctx.scene.frame_set(frame0 + i)
            apply_solution(arm, q_sel, frame0 + i, insert_keyframe=True)
            success += 1

        if p.goal_object and master_q is not None:
            fk_func = get_forward_kinematics()
            T_last = (
                compute_base_matrix(p) @
                fk_func(master_q) @
                compute_tcp_offset_matrix(p)
            )
            p.goal_object.matrix_world = Matrix(T_last)

        p.status_text = f"Fast LIN done ({success}/{steps})"
        return {'FINISHED'}
 # ──────────────────────────────────────────────────────────────

class OBJECT_OT_clear_path_visuals(bpy.types.Operator):
	bl_idname = "object.clear_path_visuals"
	bl_label = "Clear Teach Path"

	def execute(self, ctx):	
	    pv = bpy.data.collections.get("PathVisuals")	
	    if not pv:	
	        self.report({'INFO'}, "No PathVisuals")	
	        return {'CANCELLED'}	
	    for ob in pv.objects:	
	        ob.hide_viewport = True	
	        ob.hide_set(True)
	    self.report({'INFO'}, "Path hidden")	
	    return {'FINISHED'}

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_record_goal_as_empty(bpy.types.Operator):
    bl_idname = "object.record_goal_as_empty"
    bl_label  = "Record Goal (Single)"

    def execute(self, ctx):
        p   = ctx.scene.ik_motion_props
        tgt = p.goal_object
        if not tgt:
            self.report({'ERROR'}, "Goal not set")
            return {'CANCELLED'}

        coll = (bpy.data.collections.get("Teach data")
                or bpy.data.collections.new("Teach data"))
        if coll.name not in {c.name for c in ctx.scene.collection.children}:
            ctx.scene.collection.children.link(coll)

        new_idx = len([o for o in bpy.data.objects if o.name.startswith("P.")])
        dup = bpy.data.objects.new(name=f"P.{new_idx:03d}", object_data=tgt.data)
        dup.matrix_world = tgt.matrix_world
        dup.scale        = tgt.scale * 0.5
        dup["index"]     = new_idx
        dup["motion_type"]    = "LINEAR"
        dup["fixed_q3"] = p.fixed_q3
        dup["speed"] = p.motion_speed
        dup["wait_time_sec"] = p.wait_time_sec
    	
        dup.show_name = True
        p.selected_teach_point = dup
        
        if p.solutions and len(p.solutions) > p.current_index:
            dup["solution_index"] = p.current_index
            dup["joint_pose"] = list(map(float, p.solutions[p.current_index]))       
            
        coll.objects.link(dup)
        update_tcp_sorted_list()
        
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_playback_cycle_solution(bpy.types.Operator):
    """Cycle preview IK solution (Sequence Mode)"""
    bl_idname = "object.playback_cycle_solution"
    bl_label = "Cycle Solution"

    direction: bpy.props.EnumProperty(items=[('NEXT', 'Next', ''), ('PREV', 'Prev', '')])

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        tgt = p.goal_object
        arm = bpy.data.objects.get(p.armature)
        if not tgt or not arm:
            self.report({'ERROR'}, "Target or Armature not set")
            return {'CANCELLED'}
        
        q = tgt.matrix_world.to_quaternion()
        T_goal = np.eye(4)
        T_goal[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix() 
        T_goal[:3, 3]  = np.array(tgt.matrix_world.to_translation())
        
        T_flange = (
		    np.linalg.inv(compute_base_matrix(p))
		    @ T_goal
		    @ np.linalg.inv(compute_tcp_offset_matrix(p))
		)

        ik_solver = get_inverse_kinematics(p)
        sols = inverse_kinematics(T_flange)
        if not sols:
            self.report({'ERROR'}, "No IK solutions found")
            return {'CANCELLED'}

        # 3. Store new temp_solutions
        p.temp_solutions = [list(map(float, s)) for s in sols]

        # 4. Cycle index
        count = len(p.temp_solutions)
        idx = p.ik_temp_index
        if self.direction == 'NEXT':
            idx = (idx + 1) % count
        else:
            idx = (idx - 1) % count

        p.ik_temp_index = idx
        q_sel = p.temp_solutions[idx]

        # 5. Apply solution (preview only)
        ctx.scene.frame_set(ctx.scene.frame_current)
        apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)

        p.status_text = f"Preview {idx + 1}/{count}"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────
class OBJECT_OT_playback_apply_solution(bpy.types.Operator):
    bl_idname = "object.playback_apply_solution"
    bl_label = "Apply Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        name = p.selected_teach_point
        obj = bpy.data.objects.get(name)

        if not obj:
            self.report({'ERROR'}, "Teach Point not found")
            return {'CANCELLED'}

        idx = p.ik_temp_index
        if not p.temp_solutions or idx >= len(p.temp_solutions):
            self.report({'ERROR'}, "Invalid pose index")
            return {'CANCELLED'}

        q_sel = p.temp_solutions[idx]

        obj["solution_index"] = idx
        obj["joint_pose"] = list(map(float, q_sel))

        p.current_index = idx
        p.solution_index_ui = idx + 1

        p.status_text = f"Applied solution index {idx} to {obj.name}"
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_cycle_solution_ur(bpy.types.Operator):
    bl_idname = "object.cycle_solution_ur"
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

        T_base   = compute_base_matrix(p)
        T_offset = compute_tcp_offset_matrix(p)
        T_flange = np.linalg.inv(T_base) @ T_goal @ np.linalg.inv(T_offset)

        ik_solver = get_inverse_kinematics(p)
        sols = ik_solver(T_flange)
        if not sols:
            self.report({'WARNING'}, "No IK solutions")
            return {'CANCELLED'}

        if get_active_robot().startswith("UR"):
            sols = sort_solutions(sols)

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
class OBJECT_OT_apply_cycle_pose(bpy.types.Operator):
    bl_idname = "object.apply_cycle_pose"
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
 
        T_flange = (
            np.linalg.inv(compute_base_matrix(p))
            @ T_goal
            @ np.linalg.inv(compute_tcp_offset_matrix(p))
        )

        if not p.solutions:
            ik_solver = get_inverse_kinematics(p)
            sols = ik_solver(T_flange)

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

        # ✅ Also store pose to Teach Point
        obj["joint_pose"] = list(map(float, q_sel))
        obj["solution_index"] = p.current_index
        if get_active_robot() == "KUKA":
            obj["fixed_q3"] = p.fixed_q3

        p.status_text = f"Keyframed {p.current_index+1}/{len(p.solutions)}"
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_draw_teach_path(bpy.types.Operator):
    bl_idname = "object.draw_teach_path"
    bl_label  = "Draw Teach Path (Coloured)"

    samples: bpy.props.IntProperty(default=20, min=4, max=100)

    def execute(self, ctx):
        import numpy as np, bmesh
        from mathutils import Vector
        from rteach.core import get_forward_kinematics
        p = ctx.scene.ik_motion_props

        coll = bpy.data.collections.get("Teach data")
        if not coll or len(coll.objects) < 2:
            self.report({'WARNING'}, "Need ≥2 Teach Points")
            return {'CANCELLED'}
        tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        col_name = "PathVisuals"
        pv = bpy.data.collections.get(col_name) or bpy.data.collections.new(col_name)
        if pv.name not in {c.name for c in ctx.scene.collection.children}:
            ctx.scene.collection.children.link(pv)
        for ob in list(pv.objects):
            bpy.data.objects.remove(ob, do_unlink=True)

        curve = bpy.data.curves.new("TeachPath_Curve", 'CURVE')
        curve.dimensions = '3D'
        curve.bevel_depth = 0.001

        mat_lin = bpy.data.materials.get("PathLinear") or bpy.data.materials.new("PathLinear")
        mat_lin.use_nodes     = False
        mat_lin.diffuse_color = (1, 1, 0, 0.8)
        mat_lin.blend_method  = 'BLEND'

        mat_jnt = bpy.data.materials.get("PathJoint") or bpy.data.materials.new("PathJoint")
        mat_jnt.use_nodes     = False
        mat_jnt.diffuse_color = (0.6, 0, 0.9, 0.8)
        mat_jnt.blend_method  = 'BLEND'
        
        curve.materials.clear()
        curve.materials.append(mat_lin)
        curve.materials.append(mat_jnt)

        fk = get_forward_kinematics()
        N = max(2, self.samples)

        for i in range(len(tps) - 1):
            A, B = tps[i], tps[i + 1]
            motion = A.get("motion_type", "LINEAR").upper()
            spline = curve.splines.new('POLY')

            if motion == "LINEAR":
                spline.material_index = 0
                pts = [A.matrix_world.translation, B.matrix_world.translation]
            else:
                spline.material_index = 1
                q0 = np.asarray(A.get("joint_pose"), float)
                q1 = np.asarray(B.get("joint_pose"), float)
                pts = []
                for j in range(N):
                    t = j / (N - 1)
                    q = (1 - t) * q0 + t * q1
                    pts.append(Vector(fk(q)[:3, 3]))

            spline.points.add(len(pts) - 1)
            for k, v in enumerate(pts):
                spline.points[k].co = (*v, 1)

        obj = bpy.data.objects.new("TeachPath", curve)
        obj.show_in_front = False
        pv.objects.link(obj)
        self.report({'INFO'}, "Path shown")
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_playback_teach_bake(bpy.types.Operator):
    """Bake TCP sequence to keyframes (Linear & Joint)"""
    bl_idname = "object.playback_teach_bake"
    bl_label = "Bake Sequence"

    def execute(self, ctx):
        import numpy as np
        from mathutils import Matrix

        p   = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        giz = p.goal_object
        coll = bpy.data.collections.get("Teach data")
        if not (arm and giz and coll):
            self.report({'ERROR'}, "Armature / Goal / Teach data not set")
            return {'CANCELLED'}

        tps_all = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        start_idx = p.bake_start_tcp.get("index", 0) if p.bake_start_tcp else 0
        end_idx   = p.bake_end_tcp.get("index", 9999) if p.bake_end_tcp else 9999

        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        tps = [tp for tp in tps_all if start_idx <= tp.get("index", 9999) <= end_idx]

        if not tps:
            self.report({'WARNING'}, f"No TCPs between index {start_idx} and {end_idx}")
            return {'CANCELLED'}

        fps            = ctx.scene.render.fps
        default_speed  = p.motion_speed           # mm/s
        default_wait_s = p.wait_time_sec          # sec
        f0          = ctx.scene.frame_current
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

            print(f"[Bake] {tp.name} → {next_tp.name} | "
                  f"{motion} | Dist {dist_mm:.1f} mm, Speed {speed_mm} mm/s → {span} frames")

            if motion == "LINEAR":

                robot = get_active_robot()

                if robot == "KUKA" and p.precise_linear:
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
                        print(f"[Bake][{tp.name}] Precise LIN")
                        print(f"  q_start (deg): {[round(math.degrees(q),1) for q in q_start]}")
                        fps = ctx.scene.render.fps
                        speed = tp.get("speed", p.motion_speed)  

                        motion_time_sec = dist_mm / max(speed, 1e-3)
                        n_steps = round(motion_time_sec * fps)
                        auto_step_mm = dist_mm / max(n_steps, 1)

                        print(f"  dist = {round(dist_mm,2)} mm | speed = {speed} mm/s | fps = {fps}")
                        print(f"  → n_steps = {n_steps} | auto_step_mm = {round(auto_step_mm, 2)}")

                        p.step_mm = auto_step_mm
                        path = linear_move(q_start, T_goal_flange, p)
                        if path:
                            for i, q in enumerate(path):
                                ctx.scene.frame_set(f + i + 1)
                                apply_solution(arm, q, f + i + 1, insert_keyframe=True)

                            next_tp["joint_pose"] = list(map(float, path[-1]))
                            
                            f += len(path)
                            continue
                        else:
                            self.report({'WARNING'}, f"Precise LIN failed: {tp.name} → {next_tp.name}")
                    else:
                        self.report({'WARNING'}, f"[Bake] joint_pose missing in {tp.name}")

                if robot == "KUKA":
                    p.fixed_q3 = float(tp.get("fixed_q3", p.fixed_q3))

                p.goal_object   = giz
                p.linear_target = next_tp
                p.linear_frames = span

                ctx.scene.frame_set(f + 1)
                bpy.ops.object.move_l_ik('EXEC_DEFAULT')
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
class OBJECT_OT_tcp_move_up(bpy.types.Operator):
    """Move Teach Point up in order"""
    bl_idname = "object.tcp_move_up"
    bl_label = "Move Up"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        sorted_tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        i = p.tcp_list_index
        if not (0 <= i < len(sorted_tps)):
            self.report({'WARNING'}, "Invalid index")
            return {'CANCELLED'}

        if i == 0:
            self.report({'INFO'}, "Already at top")
            return {'CANCELLED'}

        obj_a = sorted_tps[i]
        obj_b = sorted_tps[i - 1]
        idx_a = obj_a.get("index", 9999)
        idx_b = obj_b.get("index", 9999)

        obj_a["index"], obj_b["index"] = idx_b, idx_a
        p.tcp_list_index = i - 1

        bpy.ops.object.draw_teach_path()
        ctx.area.tag_redraw()
        update_tcp_sorted_list()
        p.status_text = f"Moved {obj_a.name} up"
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_tcp_move_down(bpy.types.Operator):
    """Move Teach Point down in order"""
    bl_idname = "object.tcp_move_down"
    bl_label = "Move Down"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        sorted_tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        i = p.tcp_list_index
        if not (0 <= i < len(sorted_tps)):
            self.report({'WARNING'}, "Invalid index")
            return {'CANCELLED'}

        if i == len(sorted_tps) - 1:
            self.report({'INFO'}, "Already at bottom")
            return {'CANCELLED'}

        obj_a = sorted_tps[i]
        obj_b = sorted_tps[i + 1]
        idx_a = obj_a.get("index", 9999)
        idx_b = obj_b.get("index", 9999)

        obj_a["index"], obj_b["index"] = idx_b, idx_a
        p.tcp_list_index = i + 1

        bpy.ops.object.draw_teach_path()
        ctx.area.tag_redraw()
        update_tcp_sorted_list()
        p.status_text = f"Moved {obj_a.name} down"
        return {'FINISHED'}
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_tcp_update_position(bpy.types.Operator):
    bl_idname = "object.tcp_update_position"
    bl_label  = "Update TCP Pose"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        p   = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        obj = bpy.data.objects.get(self.name or (p.selected_teach_point.name if p.selected_teach_point else None))
        tgt = p.goal_object
        if not (arm and obj and tgt):
            self.report({'ERROR'}, "Goal / Teach Point / Armature not set")
            return {'CANCELLED'}

        moved = not np.allclose(np.array(tgt.matrix_world), np.array(obj.matrix_world), atol=1e-6)

        # ── Gizmo → TCP (overwrite) ─────────────────────────────
        if moved:
            obj.matrix_world = tgt.matrix_world.copy()
            obj.scale *= 0.5

            T_goal = np.array(tgt.matrix_world)
            T_flange = (
                np.linalg.inv(compute_base_matrix(p))
                @ T_goal
                @ np.linalg.inv(compute_tcp_offset_matrix(p))
            )
            ik_solver = get_inverse_kinematics(p)
            sols = ik_solver(T_flange)
            if not sols:
                self.report({'ERROR'}, "IK failed")
                return {'CANCELLED'}

            if "joint_pose" in obj:
                q_ref = np.asarray(obj["joint_pose"], float)
            else:
                q_ref = [getattr(pb.rotation_euler, ax)
                         for pb, ax in zip(arm.pose.bones, AXES)]

            def ang(a, b): return ((a - b + math.pi) % (2 * math.pi)) - math.pi
            idx = min(range(len(sols)),
                      key=lambda i: sum(abs(ang(sols[i][j], q_ref[j])) for j in range(6)))
            q_sel = sols[idx]
            apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)

            obj["solution_index"] = idx
            obj["joint_pose"]     = list(map(float, q_sel))
            if get_active_robot() == "KUKA":
                p.fixed_q3 = q_sel[2]
                obj["fixed_q3"] = p.fixed_q3

            p.current_index     = idx
            p.solution_index_ui = idx + 1
            p.status_text = f"{obj.name} updated from Gizmo"

        # ── TCP → Gizmo (preview) ──────────────────────────────
        else:
            tgt.matrix_world = obj.matrix_world.copy()
            tgt.scale        = (1, 1, 1)
            if "joint_pose" in obj:
                q_stored = obj["joint_pose"]
                apply_solution(arm, q_stored, ctx.scene.frame_current, insert_keyframe=False)
                if get_active_robot() == "KUKA":
                    p.fixed_q3 = q_stored[2]
                p.current_index     = int(obj.get("solution_index", 0))
                p.solution_index_ui = p.current_index + 1
                p.status_text = f"Preview {obj.name}"
            else:
                T_goal = np.array(tgt.matrix_world)
                T_flange = (
                    np.linalg.inv(compute_base_matrix(p))
                    @ T_goal
                    @ np.linalg.inv(compute_tcp_offset_matrix(p))
                )
                ik_solver = get_inverse_kinematics(p)
                sols = ik_solver(T_flange)
                if not sols:
                    self.report({'ERROR'}, "IK failed")
                    return {'CANCELLED'}
                q_sel = sols[0]
                apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)
                obj["solution_index"] = 0
                obj["joint_pose"]     = list(map(float, q_sel))
                p.current_index = 0
                p.solution_index_ui = 1
                p.status_text = f"{obj.name} previewed"

        bpy.ops.object.draw_teach_path()
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_tcp_delete(bpy.types.Operator):
    """Delete selected Teach Point"""
    bl_idname = "object.tcp_delete"
    bl_label = "Delete TCP Point"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        obj = bpy.data.objects.get(self.name)
        if not obj:
            self.report({'ERROR'}, f"Object '{self.name}' not found")
            return {'CANCELLED'}

        # 컬렉션에서 언링크
        for coll in obj.users_collection:
            coll.objects.unlink(obj)

        # 데이터 삭제
        bpy.data.objects.remove(obj, do_unlink=True)
        ctx.scene.ik_motion_props.status_text = f"Deleted {self.name}"
        update_tcp_sorted_list()
        return {'FINISHED'}
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_reindex_tcp_points(bpy.types.Operator):
    """Reassign sequential index values to all Teach Points"""
    bl_idname = "object.reindex_tcp_points"
    bl_label = "Reindex TCP"

    def execute(self, ctx):
        objs = bpy.data.collections.get("Teach data", {}).objects
        if not objs:
            self.report({'INFO'}, "No Teach Points found")
            return {'CANCELLED'}

        # Teach Point들을 index 기준으로 정렬
        sorted_objs = sorted(objs, key=lambda o: o.get("index", 9999))

        # index 재지정
        for i, obj in enumerate(sorted_objs):
            obj["index"] = i

        # ✅ selected_teach_point에 첫 번째 TCP 오브젝트를 Object 타입으로 설정
        props = ctx.scene.ik_motion_props
        props.selected_teach_point = sorted_objs[0] if sorted_objs else None
        props.status_text = "Teach Points reindexed"

        return {'FINISHED'}

    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_clear_all_tcp_points(bpy.types.Operator):
    bl_idname = "object.clear_all_tcp_points"
    bl_label  = "Clear All TCPs"

    def execute(self, ctx):
        coll = bpy.data.collections.get("Teach data")
        if coll:
            for ob in list(coll.objects):
                bpy.data.objects.remove(ob, do_unlink=True)

        path_coll = bpy.data.collections.get("Teach path")
        if path_coll:
            for ob in list(path_coll.objects):
                if ob.name.startswith("TeachPath"):
                    bpy.data.objects.remove(ob, do_unlink=True)

        props = ctx.scene.ik_motion_props
        props.selected_teach_point = None        

        if props.goal_object and props.goal_object.name not in bpy.data.objects:
            props.goal_object = None               

        props.status_text = "All Teach Points and Path cleared"
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
            p.fixed_q3 = obj["joint_pose"][2]
            idx = int(obj.get("solution_index", 0))
            p.solution_index_ui = idx + 1
            p.current_index = idx

        p.status_text = f"Preview {obj.name} (stored)"
        return {'FINISHED'}

    # ② 없으면 IK 재계산
    T_goal = np.array(obj.matrix_world)
    T_flange = (
        np.linalg.inv(compute_base_matrix(p)) @
        T_goal @
        np.linalg.inv(compute_tcp_offset_matrix(p))
    )
    ik_solver = get_inverse_kinematics(p)
    sols = inverse_kinematics(T_flange)
    if not sols:
        p.status_text = "IK failed"
        return {'CANCELLED'}

    idx_saved = int(obj.get("solution_index", 0))
    idx = max(0, min(idx_saved, len(sols)-1))
    if arm:
        apply_solution(arm, sols[idx], ctx.scene.frame_current, insert_keyframe=False)
    if p.goal_object:
        p.goal_object.matrix_world = obj.matrix_world
        p.goal_object.scale = (1, 1, 1)  # 🔧 Gizmo 크기 복원

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
            self.report({'INFO'}, "No Teach Points")
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
            self.report({'INFO'}, "No Teach Points")
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
            self.report({'ERROR'}, "No teach point selected")
            return {'CANCELLED'}

        if "motion_type" in obj:
            obj["motion_type"] = "LINEAR" if obj["motion_type"] == "JOINT" else "JOINT"
            self.report({'INFO'}, f"{obj.name} motion type set to {obj['motion_type']}")
        else:
            self.report({'WARNING'}, "Teach point missing 'motion_type' key")

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

        # armature key 삭제
        if arm and arm.animation_data and arm.animation_data.action:
            for fc in list(arm.animation_data.action.fcurves):
                arm.animation_data.action.fcurves.remove(fc)

        # gizmo (goal_object) key 삭제
        if goal and goal.animation_data and goal.animation_data.action:
            for fc in list(goal.animation_data.action.fcurves):
                goal.animation_data.action.fcurves.remove(fc)

        self.report({'INFO'}, "Bake keys deleted")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_snap_goal_to_active(bpy.types.Operator):
    """Snap Target Gizmo to selected object"""
    bl_idname = "object.snap_goal_to_active"
    bl_label = "Snap Target to Active"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        goal = p.goal_object
        active = ctx.active_object

        if not goal:
            self.report({'ERROR'}, "Target (goal_object) not set")
            return {'CANCELLED'}
        if not active:
            self.report({'ERROR'}, "No active object selected")
            return {'CANCELLED'}

        goal.matrix_world = active.matrix_world.copy()
        goal.scale = (1, 1, 1)  # 🔧 크기 초기화 (선택적)
        goal.select_set(True)

        p.status_text = f"Snapped target to {active.name}"
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_update_fixed_q3_from_pose(bpy.types.Operator):
    """Update fixed_q3 from current robot pose"""
    bl_idname = "object.update_fixed_q3"
    bl_label = "Update q3 from Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)

        if not arm:
            self.report({'ERROR'}, "Armature not set")
            return {'CANCELLED'}

        bone_name = "j3"
        bone_idx = 2
        pb = arm.pose.bones.get(bone_name)
        if not pb:
            self.report({'ERROR'}, "Bone j3 not found")
            return {'CANCELLED'}

        axis = AXES[bone_idx]
        if axis == 'x':
            angle_rad = pb.rotation_euler.x
        elif axis == 'y':
            angle_rad = pb.rotation_euler.y
        elif axis == 'z':
            angle_rad = pb.rotation_euler.z
        else:
            angle_rad = 0.0

        p.fixed_q3 = angle_rad # ✅ degree 단위로 UI 업데이트
        self.report({'INFO'}, f"fixed_q3 updated to {round(angle_rad, 2)}°")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_snap_gizmo_on_path(bpy.types.Operator):
    bl_idname = "object.snap_gizmo_on_path"
    bl_label = "Snap Gizmo on Path"

    def execute(self, ctx):
        import numpy as np

        p = ctx.scene.ik_motion_props
        giz = p.goal_object
        tp  = p.selected_teach_point
        coll = bpy.data.collections.get("Teach data")

        if not tp or not giz or not coll:
            self.report({'ERROR'}, "Gizmo / TCP / Teach data missing")
            return {'CANCELLED'}

        # Sort all TCPs by index
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

        p.status_text = f"Gizmo moved to {round(t*100)}% between {tp_a.name} and {tp_b.name}"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────     
class OBJECT_OT_sync_robot_type(bpy.types.Operator):
    bl_idname = "object.sync_robot_type"
    bl_label = "Sync Robot Type"

    def execute(self, ctx):
        p   = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature) or ctx.view_layer.objects.active
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Select a valid armature in Setup")
            return {'CANCELLED'}

        bone_cnt = len(arm.pose.bones)
        if p.robot_type == 'UR' and bone_cnt != 7:
            self.report({'ERROR'}, f"Armature has {bone_cnt} bones, need 7 for UR")
            return {'CANCELLED'}
        if p.robot_type == 'KUKA' and bone_cnt != 8:
            self.report({'ERROR'}, f"Armature has {bone_cnt} bones, need 8 for KUKA")
            return {'CANCELLED'}

        set_active_robot(p.robot_type)
        self.report({'INFO'}, f"Robot synced to {p.robot_type}")
        return {'FINISHED'}   
        
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_focus_on_target(bpy.types.Operator):
    bl_idname = "object.focus_on_target"
    bl_label = "Focus on Target"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        tgt = p.goal_object

        if not tgt:
            self.report({'ERROR'}, "No target set")
            return {'CANCELLED'}

        bpy.ops.object.select_all(action='DESELECT')
        tgt.select_set(True)
        ctx.view_layer.objects.active = tgt

        self.report({'INFO'}, f"Target '{tgt.name}' is now selected")
        return {'FINISHED'}
        
# ──────────────────────────────────────────────────────────────       
class OBJECT_OT_apply_global_speed(bpy.types.Operator):
    """Apply current TCP speed to all Teach Points"""
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
    """Apply current TCP wait time to all Teach Points"""
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

        tgt = p.goal_object
        arm = bpy.data.objects.get(p.armature)
        if not (tgt and arm):
            return {'CANCELLED'}

        tgt.matrix_world = obj.matrix_world.copy()
        tgt.scale = (1, 1, 1)

        if "joint_pose" in obj:
            from rteach.core import apply_solution
            apply_solution(arm, obj["joint_pose"], ctx.scene.frame_current, insert_keyframe=False)

        p.status_text = f"Snapped and previewed {obj.name}"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────         
def update_tcp_sorted_list():
    p = bpy.context.scene.ik_motion_props
    coll = bpy.data.collections.get("Teach data")
    if not coll:
        return

    p.tcp_sorted_list.clear()

    for obj in sorted(coll.objects, key=lambda o: o.get("index", 9999)):
        item = p.tcp_sorted_list.add()
        item.name = obj.name

    p.tcp_list_index = min(p.tcp_list_index, len(p.tcp_sorted_list) - 1)
    bpy.context.area.tag_redraw()
    
# ──────────────────────────────────────────────────────────────     
class OBJECT_OT_export_teach_data(bpy.types.Operator):
    """Export Teach Data (TCP pose + joint angles)"""
    bl_idname = "object.export_teach_data"
    bl_label = "Export Teach Data (.json)"

    def execute(self, ctx):
        import json
        from mathutils import Matrix, Quaternion
        from pathlib import Path

        p = ctx.scene.ik_motion_props
        base = p.base_object
        coll = bpy.data.collections.get("Teach data")
        if not base or not coll:
            self.report({'ERROR'}, "Base or Teach data not found")
            return {'CANCELLED'}

        base_inv = base.matrix_world.inverted()
        tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        data = []
        for tp in tps:
            mat_rel = base_inv @ tp.matrix_world
            pos = mat_rel.to_translation()
            quat = mat_rel.to_quaternion()

            q = tp.get("joint_pose")
            if not q:
                continue

            data.append({
                "name": tp.name,
                "position_m": [round(pos.x, 6), round(pos.y, 6), round(pos.z, 6)],
                "quaternion": [round(quat.x, 6), round(quat.y, 6), round(quat.z, 6), round(quat.w, 6)],
                "joint_pose_rad": [round(v, 6) for v in q]
            })

        if not data:
            self.report({'WARNING'}, "No teach points with joint data")
            return {'CANCELLED'}

        path = Path(bpy.path.abspath("//export_teach_data.json"))
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        self.report({'INFO'}, f"Exported {len(data)} TCPs → {path.name}")
        return {'FINISHED'}
