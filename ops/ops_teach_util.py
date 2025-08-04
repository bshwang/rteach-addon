import bpy
import math
import json
import csv
import re
import os

import numpy as np
from pathlib import Path
from mathutils import Vector
from rteach.core.robot_presets import ROBOT_CONFIGS
from rteach.core.core import get_forward_kinematics, get_BONES, get_AXES
from rteach.config.settings import delayed_workspace_update
from rteach.ext import dynamic_parent as dp
from rteach.core.core import get_tcp_object

def find_object_by_prefix(name: str) -> bpy.types.Object | None:
    """
    Find best matching object whose name starts with `name`,
    preferring the one with the highest numeric suffix (e.g., .002 > .001 > none).
    """
    def extract_suffix_index(n):
        match = re.match(rf"^{re.escape(name)}(?:\.(\d+))?$", n)
        return int(match.group(1)) if match and match.group(1) else -1

    matches = [obj for obj in bpy.data.objects if obj.name.startswith(name)]
    if not matches:
        return None

    best = max(matches, key=lambda obj: extract_suffix_index(obj.name))
    return best

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
class OBJECT_OT_draw_teach_path(bpy.types.Operator):
    bl_idname = "object.draw_teach_path"
    bl_label  = "Draw Teach Path (Coloured)"

    samples: bpy.props.IntProperty(default=20, min=4, max=100)

    def execute(self, ctx):

        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll or len(coll.objects) < 2:
            self.report({'WARNING'}, "Need ≥2 Waypoints")
            return {'CANCELLED'}

        tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        col_name = "PathVisuals"
        pv = bpy.data.collections.get(col_name) or bpy.data.collections.new(col_name)
        if pv.name not in {c.name for c in ctx.scene.collection.children}:
            ctx.scene.collection.children.link(pv)

        for ob in list(bpy.data.objects):
            if ob.name == "TeachPath":
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

            elif motion == "JOINT":
                spline.material_index = 1
                q0_raw = A.get("joint_pose")
                q1_raw = B.get("joint_pose")
                if not q0_raw or not q1_raw:
                    continue
                q0 = np.asarray(q0_raw, float)
                q1 = np.asarray(q1_raw, float)
                pts = []
                for j in range(N):
                    t = j / (N - 1)
                    q = (1 - t) * q0 + t * q1
                    T_fk = fk(q)
                    pts.append(Vector(T_fk[:3, 3]))

            else:
                continue

            spline.points.add(len(pts) - 1)
            for k, v in enumerate(pts):
                spline.points[k].co = (*v, 1)

        obj = bpy.data.objects.new("TeachPath", curve)
        obj.show_in_front = False
        pv.objects.link(obj)
        self.report({'INFO'}, "Path shown")
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_tcp_move_up(bpy.types.Operator):
    """Move Waypoint up in order"""
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
    """Move Waypoint down in order"""
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
class OBJECT_OT_tcp_delete(bpy.types.Operator):
    """Delete selected Waypoint"""
    bl_idname = "object.tcp_delete"
    bl_label = "Delete TCP Point"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        obj = bpy.data.objects.get(self.name)
        if not obj:
            self.report({'ERROR'}, f"Object '{self.name}' not found")
            return {'CANCELLED'}

        for coll in obj.users_collection:
            coll.objects.unlink(obj)

        bpy.data.objects.remove(obj, do_unlink=True)
        ctx.scene.ik_motion_props.status_text = f"Deleted {self.name}"
        update_tcp_sorted_list()
        return {'FINISHED'}
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_reindex_tcp_points(bpy.types.Operator):
    """Reassign sequential index values to all Waypoints"""
    bl_idname = "object.reindex_tcp_points"
    bl_label = "Reindex TCP"

    def execute(self, ctx):
        objs = bpy.data.collections.get("Teach data", {}).objects
        if not objs:
            self.report({'INFO'}, "No Waypoints found")
            return {'CANCELLED'}

        sorted_objs = sorted(objs, key=lambda o: o.get("index", 9999))

        for i, obj in enumerate(sorted_objs):
            obj["index"] = i

        props = ctx.scene.ik_motion_props
        props.selected_teach_point = sorted_objs[0] if sorted_objs else None
        props.status_text = "Waypoints reindexed"

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

        props.status_text = "All Waypoints and Path cleared"
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
        goal.select_set(True)

        p.status_text = f"Snapped target to {active.name}"
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_focus_on_target(bpy.types.Operator):
    bl_idname = "object.focus_on_target"
    bl_label  = "Focus on Target"

    def execute(self, ctx):
        if ctx.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        p   = ctx.scene.ik_motion_props
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
def update_tcp_sorted_list():
    p = bpy.context.scene.ik_motion_props
    coll = bpy.data.collections.get("Teach data")
    if not coll:
        p.tcp_sorted_list.clear()
        p.selected_teach_point = None
        p.tcp_list_index = -1
        return

    p.tcp_sorted_list.clear()

    for obj in sorted(coll.objects, key=lambda o: o.get("index", 9999)):
        if obj.name not in bpy.data.objects:
            continue
        item = p.tcp_sorted_list.add()
        item.name = obj.name

    p.tcp_list_index = min(p.tcp_list_index, len(p.tcp_sorted_list) - 1)

    if p.selected_teach_point and p.selected_teach_point.name in bpy.data.objects:
        obj = p.selected_teach_point
        from rteach.core.core import get_best_ik_solution
        import numpy as np

        if "joint_pose" in obj:
            q_saved = np.asarray(obj["joint_pose"], float)
            p.solutions = [q_saved]
            p.max_solutions = 1
            p.current_index = obj.get("solution_index", 0)
            p.solution_index_ui = p.current_index + 1
        else:
            T_goal = np.array(obj.matrix_world)
            q_sel, sols = get_best_ik_solution(p, T_goal)
            p.solutions = [list(map(float, s)) for s in sols]
            p.max_solutions = len(sols)
            p.current_index = obj.get("solution_index", 0)
            p.solution_index_ui = p.current_index + 1
    else:
        p.selected_teach_point = None

    bpy.context.area.tag_redraw()

# ──────────────────────────────────────────────────────────────     
class OBJECT_OT_export_teach_data(bpy.types.Operator):
    bl_idname = "object.export_teach_data"
    bl_label  = "Export Teach Data (.json)"

    def execute(self, ctx):
        p       = ctx.scene.ik_motion_props
        arm_obj = bpy.data.objects.get(p.armature)
        coll    = bpy.data.collections.get("Teach data")
        if not arm_obj or not coll:
            self.report({'ERROR'}, "Armature or Teach data missing")
            return {'CANCELLED'}

        inv = arm_obj.matrix_world.inverted()
        tps = sorted(coll.objects, key=lambda o: o.get("seq",0))

        data = []
        prev_tp = None
        for tp in tps:
            if not prev_tp:
                prev_tp = tp
                continue

            def make_point(obj):
                mat   = inv @ obj.matrix_world
                pos   = mat.to_translation()
                quat  = mat.to_quaternion()
                qp    = obj.get("joint_pose", [])
                pt    = {}
                for i, v in enumerate(qp):
                    pt[f"A{i+1}"] = round(v, 6)
                mt = obj.get("motion_type", "JOINT")
                pt["MoveType"] = "PTP" if mt=="JOINT" else "LIN"
                pt["QW"]       = round(quat.w, 6)
                pt["QX"]       = round(quat.x, 6)
                pt["QY"]       = round(quat.y, 6)
                pt["QZ"]       = round(quat.z, 6)
                pt["X"]        = round(pos.x, 6)
                pt["Y"]        = round(pos.y, 6)
                pt["Z"]        = round(pos.z, 6)
                return pt

            start_pt  = make_point(prev_tp)
            goal_pt   = make_point(tp)
            path_name = f"FROM_{prev_tp['goal']}_TO_{tp['goal']}"

            data.append({
                "Path":   path_name,
                "Points": [
                    {"StartPoint": start_pt, "GoalPoint": goal_pt}
                ],
                "Robot": arm_obj.name
            })

            prev_tp = tp

        filename = p.export_teach_filename
        filepath = bpy.path.abspath(f"//{filename}.json")
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        self.report({'INFO'}, f"Exported {len(data)} segments → {os.path.basename(filepath)}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────   
class OBJECT_OT_export_joint_graph_csv(bpy.types.Operator):
    bl_idname = "object.export_joint_graph_csv"
    bl_label = "Export Joint Graph CSV"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            self.report({'ERROR'}, "No armature found")
            return {'CANCELLED'}

        bones = get_BONES()
        axes = get_AXES()
        dof = len(bones)

        robot_key = p.robot_type.lower()
        config = ROBOT_CONFIGS.get(robot_key, {})
        stage_joints = config.get("stage_joints", [])

        frame_start = ctx.scene.frame_start
        frame_end = ctx.scene.frame_end
        frames = list(range(frame_start, frame_end + 1))

        headers = ["frame"]

        for i in range(dof):
            if i < len(p.show_plot_joints) and p.show_plot_joints[i]:
                headers.append(f"j{i+1}")

        for i, sj in enumerate(stage_joints):
            idx = dof + i
            if idx < len(p.show_plot_joints) and p.show_plot_joints[idx]:
                label = sj[1] if len(sj) > 1 else f"stage_{i}"
                headers.append(label)

        data = []
        for f in frames:
            bpy.context.scene.frame_set(f)
            row = [f]

            for i, bname in enumerate(bones):
                if i >= len(p.show_plot_joints) or not p.show_plot_joints[i]:
                    continue
                b = arm.pose.bones.get(bname)
                if not b:
                    row.append(0)
                    continue

                axis = axes[i]
                angle = (
                    b.rotation_euler.to_quaternion().angle
                    if b.rotation_mode == 'QUATERNION'
                    else getattr(b.rotation_euler, axis)
                )
                row.append(round(angle, 4))

            for i, sj in enumerate(stage_joints):
                idx = dof + i
                if idx >= len(p.show_plot_joints) or not p.show_plot_joints[idx]:
                    continue

                name = sj[0]
                axis = sj[5]
                joint_type = sj[6]
                ob = bpy.data.objects.get(name)
                if not ob:
                    row.append(0)
                    continue

                axis_idx = {"x": 0, "y": 1, "z": 2}[axis.lower()]
                val = (
                    ob.location[axis_idx]
                    if joint_type == "location"
                    else ob.rotation_euler[axis_idx]
                )
                row.append(round(val, 4))

            data.append(row)

        filename = ctx.scene.ik_motion_props.export_joint_csv_filename
        path = Path(bpy.path.abspath(f"//{filename}.csv"))
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(data)

        self.report({'INFO'}, f"Exported CSV with {len(data)} frames → {path.name}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────      
class OBJECT_OT_clear_robot_system(bpy.types.Operator):
    bl_idname = "object.clear_robot_system"
    bl_label = "🧹 Clear System"

    def execute(self, ctx):
        scene = ctx.scene
        p = scene.ik_motion_props

        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)

        target_colls = {"setup", "robot", "stage", "teach data"}
        for coll in list(bpy.data.collections):
            if coll.name.lower() in target_colls:
                try:
                    bpy.data.collections.remove(coll)
                except:
                    pass

        robot_key = p.robot_type.lower()
        config = ROBOT_CONFIGS.get(robot_key, {})
        delete_names = {"Target_Gizmo"}
        delete_names.update(config.get("setup_objects", {}).values())

        for name in delete_names:
            obj = bpy.data.objects.get(name)
            if obj:
                bpy.data.objects.remove(obj, do_unlink=True)

        bpy.ops.outliner.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)

        p.goal_object = None
        p.base_object = None
        p.ee_object = None
        p.tcp_object = None
        p.robot_type = ""
        p.armature = 'None'

        self.report({'INFO'}, "Cleared robot system and purged unused data")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────      
class OBJECT_OT_refresh_stage_sliders(bpy.types.Operator):
    bl_idname = "object.refresh_stage_sliders"
    bl_label = "Refresh Stage Sliders"

    def execute(self, context):
        p = context.scene.ik_motion_props
        props = p.stage_props

        for joint in props.joints:
            obj = bpy.data.objects.get(joint.target)
            if not obj:
                self.report({'WARNING'}, f"Target object '{joint.target}' not found")
                continue

            axis = joint.name.split("_")[-1].lower()
            is_rot = "rot" in joint.target or "tilt" in joint.target

            idx = {"x": 0, "y": 1, "z": 2}.get(axis[-1], 2)
            val = obj.rotation_euler[idx] if is_rot else obj.location[idx]

            if joint.unit_type == "mm":
                val *= 1000.0
            elif joint.unit_type == "deg":
                val = math.degrees(val)

            joint.value = val

        self.report({'INFO'}, "Stage sliders refreshed from scene")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_refresh_tcp_list(bpy.types.Operator):
    bl_idname = "object.refresh_tcp_list"
    bl_label = "Refresh TCP List"

    def execute(self, ctx):
        update_tcp_sorted_list()         
        self.report({'INFO'}, "TCP list refreshed")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_toggle_path_visibility(bpy.types.Operator):
    bl_idname = "object.toggle_path_visibility"
    bl_label = "Toggle Path Visibility"

    def execute(self, ctx):
        pv = bpy.data.collections.get("PathVisuals")
        if not pv or not pv.objects:
            self.report({'INFO'}, "No path to toggle")
            return {'CANCELLED'}

        for ob in pv.objects:
            state = not ob.visible_get()
            ob.hide_viewport = not state
            ob.hide_set(not state)

        ctx.area.tag_redraw()
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────     
class OBJECT_OT_cycle_armature_set(bpy.types.Operator):
    bl_idname = "object.cycle_armature_set"
    bl_label = "Cycle Armature Set"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        config = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        sets = config.get("armature_sets", {})

        if not sets:
            self.report({'ERROR'}, "No armature sets defined")
            return {'CANCELLED'}

        keys = list(sets.keys())
        if not keys:
            self.report({'ERROR'}, "Empty armature set list")
            return {'CANCELLED'}

        current = p.armature
        idx = keys.index(current) if current in keys else -1
        next_key = keys[(idx + 1) % len(keys)]

        cfg = sets[next_key]
        p.armature = next_key
        p.base_object = find_object_by_prefix(cfg.get("base", ""))
        p.ee_object = find_object_by_prefix(cfg.get("ee", ""))
        p.tcp_object = find_object_by_prefix(cfg.get("tcp", ""))

        self.report({'INFO'}, f"Switched to: {next_key}")

        bpy.app.timers.register(delayed_workspace_update, first_interval=0.05)
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_slide_robot_prev(bpy.types.Operator):
    bl_idname = "object.slide_robot_prev"
    bl_label = "Prev Robot"
    group: bpy.props.StringProperty()

    def execute(self, ctx):
        idx = getattr(ctx.scene, self.group, 0)
        setattr(ctx.scene, self.group, (idx - 1) % 99)
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_slide_robot_next(bpy.types.Operator):
    bl_idname = "object.slide_robot_next"
    bl_label = "Next Robot"
    group: bpy.props.StringProperty()

    def execute(self, ctx):
        idx = getattr(ctx.scene, self.group, 0)
        setattr(ctx.scene, self.group, (idx + 1) % 99)
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_pick_object(bpy.types.Operator):
    bl_idname = "object.pick_object"
    bl_label = "Pick Object"

    def execute(self, ctx):
        tcp = get_tcp_object()
        if not ctx.active_object:
            self.report({'ERROR'}, "No active object")
            return {'CANCELLED'}

        sel = ctx.selected_objects
        if not sel:
            self.report({'ERROR'}, "Nothing selected")
            return {'CANCELLED'}

        child = ctx.active_object
        if len(sel) == 1:
            parent = tcp
        else:
            if tcp in sel:
                parent = tcp
            else:
                parent = sel[0] if sel[0] != child else sel[1]

        if not parent or parent == child:
            self.report({'ERROR'}, "Parent selection failed")
            return {'CANCELLED'}
        
        const = dp.get_last_dynamic_parent_constraint(child)
        if const and const.influence == 1:
            dp.disable_constraint(child, const, ctx.scene.frame_current)
            self._set_const_interp(child, const.name)

        prev_active = ctx.view_layer.objects.active
        prev_sel = sel[:]
        for o in prev_sel:
            o.select_set(False)
        parent.select_set(True)
        child.select_set(True)
        ctx.view_layer.objects.active = child

        dp.dp_create_dynamic_parent_obj(self)

        new_const = child.constraints[-1]
        self._set_const_interp(child, new_const.name)

        for o in ctx.selected_objects:
            o.select_set(False)
        for o in prev_sel:
            o.select_set(True)
        if prev_active:
            ctx.view_layer.objects.active = prev_active

        return {'FINISHED'}

    def _set_const_interp(self, obj, const_name):
        ad = obj.animation_data
        if not ad or not ad.action:
            return
        path = f'constraints["{const_name}"].influence'
        fc = ad.action.fcurves.find(path)
        if not fc:
            return
        for kp in fc.keyframe_points:
            kp.interpolation = 'CONSTANT'
        fc.update()

class OBJECT_OT_place_object(bpy.types.Operator):
    bl_idname = "object.place_object"
    bl_label = "Place Object"

    def execute(self, ctx):
        obj = ctx.active_object
        if not obj:
            self.report({'ERROR'}, "No active object")
            return {'CANCELLED'}

        const = dp.get_last_dynamic_parent_constraint(obj)
        if not const or const.influence == 0:
            self.report({'WARNING'}, "No dynamic‑parent to disable")
            return {'CANCELLED'}

        dp.disable_constraint(obj, const, ctx.scene.frame_current)
        ad = obj.animation_data
        if ad and ad.action:
            fc = ad.action.fcurves.find(f'constraints["{const.name}"].influence')
            if fc:
                for kp in fc.keyframe_points:
                    kp.interpolation = 'CONSTANT'
                fc.update()
        return {'FINISHED'}

class OBJECT_OT_clear_dynamic_parent(bpy.types.Operator):
    bl_idname = "object.clear_dynamic_parent"
    bl_label = "Clear DP"

    def execute(self, ctx):
        obj = ctx.active_object
        if not obj:
            self.report({'ERROR'}, "No active object")
            return {'CANCELLED'}

        pbone = None
        if obj.type == 'ARMATURE':
            pbone = ctx.active_pose_bone

        dp.dp_clear(obj, pbone)
        return {'FINISHED'}
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_tcp_bake_select_all(bpy.types.Operator):
    bl_idname = "object.tcp_bake_select_all"
    bl_label  = "Select/Deselect All"
    mode: bpy.props.EnumProperty(items=[("ON","ON",""), ("OFF","OFF","")])

    def execute(self, ctx):
        tps = [o for o in bpy.data.collections.get("Teach data").objects if o.name.startswith("P.")]
        for tp in tps:
            tp["bake_enabled"] = (self.mode == "ON")
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_toggle_tcp_bake_all(bpy.types.Operator):
    bl_idname = "object.toggle_tcp_bake_all"
    bl_label  = "Toggle All TCP Bake"
    bl_description = "Toggle all bake checkboxes ON/OFF"

    def execute(self, ctx):
        scene = ctx.scene
        state = not getattr(scene, "bake_all_state", False)
        tps = bpy.data.collections.get("Teach data").objects
        for tp in tps:
            if tp.name.startswith("P."):
                tp["bake_enabled"] = state
        scene.bake_all_state = state  
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────   
classes = (
    OBJECT_OT_clear_path_visuals,
    OBJECT_OT_draw_teach_path,
    OBJECT_OT_tcp_move_up,
    OBJECT_OT_tcp_move_down,
    OBJECT_OT_tcp_delete,
    OBJECT_OT_reindex_tcp_points,
    OBJECT_OT_clear_all_tcp_points,
    OBJECT_OT_snap_goal_to_active,
    OBJECT_OT_focus_on_target,
    OBJECT_OT_export_teach_data,
    OBJECT_OT_export_joint_graph_csv,
    OBJECT_OT_clear_robot_system,
    OBJECT_OT_refresh_stage_sliders,
    OBJECT_OT_refresh_tcp_list,
    OBJECT_OT_toggle_path_visibility,
    OBJECT_OT_cycle_armature_set,
    OBJECT_OT_slide_robot_prev, 
    OBJECT_OT_slide_robot_next,
    OBJECT_OT_pick_object,
    OBJECT_OT_place_object,
    OBJECT_OT_clear_dynamic_parent,
    OBJECT_OT_tcp_bake_select_all,
    OBJECT_OT_toggle_tcp_bake_all,
)
