import bpy
import math
import re

import numpy as np

from rteach.core.robot_presets import ROBOT_CONFIGS
from rteach.config.settings import delayed_workspace_update
from rteach.ext import dynamic_parent as dp
from rteach.core.core import get_tcp_object, get_best_ik_solution

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
        p.solutions = []
        p.max_solutions = 0
        p.current_index = 0
        p.solution_index_ui = 1
        return

    p.tcp_sorted_list.clear()
    for obj in sorted(coll.objects, key=lambda o: o.get("index", 9999)):
        if obj.name not in bpy.data.objects:
            continue
        if obj.get("stage"):
            continue
        item = p.tcp_sorted_list.add()
        item.name = obj.name

    p.tcp_list_index = min(p.tcp_list_index, len(p.tcp_sorted_list) - 1)

    if p.selected_teach_point and p.selected_teach_point.name in bpy.data.objects:
        obj = p.selected_teach_point

        T_goal = np.array(obj.matrix_world)
        q_ref = None
        if "joint_pose" in obj:
            try:
                q_ref = np.asarray(obj["joint_pose"], float)
            except:
                q_ref = None

        q_best, sols = get_best_ik_solution(p, T_goal, q_ref=q_ref)
        if sols:
            p.solutions = [list(map(float, s)) for s in sols]
            p.max_solutions = len(sols)

            if q_ref is not None and q_best is not None:
                def ang_diff(a, b):
                    return ((a - b + np.pi) % (2 * np.pi)) - np.pi
                def score(i):
                    return sum(abs(ang_diff(sols[i][j], q_best[j])) for j in range(len(q_best)))
                best_idx = int(np.argmin([score(i) for i in range(len(sols))]))
            else:
                best_idx = int(obj.get("solution_index", 0))
                best_idx = max(0, min(best_idx, len(sols) - 1))

            p.current_index = best_idx
            p.solution_index_ui = best_idx + 1
        else:
            p.solutions = []
            p.max_solutions = 0
            p.current_index = 0
            p.solution_index_ui = 1
    else:
        p.selected_teach_point = None
        p.solutions = []
        p.max_solutions = 0
        p.current_index = 0
        p.solution_index_ui = 1

    bpy.context.area.tag_redraw()

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
        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        p = ctx.scene.ik_motion_props
        prev_sel = p.selected_teach_point

        objs = [o for o in coll.objects if not o.get("stage")]
        if not objs:
            update_tcp_sorted_list()
            self.report({'INFO'}, "No Waypoints found")
            return {'FINISHED'}
        objs.sort(key=lambda o: (o.get("index", 1_000_000), o.name))

        name_counter = {}
        final_names = []
        for i, obj in enumerate(objs):
            goal = (obj.get("goal", "") or "").strip() or "GOAL"
            rk = (obj.get("robot_key", "") or "").lower()
            side = "L" if "left" in rk else ("R" if "right" in rk else "X")

            key = (goal, side)
            name_counter[key] = name_counter.get(key, 0) + 1
            seq = name_counter[key]
            final_names.append(f"{goal}_{side}_{seq:02d}")

            obj["index"] = i
            if "bake_enabled" not in obj:
                obj["bake_enabled"] = True

        for i, obj in enumerate(objs):
            obj.name = f"__TMP_RTCP__{i:04d}"
        for obj, new_name in zip(objs, final_names):
            obj.name = new_name

        update_tcp_sorted_list()

        if prev_sel:
            for i, it in enumerate(p.tcp_sorted_list):
                if it.name == prev_sel.name:
                    p.tcp_list_index = i
                    break

        self.report({'INFO'}, "TCP list refreshed, renamed, and re-indexed")
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
    OBJECT_OT_tcp_move_up,
    OBJECT_OT_tcp_move_down,
    OBJECT_OT_tcp_delete,
    OBJECT_OT_clear_all_tcp_points,
    OBJECT_OT_snap_goal_to_active,
    OBJECT_OT_focus_on_target,
    OBJECT_OT_clear_robot_system,
    OBJECT_OT_refresh_stage_sliders,
    OBJECT_OT_refresh_tcp_list,
    OBJECT_OT_cycle_armature_set,
    OBJECT_OT_slide_robot_prev, 
    OBJECT_OT_slide_robot_next,
    OBJECT_OT_pick_object,
    OBJECT_OT_place_object,
    OBJECT_OT_clear_dynamic_parent,
    OBJECT_OT_tcp_bake_select_all,
    OBJECT_OT_toggle_tcp_bake_all,
)
