import bpy 
import math
import numpy as math

from mathutils import Matrix, Euler, Vector, Quaternion
from rteach.core.robot_state import get_armature_type
from rteach.core.core import (
    compute_base_matrix, compute_tcp_offset_matrix, get_forward_kinematics, get_BONES, get_AXES, get_robot_config
)
from rteach.ops.ops_teach_util import find_object_by_prefix
from rteach.config import settings_static as ss
from rteach.core.robot_presets import ROBOT_CONFIGS

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

        def trigger_workspace_visibility_refresh():
            p = bpy.context.scene.ik_motion_props
            if not p.armature or not p.robot_type:
                return 0.1  

            if p.show_workspace:
                p.show_workspace = False
                p.show_workspace = True
            else:
                pass

            return None 

        bpy.app.timers.register(trigger_workspace_visibility_refresh, first_interval=0.1)

        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────
def set_workspace_visibility():
    p = bpy.context.scene.ik_motion_props
    robot_key = p.robot_type.lower()

    for ob in bpy.data.objects:
        if ob.name.startswith("Workspace_"):
            is_current = ob.name == f"Workspace_{robot_key}"
            visible = p.show_workspace and is_current
            ob.hide_viewport = not visible
            ob.hide_render = not visible
            ob.hide_set(not visible)

class OBJECT_OT_toggle_workspace_visibility(bpy.types.Operator):
    bl_idname = "object.toggle_workspace_visibility"
    bl_label = "Toggle Workspace Visibility"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        robot_key = p.robot_type.lower()
        config = ROBOT_CONFIGS.get(robot_key, {})

        arm_solver_map = config.get("armature_solver_map", {})
        active_solver_key = arm_solver_map.get(p.armature, robot_key)

        workspace_name_expected = f"Workspace_{active_solver_key}"
        tokens = p.armature.split("_")
        suffix = tokens[1] if len(tokens) >= 2 else ""
        if suffix.upper() in {"L", "R"}:
            workspace_name_expected += f"_{suffix.lower()}"

        for ob in bpy.data.objects:
            if ob.name.startswith("Workspace_"):
                is_current = ob.name.lower() == workspace_name_expected.lower()
                visible = p.show_workspace and is_current
                ob.hide_viewport = not visible
                ob.hide_render = not visible
                ob.hide_set(not visible)

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

        print("[DEBUG] Snap to FK initiated")
        print("[DEBUG] Armature:", p.armature)
        for i, bn in enumerate(bones):
            pb = arm.pose.bones.get(bn)
            if not pb:
                print(f"[WARN] Bone not found: {bn}")
                q.append(0.0)
                continue
            axis = axes[i]
            angle = getattr(pb.rotation_euler, axis)
            q.append(angle)
            print(f"[DEBUG] Bone {bn}: {axis} = {math.degrees(angle):.2f}°")

        fk_func = get_forward_kinematics()
        T_fk = fk_func(q)
        _print = lambda tag, M: print(tag, [ [round(float(v),6) for v in row] for row in M ])

        _print("[DEBUG] FK result matrix:", T_fk)

        T_base   = compute_base_matrix(p)
        T_offset = compute_tcp_offset_matrix(p)
        T_full   = T_base @ T_fk @ T_offset

        goal.matrix_world = Matrix(T_full)

        pos = T_full[:3, 3] * 1000
        rot_euler = Matrix(T_full[:3, :3]).to_euler('XYZ')
        rot_deg = [math.degrees(a) for a in rot_euler]

        print(f"[DEBUG] Target snapped to FK position (mm): {pos.round(2)}")
        print(f"[DEBUG] FK orientation (deg): Rx={rot_deg[0]:.2f}°, Ry={rot_deg[1]:.2f}°, Rz={rot_deg[2]:.2f}°")

        _print("[DEBUG] T_base:", T_base)
        _print("[DEBUG] T_offset:", T_offset)
        _print("[DEBUG] T_full = T_base @ T_fk @ T_offset:", T_full)

        cfg = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        arm_map = cfg.get("armature_solver_map", {})
        solver_key = arm_map.get(p.armature, "?")
        print(f"[DEBUG] solver_key={solver_key}")

        for sj in cfg.get("stage_joints", []):
            key, label, unit, _minv, _maxv, axis, joint_type = sj
            obj = bpy.data.objects.get(key)
            if not obj:
                print(f"[STAGE] {key}: <not found>")
                continue
            idx = {"x":0,"y":1,"z":2}[axis]
            if joint_type == "rotation":
                val = math.degrees(obj.rotation_euler[idx])
                u = "deg"
            else:
                val = obj.location[idx] * (1000.0 if unit == "mm" else 1.0)
                u = "mm" if unit == "mm" else unit
            print(f"[STAGE] {key}: {joint_type}.{axis} = {val:.3f} {u}")

        p.status_text = "Target snapped to FK"
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_add_stage_tcp_point(bpy.types.Operator):
    bl_idname = "object.add_stage_tcp_point"
    bl_label = "Add Stage TCP Point"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        robot_key = p.robot_type
        config = get_robot_config()
        stage_joints = config.get("stage_joints", [])

        if not stage_joints:
            self.report({'ERROR'}, f"No stage joints found for robot_key: {robot_key}")
            return {'CANCELLED'}

        teach = bpy.data.collections.get("Teach data")
        if not teach:
            teach = bpy.data.collections.new("Teach data")
            ctx.scene.collection.children.link(teach)

        joint_values = {}
        for key, label, unit, mn, mx, axis, jtype in stage_joints:
            objj = bpy.data.objects.get(key)
            idx = {"x":0,"y":1,"z":2}[axis.lower()]
            if objj:
                if jtype == "location":
                    val = float(objj.location[idx])
                else:
                    val = float(objj.rotation_euler[idx])
            else:
                mid = (mn + mx) / 2.0
                if unit == "mm":
                    val = float(mid / 1000.0)
                elif unit == "deg":
                    val = float(math.radians(mid))
                else:
                    val = float(mid)
            joint_values[key] = val

        side = "L" if "left" in robot_key.lower() else ("R" if "right" in robot_key.lower() else "S")
        goal = "GOAL"
        stage_objs = [o for o in teach.objects if o.get("stage")]
        seq = sum(
            1 for o in stage_objs
            if (o.get("goal", "GOAL") or "GOAL") == goal and
               ("L" if "left" in str(o.get("robot_key","")).lower()
                else "R" if "right" in str(o.get("robot_key","")).lower()
                else "S") == side
        ) + 1

        name = f"{goal}_{side}_{seq:02d}"
        while name in bpy.data.objects:
            seq += 1
            name = f"{goal}_{side}_{seq:02d}"

        next_idx = max([o.get("index", -1) for o in stage_objs] + [-1]) + 1

        obj = bpy.data.objects.new(name, None)
        obj.empty_display_type = 'PLAIN_AXES'
        obj.empty_display_size = 0.1
        obj["stage"] = True
        obj["robot_key"] = robot_key
        obj["joint_values"] = joint_values
        obj["goal"] = goal
        obj["index"] = next_idx
        obj.hide_viewport = True

        teach.objects.link(obj)

        bpy.ops.object.refresh_stage_tcp_list()

        p.stage_tcp_list_index = len(p.stage_tcp_sorted_list) - 1
        p.selected_stage_tcp = obj

        for area in ctx.screen.areas:
            if area.type in {'VIEW_3D', 'PROPERTIES'}:
                area.tag_redraw()

        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
def update_stage_tcp_sorted_list():
    p = bpy.context.scene.ik_motion_props
    coll = bpy.data.collections.get("Teach data")
    if not coll:
        p.stage_tcp_sorted_list.clear()
        p.selected_stage_tcp = None
        p.stage_tcp_list_index = -1
        return

    p.stage_tcp_sorted_list.clear()

    for obj in sorted(coll.objects, key=lambda o: o.get("index", 9999)):
        if obj.name not in bpy.data.objects:
            continue
        if not obj.get("stage"):
            continue  # Only include stage TCPs
        item = p.stage_tcp_sorted_list.add()
        item.name = obj.name

    p.stage_tcp_list_index = min(p.stage_tcp_list_index, len(p.stage_tcp_sorted_list) - 1)

    if p.selected_stage_tcp and p.selected_stage_tcp.name in bpy.data.objects:
        p.selected_stage_tcp = bpy.data.objects[p.selected_stage_tcp.name]
    else:
        p.selected_stage_tcp = None

    bpy.context.area.tag_redraw()

class OBJECT_OT_refresh_stage_tcp_list(bpy.types.Operator):
    bl_idname = "object.refresh_stage_tcp_list"
    bl_label = "Refresh Stage TCP List"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props

        prev_obj = None
        if p.stage_tcp_sorted_list and 0 <= p.stage_tcp_list_index < len(p.stage_tcp_sorted_list):
            prev_name = p.stage_tcp_sorted_list[p.stage_tcp_list_index].name
            prev_obj = bpy.data.objects.get(prev_name)

        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        name_counter = {}
        stage_objs = [obj for obj in coll.objects if obj.get("stage")]
        stage_objs = sorted(stage_objs, key=lambda o: o.get("index", 0))

        for i, obj in enumerate(stage_objs):
            goal = (obj.get("goal", "") or "GOAL").strip()
            robot_key = (obj.get("robot_key", "") or "").lower()
            side = "L" if "left" in robot_key else "R" if "right" in robot_key else "S"

            key = (goal, side)
            name_counter[key] = name_counter.get(key, 0) + 1
            seq = name_counter[key]

            if not obj.get("name_preserve", False):
                new_name = f"{goal}_{side}_{seq:02d}"
                if obj.name != new_name:
                    obj.name = new_name

            obj["index"] = i

        update_stage_tcp_sorted_list()

        if prev_obj and prev_obj.name in [it.name for it in p.stage_tcp_sorted_list]:
            for i, it in enumerate(p.stage_tcp_sorted_list):
                if it.name == prev_obj.name:
                    p.stage_tcp_list_index = i
                    break

        if hasattr(ctx, "area") and ctx.area:
            ctx.area.tag_redraw()

        self.report({'INFO'}, "Stage TCP list refreshed")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_stage_tcp_move_up(bpy.types.Operator):
    bl_idname = "object.stage_tcp_move_up"
    bl_label = "Move Stage TCP Up"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll:
            return {'CANCELLED'}

        stage_objs = [o for o in coll.objects if o.get("stage")]
        if not stage_objs:
            return {'CANCELLED'}

        sorted_objs = sorted(stage_objs, key=lambda o: o.get("index", 9999))
        i = p.stage_tcp_list_index
        if not (0 <= i < len(sorted_objs)):
            return {'CANCELLED'}
        if i == 0:
            return {'CANCELLED'}

        a = sorted_objs[i]
        b = sorted_objs[i - 1]
        ia = a.get("index", i)
        ib = b.get("index", i - 1)
        a["index"], b["index"] = ib, ia

        update_stage_tcp_sorted_list()
        p.stage_tcp_list_index = i - 1
        p.selected_stage_tcp = bpy.data.objects.get(sorted_objs[i - 1].name)
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_stage_tcp_move_down(bpy.types.Operator):
    bl_idname = "object.stage_tcp_move_down"
    bl_label = "Move Stage TCP Down"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll:
            return {'CANCELLED'}

        stage_objs = [o for o in coll.objects if o.get("stage")]
        if not stage_objs:
            return {'CANCELLED'}

        sorted_objs = sorted(stage_objs, key=lambda o: o.get("index", 9999))
        i = p.stage_tcp_list_index
        if not (0 <= i < len(sorted_objs)):
            return {'CANCELLED'}
        if i == len(sorted_objs) - 1:
            return {'CANCELLED'}

        a = sorted_objs[i]
        b = sorted_objs[i + 1]
        ia = a.get("index", i)
        ib = b.get("index", i + 1)
        a["index"], b["index"] = ib, ia

        update_stage_tcp_sorted_list()
        p.stage_tcp_list_index = i + 1
        p.selected_stage_tcp = bpy.data.objects.get(sorted_objs[i + 1].name)
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_delete_stage_tcp_point(bpy.types.Operator):
    bl_idname = "object.delete_stage_tcp_point"
    bl_label = "Delete Stage TCP Point"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        obj = bpy.data.objects.get(self.name)
        if not obj:
            self.report({'ERROR'}, f"Stage TCP '{self.name}' not found")
            return {'CANCELLED'}

        for coll in obj.users_collection:
            coll.objects.unlink(obj)

        bpy.data.objects.remove(obj, do_unlink=True)
        ctx.scene.ik_motion_props.status_text = f"Deleted stage TCP {self.name}"
        update_stage_tcp_sorted_list()
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_clear_all_stage_tcp_points(bpy.types.Operator):
    bl_idname = "object.clear_all_stage_tcp_points"
    bl_label = "Clear All Stage TCPs"

    def execute(self, ctx):
        stage_objs = [obj for obj in bpy.data.objects if obj.get("stage") is True]
        for obj in stage_objs:
            bpy.data.objects.remove(obj, do_unlink=True)
        return bpy.ops.object.refresh_stage_tcp_list()
    
# ──────────────────────────────────────────────────────────────  
def _apply_stage_tcp_values(robot_key: str, joint_values):
    cfg_key = (robot_key or bpy.context.scene.ik_motion_props.robot_type or "").lower()
    cfg = ROBOT_CONFIGS.get(cfg_key, {})
    stage = cfg.get("stage_joints", [])

    def _as_map(jv):
        if isinstance(jv, dict):
            return {k: float(jv[k]) for k in jv.keys()}
        if hasattr(jv, "keys") and hasattr(jv, "__getitem__"):
            return {k: float(jv[k]) for k in jv.keys()}
        return None

    m = _as_map(joint_values)

    for key, label, unit, mn, mx, axis, jtype in stage:
        obj = find_object_by_prefix(key)
        if not obj:
            continue
        v = m.get(key) if m and key in m else None
        if v is None:
            mid = (mn + mx) / 2.0
            v = mid/1000.0 if unit == "mm" else math.radians(mid) if unit == "deg" else mid
        idx = {"x":0,"y":1,"z":2}[axis.lower()]
        if jtype == "location":
            obj.location[idx] = float(v)
        else:
            obj.rotation_mode = 'XYZ'
            obj.rotation_euler[idx] = float(v)

    bpy.context.view_layer.update()

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_keyframe_all_stage_joints(bpy.types.Operator):
    bl_idname = "object.keyframe_all_stage_joints"
    bl_label = "Keyframe All Stage Joints"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        config = ROBOT_CONFIGS.get(p.robot_type, {})
        stage_joints = config.get("stage_joints", [])
        frame = ctx.scene.frame_current

        count = 0
        for sj in stage_joints:
            name, _, _, _, _, axis, jtype = sj
            obj = bpy.data.objects.get(name)
            if not obj:
                continue
            idx = {"x": 0, "y": 1, "z": 2}[axis.lower()]
            if jtype == "location":
                obj.keyframe_insert(data_path="location", frame=frame, index=idx)
            else:
                obj.rotation_mode = 'XYZ'
                obj.keyframe_insert(data_path="rotation_euler", frame=frame, index=idx)
            count += 1

        self.report({'INFO'}, f"Keyframed {count} stage joints at frame {frame}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_stage_home_preview(bpy.types.Operator):
    bl_idname = "object.stage_home_preview"
    bl_label = "Stage Home Preview"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        config = ROBOT_CONFIGS.get(p.robot_type, {})
        stage_joints = config.get("stage_joints", [])

        for sj in stage_joints:
            name, _, _, _, _, axis, jtype = sj
            obj = bpy.data.objects.get(name)
            if not obj:
                continue
            idx = {"x": 0, "y": 1, "z": 2}[axis.lower()]
            if jtype == "location":
                loc = list(obj.location)
                loc[idx] = 0.0
                obj.location = loc
            else:
                obj.rotation_mode = 'XYZ'
                rot = list(obj.rotation_euler)
                rot[idx] = 0.0
                obj.rotation_euler = rot

        ctx.view_layer.update()
        self.report({'INFO'}, "Stage moved to Home (all zeros)")
        return {'FINISHED'}
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_update_selected_stage_tcp(bpy.types.Operator):
    bl_idname = "object.update_selected_stage_tcp"
    bl_label = "Update Selected Stage TCP"

    name: bpy.props.StringProperty(default="")

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        obj = bpy.data.objects.get(self.name) if self.name else None
        if not obj:
            if p.stage_tcp_sorted_list and 0 <= p.stage_tcp_list_index < len(p.stage_tcp_sorted_list):
                obj = bpy.data.objects.get(p.stage_tcp_sorted_list[p.stage_tcp_list_index].name)
        if not obj:
            self.report({'ERROR'}, "No stage TCP selected")
            return {'CANCELLED'}

        cfg = ROBOT_CONFIGS.get(p.robot_type, {})
        stage = cfg.get("stage_joints", [])
        jv = {}
        for key, _, _, _, _, axis, jtype in stage:
            tgt = bpy.data.objects.get(key)
            if not tgt:
                continue
            idx = {"x": 0, "y": 1, "z": 2}[axis.lower()]
            val = tgt.location[idx] if jtype == "location" else tgt.rotation_euler[idx]
            jv[key] = float(val)

        obj["joint_values"] = jv
        self.report({'INFO'}, f"Updated {obj.name}")
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_clear_stage_joint_keys(bpy.types.Operator):
    bl_idname = "object.clear_stage_joint_keys"
    bl_label = "Clear Stage Joint Keys"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        obj = bpy.data.objects.get(self.name)
        if not obj:
            self.report({'ERROR'}, f"Joint '{self.name}' not found")
            return {'CANCELLED'}

        ad = obj.animation_data
        removed_fc = 0
        removed_nla = 0

        if ad:
            if ad.action:
                for fc in list(ad.action.fcurves):
                    ad.action.fcurves.remove(fc)
                    removed_fc += 1
                ad.action = None

            for tr in list(ad.nla_tracks):
                for _strip in list(tr.strips):
                    tr.strips.remove(_strip)
                obj.animation_data.nla_tracks.remove(tr)
                removed_nla += 1

        self.report({'INFO'}, f"Deleted keys for '{self.name}' (fcurves: {removed_fc}, nla_tracks: {removed_nla})")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_preview_stage_tcp_prev_pose(bpy.types.Operator):
    bl_idname = "object.preview_stage_tcp_prev_pose"
    bl_label = "Prev Stage TCP + Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        lst = p.stage_tcp_sorted_list
        if not lst:
            return {'CANCELLED'}
        if p.stage_tcp_list_index > 0:
            p.stage_tcp_list_index -= 1
            it = lst[p.stage_tcp_list_index]
            obj = bpy.data.objects.get(it.name)
            if obj:
                p.selected_stage_tcp = obj
                bpy.ops.object.preview_stage_tcp_pose('EXEC_DEFAULT')
        return {'FINISHED'}


class OBJECT_OT_preview_stage_tcp_next_pose(bpy.types.Operator):
    bl_idname = "object.preview_stage_tcp_next_pose"
    bl_label = "Next Stage TCP + Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        lst = p.stage_tcp_sorted_list
        if not lst:
            return {'CANCELLED'}
        if p.stage_tcp_list_index < len(lst) - 1:
            p.stage_tcp_list_index += 1
            it = lst[p.stage_tcp_list_index]
            obj = bpy.data.objects.get(it.name)
            if obj:
                p.selected_stage_tcp = obj
                bpy.ops.object.preview_stage_tcp_pose('EXEC_DEFAULT')
        return {'FINISHED'}

class OBJECT_OT_preview_stage_tcp_pose(bpy.types.Operator):
    bl_idname = "object.preview_stage_tcp_pose"
    bl_label = "Preview Stage Pose"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props

        sel = None
        if p.stage_tcp_sorted_list and 0 <= p.stage_tcp_list_index < len(p.stage_tcp_sorted_list):
            sel = p.stage_tcp_sorted_list[p.stage_tcp_list_index].name
        if not sel:
            self.report({'WARNING'}, "No stage TCP selected")
            return {'CANCELLED'}

        obj = bpy.data.objects.get(sel)
        if not obj:
            self.report({'ERROR'}, f"Stage TCP '{sel}' not found")
            return {'CANCELLED'}

        jv = obj.get("joint_values")
        if not jv:
            self.report({'WARNING'}, "No joint_values on selected stage TCP")
            return {'CANCELLED'}

        cfg = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        sj = cfg.get("stage_joints", [])
        for key, _, unit, _, _, axis, jtype in sj:
            val = jv.get(key, None) if hasattr(jv, "get") else None
            if val is None:
                continue
            target = bpy.data.objects.get(key)
            if not target:
                continue
            idx = {"x": 0, "y": 1, "z": 2}[axis.lower()]
            if jtype == "location":
                target.location[idx] = float(val)
            else:
                target.rotation_mode = 'XYZ'
                target.rotation_euler[idx] = float(val)

        ctx.view_layer.update()
        p.selected_stage_tcp = obj
        return {'FINISHED'}

class OBJECT_OT_update_tp_from_jointpose(bpy.types.Operator):
    bl_idname = "object.update_tp_from_jointpose"
    bl_label = "Update TP from JointPose"
    bl_options = {'REGISTER', 'UNDO'}

    update_all: bpy.props.BoolProperty(name="Update All", default=False)
    use_fk: bpy.props.BoolProperty(name="Use FK if available", default=True)
    also_refresh_lists: bpy.props.BoolProperty(name="Refresh Lists", default=True)
    try_update_all_tcp_poses: bpy.props.BoolProperty(name="Call update_all_tcp_poses()", default=False)

    def list16_to_matrix(self, L):
        return Matrix(((L[0], L[1], L[2], L[3]),
                       (L[4], L[5], L[6], L[7]),
                       (L[8], L[9], L[10], L[11]),
                       (L[12], L[13], L[14], L[15])))

    def build_matrix_from_locquat(self, lp, lq):
        T = Matrix.Translation(Vector((float(lp[0]), float(lp[1]), float(lp[2]))))
        Q = Quaternion((float(lq[0]), float(lq[1]), float(lq[2]), float(lq[3])))
        return T @ Q.to_matrix().to_4x4()

    def try_fk_local(self, robot_key: str, q_list: list):
        try:
            import importlib
            cand = [
                ("rteach.core_ur", "fk_ur"),
                ("rteach.core_ur", "forward_kinematics"),
                ("rteach.core.core_ur", "fk_ur"),
                ("rteach.core.core_ur", "forward_kinematics"),
            ]
            for mod_name, fn_name in cand:
                try:
                    mod = importlib.import_module(mod_name)
                    fn = getattr(mod, fn_name, None)
                    if callable(fn):
                        try:
                            M = fn(q_list, robot_key)
                        except TypeError:
                            M = fn(q_list)
                        if hasattr(M, "__getitem__"):
                            return Matrix(((M[0][0], M[0][1], M[0][2], M[0][3]),
                                           (M[1][0], M[1][1], M[1][2], M[1][3]),
                                           (M[2][0], M[2][1], M[2][2], M[2][3]),
                                           (0.0, 0.0, 0.0, 1.0)))
                except Exception as e:
                    print(f"[TP-UPD][FK] provider {mod_name}.{fn_name} failed: {e}")
        except Exception as e:
            print(f"[TP-UPD][FK] import failed: {e}")
        return None

    def execute(self, ctx):
        scene = ctx.scene
        p = scene.ik_motion_props

        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        if self.update_all:
            targets = [o for o in coll.objects if not o.get("stage")]
        else:
            targets = []
            if p.tcp_sorted_list and 0 <= p.tcp_list_index < len(p.tcp_sorted_list):
                name = p.tcp_sorted_list[p.tcp_list_index].name
                obj = bpy.data.objects.get(name)
                if obj and not obj.get("stage"):
                    targets.append(obj)

        if not targets:
            self.report({'WARNING'}, "No target TP found")
            return {'CANCELLED'}

        count = 0
        for obj in targets:
            base_name = obj.get("base_name", "")
            base = bpy.data.objects.get(base_name) if base_name else None
            if not base:
                print(f"[TP-UPD][WARN] Base not found for {obj.name}: {base_name}")
                continue

            T_local = None

            if self.use_fk:
                q = obj.get("joint_pose", None)
                if q and isinstance(q, (list, tuple)) and len(q) >= 6:
                    T_fk = self.try_fk_local(obj.get("robot_key", ""), list(map(float, q)))
                    if T_fk:
                        T_local = T_fk
                        print(f"[TP-UPD][FK] Using FK for {obj.name}")

            if T_local is None:
                L = obj.get("local_mx_import", None)
                if L and isinstance(L, (list, tuple)) and len(L) == 16:
                    T_local = self.list16_to_matrix(L)
                    print(f"[TP-UPD] Using stored local_mx_import for {obj.name}")

            if T_local is None:
                lp = obj.get("local_pos", None)
                lq = obj.get("local_quat", None)
                if lp and lq and len(lp) == 3 and len(lq) == 4:
                    T_local = self.build_matrix_from_locquat(lp, lq)
                    print(f"[TP-UPD] Using local_pos/local_quat for {obj.name}")

            if T_local is None:
                print(f"[TP-UPD][WARN] No local pose for {obj.name}")
                continue

            M_world = base.matrix_world @ T_local
            obj.matrix_world = M_world
            count += 1

            b = base.matrix_world.decompose()[0]
            t = M_world.decompose()[0]
            print(f"[TP-UPD] Updated {obj.name}  base=({b.x:.6f},{b.y:.6f},{b.z:.6f})  tcp=({t.x:.6f},{t.y:.6f},{t.z:.6f})")

        if self.also_refresh_lists:
            try:
                bpy.ops.object.refresh_tcp_list()
            except Exception as e:
                print(f"[TP-UPD][WARN] refresh_tcp_list failed: {e}")

        if self.try_update_all_tcp_poses:
            try:
                bpy.ops.object.update_all_tcp_poses()
            except Exception as e:
                print(f"[TP-UPD][WARN] update_all_tcp_poses skipped/failed: {e}")

        self.report({'INFO'}, f"Updated {count} TP(s)")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
classes = (
    OBJECT_OT_keyframe_joint_pose,
    OBJECT_OT_go_home_pose,
    OBJECT_OT_setup_tcp_from_gizmo,
    OBJECT_OT_sync_robot_type,
    OBJECT_OT_toggle_workspace_visibility,
    OBJECT_OT_keyframe_stage_joint,
    OBJECT_OT_focus_stage_joint,
    OBJECT_OT_snap_target_to_fk,
    OBJECT_OT_add_stage_tcp_point,
    OBJECT_OT_refresh_stage_tcp_list,
    OBJECT_OT_stage_tcp_move_up,
    OBJECT_OT_stage_tcp_move_down,
    OBJECT_OT_delete_stage_tcp_point,
    OBJECT_OT_clear_all_stage_tcp_points,
    OBJECT_OT_keyframe_all_stage_joints,
    OBJECT_OT_stage_home_preview,
    OBJECT_OT_update_selected_stage_tcp,
    OBJECT_OT_clear_stage_joint_keys,
    OBJECT_OT_preview_stage_tcp_prev_pose,
    OBJECT_OT_preview_stage_tcp_next_pose,
    OBJECT_OT_preview_stage_tcp_pose,
    OBJECT_OT_update_tp_from_jointpose,
)
