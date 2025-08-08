import bpy 
import math
import numpy as math

from mathutils import Matrix, Euler
from rteach.core.robot_state import get_armature_type
from rteach.core.core import (
    compute_base_matrix, compute_tcp_offset_matrix, get_forward_kinematics, get_BONES, get_AXES, get_robot_config
)
from rteach.ops.ops_teach_util import find_object_by_prefix
from rteach.config import settings_static as ss
from rteach.core.robot_presets import ROBOT_CONFIGS
    
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
        import math
        from mathutils import Matrix, Euler

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
        T = fk_func(q)

        print("[DEBUG] FK result matrix:")
        for row in T:
            print("   ", [round(v, 4) for v in row])

        T_full = compute_base_matrix(p) @ T @ compute_tcp_offset_matrix(p)
        goal.matrix_world = Matrix(T_full)

        pos = T_full[:3, 3] * 1000
        rot_euler = Matrix(T_full[:3, :3]).to_euler('XYZ')
        rot_deg = [math.degrees(a) for a in rot_euler]

        print(f"[DEBUG] Target snapped to FK position (mm): {pos.round(2)}")
        print(f"[DEBUG] FK orientation (deg): Rx={rot_deg[0]:.2f}°, Ry={rot_deg[1]:.2f}°, Rz={rot_deg[2]:.2f}°")

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
        stage = config.get("stage", [])

        if not stage:
            self.report({'ERROR'}, f"No stage joints found for robot_key: {robot_key}")
            return {'CANCELLED'}

        joint_values = []
        for j in stage:
            val = j.get("default", 0.0)
            joint_values.append(val)

        name_base = "STAGE"
        existing = [o.name for o in bpy.data.objects if o.name.startswith(name_base)]
        i = 1
        while f"{name_base}_{i:02}" in existing:
            i += 1
        name = f"{name_base}_{i:02}"

        obj = bpy.data.objects.new(name, None)
        obj.empty_display_type = 'PLAIN_AXES'
        obj.empty_display_size = 0.1
        obj["stage"] = True
        obj["robot_key"] = robot_key
        obj["joint_values"] = joint_values

        ctx.scene.collection.objects.link(obj)
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
        update_stage_tcp_sorted_list()

        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        name_counter = {}
        stage_objs = [obj for obj in coll.objects if obj.get("stage")]
        stage_objs = sorted(stage_objs, key=lambda o: o.get("index", 0))

        print("\n===== [STAGE TCP RENAME LOG] =====")
        for i, obj in enumerate(stage_objs):

            goal = obj.get("goal", "").strip() or "GOAL"
            robot_key = obj.get("robot_key", "").lower()

            if "left" in robot_key:
                side = "L"
            elif "right" in robot_key:
                side = "R"
            else:
                side = "S"

            key = (goal, side)
            name_counter.setdefault(key, 0)
            name_counter[key] += 1
            seq = name_counter[key]

            new_name = f"{goal}_{side}_{seq:02d}"
            obj["index"] = i

            if obj.name != new_name:
                print(f"[{i+1:02}] RENAME {obj.name} → {new_name}")
                obj.name = new_name
            else:
                print(f"[{i+1:02}] OK      {obj.name}")

        print("===== [STAGE TCP RENAME DONE] =====\n")

        self.report({'INFO'}, "Stage TCP list refreshed, renamed, and re-indexed")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_stage_tcp_move_up(bpy.types.Operator):
    bl_idname = "object.stage_tcp_move_up"
    bl_label = "Move Stage TCP Up"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        idx = p.stage_tcp_list_index
        if idx > 0:
            p.stage_tcp_sorted_list.move(idx, idx - 1)
            p.stage_tcp_list_index -= 1
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_stage_tcp_move_down(bpy.types.Operator):
    bl_idname = "object.stage_tcp_move_down"
    bl_label = "Move Stage TCP Down"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        idx = p.stage_tcp_list_index
        if idx < len(p.stage_tcp_sorted_list) - 1:
            p.stage_tcp_sorted_list.move(idx, idx + 1)
            p.stage_tcp_list_index += 1
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
)
