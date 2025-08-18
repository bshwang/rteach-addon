import bpy
import os
import math

import rteach.core.core_ur as core_ur
import rteach.core.core_iiwa as core_iiwa

from bpy.utils import previews
from rteach.core.core import get_BONES, get_robot_config
from rteach.config.settings_static  import get_joint_limits
from rteach.core.robot_presets import ROBOT_CONFIGS

preview_collections = {}

def get_robot_preview(robot_key):
    if "robot_thumbs" not in preview_collections:
        print("[WARN] robot_thumbs not found — creating preview collection")
        pcoll = previews.new()
        preview_collections["robot_thumbs"] = pcoll
    else:
        pcoll = preview_collections["robot_thumbs"]

    config = ROBOT_CONFIGS.get(robot_key, {})
    thumb_rel_path = config.get("thumbnail", "")
    addon_dir = os.path.dirname(os.path.dirname(__file__))
    thumb_abs_path = os.path.join(addon_dir, thumb_rel_path)

    if not os.path.exists(thumb_abs_path):
        print(f"[ERROR] Thumbnail not found: {thumb_abs_path}")
        return None

    if robot_key not in pcoll:
        try:
            pcoll.load(robot_key, thumb_abs_path, 'IMAGE')
        except Exception as e:
            print(f"[FAIL] Failed to load thumbnail for {robot_key}: {e}")
            return None

    return pcoll.get(robot_key)

def get_stage_label_from_key(robot_key):
    return get_robot_config().get("label", robot_key)

def get_stage_joint_labels(robot_key):
    return [j.get("label", f"Joint {i+1}") for i, j in enumerate(get_robot_config().get("stage", []))]

def get_stage_joint_unit(robot_key, idx):
    j = get_robot_config().get("stage", [])[idx]
    return "mm" if j.get("unit") == "mm" else "deg"

def get_addon_prefs():
    return bpy.context.preferences.addons["rteach"].preferences

class OBJECT_OT_import_robot_from_grid(bpy.types.Operator):
    bl_idname = "object.import_robot_from_grid"
    bl_label = "Import Robot"
    bl_options = {'INTERNAL'}

    robot_key: bpy.props.StringProperty()

    def execute(self, context):
        bpy.ops.object.import_robot_system('EXEC_DEFAULT', system=self.robot_key)
        return {'FINISHED'}

class UI_UL_tcp_list(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if item and item.name:
            obj = bpy.data.objects.get(item.name)

            if obj and "bake_enabled" not in obj:
                obj["bake_enabled"] = True  

            idx = obj.get("index", "?") if obj else "?"
            label = f"{item.name} [#{idx}]"

            row = layout.row(align=True)
            if obj:
                row.prop(obj, '["bake_enabled"]', text="")  
            op = row.operator("object.tcp_list_select", text=label, emboss=False, icon='EMPTY_AXIS')
            op.index = index

class UI_UL_stage_tcp_list(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if item and item.name:
            obj = bpy.data.objects.get(item.name)
            base_name = obj.name if obj else item.name
            label = f"{base_name} [#{index}]"
            layout.label(text=label, icon='EMPTY_AXIS')

def draw_robot_slide_section(layout, keys, slide_prop_name):
    if not keys:
        return

    scene = bpy.context.scene
    idx = getattr(scene, slide_prop_name, 0) % len(keys)
    key = keys[idx]
    cfg = ROBOT_CONFIGS[key]
    thumb = get_robot_preview(key)

    label_text = cfg.get("display_name", key)

    row = layout.row(align=True)

    col_prev = row.column(align=True)
    col_prev.scale_y = 7
    op_prev = col_prev.operator("object.slide_robot_prev", text="", icon='TRIA_LEFT')
    op_prev.group = slide_prop_name

    col = row.column(align=True)
    box = col.box()
    if thumb:
        box.template_icon(icon_value=thumb.icon_id, scale=5.55)
    else:
        box.label(text="(No image)")

    op = col.operator("object.import_robot_from_grid", text=label_text, icon='IMPORT')
    op.robot_key = key

    col_next = row.column(align=True)
    col_next.scale_y = 7
    op_next = col_next.operator("object.slide_robot_next", text="", icon='TRIA_RIGHT')
    op_next.group = slide_prop_name

class VIEW3D_PT_ur_ik(bpy.types.Panel):
    bl_label = "Robot Motion Simulator"
    bl_idname = "VIEW3D_PT_ur_ik"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Robot Sim'

    def draw(self, ctx):
        L = self.layout
        self.draw_robot_selector(L, ctx)
        self.draw_setup_section(L, ctx)
        self.draw_stage_motion(L, ctx)
        self.draw_robot_motion(L, ctx)
        self.draw_io_section(L, ctx)

    def draw_robot_selector(self, L, ctx):
        p = ctx.scene.ik_motion_props
        config = ROBOT_CONFIGS.get(p.robot_type.lower(), {})

        box = L.box()
        box.label(text="Robot Selector")

        box.prop(p, "show_robot_library", text="Show Library", toggle=True, icon='ASSET_MANAGER')

        row = box.row(align=True)
        row.label(text=p.robot_type)
        row.operator("object.clear_robot_system", text="Clear", icon='TRASH')
        row.operator("object.sync_robot_type", text="Sync", icon='FILE_REFRESH')
        box.separator(factor=1.5)

        if not p.show_robot_library:
            return

        single_keys = []
        custom_keys = []
        for key, cfg in ROBOT_CONFIGS.items():
            stage = cfg.get("stage_joints", [])
            if stage:
                custom_keys.append(key)
            else:
                single_keys.append(key)

        box.label(text="Single Robots")
        draw_robot_slide_section(box, single_keys, "robot_slide_single_idx")

        box.separator(factor=1.5)

        box.label(text="Custom Systems")
        draw_robot_slide_section(box, custom_keys, "robot_slide_custom_idx")

    def draw_setup_section(self, layout, context):
        p = context.scene.ik_motion_props
        config = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        box = layout.box()

        row = box.row()
        icon = 'TRIA_DOWN' if p.show_setup else 'TRIA_RIGHT'
        row.prop(p, "show_setup", icon=icon, text="Setup", emboss=False)
        if not p.show_setup:
            return

        box.prop(p, "armature", text="Armature")
        box.prop(p, "base_object", text="Robot Base")
        box.prop(p, "ee_object", text="Flange")

        row = box.row(align=True)
        row.prop(p, "tcp_object", text="TCP")
        row.operator("object.setup_tcp_from_gizmo", text="", icon='EMPTY_ARROWS')

        icon = 'CHECKBOX_HLT' if p.show_workspace else 'CHECKBOX_DEHLT'
        row = box.row()
        row.prop(p, "show_workspace", text="Show Workspace", toggle=True, icon=icon)

        # Solver status (auto-fallback aware)
        prefs = bpy.context.preferences.addons['rteach'].preferences
        solver_str = ""
        rt = p.robot_type.lower()

        if "ur" in rt:
            mode = prefs.ur_solver_choice
            if mode == 'PY':
                active = 'PY'
            else:
                active = core_ur.UR_SOLVER_TYPE
            solver_str = f"UR Solver: {active}"
        elif "iiwa" in rt or "kuka" in rt:
            mode = prefs.kuka_solver_choice
            if mode == 'PY':
                active = 'PY'
            else:
                active = core_iiwa.KUKA_SOLVER_TYPE
            solver_str = f"KUKA Solver: {active}"

        if solver_str:
            box.label(text=solver_str, icon='INFO')

        if config.get("armature_sets"):
            row = box.row(align=True)
            row.label(text="Quick Switch:")
            row.operator("object.cycle_armature_set", text="⟳ Switch Robot")

    def draw_target_section(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_target else 'TRIA_RIGHT'
        row.prop(p, "show_target", icon=icon, text="Target Pose", emboss=False)
        if p.show_target and p.goal_object:
            row = box.row(align=True)
            row.prop(p, "goal_object", text="Target")
            row.operator("object.focus_on_target", text="", icon='RESTRICT_SELECT_OFF')
            row = box.row(align=True)
            row.prop(p.goal_object, 'location', index=0, text="X")
            row.prop(p.goal_object, 'rotation_euler', index=0, text="RX")
            row = box.row(align=True)
            row.prop(p.goal_object, 'location', index=1, text="Y")
            row.prop(p.goal_object, 'rotation_euler', index=1, text="RY")
            row = box.row(align=True)
            row.prop(p.goal_object, 'location', index=2, text="Z")
            row.prop(p.goal_object, 'rotation_euler', index=2, text="RZ")

    def draw_robot_motion(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_robot_motion else 'TRIA_RIGHT'
        row.prop(p, "show_robot_motion", icon=icon, text="Robot Motion", emboss=False)
        if not p.show_robot_motion:
            return

        self.draw_jog_section(box, ctx)
        self.draw_robot_tp_manager(box, ctx)
        self.draw_target_tools(box, ctx)  

    def draw_target_tools(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        box.label(text="🔸 Tools")

        row = box.row(align=True)
        row.operator("object.focus_on_target", text="Select",  icon='RESTRICT_SELECT_OFF')
        row.operator("object.snap_target_to_fk", text="to FK", icon='PIVOT_ACTIVE')
        row.operator("object.snap_goal_to_active", text="to Active", icon='PIVOT_ACTIVE')

        row = box.row(align=True)
        row.operator("object.preview_goal_pose", text="Preview", icon='HIDE_OFF')
        row.operator("object.record_goal_pose",  text="Record",  icon='REC')

        row = box.row(align=True)
        row.operator("object.update_tcp_pose",      text="Update",        icon='EXPORT')
        row.operator("object.update_all_tcp_poses", text="Recompute All", icon='FILE_REFRESH')
        row.operator("object.update_tp_from_jointpose", text="Update f/ Joint", icon='RECOVER_LAST')

        row = box.row(align=True)
        row.prop(p, "bake_start_frame", text="Start Frame")
        row.operator("object.bake_teach_sequence", text="Bake", icon='FILE_TICK')
        row.operator("object.clear_bake_keys",     text="",     icon='TRASH')

        row = box.row(align=True)
        row.operator("object.pick_object",           text="Pick",  icon='LINKED')
        row.operator("object.place_object",          text="Place", icon='UNLINKED')
        row.operator("object.clear_dynamic_parent",  text="",      icon='TRASH')

    def draw_stage_motion(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_stage_motion else 'TRIA_RIGHT'
        row.prop(p, "show_stage_motion", icon=icon, text="Stage Motion", emboss=False)
        if not p.show_stage_motion:
            return

        self.draw_stage_jog_section(box, ctx)
        self.draw_stage_tp_manager(box, ctx)

    def draw_jog_section(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        box.label(text="🔸 Jog Mode")

        if not p.robot_type:
            box.label(text="No robot selected")
            return

        jog = ctx.scene.jog_props
        bones = get_BONES()
        limits = get_joint_limits(p.robot_type)

        expected_dof = len(bones)
        actual_props = [k for k in dir(jog) if k.startswith("joint_")]
        if len(actual_props) < expected_dof:
            box.label(text="⚠️ Jog sliders not fully registered yet", icon='ERROR')
            return

        if not hasattr(jog, "joint_0"):
            box.label(text="⚠️ No robot system loaded", icon='ERROR')
            return

        for i, bn in enumerate(bones):
            row = box.row(align=True)
            min_deg, max_deg = limits[i]
            row.label(text=f"{bn}:")
            row.label(text=f"{min_deg:.0f}° ")
            sub = row.row()
            sub.scale_x = 1.5
            sub.prop(jog, f"joint_{i}", text="", slider=True)
            row.label(text=f"  {max_deg:.0f}°")

        box.separator()
        row = box.row(align=True)
        row.operator("object.keyframe_joint_pose", text="Keyframe", icon='KEY_HLT')
        row.operator("object.record_tcp_from_jog", text="Waypoint", icon='EMPTY_AXIS')
        row.operator("object.go_home_pose", text="Home", icon='HOME')

    def draw_robot_tp_manager(self, L, ctx):
        p = ctx.scene.ik_motion_props
        obj = p.selected_teach_point

        box = L.box()
        box.label(text="🔸 TP manager")

        row = box.row()
        list_col = row.column()
        list_col.template_list("UI_UL_tcp_list", "", p, "tcp_sorted_list", p, "tcp_list_index", rows=5)

        btn_col = row.column(align=True)
        btn_col.operator("object.refresh_tcp_list", text="", icon='FILE_REFRESH')
        btn_col.operator("object.tcp_move_up", text="", icon='TRIA_UP')
        btn_col.operator("object.tcp_move_down", text="", icon='TRIA_DOWN')
        btn_col.operator("object.tcp_delete", text="", icon='X').name = obj.name if obj else ""
        btn_col.operator("object.clear_all_tcp_points", text="", icon='TRASH')
        btn_col.operator("object.toggle_tcp_bake_all", text="", icon='CHECKBOX_HLT' if getattr(ctx.scene, "bake_all_state", False) else 'CHECKBOX_DEHLT')

        row = box.row(align=True)
        if obj:
            idx = obj.get("index", "?")
            row.label(text=f"Selected: {obj.name} [#{idx}]", icon='PINNED')
            row.operator("object.preview_tcp_prev_pose", text="", icon='BACK')
            row.operator("object.preview_tcp_next_pose", text="", icon='FORWARD')
        else:
            row.label(text="Selected: None", icon='PINNED')

        row = box.row(align=True)
        row.label(text="Goal label:")
        if obj:
            row.prop(obj, '["goal"]', text="")
        else:
            row.label(text="None")

        row = box.row(align=True)
        row.label(text="Motion:")
        if obj:
            row.operator("object.toggle_motion_type", text="", icon='FILE_REFRESH')
            row.label(text=obj.get("motion_type", "?"))
        else:
            row.label(text="None")

        row = box.row(align=True)
        row.label(text="Speed (mm/s):")
        if obj:
            row.prop(p, "sel_tcp_speed", text="", slider=True)
            row.operator("object.apply_global_speed", text="", icon='PASTEDOWN')
        else:
            row.label(text="None")

        row = box.row(align=True)
        row.label(text="Wait time (sec):")
        if obj:
            row.prop(p, "sel_tcp_wait", text="", slider=True)
            row.operator("object.apply_global_wait", text="", icon='PASTEDOWN')
        else:
            row.label(text="None")

        row = box.row()
        split = row.split(factor=0.45)
        split.label(text="IK Index:")
        if p.solutions:
            right = split.split(factor=0.35)
            right.prop(p, "solution_index_ui", text="")
            tail = right.split(factor=0.7)
            tail.label(text=f"/ {p.max_solutions}")
            if obj:
                tail.operator("object.apply_preview_pose", text="", icon='EXPORT')
        else:
            split.label(text="None")

        if "kuka" in p.armature.lower():
            row = box.row(align=True)
            row.label(text="R angle (q3)")
            sub = row.row()
            sub.scale_x = 1.2
            sub.prop(p, "fixed_q3_deg", text="", slider=True)

    def draw_bake_and_pick(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        box.label(text="🔸 Bake Motion")

        row = box.row(align=True)
        row.prop(p, "bake_start_frame", text="Start Frame")
        if "iiwa" in p.robot_type.lower():
            row.prop(p, "precise_linear", text="LIN se(3)", icon='CONSTRAINT', toggle=True)

        row = box.row(align=True)
        row.operator("object.bake_teach_sequence", text="Bake", icon='FILE_TICK')
        row.operator("object.clear_bake_keys", text="", icon='TRASH')

        box.separator()
        box.label(text="🔸 Pick / Place")

        row = box.row(align=True)
        row.operator("object.pick_object", text="Pick", icon='LINKED')
        row.operator("object.place_object", text="Place", icon='UNLINKED')
        row.operator("object.clear_dynamic_parent", text="", icon='TRASH')

    def draw_stage_tp_manager(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        box.label(text="🔸 Stage TP Manager")

        row = box.row()
        row.template_list("UI_UL_stage_tcp_list", "", p, "stage_tcp_sorted_list", p, "stage_tcp_list_index", rows=4)

        sel_item = p.stage_tcp_sorted_list[p.stage_tcp_list_index] if p.stage_tcp_sorted_list and p.stage_tcp_list_index >= 0 else None
        sel_name = sel_item.name if sel_item else ""
        sel_obj = bpy.data.objects.get(sel_name) if sel_name else None

        col = row.column(align=True)
        col.operator("object.refresh_stage_tcp_list", text="", icon='FILE_REFRESH')
        col.operator("object.stage_tcp_move_up", text="", icon='TRIA_UP')
        col.operator("object.stage_tcp_move_down", text="", icon='TRIA_DOWN')
        op = col.operator("object.delete_stage_tcp_point", text="", icon='X')
        op.name = sel_name
        col.operator("object.clear_all_stage_tcp_points", text="", icon='TRASH')

        row = box.row(align=True)
        if sel_obj:
            idx = sel_obj.get("index", "?")
            row.label(text=f"Selected: {sel_obj.name} [#{idx}]", icon='PINNED')
            row.operator("object.preview_stage_tcp_prev_pose", text="", icon='BACK')
            row.operator("object.preview_stage_tcp_next_pose", text="", icon='FORWARD')
        else:
            row.label(text="Selected: None", icon='PINNED')

        row = box.row(align=True)
        row.label(text="Goal label:")
        if sel_obj:
            row.prop(sel_obj, '["goal"]', text="")
        else:
            row.label(text="None")

        row = box.row(align=True)
        row.operator("object.update_selected_stage_tcp", text="Update", icon='EXPORT')

    def draw_stage_jog_section(self, layout, ctx):
        prefs = get_addon_prefs()
        if not prefs.show_stage:
            return
        p = ctx.scene.ik_motion_props
        props = ctx.scene.stage_props
        box = layout.box()
        box.label(text="🔸 Stage Jog Mode")

        config = ROBOT_CONFIGS.get(p.robot_type, {})
        joints = config.get("stage_joints", [])
        if not joints:
            box.label(text="This robot has no stage joints")
            return

        for joint in joints:
            key, label, unit, *_ = joint
            if hasattr(props, key):
                row = box.row(align=True)
                row.prop(props, key, text=label, slider=True)
                row.operator("object.focus_stage_joint", text="", icon='RESTRICT_SELECT_OFF').name = key
                op = row.operator("object.keyframe_stage_joint", text="", icon='KEY_HLT')
                op.name = key
                row.operator("object.clear_stage_joint_keys", text="", icon='TRASH').name = key
            else:
                box.label(text=f"⚠ Missing: {key}")

        row = box.row(align=True)
        row.operator("object.keyframe_all_stage_joints", text="Keyframe", icon='KEY_HLT')
        row.operator("object.add_stage_tcp_point",      text="Waypoint", icon='EMPTY_AXIS')
        row.operator("object.stage_home_preview",       text="Home",     icon='HOME')

    def draw_io_section(self, layout, context):
        prefs = get_addon_prefs()
        if not prefs.show_io:
            return
        p = context.scene.ik_motion_props

        box = layout.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_io else 'TRIA_RIGHT'
        row.prop(p, "show_io", icon=icon, text="Import / Export", emboss=False)
        if not p.show_io:
            return

        # Frame Range
        box.label(text="🔸 Frame Range")
        row = box.row()
        split = row.split(factor=0.8)
        sub = split.row(align=True)
        sub.enabled = not p.frame_all
        sub.prop(p, "frame_start", text="Start")
        sub.prop(p, "frame_end",   text="End")
        split.prop(p, "frame_all", text="All", toggle=True)

        # Joint Selection
        box.separator()
        box.label(text="🔸 Joint Selection")
        grid = box.grid_flow(columns=4, align=True)
        grid.prop(p, "export_robot_all", text="Robot")
        cfg = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        for i, sj in enumerate(cfg.get("stage_joints", [])):
            grid.prop(p, "show_plot_stage_joints", index=i, text=sj[1])

        # Plot Graph
        box.separator()
        box.label(text="🔸 Plot Graph")
        row = box.row()
        split = row.split(factor=0.7)
        try:
            import matplotlib.pyplot
            split.operator("rteach.show_joint_graph", text="Plot Graph", icon='IMAGE_DATA')
        except ImportError:
            split.enabled = False
            split.label(icon='ERROR', text="Plot Graph (requires matplotlib)")
        split.prop(p, "use_cycle", text="Cycle (%)")

        # Import Data (JSON)
        box.separator()
        box.label(text="🔸 Import Data")
        row = box.row(align=True)
        split = row.split(factor=0.85)
        split.prop(p, "import_teach_filename", text="")
        split.operator("object.import_teach_data", text="", icon='IMPORT')
        
        row = box.row(align=True)
        split = row.split(factor=0.85)
        split.prop(p, "import_joint_csv_filename", text="")
        split.operator("object.import_joint_csv", text="", icon='IMPORT')

        # PMBot Beta Import
        row = box.row(align=True)
        row.operator("object.import_pmbot_beta_data", text="Import PMBot Beta", icon='IMPORT')
        row = box.row(align=True)
        row.operator("object.bake_keyframes_from_pmbot_xml", text="Bake Keyframes from PMBot XML", icon='KEY_HLT')

        # Export Data
        box.separator()
        box.label(text="🔸 Export Data")
        row = box.row(align=True)
        split = row.split(factor=0.85)
        split.prop(p, "export_teach_filename", text="")
        split.operator("object.export_teach_data", text="", icon='EXPORT')

        row = box.row(align=True)
        split = row.split(factor=0.85)
        split.prop(p, "export_joint_csv_filename", text="")
        split.operator("object.export_joint_graph_csv", text="", icon='EXPORT')

classes = [
    UI_UL_tcp_list,
    UI_UL_stage_tcp_list,
    VIEW3D_PT_ur_ik,
    OBJECT_OT_import_robot_from_grid,
]
