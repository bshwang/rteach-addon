import os
import bpy

from bpy.utils import previews
from rteach.core.core import get_BONES, get_joint_limits
from rteach.core.robot_presets import ROBOT_CONFIGS
import rteach.core.core_ur as core_ur
import rteach.core.core_iiwa as core_iiwa

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
        p = ctx.scene.ik_motion_props

        self.draw_robot_selector(L, ctx)
        self.draw_setup_section(L, ctx)
        self.draw_target_section(L, ctx)
        self.draw_jog_section(L, ctx)
        self.draw_stage_jog_section(L, ctx)
        self.draw_tcp_manager(L, ctx)
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

    def draw_jog_section(self, L, ctx):

        p = ctx.scene.ik_motion_props
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_jog else 'TRIA_RIGHT'
        row.prop(p, "show_jog", icon=icon, text="Robot Jog Mode", emboss=False)

        if not p.robot_type:
            box.label(text="No robot selected")
            return

        if not p.show_jog:
            return

        jog = ctx.scene.jog_props
        bones = get_BONES()
        limits = get_joint_limits()
        
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

    def draw_tcp_manager(self, L, ctx):
        p = ctx.scene.ik_motion_props
        obj = p.selected_teach_point
        icon = 'TRIA_DOWN' if p.show_teach else 'TRIA_RIGHT'

        box = L.box()
        row = box.row()
        row.prop(p, "show_teach", icon=icon, text="Motion Teaching", emboss=False)

        if not p.show_teach:
            return

        box.label(text="🔸 Waypoint manager")

        row = box.row()

        list_col = row.column()
        list_col.template_list("UI_UL_tcp_list", "", p, "tcp_sorted_list", p, "tcp_list_index", rows=6)

        btn_col = row.column(align=True)
        btn_col.operator("object.refresh_tcp_list", text="", icon='FILE_REFRESH')
        btn_col.operator("object.reindex_tcp_points", text="", icon='SORTALPHA')
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

        # Motion
        row = box.row(align=True)
        row.label(text="Motion:")
        if obj:
            row.operator("object.toggle_motion_type", text="", icon='FILE_REFRESH')
            motion = obj.get("motion_type", "?")
            row.label(text=motion)
        else:
            row.label(text="None")

        # Speed
        row = box.row(align=True)
        row.label(text="Speed (mm/s):")
        if obj:
            row.prop(p, "sel_tcp_speed", text="", slider=True)
            row.operator("object.apply_global_speed", text="", icon='PASTEDOWN')
        else:
            row.label(text="None")

        # Wait
        row = box.row(align=True)
        row.label(text="Wait time (sec):")
        if obj:
            row.prop(p, "sel_tcp_wait", text="", slider=True)
            row.operator("object.apply_global_wait", text="", icon='PASTEDOWN')
        else:
            row.label(text="None")

        # IK Index (based on solution list)
        row = box.row(align=True)
        row.label(text="IK Index:")
        if p.solutions:
            row.prop(p, "solution_index_ui", text="")
            if obj:
                row.operator("object.apply_preview_pose", text="", icon='EXPORT')
        else:
            row.label(text="None")
            
        if "iiwa" in p.armature.lower():
            row = box.row(align=True)
            row.label(text="R angle (q3)")
            sub = row.row()
            sub.scale_x = 1.2
            sub.prop(p, "fixed_q3_deg", text="", slider=True)

        # Target Position (always visible)
        box.separator()
        box.label(text="🔸 Target Position")

        row = box.row(align=True)
        row.operator("object.focus_on_target", text="Select", icon='RESTRICT_SELECT_OFF')
        row.operator("object.snap_target_to_fk", text="to FK", icon='PIVOT_ACTIVE')
        row.operator("object.snap_goal_to_active", text="to Active", icon='PIVOT_ACTIVE')

        row = box.row(align=True)
        row.operator("object.preview_goal_pose", text="Preview", icon='HIDE_OFF')
        row.operator("object.record_goal_pose",  text="Record",  icon='REC')
        row.operator("object.update_tcp_pose", text="Update", icon='EXPORT')

        if obj:
            next_idx = obj.get("index", 0) + 1
            coll = bpy.data.collections.get("Teach data")
            if coll:
                tps = sorted(
                    (o for o in coll.objects if o.name.startswith("P.")),
                    key=lambda o: o.get("index", 9999)
                )
                next_point = next(
                    (tp for tp in tps if tp.get("index") == next_idx),
                    None
                )
                if next_point:
                    row = box.row(align=True)
                    row.label(text=f"Path {obj.name} → {next_point.name}")
                    row.prop(p, "path_percent", text="", slider=True)
                    row.operator(
                        "object.snap_gizmo_on_path",
                        text="",
                        icon='RESTRICT_SELECT_OFF'
                    )

        box.separator()
        box.label(text="🔸 Bake Motion")

        row = box.row(align=True)
        row.prop(p, "bake_start_frame", text="Start Frame")
        row.prop(p, "precise_linear", text="LIN se(3)", icon='CONSTRAINT', toggle=True)

        row = box.row(align=True)
        row.operator("object.bake_teach_sequence", text="Bake", icon='FILE_TICK')
        row.operator("object.clear_bake_keys", text="", icon='TRASH')

        row = box.row(align=True)
        row.operator("object.draw_teach_path", text="Draw Path", icon='TRACKING_FORWARDS')
        row.operator("object.toggle_path_visibility", text="Show/Hide Path", icon='HIDE_OFF')

        box.separator()
        box.label(text="🔸 Pick / Place")

        row = box.row(align=True)
        row.operator("object.pick_object", text="Pick", icon='LINKED')
        row.operator("object.place_object", text="Place", icon='UNLINKED')
        row.operator("object.clear_dynamic_parent", text="", icon='TRASH')

    def draw_stage_jog_section(self, layout, ctx):
        prefs = get_addon_prefs()
        if not prefs.show_stage:
            return
        p = ctx.scene.ik_motion_props
        props = ctx.scene.stage_props
        box = layout.box()
    
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_stage else 'TRIA_RIGHT'
        row.prop(p, "show_stage", icon=icon, text="Stage Jog Mode", emboss=False)
    
        if not p.show_stage:
            return
    
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
                op = row.operator("object.keyframe_stage_joint", text="", icon='KEY_HLT')
                op.name = key
                row.operator("object.focus_stage_joint", text="", icon='RESTRICT_SELECT_OFF').name = key
            else:
                box.label(text=f"⚠ Missing: {key}")
            
    def draw_io_section(self, L, ctx):
        prefs = get_addon_prefs()
        if not prefs.show_io:
            return
        p = ctx.scene.ik_motion_props
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if getattr(p, "show_io", True) else 'TRIA_RIGHT'
        row.prop(p, "show_io", icon=icon, text="Import / Export", emboss=False)

        if not p.show_io:
            return

        grid = box.grid_flow(columns=4, align=True)

        config = ROBOT_CONFIGS.get(p.robot_type, {})
        joint_labels = [f"j{i+1}" for i in range(len(config.get("axes", [])))]
        stage_labels = [j[1] for j in config.get("stage_joints", [])]

        for i, label in enumerate(joint_labels):
            if i < len(p.show_plot_joints):
                grid.prop(p, 'show_plot_joints', index=i, text=label)
        for i, label in enumerate(stage_labels):
            idx = len(joint_labels) + i
            if idx < len(p.show_plot_joints):
                grid.prop(p, 'show_plot_joints', index=idx, text=label)

        box.operator("object.export_joint_graph_csv", text="Export Joint CSV", icon='EXPORT')
        box.operator("object.export_teach_data", text="Export Teach Data (.json)", icon='EXPORT')

classes = [
    UI_UL_tcp_list,
    VIEW3D_PT_ur_ik,
    OBJECT_OT_import_robot_from_grid,
]
