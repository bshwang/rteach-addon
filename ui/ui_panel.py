import os
import bpy

from bpy.utils import previews
from rteach.core.core import get_BONES, get_joint_limits
from rteach.core.robot_presets import ROBOT_CONFIGS
from rteach.core.robot_state import get_armature_type

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
            idx = obj.get("index", "?") if obj else "?"
            label = f"{item.name} [#{idx}]"
            op = layout.operator("object.tcp_list_select", text=label, emboss=False, icon='EMPTY_AXIS')
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
        self.draw_pick_place_section(L, ctx)
        self.draw_jog_section(L, ctx)
        self.draw_stage_jog_section(L, ctx)
        self.draw_step1(L, ctx)
        self.draw_step2(L, ctx)
        self.draw_step3(L, ctx)
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

    def draw_setup_section(self, L, ctx):
        p = ctx.scene.ik_motion_props
        config = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        box = L.box()

        row = box.row()
        icon = 'TRIA_DOWN' if p.show_setup else 'TRIA_RIGHT'
        row.prop(p, "show_setup", icon=icon, text="Setup", emboss=False)

        if not p.show_setup:
            return

        box.prop(p, "armature", text="Armature")
        box.prop(p, "base_object", text="Robot Base")
        box.prop(p, "ee_object", text="Flange")
        box.prop(p, "tcp_object", text="TCP")

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
            box.prop(p, "goal_object", text="Target")
            row = box.row(align=True)
            row.prop(p.goal_object, 'location', index=0, text="X")
            row.prop(p.goal_object, 'rotation_euler', index=0, text="RX")
            row = box.row(align=True)
            row.prop(p.goal_object, 'location', index=1, text="Y")
            row.prop(p.goal_object, 'rotation_euler', index=1, text="RY")
            row = box.row(align=True)
            row.prop(p.goal_object, 'location', index=2, text="Z")
            row.prop(p.goal_object, 'rotation_euler', index=2, text="RZ")
            row = box.row(align=True)
            row.operator("object.focus_on_target", text="Select", icon='RESTRICT_SELECT_OFF')
            row.operator("object.snap_goal_to_active", text="Snap to Active", icon='PIVOT_ACTIVE')
            row = box.row(align=True)
            row.operator("object.snap_target_to_fk", text="Snap to FK", icon='CONSTRAINT')
            row.operator("object.setup_tcp_from_gizmo", text="Set as TCP", icon='EMPTY_ARROWS')

    def draw_pick_place_section(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_pick_place else 'TRIA_RIGHT'
        row.prop(p, "show_pick_place", icon=icon, text="Pick / Place", emboss=False)

        if not p.show_pick_place:
            return

        row = box.row(align=True)
        row.operator("object.pick_object", text="Pick", icon='LINKED')
        row.operator("object.place_object", text="Place", icon='UNLINKED')

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

        icon = 'CHECKBOX_HLT' if p.show_workspace else 'CHECKBOX_DEHLT'
        row = box.row()
        row.prop(p, "show_workspace", text="Show Workspace", toggle=True, icon=icon)

    def draw_step1(self, L, ctx):

        p = ctx.scene.ik_motion_props
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_step1 else 'TRIA_RIGHT'
        row.prop(p, "show_step1", icon=icon, text="Step 1: Teach Point", emboss=False)
        if not p.show_step1:
            return

        row = box.row(align=True)
        row.operator("object.teach_pose", text="Go To", icon='VIEW_CAMERA')
        row.prop(p, "auto_record", text="", toggle=True, icon='REC')

        if get_armature_type(p.robot_type) == "KUKA":
            row = box.row(align=True)
            row.label(text="R angle(q3)")
            sub = row.row()
            sub.scale_x = 1.2
            sub.prop(p, "fixed_q3_deg", text="", slider=True)

        split = box.split(factor=0.4, align=True)
        split.prop(p, "solution_index_ui", text="Index")

        right = split.split(factor=0.7, align=True)
        left_btns = right.row(align=True)
        left_btns.operator("object.cycle_pose_preview", text="◀").direction = 'PREV'
        left_btns.operator("object.cycle_pose_preview", text="▶").direction = 'NEXT'
        right.operator("object.apply_preview_pose", text="", icon='CHECKMARK')

    def draw_step2(self, L, ctx):
        p = ctx.scene.ik_motion_props
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_step2 else 'TRIA_RIGHT'
        row.prop(p, "show_step2", icon=icon, text="Step 2: Path & Editing", emboss=False)
        if not p.show_step2:
            return

        row = box.row(align=True)
        row.template_list("UI_UL_tcp_list", "", p, "tcp_sorted_list", p, "tcp_list_index")

        col = row.column(align=True)
        col.operator("object.tcp_move_up", text="", icon='TRIA_UP')
        col.operator("object.tcp_move_down", text="", icon='TRIA_DOWN')
        if p.selected_teach_point:
            op = col.operator("object.tcp_delete", text="", icon='X')
            op.name = p.selected_teach_point.name
        else:
            col.label(icon='BLANK1')
        col.operator("object.clear_all_tcp_points", text="", icon='TRASH')
        col.operator("object.refresh_tcp_list", text="", icon='FILE_REFRESH')
        col.operator("object.reindex_tcp_points", text="", icon='SORTALPHA')

        row = box.row(align=True)
        row.prop(p, "selected_teach_point", text="")
        row.operator("object.preview_tcp_prev", text="", icon='FRAME_PREV')
        row.operator("object.preview_tcp_next", text="", icon='FRAME_NEXT')
        row.operator("object.update_tcp_pose", text="Update WP", icon='EXPORT')
        row = box.row(align=True)
        
        row.operator("object.toggle_path_visibility", text="Show/Hide Path", icon='HIDE_OFF')
        row.operator("object.draw_teach_path", text="", icon='FILE_REFRESH')

        obj = p.selected_teach_point
        if obj:
            box.label(text="Path Interpolation")
            row = box.row(align=True)
            row.prop(p, "path_percent", text="Path %")
            row.operator("object.snap_gizmo_on_path", text="Move Gizmo")

    def draw_step3(self, L, ctx):

        p = ctx.scene.ik_motion_props
        obj = p.selected_teach_point
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_step3 else 'TRIA_RIGHT'
        row.prop(p, "show_step3", icon=icon, text="Step 3: Bake Motion", emboss=False)
        if not p.show_step3:
            return

        if obj:
            box.separator()
            box.label(text=f"Selected: {obj.name}", icon='PINNED')
            if obj.get("motion_type") in {"LINEAR", "JOINT"}:
                row = box.row(align=True)
                row.label(text=f"Motion Type: {obj['motion_type']}")
                row.operator("object.toggle_motion_type", text="", icon='FILE_REFRESH')

                row = box.row(align=True)
                row.prop(p, "sel_tcp_speed", text="Speed (mm/s)", slider=True)
                row.operator("object.apply_global_speed", text="", icon='PASTEDOWN')

                row = box.row(align=True)
                row.prop(p, "sel_tcp_wait", text="Wait (sec)", slider=True)
                row.operator("object.apply_global_wait", text="", icon='PASTEDOWN')
            else:
                box.label(text="< timing props missing – legacy point >", icon='ERROR')

        row = box.row(align=True)
        outer_split = row.split(factor=0.8, align=True)

        left = outer_split.split(factor=0.5, align=True)
        col1 = left.row(align=True)
        col1.enabled = not p.bake_all_tcp
        col1.prop(p, "bake_start_idx", text="Start idx")

        col2 = left.row(align=True)
        col2.enabled = not p.bake_all_tcp
        col2.prop(p, "bake_end_idx", text="End idx")

        outer_split.prop(p, "bake_all_tcp", text="All")

        row = box.row(align=True)
        row.prop(p, "bake_start_frame", text="Start Frame")

        row = box.row(align=True)
        row.operator("object.bake_teach_sequence", text="Bake", icon='FILE_TICK')
        row.operator("object.clear_bake_keys", text="", icon='TRASH')
        
        row = box.row(align=True)
        row.scale_x = 1.1
        row.label(text="High Precision Linear Mode")
        row.prop(p, "precise_linear", text="", icon='CONSTRAINT', toggle=True)

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
    
        from rteach.core.robot_presets import ROBOT_CONFIGS
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
        row.prop(p, "show_io", icon=icon, text="📂 Import / Export", emboss=False)

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
