# ui_panel.py 
import bpy
from .settings import IKMotionProperties, JogProperties
from .robot_state import get_active_robot
from .core import get_BONES, get_armature_bones, get_joint_limits

class UI_UL_tcp_list(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if item and item.name:
            op = layout.operator("object.tcp_list_select", text=item.name, emboss=False, icon='EMPTY_AXIS')
            op.index = index

class VIEW3D_PT_ur_ik(bpy.types.Panel):
    bl_label = "Robot Motion Simulator"
    bl_idname = "VIEW3D_PT_ur_ik"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'IK Solver'

    def draw(self, ctx):
        p = ctx.scene.ik_motion_props
        selected = ctx.view_layer.objects.active

        L = self.layout
        L.use_property_split = True
        L.use_property_decorate = False
        
        # ▶ Robot Type Selection
        row = L.row(align=True)
        row.prop(p, "robot_type", text="")
        row.operator("object.sync_robot_type", text="Sync", icon='FILE_REFRESH')

        # ▶ Setup
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_setup else 'TRIA_RIGHT'
        row.prop(p, "show_setup", icon=icon, text="Setup", emboss=False)
        if p.show_setup:
            box.prop(p, "armature", text="Armature")
            box.prop(p, "base_object", text="Robot Base")
            box.prop(p, "ee_object", text="Flange")
            box.prop(p, "tcp_object", text="TCP")

        # ▶ Target Pose
        L.separator()
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_target else 'TRIA_RIGHT'
        row.prop(p, "show_target", icon=icon, text="Target Pose", emboss=False)
        if p.show_target:
            box.prop(p, "goal_object", text="Target")
            if p.goal_object:
                col = box.column(align=True)
                col.prop(p.goal_object, 'location', index=0, text="X")
                col.prop(p.goal_object, 'location', index=1, text="Y")
                col.prop(p.goal_object, 'location', index=2, text="Z")
                col.prop(p.goal_object, 'rotation_euler', index=0, text="RX")
                col.prop(p.goal_object, 'rotation_euler', index=1, text="RY")
                col.prop(p.goal_object, 'rotation_euler', index=2, text="RZ")
                box.operator("object.focus_on_target", text="Select Target", icon='RESTRICT_SELECT_OFF')
                box.operator("object.snap_goal_to_active", text="Snap Target to Selected", icon='PIVOT_ACTIVE')

        # ── Jog Mode ──
        L.separator()
        box_j = L.box()
        row = box_j.row()
        icon = 'TRIA_DOWN' if p.show_jog else 'TRIA_RIGHT'
        row.prop(p, "show_jog", icon=icon, text="Jog Mode", emboss=False)
        if p.show_jog:
            jog = ctx.scene.jog_props
            bones = get_armature_bones()
            limits = get_joint_limits()
            for i, bn in enumerate(bones):
                row = box_j.row(align=True)
                min_deg, max_deg = limits[i]
                row.label(text=f"{bn}:")
                row.label(text=f"{min_deg:.0f}° ")
                sub = row.row()
                sub.scale_x = 1.5
                sub.prop(jog, f"joint_{i}", text="", slider=True)
                row.label(text=f"  {max_deg:.0f}°")

        # ▶ Sequence Mode
        L.separator()
        box = L.box()
        row = box.row()
        icon = 'TRIA_DOWN' if p.show_teach else 'TRIA_RIGHT'
        row.prop(p, "show_teach", icon=icon, text="Sequence Mode", emboss=False)
        
        if p.show_teach and p.status_text:
            if "fail" in p.status_text.lower():
                alert_box = box.box()
                alert_box.alert = True
                alert_box.label(text=f"⚠ {p.status_text.upper()} ⚠", icon='CANCEL')
            else:
                box.label(text=p.status_text, icon='INFO')

        if p.show_teach:
            box1 = box.box()
            box1.label(text="Step 1: Teach Pose")
            
            if p.robot_type == "KUKA":
                row = box1.row(align=True)
                row.prop(p, "fixed_q3_deg", text="R angle(q3)", slider=True)

            row = box1.row(align=True)
            row.operator("object.teach_pose", text="Go To", icon='VIEW_CAMERA')
            row.prop(p, "auto_record", text="Record", toggle=True, icon='REC')

            pose_box = box1.box()
            pose_box.label(text=f"Pose {p.current_index + 1}/{len(p.solutions)}")
            row = pose_box.row(align=True)
            row.prop(p, "solution_index_ui", text="Pose Index")
            row.operator("object.cycle_pose_preview", text="◀").direction = 'PREV'
            row.operator("object.cycle_pose_preview", text="▶").direction = 'NEXT'
            pose_box.prop(p, "use_last_pose", text="Keep Last Pose")
            row = pose_box.row(align=True)
            row.operator("object.apply_preview_pose", text="Apply Pose", icon='KEY_HLT')

            # ▶ Step 2
            box2 = box.box()
            box2.label(text="Step 2: Path & Editing")
            row = box2.row(align=True)
            row.operator("object.reindex_tcp_points", text="Reindex", icon='SORTALPHA')
            row.operator("object.clear_all_tcp_points", text="Clear All TCPs", icon='TRASH')
            row = box2.row(align=True)
            row.operator("object.draw_teach_path", text="Show Path", icon='OUTLINER_OB_CURVE')
            row.operator("object.clear_path_visuals", text="Hide Path", icon='HIDE_OFF')

            row = box2.row()
            row.template_list("UI_UL_tcp_list", "", p, "tcp_sorted_list", p, "tcp_list_index")

            row = box2.row(align=True)
            row.operator("object.tcp_move_up", text="", icon='TRIA_UP')
            row.operator("object.tcp_move_down", text="", icon='TRIA_DOWN')
            row.operator("object.tcp_delete", text="", icon='X')  
            row.operator("object.update_tcp_pose", text="Pose Update", icon='EXPORT')

            row = box2.row(align=True)
            row.prop(p, "selected_teach_point", text="Selected")
            row.operator("object.preview_tcp_prev", text="", icon='FRAME_PREV')
            row.operator("object.preview_tcp_next", text="", icon='FRAME_NEXT')
            row = box2.row(align=True)
            row.operator("object.export_teach_data", text="Export Teach Data", icon='EXPORT')

            obj = p.selected_teach_point
            if obj:
                row = box2.row(align=True)
                box2.label(text="Path Interpolation")
                row = box2.row(align=True)
                row.prop(p, "path_percent", text="Path %")
                row.operator("object.snap_gizmo_on_path", text="Move Gizmo")

            # ▶ Step 3
            box3 = box.box()
            box3.label(text="Step 3: Bake Motion")
            box3.separator()
            row = box3.row(align=True)
            row.prop(ctx.scene, "frame_current", text="Start Frame")
            row.operator("screen.frame_jump", text="", icon='REW').end = False
            row.operator("screen.frame_jump", text="", icon='FF').end = True
            row.operator("screen.keyframe_jump", text="", icon='PREV_KEYFRAME').next = False
            row.operator("screen.keyframe_jump", text="", icon='NEXT_KEYFRAME').next = True

            tcp_list = [o for o in bpy.data.objects if o.name.startswith("P.")]

            box3.label(text="Bake Range (TCP)", icon='BORDERMOVE')

            start_idx = p.bake_start_tcp.get("index", -1) if p.bake_start_tcp else -1
            end_idx   = p.bake_end_tcp.get("index", -1) if p.bake_end_tcp else -1

            if start_idx >= 0 and end_idx >= 0:
                if start_idx > end_idx:
                    box3.label(text=f"⚠ Start TCP (P.{start_idx:03}) comes after End TCP (P.{end_idx:03})", icon='ERROR')
                    box3.label(text="Will auto-correct to ascending order", icon='SORTSIZE')
                else:
                    box3.label(text=f"P.{start_idx:03} → P.{end_idx:03}", icon='SORTSIZE')

            row = box3.row(align=True)
            row.label(text="Start")
            row.prop_search(p, "bake_start_tcp", bpy.data, "objects", text="")
            row = box3.row(align=True)
            row.label(text="End")
            row.prop_search(p, "bake_end_tcp", bpy.data, "objects", text="")

            row = box3.row(align=True)
            row.operator("object.bake_teach_sequence", text="Bake", icon='FILE_TICK')
            row.operator("object.clear_bake_keys", text="", icon='TRASH')

            if obj:
                box3.separator()
                box3.label(text=f"Selected: {obj.name}", icon='PINNED')

                if obj.get("motion_type") in {"LINEAR", "JOINT"}:

                    row = box3.row(align=True)
                    row.label(text=f"Motion Type: {obj['motion_type']}")
                    row.operator("object.toggle_motion_type", text="", icon='FILE_REFRESH')

                    # ✅ speed 
                    row = box3.row(align=True)
                    row.prop(obj, '["speed"]', text="Speed (mm/s)", slider=True)
                    row.operator("object.apply_global_speed", text="", icon='PASTEDOWN')

                    # ✅ wait_time_sec 
                    row = box3.row(align=True)
                    row.prop(obj, '["wait_time_sec"]', text="Wait (sec)", slider=True)
                    row.operator("object.apply_global_wait", text="", icon='PASTEDOWN')
                else:
                    box3.label(text="< timing props missing – legacy point >", icon='ERROR')
                    
            # ▶ Linear Motion option
            box3.separator()
            row = box3.row(align=True)
            row.scale_x = 1.1
            row.label(text="High Precision Linear Mode")
            row.prop(p, "precise_linear", text="", icon='CONSTRAINT', toggle=True)
            
classes = [
    UI_UL_tcp_list,
    VIEW3D_PT_ur_ik,
]