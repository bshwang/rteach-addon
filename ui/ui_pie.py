import bpy
from bpy.app.handlers import persistent

addon_keymaps = []

# ──────────────────────────────────────────────────────────────
# Pie Menu: Target Pose "Q"
# ──────────────────────────────────────────────────────────────
class VIEW3D_MT_robot_target_pie(bpy.types.Menu):
    bl_label = "Target Gizmo Menu"
    bl_idname = "VIEW3D_MT_robot_target_pie"

    def draw(self, context):
        pie = self.layout.menu_pie()
        pie.operator("object.focus_on_target", text="Select", icon='RESTRICT_SELECT_OFF')
        pie.operator("object.snap_target_to_fk", text="Snap to FK Position", icon='CONSTRAINT')
        pie.operator("object.setup_tcp_from_gizmo", text="Set as TCP offset", icon='EMPTY_ARROWS')
        pie.operator("object.snap_goal_to_active", text="Snap to Active", icon='PIVOT_ACTIVE')

# ──────────────────────────────────────────────────────────────
# Pie Menu: Jog Mode "Shift Q"
# ──────────────────────────────────────────────────────────────
class VIEW3D_MT_robot_jog_pie(bpy.types.Menu):
    bl_label = "Jog Mode"
    bl_idname = "VIEW3D_MT_robot_jog_pie"

    def draw(self, context):
        pie = self.layout.menu_pie()
        pie.operator("object.keyframe_joint_pose", text="Keyframe Pose", icon='KEY_HLT')
        pie.operator("object.record_tcp_from_jog", text="Record TCP", icon='EMPTY_AXIS')
        pie.operator("object.go_home_pose", text="Home Position", icon='HOME')

# ──────────────────────────────────────────────────────────────
# Pie Menu: Step 1 (Teach) "Ctrl Q"
# ──────────────────────────────────────────────────────────────
class VIEW3D_MT_robot_step1_pie(bpy.types.Menu):
    bl_label = "Teach Menu"
    bl_idname = "VIEW3D_MT_robot_step1_pie"

    def draw(self, context):
        pie = self.layout.menu_pie()
        pie.operator("object.teach_pose", text="Go To Pose", icon='VIEW_CAMERA')
        pie.operator("object.apply_preview_pose", text="Apply Pose", icon='KEY_HLT')
        pie.operator("object.cycle_pose_preview", text="Next Pose").direction = 'NEXT'
        pie.operator("object.cycle_pose_preview", text="Prev Pose").direction = 'PREV'

classes = (
    VIEW3D_MT_robot_target_pie,
    VIEW3D_MT_robot_jog_pie,
    VIEW3D_MT_robot_step1_pie,
)

# ──────────────────────────────
# KEYMAP REGISTRATION
# ──────────────────────────────

@persistent
def _delayed_keymap_registration(dummy=None):
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if not kc:
        print("[PieMenu] Keyconfig not ready, retrying...")
        return 0.5  # retry later

    km = kc.keymaps.get('3D View') or kc.keymaps.new(name='3D View', space_type='VIEW_3D')

    kmi1 = km.keymap_items.new('wm.call_menu_pie', 'Q', 'PRESS')
    kmi1.properties.name = "VIEW3D_MT_robot_target_pie"

    kmi2 = km.keymap_items.new('wm.call_menu_pie', 'Q', 'PRESS', shift=True)
    kmi2.properties.name = "VIEW3D_MT_robot_jog_pie"

    kmi3 = km.keymap_items.new('wm.call_menu_pie', 'Q', 'PRESS', ctrl=True)
    kmi3.properties.name = "VIEW3D_MT_robot_step1_pie"

    addon_keymaps.clear()
    addon_keymaps.extend([(km, [kmi1, kmi2, kmi3])])
    print("[PieMenu] Keymaps registered")
    return None  # stop retrying

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.app.handlers.load_post.append(_delayed_keymap_registration)

def unregister():
    for km, kmi_list in addon_keymaps:
        for kmi in kmi_list:
            km.keymap_items.remove(kmi)
    addon_keymaps.clear()

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

    if _delayed_keymap_registration in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_delayed_keymap_registration)
