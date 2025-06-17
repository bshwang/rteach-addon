import bpy
from bpy.props import EnumProperty
from .robot_presets import ROBOT_CONFIGS

def get_robot_enum_items():
    return [(k, k, f"Load system: {k}") for k in ROBOT_CONFIGS.keys()]

class OBJECT_OT_import_robot_system(bpy.types.Operator):
    bl_idname = "object.import_robot_system"
    bl_label = "Import Robot System"

    system: EnumProperty(name="System", items=get_robot_enum_items())

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "system", text="Robot System")

    def execute(self, ctx):
        entry = ROBOT_CONFIGS.get(self.system)
        if not entry:
            self.report({'ERROR'}, f"Preset '{self.system}' not found")
            return {'CANCELLED'}

        p = ctx.scene.ik_motion_props
        p.robot_type = self.system
        p.preset_key = self.system  # JSON 대체 키, 유지해도 무방
        print(f"[INFO] Robot system set to '{self.system}'")

        # ▸ Armature auto-detect
        arm_name = entry.get("armature")
        if arm_name and arm_name in bpy.data.objects:
            p.armature = arm_name
            print(f"[INFO] Armature set to '{arm_name}'")

        # ▸ Setup object auto-linking
        setup_keys = ["goal_object", "base_object", "tcp_object", "ee_object"]
        for key in setup_keys:
            name = entry.get(key)
            if name and name in bpy.data.objects:
                setattr(p, key, bpy.data.objects[name])
                print(f"[INFO] {key} set to '{name}'")

        self.report({'INFO'}, f"Imported system: {self.system}")
        return {'FINISHED'}

    def invoke(self, ctx, event):
        return ctx.window_manager.invoke_props_dialog(self)

def register():
    bpy.utils.register_class(OBJECT_OT_import_robot_system)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_import_robot_system)
