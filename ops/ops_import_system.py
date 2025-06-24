import bpy
import os
from bpy.props import EnumProperty
from rteach.core.robot_presets import ROBOT_CONFIGS

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
        
        addon_root = os.path.dirname(os.path.dirname(__file__))  
        blend_path = os.path.join(addon_root, "resources", "robot_assets", f"{self.system}.blend")

        if not os.path.exists(blend_path):
            self.report({'ERROR'}, f".blend file not found: {blend_path}")
            return {'CANCELLED'}

        with bpy.data.libraries.load(blend_path, link=False) as (data_from, data_to):
            data_to.collections = [
                name for name in data_from.collections
                if name not in bpy.data.collections
            ]

        for coll in data_to.collections:
            if coll:
                ctx.scene.collection.children.link(coll)

        p = ctx.scene.ik_motion_props
        p.robot_type = self.system
        p.preset_key = self.system
        p.preview_only = False
        print(f"[INFO] Robot system set to '{self.system}'")

        arm_name = entry.get("armature")
        if arm_name and arm_name in bpy.data.objects:
            p.armature = arm_name
            print(f"[INFO] Armature set to '{arm_name}'")

        setup = entry.get("setup_objects", {})
        for key in ["goal", "base", "tcp", "ee"]:
            name = setup.get(key)
            if name and name in bpy.data.objects:
                obj = bpy.data.objects[name]
                in_setup = any(c.name == "Setup" and obj.name in c.objects for c in bpy.data.collections)
                if in_setup:
                    setattr(p, f"{key}_object", obj)
                    print(f"[INFO] {key}_object set to '{name}'")
                    
                else:
                    print(f"[SKIP] {key}_object '{name}' is not in 'Setup' collection")

        self.report({'INFO'}, f"Imported system: {self.system}")

        def delayed_sync():
            print("[DEBUG] Delayed sync_robot_type triggered")
            bpy.ops.object.sync_robot_type('INVOKE_DEFAULT')
            return None  

        bpy.app.timers.register(delayed_sync, first_interval=0.5)

        return {'FINISHED'}

    def invoke(self, ctx, event):
        return ctx.window_manager.invoke_props_dialog(self)

# ──────────────── Register
def register():
    bpy.utils.register_class(OBJECT_OT_import_robot_system)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_import_robot_system)
