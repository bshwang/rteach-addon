import bpy
import os
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

        # ① .blend 자산 불러오기
        addon_dir = os.path.dirname(__file__)
        blend_path = os.path.join(addon_dir, "robot_assets", f"{self.system}.blend")

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

        # ② 로봇 설정값 저장
        p = ctx.scene.ik_motion_props
        p.robot_type = self.system
        p.preset_key = self.system
        print(f"[INFO] Robot system set to '{self.system}'")

        # ③ Armature 자동 설정
        arm_name = entry.get("armature")
        if arm_name and arm_name in bpy.data.objects:
            p.armature = arm_name
            print(f"[INFO] Armature set to '{arm_name}'")

        # ④ Setup 오브젝트 지연 할당 (goal/base/tcp/ee)
        def delayed_setup_assignment():
            for key in ["goal_object", "base_object", "tcp_object", "ee_object"]:
                name = entry.get(key)
                if not name:
                    continue
                obj = bpy.data.objects.get(name)
                if not obj:
                    print(f"[WARN] {key} target '{name}' not found")
                    continue
                valid = any(c.name == "Setup" and obj.name in c.objects for c in bpy.data.collections)
                if valid:
                    setattr(p, key, obj)
                    print(f"[INFO] {key} set to '{name}'")
                else:
                    print(f"[SKIP] {key} '{name}' is not in 'Setup' collection")

            return None  # run once

        # 지연 등록: 컬렉션 연결 완료 후 실행
        bpy.app.timers.register(delayed_setup_assignment, first_interval=0.1)

        self.report({'INFO'}, f"Imported system: {self.system}")
        return {'FINISHED'}

    def invoke(self, ctx, event):
        return ctx.window_manager.invoke_props_dialog(self)

# ──────────────────────────────────────────────
def register():
    bpy.utils.register_class(OBJECT_OT_import_robot_system)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_import_robot_system)
