import bpy
import os
import json
import math
from bpy.props import EnumProperty
from .settings import StageJogProperties, JogProperties, create_getter, create_setter
from .settings import register_stage_properties, re_register_jog_properties
from .robot_state import get_active_robot
from .core import get_AXES, get_joint_limits

ROBOT_PRESET_PATH = os.path.join(os.path.dirname(__file__), "presets", "robots.json")

def load_robot_presets():
    with open(ROBOT_PRESET_PATH, "r", encoding="utf-8") as f:
        return json.load(f)

def get_robot_enum_items():
    try:
        data = load_robot_presets()
        return [(k, k, f"Load system: {k}") for k in data.keys()]
    except Exception as e:
        print(f"[ERROR] Failed to load robot presets: {e}")
        return []

def update_jog_properties():
    from .core import get_joint_limits, get_AXES, get_BONES
    from .settings import create_getter, create_setter

    JogProperties.__annotations__.clear()

    limits = get_joint_limits()
    axes = get_AXES()
    bones = get_BONES()

    print(f"[DEBUG] DOF={len(limits)}, bones={bones}, axes={axes}")

    for i in range(len(limits)):
        deg_min, deg_max = limits[i]
        min_r = math.radians(deg_min)
        max_r = math.radians(deg_max)

        JogProperties.__annotations__[f"joint_{i}"] = bpy.props.FloatProperty(
            name=f"Joint {i}",
            subtype='ANGLE',
            unit='ROTATION',
            min=min_r,
            max=max_r,
            get=create_getter(i),
            set=create_setter(i),
        )
        
    bpy.context.area.tag_redraw()

class OBJECT_OT_import_robot_system(bpy.types.Operator):
    bl_idname = "object.import_robot_system"
    bl_label = "Import Robot System"

    system: EnumProperty(name="System", items=get_robot_enum_items())

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "system", text="Robot System")

    def execute(self, ctx):
        data = load_robot_presets()
        entry = data.get(self.system)
        if not entry:
            self.report({'ERROR'}, f"Preset '{self.system}' not found")
            return {'CANCELLED'}

        addon_dir = os.path.dirname(__file__)
        blend_path = os.path.join(addon_dir, "robot_assets", entry["blend"])

        if not os.path.exists(blend_path):
            self.report({'ERROR'}, f".blend file not found: {blend_path}")
            return {'CANCELLED'}

        # Load collections instead of individual objects
        with bpy.data.libraries.load(blend_path, link=False) as (data_from, data_to):
            data_to.collections = [name for name in data_from.collections if name not in bpy.data.collections]

        for coll in data_to.collections:
            if coll:
                ctx.scene.collection.children.link(coll)

        # Sync metadata
        p = ctx.scene.ik_motion_props
        robot_type_raw = entry.get("robot_type", "").strip().upper()
        print(f"[DEBUG] robot_type loaded from JSON: '{robot_type_raw}'")
        
        p.preset_key = self.system.strip()
        print(f"[DEBUG] preset_key set to '{p.preset_key}'")

        # Broad category check
        if not any(robot_type_raw.startswith(prefix) for prefix in ("UR", "KUKA")):
            self.report({'ERROR'}, f"Invalid robot_type '{robot_type_raw}'. Must start with 'UR' or 'KUKA'.")
            return {'CANCELLED'}

        # Full type is allowed (e.g., UR5E, UR16E, KUKA)
        p.robot_type = robot_type_raw

        # ▸ Armature auto-detect
        for arm_name in entry.get("armatures", []):
            if arm_name in bpy.data.objects:
                p.armature = arm_name
                break

        print(f"[DEBUG] system={self.system} → robot_type={p.robot_type}, armature={p.armature}")

        arm = bpy.data.objects.get(p.armature)
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, f"Valid armature '{p.armature}' not found")
            return {'CANCELLED'}

        # ▸ Axes override
        if "axes" in entry:
            from . import core_ur, core_iiwa
            if p.robot_type == "UR":
                core_ur.AXES = entry["axes"]
            elif p.robot_type == "KUKA":
                core_iiwa.AXES = entry["axes"]

        # ▸ Joint limits
        if "joint_limits_deg" in entry and entry["joint_limits_deg"]:
            from . import core
            core.CUSTOM_JOINT_LIMITS = entry["joint_limits_deg"]

        # ▸ Jog + Stage sliders
        update_jog_properties()
        register_stage_properties(entry)

        # ▸ Setup Objects (goal/base/tcp/ee)
        if "setup_objects" in entry:
            for key in ["goal", "base", "tcp", "ee"]:
                name = entry["setup_objects"].get(key)
                obj = bpy.data.objects.get(name)
                if name and obj:
                    setattr(p, f"{key}_object", obj)

        register_stage_properties(entry)
        re_register_jog_properties()
        update_jog_properties()

        self.report({'INFO'}, f"Imported and synced system: {self.system}")
        return {'FINISHED'}
    
    def invoke(self, ctx, event):
        return ctx.window_manager.invoke_props_dialog(self)

def register():
    bpy.utils.register_class(OBJECT_OT_import_robot_system)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_import_robot_system)

if __name__ == "__main__":
    register()
