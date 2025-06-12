import bpy import os import json import math from bpy.props import EnumProperty from .settings import StageJogProperties, JogProperties, get_joint_limits, get_AXES, create_getter, create_setter from .robot_state import get_active_robot

ROBOT_PRESET_PATH = os.path.join(os.path.dirname(file), "presets", "robots.json")

def load_robot_presets(): with open(ROBOT_PRESET_PATH, "r", encoding="utf-8") as f: return json.load(f)

def get_robot_enum_items(): try: data = load_robot_presets() return [(k, k, f"Load system: {k}") for k in data.keys()] except Exception as e: print(f"[ERROR] Failed to load robot presets: {e}") return []

def register_stage_properties(preset): annotations = {} for item in preset.get("stage_joints", []): name = item["name"] label = item.get("label", name) axis = item["axis"].lower() is_angle = item.get("unit") in {"deg", "rad", "ROTATION"}

min_val = item["min"] / 1000.0 if item["unit"] == "mm" else item["min"]
    max_val = item["max"] / 1000.0 if item["unit"] == "mm" else item["max"]

    def make_get(name=name, axis=axis):
        def _get(self):
            obj = bpy.data.objects.get(name)
            if obj:
                return getattr(obj.rotation_euler if is_angle else obj.location, axis, 0.0)
            return 0.0
        return _get

    def make_set(name=name, axis=axis):
        def _set(self, val):
            obj = bpy.data.objects.get(name)
            if obj:
                if is_angle:
                    setattr(obj.rotation_euler, axis, val)
                else:
                    setattr(obj.location, axis, val)
        return _set

    annotations[name] = bpy.props.FloatProperty(
        name=label,
        subtype='ANGLE' if is_angle else 'DISTANCE',
        unit='ROTATION' if is_angle else 'LENGTH',
        min=min_val,
        max=max_val,
        get=make_get(),
        set=make_set()
    )

StageJogProperties.__annotations__.clear()
StageJogProperties.__annotations__.update(annotations)

def update_jog_properties(): JogProperties.annotations.clear() limits = get_joint_limits() axes = get_AXES() print(f"[DEBUG] update_jog_properties: robot={get_active_robot()} | DOF={len(limits)}") for i in range(len(limits)): deg_min, deg_max = limits[i] min_r = math.radians(deg_min) max_r = math.radians(deg_max) JogProperties.annotations[f"joint_{i}"] = bpy.props.FloatProperty( name=f"Joint {i}", subtype='ANGLE', unit='ROTATION', min=min_r, max=max_r, get=create_getter(i), set=create_setter(i), )

def move_objects_to_collection(objects, collection_name): scene = bpy.context.scene target_coll = bpy.data.collections.get(collection_name) if not target_coll: target_coll = bpy.data.collections.new(collection_name) scene.collection.children.link(target_coll) for obj in objects: if obj.name not in target_coll.objects: target_coll.objects.link(obj) for coll in list(obj.users_collection): if coll != target_coll: coll.objects.unlink(obj)

class OBJECT_OT_import_robot_system(bpy.types.Operator): bl_idname = "object.import_robot_system" bl_label = "Import Robot System"

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

    existing_names = set(bpy.data.objects.keys())
    with bpy.data.libraries.load(blend_path, link=False) as (data_from, data_to):
        data_to.objects = [name for name in data_from.objects if name not in existing_names]

    imported_objs = []
    for obj in data_to.objects:
        if obj is not None:
            ctx.collection.objects.link(obj)
            imported_objs.append(obj)

    # Sync armature name to property
    p = ctx.scene.ik_motion_props
    for arm_name in entry.get("armatures", []):
        if arm_name in bpy.data.objects:
            p.armature = arm_name
            break

    print(f"[DEBUG] Sync system={self.system}, armature={p.armature}")
    p.robot_type = self.system

    arm = bpy.data.objects.get(p.armature)
    if not arm or arm.type != 'ARMATURE':
        self.report({'ERROR'}, f"Valid armature '{p.armature}' not found")
        return {'CANCELLED'}

    # Override axes
    if "axes" in entry:
        from . import core_ur, core_iiwa
        if self.system.startswith("ur"):
            core_ur.AXES = entry["axes"]
        else:
            core_iiwa.AXES = entry["axes"]

    if "joint_limits_deg" in entry:
        from . import core
        core.CUSTOM_JOINT_LIMITS = entry["joint_limits_deg"]

    update_jog_properties()
    register_stage_properties(entry)

    setup_objs = []
    if "setup_objects" in entry:
        for key in ["goal", "base", "tcp", "ee"]:
            name = entry["setup_objects"].get(key)
            obj = bpy.data.objects.get(name)
            if name and obj:
                setattr(p, f"{key}_object", obj)
                setup_objs.append(obj)

    if setup_objs:
        move_objects_to_collection(setup_objs, "Setup")

    self.report({'INFO'}, f"Imported and synced system: {self.system}")
    return {'FINISHED'}

def invoke(self, ctx, event):
    return ctx.window_manager.invoke_props_dialog(self)

def register(): bpy.utils.register_class(OBJECT_OT_import_robot_system)

def unregister(): bpy.utils.unregister_class(OBJECT_OT_import_robot_system)

if name == "main": register()

