import bpy
import math
from rteach.core.robot_presets import ROBOT_CONFIGS

MAX_JOINTS = 8

# ──────────────────────────────────────────────
class JogProperties(bpy.types.PropertyGroup):
    pass

JogProperties.__annotations__ = {}

def get_robot_axes():
    p = bpy.context.scene.ik_motion_props
    config = ROBOT_CONFIGS.get(getattr(p, "robot_type", ""), {})
    return config.get("axes", ["y"] * MAX_JOINTS)

def create_joint_getter(i):
    def getter(self):
        p = bpy.context.scene.ik_motion_props
        axes = get_robot_axes()
        if i >= len(axes): return 0.0
        arm = bpy.data.objects.get(p.armature)
        if not arm: return 0.0
        bone = arm.pose.bones.get(f"j{i+1}")
        if not bone: return 0.0
        return getattr(bone.rotation_euler, axes[i])
    return getter

def create_joint_setter(i):
    def setter(self, value):
        p = bpy.context.scene.ik_motion_props
        axes = get_robot_axes()
        if i >= len(axes): return
        arm = bpy.data.objects.get(p.armature)
        if not arm: return
        bone = arm.pose.bones.get(f"j{i+1}")
        if not bone: return
        bone.rotation_mode = 'XYZ'
        setattr(bone.rotation_euler, axes[i], value)
        bpy.context.view_layer.update()
    return setter

for i in range(MAX_JOINTS):
    JogProperties.__annotations__[f"joint_{i}"] = bpy.props.FloatProperty(
        name=f"Joint {i+1}", subtype='ANGLE', unit='ROTATION',
        min=-math.pi, max=math.pi,
        get=create_joint_getter(i), set=create_joint_setter(i)
    )

# ──────────────────────────────────────────────
def create_stage_getter(obj_name, axis, unit, joint_type):
    def getter(self):
        obj = bpy.data.objects.get(obj_name)
        if not obj: return 0.0
        idx = {"x":0, "y":1, "z":2}[axis.lower()]
        if joint_type == "rotation":
            return obj.rotation_euler[idx]
        else:
            return obj.location[idx]  
    return getter

def create_stage_setter(obj_name, axis, unit, joint_type):
    def setter(self, value):
        obj = bpy.data.objects.get(obj_name)
        if not obj: return
        idx = {"x":0, "y":1, "z":2}[axis.lower()]
        if joint_type == "rotation":
            obj.rotation_euler[idx] = value
        else:
            obj.location[idx] = value
        bpy.context.view_layer.update()
    return setter

# ──────────────────────────────────────────────
class StageJogProperties(bpy.types.PropertyGroup): pass
StageJogProperties.__annotations__ = {}

for config in ROBOT_CONFIGS.values():
    for joint in config.get("stage_joints", []):
        key, label, unit, min_val, max_val, axis, joint_type = joint
        if key in StageJogProperties.__annotations__:
            continue  
        
        if unit == "mm":
            min_val /= 1000.0
            max_val /= 1000.0
        elif unit == "deg":
            min_val = math.radians(min_val)
            max_val = math.radians(max_val)

        subtype = 'ANGLE' if unit == "deg" else 'DISTANCE'
        blender_unit = 'ROTATION' if unit == "deg" else 'LENGTH'

        StageJogProperties.__annotations__[key] = bpy.props.FloatProperty(
            name=label, subtype=subtype, unit=blender_unit,
            min=min_val, max=max_val,
            get=create_stage_getter(key, axis, unit, joint_type),
            set=create_stage_setter(key, axis, unit, joint_type)
        )

# ──────────────────────────────────────────────
def register_static_properties():
    bpy.utils.register_class(JogProperties)
    bpy.utils.register_class(StageJogProperties)
    bpy.types.Scene.jog_props = bpy.props.PointerProperty(type=JogProperties)
    bpy.types.Scene.stage_props = bpy.props.PointerProperty(type=StageJogProperties)

def unregister_static_properties():
    del bpy.types.Scene.jog_props
    del bpy.types.Scene.stage_props
    bpy.utils.unregister_class(StageJogProperties)
    bpy.utils.unregister_class(JogProperties)
