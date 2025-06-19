import bpy
import math
from rteach.core.robot_presets import ROBOT_CONFIGS, get_joint_limits_deg, get_robot_axes

MAX_JOINTS = 10


class JogProperties(bpy.types.PropertyGroup):
    pass

JogProperties.__annotations__ = {}

def get_robot_axes():
    p = bpy.context.scene.ik_motion_props
    config = ROBOT_CONFIGS.get(getattr(p, "robot_type", ""), {})
    return config.get("axes", ["y"] * MAX_JOINTS)

def get_joint_limits():
    p = bpy.context.scene.ik_motion_props
    config = ROBOT_CONFIGS.get(getattr(p, "robot_type", ""), {})
    return config.get("joint_limits_deg", [[-180, 180]] * MAX_JOINTS)

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

try:
    limits = get_joint_limits()
except:
    limits = [[-180, 180]] * MAX_JOINTS

for i in range(MAX_JOINTS):
    lim = limits[i] if i < len(limits) else [-180, 180]
    min_rad = math.radians(lim[0])
    max_rad = math.radians(lim[1])

    JogProperties.__annotations__[f"joint_{i}"] = bpy.props.FloatProperty(
        name=f"Joint {i+1}", subtype='ANGLE', unit='ROTATION',
        min=min_rad, max=max_rad,
        get=create_joint_getter(i), set=create_joint_setter(i)
    )

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

def register_static_properties(robot_type="ur16e"):
    print(f"[DEBUG] register_static_properties({robot_type})")

    limits_deg = get_joint_limits_deg(robot_type)
    axes = get_robot_axes(robot_type)

    JogProperties.__annotations__.clear()
    for i in range(len(axes)):
        min_deg, max_deg = limits_deg[i]
        JogProperties.__annotations__[f"joint_{i}"] = bpy.props.FloatProperty(
            name=f"Joint {i+1}",
            default=0.0,
            min=math.radians(min_deg),
            max=math.radians(max_deg),
            subtype='ANGLE'
        )

def unregister_static_properties():
    if hasattr(bpy.types.Scene, "jog_props"):
        del bpy.types.Scene.jog_props
    try:
        bpy.utils.unregister_class(JogProperties)
    except Exception:
        pass
