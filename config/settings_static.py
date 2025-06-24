import bpy
import math
from rteach.core.robot_presets import ROBOT_CONFIGS, get_joint_limits_deg

MAX_JOINTS = 10

# ──────────────────────────────────────────────────────────────
# Dynamic Jog slider (robot joints)
# ──────────────────────────────────────────────────────────────
class JogProperties(bpy.types.PropertyGroup):
    pass

JogProperties.__annotations__ = {}

def get_robot_axes(robot_type: str = ""):
    """
    Return axis list for given robot_type (lower‑cased).  
    Fallback: ['y'] * MAX_JOINTS
    """
    config = ROBOT_CONFIGS.get(robot_type.lower(), {})
    return config.get("axes", ["y"] * MAX_JOINTS)

def get_joint_limits(robot_type: str = ""):
    """
    Return joint‑limit list (deg) for given robot_type.  
    Fallback: 10 universal [-180,180] pairs.
    """
    config = ROBOT_CONFIGS.get(robot_type.lower(), {})
    return config.get("joint_limits_deg", [[-180, 180]] * MAX_JOINTS)

def _make_joint_getter(i):
    def getter(self):
        p = bpy.context.scene.ik_motion_props
        axes = get_robot_axes(p.robot_type)
        if i >= len(axes):
            return 0.0
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            return 0.0
        bone = arm.pose.bones.get(f"j{i+1}")
        if not bone:
            return 0.0
        return getattr(bone.rotation_euler, axes[i])
    return getter

def _make_joint_setter(i):
    def setter(self, value):
        p = bpy.context.scene.ik_motion_props
        axes = get_robot_axes(p.robot_type)
        if i >= len(axes):
            return
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            return
        bone = arm.pose.bones.get(f"j{i+1}")
        if not bone:
            return
        bone.rotation_mode = 'XYZ'
        setattr(bone.rotation_euler, axes[i], value)
        bpy.context.view_layer.update()
    return setter

# create default (placeholder) joint sliders so add‑on can register even
# before a robot is selected
for i in range(MAX_JOINTS):
    JogProperties.__annotations__[f"joint_{i}"] = bpy.props.FloatProperty(
        name=f"Joint {i+1}",
        subtype='ANGLE',
        unit='ROTATION',
        min=math.radians(-180),
        max=math.radians(180),
        get=_make_joint_getter(i),
        set=_make_joint_setter(i),
    )

# ──────────────────────────────────────────────────────────────
# Dynamic Stage‑jog sliders (external axes)
# ──────────────────────────────────────────────────────────────
class StageJogProperties(bpy.types.PropertyGroup):
    pass

StageJogProperties.__annotations__ = {}

def _make_stage_getter(obj_name, axis, joint_type):
    idx = {"x": 0, "y": 1, "z": 2}[axis.lower()]

    def getter(self):
        obj = bpy.data.objects.get(obj_name)
        if not obj:
            return 0.0
        return obj.rotation_euler[idx] if joint_type == "rotation" else obj.location[idx]

    return getter


def _make_stage_setter(obj_name, axis, joint_type):
    idx = {"x": 0, "y": 1, "z": 2}[axis.lower()]

    def setter(self, value):
        obj = bpy.data.objects.get(obj_name)
        if not obj:
            return
        if joint_type == "rotation":
            obj.rotation_euler[idx] = value
        else:
            obj.location[idx] = value
        bpy.context.view_layer.update()

    return setter


for cfg in ROBOT_CONFIGS.values():
    for joint in cfg.get("stage_joints", []):
        key, label, unit, mn, mx, axis, jtype = joint
        if key in StageJogProperties.__annotations__:
            continue

        if unit == "mm":
            mn, mx = mn / 1000.0, mx / 1000.0
            subtype, bl_unit = 'DISTANCE', 'LENGTH'
        else:  # deg
            mn, mx = math.radians(mn), math.radians(mx)
            subtype, bl_unit = 'ANGLE', 'ROTATION'

        StageJogProperties.__annotations__[key] = bpy.props.FloatProperty(
            name=label,
            subtype=subtype,
            unit=bl_unit,
            min=mn,
            max=mx,
            get=_make_stage_getter(key, axis, jtype),
            set=_make_stage_setter(key, axis, jtype),
        )


# ──────────────────────────────────────────────────────────────
# Helper: (re)register properties for selected robot
# ──────────────────────────────────────────────────────────────
def register_static_properties(robot_type: str = ""):
    """
    Rebuild JogProperties sliders to match the DOF & limits of `robot_type`.
    Must be called *before* re‑registering JogProperties.
    """
    print(f"[DEBUG] register_static_properties({robot_type})")

    axes = get_robot_axes(robot_type)
    limits_deg = get_joint_limits_deg(robot_type.lower())

    JogProperties.__annotations__.clear()
    for i, axis in enumerate(axes):
        mn, mx = limits_deg[i] if i < len(limits_deg) else (-180, 180)
        JogProperties.__annotations__[f"joint_{i}"] = bpy.props.FloatProperty(
            name=f"Joint {i+1}",
            subtype='ANGLE',
            unit='ROTATION',
            min=math.radians(mn),
            max=math.radians(mx),
            get=_make_joint_getter(i),
            set=_make_joint_setter(i),
        )


def unregister_static_properties():
    """
    Remove JogProperties pointer & class (called before rebuild).
    """
    if hasattr(bpy.types.Scene, "jog_props"):
        del bpy.types.Scene.jog_props
    try:
        bpy.utils.unregister_class(JogProperties)
    except Exception:
        pass
