import bpy
import math
from .robot_presets import ROBOT_CONFIGS

MAX_JOINTS = 8  # 최대 로봇 자유도

# ──────────────────────────────────────────────
class JogProperties(bpy.types.PropertyGroup):
    pass

JogProperties.__annotations__ = {}

def get_robot_axes():
    p = bpy.context.scene.ik_motion_props
    robot = getattr(p, "robot_type", "").strip()
    config = ROBOT_CONFIGS.get(robot)
    if config and "axes" in config:
        return config["axes"]
    return ["y"] * MAX_JOINTS

def create_joint_getter(i):
    def getter(self):
        p = bpy.context.scene.ik_motion_props
        axes = get_robot_axes()
        if i >= len(axes):
            return 0.0
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            return 0.0
        bone = arm.pose.bones.get(f"j{i+1}")
        if not bone:
            return 0.0
        axis = axes[i]
        return getattr(bone.rotation_euler, axis)
    return getter

def create_joint_setter(i):
    def setter(self, value):
        p = bpy.context.scene.ik_motion_props
        axes = get_robot_axes()
        if i >= len(axes):
            return
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            return
        bone = arm.pose.bones.get(f"j{i+1}")
        if not bone:
            return
        axis = axes[i]
        bone.rotation_mode = 'XYZ'
        setattr(bone.rotation_euler, axis, value)
        bpy.context.view_layer.update()
    return setter

for i in range(MAX_JOINTS):
    JogProperties.__annotations__[f"joint_{i}"] = bpy.props.FloatProperty(
        name=f"Joint {i+1}",
        subtype='ANGLE',
        unit='ROTATION',
        min=-math.pi,
        max=math.pi,
        get=create_joint_getter(i),
        set=create_joint_setter(i)
    )

# ──────────────────────────────────────────────
# Stage getter/setter 생성
def create_stage_getter(obj_name, axis="z", unit="mm", joint_type="location"):
    def getter(self):
        obj = bpy.data.objects.get(obj_name)
        if not obj:
            return 0.0
        idx = {"x": 0, "y": 1, "z": 2}[axis[-1].lower()]
        if joint_type.startswith("rotation"):
            val = obj.rotation_euler[idx]
            return math.degrees(val) if unit == "deg" else val
        else:
            val = obj.location[idx]
            return val * 1000 if unit == "mm" else val
    return getter

def create_stage_setter(obj_name, axis="z", unit="mm", joint_type="location"):
    def setter(self, value):
        obj = bpy.data.objects.get(obj_name)
        if not obj:
            return
        idx = {"x": 0, "y": 1, "z": 2}[axis[-1].lower()]
        if joint_type.startswith("rotation"):
            val = math.radians(value) if unit == "deg" else value
            rot = list(obj.rotation_euler)
            rot[idx] = val
            obj.rotation_euler = rot
        else:
            val = value / 1000 if unit == "mm" else value
            loc = list(obj.location)
            loc[idx] = val
            obj.location = loc
        bpy.context.view_layer.update()
    return setter

# ──────────────────────────────────────────────
class StageJogProperties(bpy.types.PropertyGroup):
    pass

# 모든 사용 가능한 Stage 조인트 정의
StageJogProperties.__annotations__ = {
    "joint_ev_z": bpy.props.FloatProperty(
        name="EV_Z", unit='LENGTH', subtype='DISTANCE',
        min=-0.58, max=0.22,
        get=create_stage_getter("joint_ev_z", "z", "mm", "location"),
        set=create_stage_setter("joint_ev_z", "z", "mm", "location")
    ),
    "joint_ev_y": bpy.props.FloatProperty(
        name="EV_Y", unit='LENGTH', subtype='DISTANCE',
        min=-0.4, max=0.0,
        get=create_stage_getter("joint_ev_y", "y", "mm", "location"),
        set=create_stage_setter("joint_ev_y", "y", "mm", "location")
    ),
    "joint_stage_x": bpy.props.FloatProperty(
        name="Stage_X", unit='LENGTH', subtype='DISTANCE',
        min=0.0, max=0.4,
        get=create_stage_getter("joint_stage_x", "x", "mm", "location"),
        set=create_stage_setter("joint_stage_x", "x", "mm", "location")
    ),
    "joint_stage_y": bpy.props.FloatProperty(
        name="Stage_Y", unit='LENGTH', subtype='DISTANCE',
        min=-0.12, max=0.28,
        get=create_stage_getter("joint_stage_y", "y", "mm", "location"),
        set=create_stage_setter("joint_stage_y", "y", "mm", "location")
    ),
    "joint_stage_z": bpy.props.FloatProperty(
        name="Stage_Z", unit='LENGTH', subtype='DISTANCE',
        min=-0.25, max=0.55,
        get=create_stage_getter("joint_stage_z", "z", "mm", "location"),
        set=create_stage_setter("joint_stage_z", "z", "mm", "location")
    ),
    "joint_holder_tilt": bpy.props.FloatProperty(
        name="Holder_Tilt", unit='ROTATION', subtype='ANGLE',
        min=0.0, max=35.0,
        get=create_stage_getter("joint_holder_tilt", "x", "deg", "rotation"),
        set=create_stage_setter("joint_holder_tilt", "x", "deg", "rotation")
    ),
    "joint_holder_rot": bpy.props.FloatProperty(
        name="Holder_Rot", unit='ROTATION', subtype='ANGLE',
        min=0.0, max=135.0,
        get=create_stage_getter("joint_holder_rot", "z", "deg", "rotation"),
        set=create_stage_setter("joint_holder_rot", "z", "deg", "rotation")
    ),
    "joint_z": bpy.props.FloatProperty(
        name="Elevation", unit='LENGTH', subtype='DISTANCE',
        min=0.0, max=1.0,
        get=create_stage_getter("joint_z", "z", "mm", "location"),
        set=create_stage_setter("joint_z", "z", "mm", "location")
    ),
    "joint_x": bpy.props.FloatProperty(
        name="Linear", unit='LENGTH', subtype='DISTANCE',
        min=-0.5, max=0.5,
        get=create_stage_getter("joint_x", "x", "mm", "location"),
        set=create_stage_setter("joint_x", "x", "mm", "location")
    ),
    "joint_rot": bpy.props.FloatProperty(
        name="Rotation", unit='ROTATION', subtype='ANGLE',
        min=-180.0, max=180.0,
        get=create_stage_getter("joint_rot", "z", "deg", "rotation"),
        set=create_stage_setter("joint_rot", "z", "deg", "rotation")
    ),
    "joint_torso": bpy.props.FloatProperty(
        name="Outtrigger", unit='ROTATION', subtype='ANGLE',
        min=-90.0, max=90.0,
        get=create_stage_getter("joint_torso", "z", "deg", "rotation"),
        set=create_stage_setter("joint_torso", "z", "deg", "rotation")
    )
}

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
