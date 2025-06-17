import bpy
import math
from .robot_presets import ROBOT_CONFIGS

MAX_JOINTS = 8  # 최대 지원 관절 수

class JogProperties(bpy.types.PropertyGroup):
    pass

JogProperties.__annotations__ = {}

def get_robot_axes():
    p = bpy.context.scene.ik_motion_props
    robot = getattr(p, "robot_type", "").strip()
    config = ROBOT_CONFIGS.get(robot)
    if config and "axes" in config:
        return config["axes"]
    return ["y"] * MAX_JOINTS  # 기본은 KUKA 기준

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
        val = getattr(bone.rotation_euler, axis)
        print(f"[DEBUG][GET] j{i+1} axis={axis}, val={round(math.degrees(val),1)} deg")
        return val
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
        print(f"[DEBUG][SET] j{i+1} axis={axis}, val={round(math.degrees(value),1)} deg")
    return setter

# joint_0 ~ joint_7 슬라이더 등록
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

# Stage 조인트 superset 등록
class StageJogProperties(bpy.types.PropertyGroup):
    joint_ev_z: bpy.props.FloatProperty(name="EV_Z", unit='LENGTH', min=-0.58, max=0.22)
    joint_ev_y: bpy.props.FloatProperty(name="EV_Y", unit='LENGTH', min=-0.4, max=0.0)
    joint_stage_x: bpy.props.FloatProperty(name="Stage_X", unit='LENGTH', min=0.0, max=0.4)
    joint_stage_y: bpy.props.FloatProperty(name="Stage_Y", unit='LENGTH', min=-0.12, max=0.28)
    joint_holder_tilt: bpy.props.FloatProperty(name="Holder_Tilt", unit='ROTATION', min=0.0, max=math.radians(35))
    joint_holder_rot: bpy.props.FloatProperty(name="Holder_Rot", unit='ROTATION', min=0.0, max=math.radians(135))
    joint_z: bpy.props.FloatProperty(name="Z Axis", unit='LENGTH', min=0.0, max=1.0)
    joint_x: bpy.props.FloatProperty(name="X Axis", unit='LENGTH', min=-0.5, max=0.5)
    joint_rot: bpy.props.FloatProperty(name="Rotation", unit='ROTATION', min=-math.pi, max=math.pi)
    joint_torso: bpy.props.FloatProperty(name="Torso", unit='ROTATION', min=-math.pi/2, max=math.pi/2)

# 등록 함수
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
