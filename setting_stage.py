import bpy
import math

class StageJogProperties(bpy.types.PropertyGroup):

    joint_ev_z: bpy.props.FloatProperty(
        name="EV_Z", unit='LENGTH', subtype='DISTANCE',
        min=-0.58, max=0.22, default=0.0
    )
    joint_ev_y: bpy.props.FloatProperty(
        name="EV_Y", unit='LENGTH', subtype='DISTANCE',
        min=-0.4, max=0.0, default=0.0
    )
    joint_stage_x: bpy.props.FloatProperty(
        name="Stage_X", unit='LENGTH', subtype='DISTANCE',
        min=0.0, max=0.4, default=0.0
    )
    joint_stage_y: bpy.props.FloatProperty(
        name="Stage_Y", unit='LENGTH', subtype='DISTANCE',
        min=-0.12, max=0.28, default=0.0
    )
    joint_stage_z: bpy.props.FloatProperty(
        name="Stage_Z", unit='LENGTH', subtype='DISTANCE',
        min=-0.25, max=0.55, default=0.0
    )
    joint_holder_tilt: bpy.props.FloatProperty(
        name="Holder_Tilt", unit='ROTATION', subtype='ANGLE',
        min=0.0, max=math.radians(35), default=0.0
    )
    joint_holder_rot: bpy.props.FloatProperty(
        name="Holder_Rot", unit='ROTATION', subtype='ANGLE',
        min=0.0, max=math.radians(135), default=0.0
    )

# register 시:
def register_stage_properties():
    bpy.utils.register_class(StageJogProperties)
    bpy.types.Scene.stage_props = bpy.props.PointerProperty(type=StageJogProperties)

def unregister_stage_properties():
    del bpy.types.Scene.stage_props
    bpy.utils.unregister_class(StageJogProperties)
