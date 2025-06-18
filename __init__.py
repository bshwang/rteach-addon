bl_info = {
    "name": "Robot Simulator for Blender",
    "author": "Beomsoo Hwang",
    "version": (1, 0, 4),
    "blender": (4, 3, 0),
    "location": "View3D > Sidebar > IK Solver",
    "description": "UR/KUKA robot motion teaching add-on",
    "category": "Animation"
}

import os
import sys
import bpy

addon_dir = os.path.dirname(__file__)
if addon_dir not in sys.path:
    sys.path.append(addon_dir)

from bpy.props import EnumProperty, PointerProperty
from rteach.core.core import *
from rteach.ops.ops_teach_main import classes as main_classes
from rteach.ops.ops_teach_util import classes as util_classes
from rteach.ui.ui_panel import classes as ui_classes
from rteach.ops.ops_import_system import OBJECT_OT_import_robot_system
from rteach.ui import ui_pie
from rteach.ui import ui_overlay

from rteach.config.settings import IKMotionProperties, TcpItem
from rteach.config.settings_static import (
    register_static_properties, unregister_static_properties, JogProperties, StageJogProperties
)
from rteach.core.robot_presets import ROBOT_CONFIGS

if not hasattr(bpy.types.Object, "motion_enum"):
    bpy.types.Object.motion_enum = EnumProperty(
        name="Motion Type",
        items=[
            ("JOINT",  "Joint",  "Joint-space interpolation"),
            ("LINEAR", "Linear", "Linear TCP motion"),
        ],
        get=lambda self: self.get("motion_type", "JOINT"),
        set=lambda self, v: self.__setitem__("motion_type", v),
    )

classes = (
    TcpItem,
    StageJogProperties, 
    IKMotionProperties,
    JogProperties,
    OBJECT_OT_import_robot_system,
    *main_classes,
    *util_classes,
    *ui_classes, 
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.ik_motion_props = bpy.props.PointerProperty(type=IKMotionProperties)
    bpy.types.Scene.jog_props = bpy.props.PointerProperty(type=JogProperties)
    bpy.types.Scene.stage_props = bpy.props.PointerProperty(type=StageJogProperties)

    ui_pie.register()
    ui_overlay.register()
        
def unregister():
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except Exception:
            pass
    try:
        ui_pie.unregister()
        ui_overlay.unregister()
    except:
        pass
    try:
        del bpy.types.Scene.jog_props
    except:
        pass
    try:
        del bpy.types.Scene.stage_props
    except:
        pass
