#init.py 

bl_info = {
    "name": "Robot Simulator for Blender",
    "author": "Beomsoo Hwang",
    "version": (1, 0, 3),
    "blender": (4, 3, 0),
    "location": "View3D > Sidebar > IK Solver",
    "description": "UR/KUKA robot motion teaching add-on",
    "category": "Animation"
}

import os
import sys

addon_dir = os.path.dirname(__file__)
if addon_dir not in sys.path:
    sys.path.append(addon_dir)

import bpy
from bpy.props import EnumProperty, PointerProperty

from .core import *
from .ops_teach_main import classes as main_classes
from .ops_teach_util import classes as util_classes
from .ui_panel import classes as ui_classes

from .settings import IKMotionProperties, JogProperties, TcpItem, StageJogProperties
from .settings import register_stage_properties, re_register_stage_properties
from .ops_import_system import OBJECT_OT_import_robot_system, update_jog_properties
from . import ui_pie
from . import ui_overlay

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

    try:
        register_stage_properties({
            "stage_joints": [
                {
                    "name": "placeholder_stage",
                    "label": "Placeholder",
                    "type": "location",
                    "axis": "x",
                    "min": 0,
                    "max": 1,
                    "unit": "mm"
                }
            ]
        })
    except Exception as e:
        print(f"[WARNING] register_stage_properties failed during register: {e}")

    try:
        re_register_stage_properties()
    except Exception as e:
        print(f"[WARNING] re_register_stage_properties failed during register: {e}")

    def delayed_jog_update():
        try:
            update_jog_properties()
        except Exception as e:
            print(f"[WARNING] update_jog_properties (delayed) failed: {e}")
        return None 

    bpy.app.timers.register(delayed_jog_update, first_interval=0.1)
        
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
