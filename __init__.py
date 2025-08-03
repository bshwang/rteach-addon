bl_info = {
    "name": "Robot Simulator for Blender",
    "author": "Beomsoo Hwang",
    "version": (1, 2, 1),
    "blender": (4, 3, 0),
    "location": "View3D > Sidebar > IK Solver",
    "description": "Robot motion simulation add-on",
    "category": "Animation"
}

import os
import sys
import bpy

addon_dir = os.path.dirname(__file__)
if addon_dir not in sys.path:
    sys.path.append(addon_dir)

from bpy.props import EnumProperty
from rteach.core.core import *
from rteach.ops.ops_teach_main import classes as main_classes
from rteach.ops.ops_teach_sub import classes as sub_classes
from rteach.ops.ops_teach_util import classes as util_classes
from rteach.ui.ui_panel import classes as ui_classes
from rteach.ops.ops_import_system import OBJECT_OT_import_robot_system
from rteach.ui import ui_pie
from rteach.ui import ui_overlay

from rteach.config.settings import IKMotionProperties, TcpItem
from rteach.config.settings_static import JogProperties, StageJogProperties

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
    *sub_classes,
    *util_classes,
    *ui_classes, 
)

class RobotSimPreferences(bpy.types.AddonPreferences):
    bl_idname = __name__

    show_overlay: bpy.props.BoolProperty(
        name="Show Overlay",
        description="Display overlay text in 3D view",
        default=True
    )

    show_stage: bpy.props.BoolProperty(
        name="Show Stage Jog Mode",
        default=True
    )

    show_io: bpy.props.BoolProperty(
        name="Show Import/Export",
        default=True
    )

    ur_solver_choice: bpy.props.EnumProperty(
        name="UR Solver",
        description="Choose which UR analytic IK solver to use",
        items=[
            ('PYD', "Use .pyd (C++ fast)", "Use compiled solver module"),
            ('PY',  "Use .py (debug/fallback)", "Use fallback pure Python solver")
        ],
        default='PYD'
    )
    kuka_solver_choice: bpy.props.EnumProperty(
        name="KUKA Solver",
        description="Choose which KUKA analytic IK solver to use",
        items=[
            ('PYD', "Use .pyd (C++ fast)", "Use compiled solver module"),
            ('PY',  "Use .py (debug/fallback)", "Use fallback pure Python solver")
        ],
        default='PYD'
    )


    def draw(self, context):
        layout = self.layout
        layout.label(text="Optional UI Sections:")
        layout.prop(self, "show_overlay")
        layout.prop(self, "show_stage")
        layout.prop(self, "show_io")
        layout.separator(factor=1.5)
        layout.label(text="UR Solver Preference:")
        layout.prop(self, "ur_solver_choice", expand=True)
        layout.separator(factor=1.5)
        layout.label(text="KUKA Solver Preference:")
        layout.prop(self, "kuka_solver_choice", expand=True)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.ik_motion_props = bpy.props.PointerProperty(type=IKMotionProperties)
    bpy.types.Scene.jog_props = bpy.props.PointerProperty(type=JogProperties)
    bpy.types.Scene.stage_props = bpy.props.PointerProperty(type=StageJogProperties)
    bpy.utils.register_class(RobotSimPreferences)
    bpy.types.Scene.robot_slide_single_idx = bpy.props.IntProperty(default=0)
    bpy.types.Scene.robot_slide_custom_idx = bpy.props.IntProperty(default=0)
    bpy.types.Scene.bake_all_state = bpy.props.BoolProperty(
        name="Bake All TCPs",
        default=False,
        description="Toggle all bake checkboxes ON/OFF"
    )

    ui_pie.register()
    ui_overlay.register()
        
def unregister():
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except Exception:
            pass
    bpy.utils.unregister_class(RobotSimPreferences)

    if hasattr(bpy.types.Scene, "robot_slide_single_idx"):
        del bpy.types.Scene.robot_slide_single_idx
    if hasattr(bpy.types.Scene, "robot_slide_custom_idx"):
        del bpy.types.Scene.robot_slide_custom_idx
    if hasattr(bpy.types.Scene, "bake_all_state"):
        del bpy.types.Scene.bake_all_state

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
