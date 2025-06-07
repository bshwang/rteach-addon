#init.py (0604)

bl_info = {
    "name": "Robot Simulator for Blender",
    "author": "Beomsoo Hwang",
    "version": (1, 0),
    "blender": (4, 3, 0),
    "location": "View3D > Sidebar > IK Solver",
    "description": "UR/KUKA robot motion teaching add-on",
    "category": "Animation"
}

import bpy
from bpy.props import EnumProperty, PointerProperty

from .settings import IKMotionProperties
from .settings import JogProperties
from .core import *

from .ops_teach import *
from .ui_panel import *
from .settings import TcpItem

# Define EnumProperty immediately on import (before class registration)
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
    IKMotionProperties,
    JogProperties,
    OBJECT_OT_compute_ik_ur,
    OBJECT_OT_move_l,
    OBJECT_OT_record_goal_as_empty,
    OBJECT_OT_cycle_solution_ur,
    OBJECT_OT_clear_path_visuals,
    OBJECT_OT_apply_cycle_pose,
    VIEW3D_PT_ur_ik,
    OBJECT_OT_draw_teach_path,
    OBJECT_OT_playback_teach_bake,
    OBJECT_OT_playback_cycle_solution,
    OBJECT_OT_playback_apply_solution,
    OBJECT_OT_tcp_move_up,
    OBJECT_OT_tcp_move_down,
    OBJECT_OT_tcp_update_position,
    OBJECT_OT_tcp_delete,
    OBJECT_OT_reindex_tcp_points,
    OBJECT_OT_clear_all_tcp_points,
    OBJECT_OT_preview_tcp_next,
    OBJECT_OT_preview_tcp_prev,
    OBJECT_OT_toggle_motion_type,
    OBJECT_OT_clear_bake_keys,
    OBJECT_OT_snap_goal_to_active,
    OBJECT_OT_update_fixed_q3_from_pose,
    OBJECT_OT_snap_gizmo_on_path,
    OBJECT_OT_sync_robot_type,
    OBJECT_OT_focus_on_target,
    OBJECT_OT_apply_global_wait,
    OBJECT_OT_apply_global_speed,
    OBJECT_OT_tcp_list_select,
    UI_UL_tcp_list,
    OBJECT_OT_export_teach_data,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.ik_motion_props = bpy.props.PointerProperty(type=IKMotionProperties)
    bpy.types.Scene.jog_props       = bpy.props.PointerProperty(type=JogProperties)

def unregister():
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except Exception:
            pass
