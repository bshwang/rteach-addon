# ui_overlay.py

import bpy
import blf
from bpy.app.handlers import persistent
from math import degrees
from rteach.ui.ui_panel import get_addon_prefs
from rteach.core.robot_state import get_armature_type

handler_ref = []

def draw_overlay_text():
    prefs = get_addon_prefs()
    if not prefs.show_overlay:
        return
    p = bpy.context.scene.ik_motion_props
    if not getattr(p, "show_overlay", True):
        return

    obj  = p.selected_teach_point
    goal = p.goal_object

    lines = []
    if p.status_text:
        lines.append(f"Status: {p.status_text}")
    if goal:
        pos = goal.location
        rot = goal.rotation_euler.to_matrix().to_euler()
        lines.append(
            f"Goal Pos: X={pos.x*1000:.1f} Y={pos.y*1000:.1f} Z={pos.z*1000:.1f}"
        )
        lines.append(
            f"Goal Rot: RX={degrees(rot.x):.1f}° RY={degrees(rot.y):.1f}° RZ={degrees(rot.z):.1f}°"
        )
    if obj:
        lines.append(f"Selected TCP: {obj.name}")
        pose_idx = p.current_index + 1 if p.max_solutions > 0 else "N/A"
        lines.append(f"Solution: {pose_idx}/{p.max_solutions}")
        mt = obj.get("motion_type", "?")
        lines.append(f"MoveType: {mt}")
        if get_armature_type(p.robot_type) == "KUKA":
            lines.append(f"fixed_q3: {round(degrees(p.fixed_q3),1)}°")
        goal_val = obj.get("goal", "")
        lines.append(f"GoalLabel: {goal_val}")
    else:
        lines.append("Selected TCP: None")

    font_id = 0
    dpi = bpy.context.preferences.system.dpi
    ui_scale = bpy.context.preferences.view.ui_scale
    scale = dpi/72/ui_scale
    size = int(14 * scale)
    blf.size(font_id, size)

    spacing = int(size*1.4)
    for i, text in enumerate(reversed(lines)):
        blf.position(font_id, 20, 30 + i*spacing, 0)
        blf.draw(font_id, text)

@persistent
def enable_overlay(dummy=None):
    if draw_overlay_text not in handler_ref:
        handler = bpy.types.SpaceView3D.draw_handler_add(
            draw_overlay_text, (), 'WINDOW', 'POST_PIXEL'
        )
        handler_ref.clear()
        handler_ref.append(handler)

def disable_overlay():
    if handler_ref:
        bpy.types.SpaceView3D.draw_handler_remove(handler_ref[0], 'WINDOW')
        handler_ref.clear()

def register():
    enable_overlay()
    bpy.app.handlers.load_post.append(enable_overlay)

def unregister():
    disable_overlay()
    if enable_overlay in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(enable_overlay)
