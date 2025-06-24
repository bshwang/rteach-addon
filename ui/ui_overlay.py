import bpy
import blf
from bpy.app.handlers import persistent
from math import degrees
from rteach.ui.ui_panel import get_addon_prefs
from rteach.core.robot_state import get_armature_type

handler_ref = []

def draw_overlay_text():
    region = bpy.context.region
    if not region:
        return

    prefs = get_addon_prefs()
    if not prefs.show_overlay:
        return

    p = bpy.context.scene.ik_motion_props
    if not getattr(p, "show_overlay", True):
        return

    if not hasattr(p, "selected_teach_point") or not hasattr(p, "goal_object"):
        return

    obj = p.selected_teach_point
    goal = p.goal_object

    lines = []

    if p.status_text:
        lines.append(f"Status: {p.status_text}")

    if goal:
        pos = goal.location
        rot = goal.rotation_euler.to_matrix().to_euler()
        lines.append(
            f"Goal: X={pos.x * 1000:.1f} Y={pos.y * 1000:.1f} Z={pos.z * 1000:.1f} "
            f"RX={degrees(rot.x):.1f}° RY={degrees(rot.y):.1f}° RZ={degrees(rot.z):.1f}°"
        )

    if obj:
        lines.append(f"Selected: {obj.name}")
        pose_str = f"{p.current_index+1}" if p.max_solutions > 0 else "N/A"
        motion_type = obj.get("motion_type", "?")
        speed = obj.get("speed", None)
        wait = obj.get("wait_time_sec", None)

        line = f"Pose: {pose_str} | Motion: {motion_type}"
        if speed is not None:
            line += f" | Speed: {speed:.0f} mm/s"
        if wait is not None:
            line += f" | Wait: {wait:.1f} s"
        lines.append(line)

        if get_armature_type(p.robot_type) == "KUKA":
            lines.append(f"fixed_q3: {round(degrees(p.fixed_q3), 1)}°")
    else:
        lines.append("Selected: None")

    font_id = 0
    dpi = bpy.context.preferences.system.dpi
    ui_scale = bpy.context.preferences.view.ui_scale
    ref_dpi = 72
    scale = dpi / ref_dpi / ui_scale
    base_size = 14
    adjusted_size = int(base_size * scale)
    blf.size(font_id, adjusted_size)

    line_spacing = int(adjusted_size * 1.5)
    for i, line in enumerate(reversed(lines)):
        blf.position(font_id, 20, 40 + i * line_spacing, 0)
        blf.draw(font_id, line)

@persistent
def enable_overlay(dummy=None):
    if draw_overlay_text not in handler_ref:
        handler = bpy.types.SpaceView3D.draw_handler_add(draw_overlay_text, (), 'WINDOW', 'POST_PIXEL')
        handler_ref.clear()
        handler_ref.append(draw_overlay_text)
        print("[Overlay] Enabled")

def disable_overlay():
    if handler_ref:
        bpy.types.SpaceView3D.draw_handler_remove(handler_ref[0], 'WINDOW')
        handler_ref.clear()
        print("[Overlay] Disabled")

def register():
    enable_overlay()
    bpy.app.handlers.load_post.append(enable_overlay)

def unregister():
    disable_overlay()
    if enable_overlay in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(enable_overlay)
