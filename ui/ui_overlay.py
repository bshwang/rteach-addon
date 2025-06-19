import bpy
import blf
from bpy.app.handlers import persistent
from math import degrees

handler_ref = []

def draw_overlay_text():
    region = bpy.context.region
    if not region:
        return

    p = bpy.context.scene.ik_motion_props
    if not getattr(p, "show_overlay", True):
        return  

    obj = p.selected_teach_point
    goal = p.goal_object

    lines = []

    if obj:
        lines.append(f"Selected: {obj.name}")
        if p.max_solutions > 0:
            lines.append(f"Pose: {p.current_index+1} / {p.max_solutions}")
        if hasattr(p, 'fixed_q3'):
            lines.append(f"fixed_q3: {round(degrees(p.fixed_q3), 1)}°")
    else:
        lines.append("Selected: None")

    if p.status_text:
        lines.append(f"Status: {p.status_text}")

    if goal:
        pos = goal.location
        lines.append(f"Goal: X={pos.x * 1000:.1f} Y={pos.y * 1000:.1f} Z={pos.z * 1000:.1f}")
        rot = goal.rotation_euler.to_matrix().to_euler()
        lines.append(f"Rot: RX={degrees(rot.x):.1f}° RY={degrees(rot.y):.1f}° RZ={degrees(rot.z):.1f}°")

    if obj:
        motion_type = obj.get("motion_type", "?")
        speed = obj.get("speed", None)
        wait = obj.get("wait_time_sec", None)
        lines.append(f"Motion: {motion_type}")
        if speed is not None:
            lines.append(f"Speed: {speed:.0f} mm/s")
        if wait is not None:
            lines.append(f"Wait: {wait:.1f} s")

    font_id = 0
    blf.size(font_id, 12)

    for i, line in enumerate(lines):
        blf.position(font_id, 20, 40 + i * 20, 0)
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

# 애드온 등록용
def register():
    enable_overlay()
    bpy.app.handlers.load_post.append(enable_overlay)

def unregister():
    disable_overlay()
    if enable_overlay in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(enable_overlay)
