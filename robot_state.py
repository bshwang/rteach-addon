_active_robot = "KUKA"

def set_active_robot(name: str):
    global _active_robot
    _active_robot = name

def get_active_robot() -> str:
    from bpy import context
    p = context.scene.ik_motion_props if context.scene else None
    t = (p.robot_type.lower() if p and hasattr(p, "robot_type") else "").strip()
    
    print(f"[DEBUG] get_active_robot(): raw robot_type = {t}")
    
    if "kuka" in t or "iiwa" in t or "prb" in t:
        print("[DEBUG] robot classified as KUKA")
        return "KUKA"
    elif "ur" in t:
        print("[DEBUG] robot classified as UR")
        return "UR"
    print("[DEBUG] robot default fallback =", t.upper())
    return t.upper()
