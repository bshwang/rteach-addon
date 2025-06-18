def set_active_robot(name: str):
    from bpy import context
    if hasattr(context.scene, "ik_motion_props"):
        context.scene.ik_motion_props.robot_type = name.upper()

def get_active_robot(safe=False) -> str:
    from bpy import context
    p = context.scene.ik_motion_props if context.scene else None
    t = (str(p.robot_type).strip().upper() if p and hasattr(p, "robot_type") else "")

    if not t:
        if safe:
            return "UNKNOWN"
        raise ValueError("[get_active_robot] robot_type is empty or missing")

    return t

