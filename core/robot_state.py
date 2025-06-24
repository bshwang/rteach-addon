def set_active_robot(name: str):
    from bpy import context
    if hasattr(context.scene, "ik_motion_props"):
        context.scene.ik_motion_props.robot_type = name

def get_armature_type(robot_type: str) -> str:
    from rteach.core.robot_presets import ROBOT_CONFIGS
    config = ROBOT_CONFIGS.get(robot_type.lower(), {})
    return config.get("armature_type", "UNKNOWN")
