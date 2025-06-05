# robot_state.py(0604)

_active_robot = "KUKA"

def set_active_robot(name: str):
    global _active_robot
    _active_robot = name

def get_active_robot() -> str:
    return _active_robot