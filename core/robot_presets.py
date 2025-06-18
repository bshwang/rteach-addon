import math

ROBOT_CONFIGS = {
    "robot_prb": {
        "armature": "KUKA_iiwa14_Arm",
        "axes": ["y"] * 7,
        "joint_limits_deg": [
            (-170, 170), (-120, 120), (-170, 170), (-120, 120),
            (-170, 170), (-120, 120), (-175, 175)
        ],
        "stage_joints": [
            ("joint_ev_z", "EV_Z", "mm", -580, 220, "z", "location"),
            ("joint_ev_y", "EV_Y", "mm", -400, 0, "y", "location"),
            ("joint_stage_x", "Holder_X", "mm", 0, 400, "x", "location"),
            ("joint_stage_y", "Holder_Y", "mm", -120, 280, "y", "location"),
            ("joint_stage_z", "Holder_Z", "mm", -250, 550, "z", "location"),
            ("joint_holder_tilt", "Holder_Tilt", "deg", 0, 35, "x", "rotation"),
            ("joint_holder_rot", "Holder_Rot", "deg", 0, 135, "z", "rotation"),
        ],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "KUKA_Base",
            "tcp": "KUKA_TCP",
            "ee": "KUKA_EE"
        }
    },
    "robot_pmbot": {
        "armature": "UR16e_Arm",
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [
            ("joint_z", "Elevation", "mm", 0, 1000, "z", "location"),
            ("joint_rot", "Rotation", "deg", -180, 180, "z", "rotation"),
            ("joint_x", "Linear", "mm", -500, 500, "x", "location"),
            ("joint_torso", "Outtrigger", "deg", -90, 90, "z", "rotation")
        ],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR16e_Base",
            "tcp": "UR16e_TCP",
            "ee": "UR16e_EE"
        }
    },
    "ur16e": {
        "armature": "UR16e_Arm",
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR16e_Base",
            "tcp": "UR16e_TCP",
            "ee": "UR16e_EE"
        }
    },
    "ur5e": {
        "armature": "UR5e_Arm",
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR5e_Base",
            "tcp": "UR5e_TCP",
            "ee": "UR5e_EE"
        }
    },
    "iiwa14": {
        "armature": "KUKA_iiwa14_Arm",
        "axes": ["y"] * 7,
        "joint_limits_deg": [
            (-170, 170), (-120, 120), (-170, 170), (-120, 120),
            (-170, 170), (-120, 120), (-175, 175)
        ],
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "KUKA_Base",
            "tcp": "KUKA_TCP",
            "ee": "KUKA_EE"
        }
    }
}
