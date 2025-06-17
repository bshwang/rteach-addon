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
            ("joint_ev_z", "EV_Z", "LENGTH", -0.58, 0.22),
            ("joint_ev_y", "EV_Y", "LENGTH", -0.4, 0.0),
            ("joint_stage_x", "Stage_X", "LENGTH", 0.0, 0.4),
            ("joint_stage_y", "Stage_Y", "LENGTH", -0.12, 0.28),
            ("joint_holder_tilt", "Holder_Tilt", "ROTATION", 0.0, math.radians(35)),
            ("joint_holder_rot", "Holder_Rot", "ROTATION", 0.0, math.radians(135)),
        ]
    },
    "robot_pmbot": {
        "armature": "UR16e_Arm",
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [
            (-360, 360)] * 6,
        "stage_joints": [
            ("joint_z", "Z Axis", "LENGTH", 0.0, 1.0),
            ("joint_x", "X Axis", "LENGTH", -0.5, 0.5),
            ("joint_rot", "Rotation", "ROTATION", -math.pi, math.pi),
            ("joint_torso", "Torso", "ROTATION", -math.pi/2, math.pi/2)
        ]
    }
}
