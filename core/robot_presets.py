ROBOT_CONFIGS = {
    "prb_iiwa": {
        "armature_type": "KUKA",
        "armature": "KUKA_iiwa14_Arm",
        "armature_solver_map": {   
            "KUKA_iiwa14_Arm": "iiwa14"
        },
        "axes": ["y"] * 7,
        "joint_limits_deg": [
            (-170, 170), (-120, 120), (-170, 170), (-120, 120),
            (-170, 170), (-120, 120), (-175, 175)
        ],
        "stage_joints": [
            ("joint_ev_z", "EV_Z", "mm", -100, 680, "z", "location"),
            ("joint_ev_y", "EV_Y", "mm", -400, 0, "y", "location"),
            ("joint_stage_x", "Stage_X", "mm", 0, 400, "x", "location"),
            ("joint_stage_y", "Stage_Y", "mm", -400, 0, "y", "location"),
            ("joint_stage_z", "Stage_Z", "mm", 0, 780, "z", "location"),
            ("joint_rot", "Holder_Rot", "deg", -90, 90, "z", "rotation"),
            ("joint_tilt", "Holder_Tilt", "deg", 0, 60, "x", "rotation"),
            ("joint_tb_low", "Toolbox", "mm", 0, 280, "z", "location"),
        ],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "KUKA_Base",
            "ee": "KUKA_EE",
            "tcp": "KUKA_TCP"
        },
        "thumbnail": "resources/robot_thumbs/prb_iiwa.png"
    },

    "prb_ur": {
        "armature_type": "UR",
        "armature": "UR16e_Arm",
        "armature_solver_map": {   
            "UR16e_Arm": "ur16e"
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [
            ("joint_ev_z", "EV_Z", "mm", -100, 680, "z", "location"),
            ("joint_ev_y", "EV_Y", "mm", -400, 0, "y", "location"),
            ("joint_stage_x", "Stage_X", "mm", 0, 400, "x", "location"),
            ("joint_stage_y", "Stage_Y", "mm", -400, 0, "y", "location"),
            ("joint_stage_z", "Stage_Z", "mm", 0, 780, "z", "location"),
            ("joint_rot", "Holder_Rot", "deg", -90, 90, "z", "rotation"),
            ("joint_tilt", "Holder_Tilt", "deg", 0, 60, "x", "rotation"),
            ("joint_tb_low", "Toolbox", "mm", 0, 280, "z", "location"),
        ],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR16e_Base",
            "ee": "UR16e_EE",
            "tcp": "UR16e_TCP"
        },
        "thumbnail": "resources/robot_thumbs/prb_ur.png"
    },

    "prb_beta": {
        "armature_type": "KUKA",
        "armature_solver_map": {
            "KUKA_L_Arm": "iiwa14",
            "KUKA_R_Arm": "iiwa14"
        },
        "armature_sets": {
            "KUKA_L_Arm": {
                "base": "KUKA_L_Base",
                "ee":   "KUKA_L_EE",
                "tcp":  "KUKA_L_TCP"
            },
            "KUKA_R_Arm": {
                "base": "KUKA_R_Base",
                "ee":   "KUKA_R_EE",
                "tcp":  "KUKA_R_TCP"
            }
        },
        "axes": ["y"] * 7,
        "joint_limits_deg": [
            (-170, 170), (-120, 120), (-170, 170), (-120, 120),
            (-170, 170), (-120, 120), (-175, 175)
        ],
        "stage_joints": [
            ("joint_ev", "EV", "mm", 0, 680, "z", "location"),
            ("joint_x", "Stage_X", "mm", 0, 400, "x", "location"),
            ("joint_y", "Stage_Y", "mm", -500, 0, "y", "location"),
            ("joint_z", "Stage_Z", "mm", 0, 680, "z", "location"),
            ("joint_rot", "Holder_Rot", "deg", -90, 90, "z", "rotation"),
            ("joint_tilt", "Holder_Tilt", "deg", 0, 60, "x", "rotation"),
            ("joint_btfinger", "TCG_BTF", "mm", 0, 45, "y", "location"),
            ("joint_socket", "TCG_Socket", "deg", -900, 900, "z", "rotation"),
            ("joint_ncfinger", "NCG_NCF", "deg", -900, 900, "z", "rotation"),
            ("joint_hin", "H_in", "mm", 0, 10, "x", "location"),
            ("joint_hout", "H_out", "mm", 0, 10, "x", "location"),
        ],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "KUKA_L_Base",  
            "tcp":  "KUKA_L_TCP",
            "ee":   "KUKA_L_EE"
        },
        "thumbnail": "resources/robot_thumbs/prb_beta.png"
    },

    "pmbot_beta": {
        "armature_type": "UR",
        "armature_solver_map": {
            "UR5e_Arm": "ur5e",
            "UR16e_Arm": "ur16e"
        },
        "armature_sets": {
            "UR5e_Arm": {
                "base": "UR5e_Base",
                "ee":   "UR5e_EE",
                "tcp":  "UR5e_TCP"
            },
            "UR16e_Arm": {
                "base": "UR16e_Base",
                "ee":   "UR16e_EE",
                "tcp":  "UR16e_TCP"
            }
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [
            ("joint_elevation", "Elevation", "mm", 0, 250, "z", "location"),
            ("joint_rotation", "Rotation", "deg", -180, 180, "z", "rotation"),
            ("joint_linear", "Linear", "mm", -170, 170, "x", "location"),
            ("joint_ots", "Outtrigger", "mm", -40, 0, "z", "location")
        ],
        "setup_objects": {
            "goal": "Target_Gizmo",
        },
        "thumbnail": "resources/robot_thumbs/pmbot_beta.png"
    },

    "ur3e": {
        "armature_type": "UR",
        "armature": "UR3e_Arm",
        "armature_solver_map": {         
            "UR3e_Arm": "ur3e"
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR3e_Base",
            "tcp": "UR3e_TCP",
            "ee": "UR3e_EE"
        },
        "thumbnail": "resources/robot_thumbs/ur3e.png"
    },

    "ur5e": {
        "armature_type": "UR",
        "armature": "UR5e_Arm",
        "armature_solver_map": {         
            "UR5e_Arm": "ur5e"
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR5e_Base",
            "tcp": "UR5e_TCP",
            "ee": "UR5e_EE"
        },
        "thumbnail": "resources/robot_thumbs/ur5e.png"
    },

    "ur10e": {
        "armature_type": "UR",
        "armature": "UR10e_Arm",
        "armature_solver_map": {         
            "UR10e_Arm": "ur10e"
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR10e_Base",
            "tcp": "UR10e_TCP",
            "ee": "UR10e_EE"
        },
        "thumbnail": "resources/robot_thumbs/ur10e.png"
    },
 
    "ur16e": {
        "armature_type": "UR",
        "armature": "UR16e_Arm",
        "armature_solver_map": {         
            "UR16e_Arm": "ur16e"
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR16e_Base",
            "tcp": "UR16e_TCP",
            "ee": "UR16e_EE"
        },
        "thumbnail": "resources/robot_thumbs/ur16e.png"
    },

    "ur15": {
        "armature_type": "UR",
        "armature": "UR15_Arm",
        "armature_solver_map": {         
            "UR15_Arm": "ur15"
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR15_Base",
            "tcp": "UR15_TCP",
            "ee": "UR15_EE"
        },
        "thumbnail": "resources/robot_thumbs/ur15.png"
    },

    "ur20": {
        "armature_type": "UR",
        "armature": "UR20_Arm",
        "armature_solver_map": {         
            "UR20_Arm": "ur20"
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR20_Base",
            "tcp": "UR20_TCP",
            "ee": "UR20_EE"
        },
        "thumbnail": "resources/robot_thumbs/ur20.png"
    },

    "ur30": {
        "armature_type": "UR",
        "armature": "UR30_Arm",
        "armature_solver_map": {         
            "UR30_Arm": "ur30"
        },
        "axes": ["z", "x", "x", "z", "x", "z"],
        "joint_limits_deg": [[-360, 360]] * 6,
        "stage_joints": [],
        "setup_objects": {
            "goal": "Target_Gizmo",
            "base": "UR30_Base",
            "tcp": "UR30_TCP",
            "ee": "UR30_EE"
        },
        "thumbnail": "resources/robot_thumbs/ur30.png"
    },

    "iiwa14": {
        "armature_type": "KUKA",
        "armature": "KUKA_iiwa14_Arm",
        "armature_solver_map": {         
            "KUKA_iiwa14_Arm": "iiwa14"
        },
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
        },
        "thumbnail": "resources/robot_thumbs/iiwa14.png"
    }
}

def get_joint_limits_deg(robot_type):
    return ROBOT_CONFIGS[robot_type.lower()]["joint_limits_deg"]

def get_robot_axes(robot_type):
    return ROBOT_CONFIGS[robot_type.lower()]["axes"]
