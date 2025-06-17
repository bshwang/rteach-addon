# 로봇 ID에 따라 사용할 조인트 이름을 반환

def get_stage_labels(robot_type):
    if robot_type == "robot_prb":
        return [
            "joint_ev_z", "joint_ev_y", "joint_stage_x", "joint_stage_y",
            "joint_holder_tilt", "joint_holder_rot"
        ]
    elif robot_type == "robot_pmbot":
        return [
            "joint_stage_x", "joint_stage_y", "joint_stage_z"
        ]
    elif robot_type == "robot_prb_ur":
        return [
            "joint_ev_z", "joint_ev_y", "joint_holder_tilt"
        ]
    else:
        return []  # 기본값 또는 비Stage 로봇
