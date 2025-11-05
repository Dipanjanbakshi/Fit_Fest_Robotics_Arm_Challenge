from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.0]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [1.57]


def joint_names() -> List[str]:
    return [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_flex",
        "wrist_roll",
    ]


def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "hand"


def gripper_joint_names() -> List[str]:
    return [
        "grip_left"
    ]
