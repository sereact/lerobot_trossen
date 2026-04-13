from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from lerobot.teleoperators.config import TeleoperatorConfig


class ActionSpace(str, Enum):
    JOINTS = "joint_state"
    CARTESIAN = "cartesian_pose"


@TeleoperatorConfig.register_subclass("widowxai_leader_teleop")
@dataclass
class WidowXAILeaderTeleopConfig(TeleoperatorConfig):
    # IP address of the arm
    ip_address: str

    # Joint names for the WidowX AI leader arm
    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint_0",
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "left_carriage_joint",
        ]
    )

    # Action space for the robot, either joints or cartesian
    action_space: ActionSpace = ActionSpace.JOINTS

    # Force feedback gain for haptic feedback
    force_feedback_gain: float = 0

    # "Staged" positions in rad for the arm and m for the gripper
    #
    # The robot will move to these positions when first started and before the arm is sent to the
    # sleep position.
    staged_positions: list[float] = field(
        default_factory=lambda: [0, np.pi / 3, np.pi / 6, np.pi / 5, 0, 0, 0]
    )

    def __post_init__(self):
        if self.action_space == ActionSpace.CARTESIAN:
            self.joint_names = [
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
                "carriage",
            ]
