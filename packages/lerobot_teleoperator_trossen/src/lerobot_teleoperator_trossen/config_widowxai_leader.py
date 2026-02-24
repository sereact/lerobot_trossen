from dataclasses import dataclass, field

import numpy as np
from lerobot.teleoperators.config import TeleoperatorConfig


@dataclass
class HapticFeedback:
    # Haptic feedback sensitivity
    gain: float = 0.2

    # Minimum measured force (N) before feedback activates (removes noise/jitter)
    deadband: float = 10.0

    # Force scale (N) controlling how quickly feedback saturates (smaller = more aggressive)
    f0: float = 50.0

    # Maximum haptic feedback command
    u_max: float = 10.0


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

    feedback: HapticFeedback | None = field(default_factory=lambda: HapticFeedback(0))

    # "Staged" positions in rad for the arm and m for the gripper
    #
    # The robot will move to these positions when first started and before the arm is sent to the
    # sleep position.
    staged_positions: list[float] = field(
        default_factory=lambda: [0, np.pi / 3, np.pi / 6, np.pi / 5, 0, 0, 0]
    )
