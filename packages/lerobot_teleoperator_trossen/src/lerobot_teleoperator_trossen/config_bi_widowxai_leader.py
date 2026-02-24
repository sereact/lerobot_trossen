from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot_teleoperator_trossen.config_widowxai_leader import HapticFeedback


@TeleoperatorConfig.register_subclass("bi_widowxai_leader_teleop")
@dataclass
class BiWidowXAILeaderRobotConfig(TeleoperatorConfig):
    left_arm_ip_address: str
    right_arm_ip_address: str

    feedback: HapticFeedback | None = field(default_factory=lambda: HapticFeedback())
