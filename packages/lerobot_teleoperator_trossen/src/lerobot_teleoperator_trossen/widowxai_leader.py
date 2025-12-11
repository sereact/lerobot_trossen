import logging
import time

import trossen_arm
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot_teleoperator_trossen.config_widowxai_leader import (
    WidowXAILeaderTeleopConfig,
)

logger = logging.getLogger(__name__)


class WidowXAILeaderTeleop(Teleoperator):
    """
    [WidowX AI](https://www.trossenrobotics.com/widowx-ai) by Trossen Robotics
    """

    config_class = WidowXAILeaderTeleopConfig
    name = "widowxai_leader_teleop"

    def __init__(self, config: WidowXAILeaderTeleopConfig):
        super().__init__(config)
        self.config = config
        self.driver = trossen_arm.TrossenArmDriver()

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint_name}.pos": float for joint_name in self.config.joint_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {f"{joint_name}.eff": float for joint_name in self.config.joint_names}

    @property
    def is_connected(self) -> bool:
        return self.driver.get_is_configured()

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.driver.configure(
            model=trossen_arm.Model.wxai_v0,
            end_effector=trossen_arm.StandardEndEffector.wxai_v0_leader,
            serv_ip=self.config.ip_address,
            clear_error=True,
        )
        if not self.is_calibrated and calibrate:
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        # Trossen Arm robots do not require calibration
        return True

    def calibrate(self) -> None:
        # Trossen Arm robots do not require calibration
        pass

    def configure(self) -> None:
        self.driver.set_all_modes(trossen_arm.Mode.position)
        # Move the arm to the staged positions
        self.driver.set_all_positions(
            self.config.staged_positions,
            goal_time=2.0,
            blocking=True,
        )
        self.driver.set_all_modes(trossen_arm.Mode.external_effort)
        # Set all external efforts to 0.0 to enable gravity compensation
        self.driver.set_all_external_efforts(
            [0.0] * len(self.config.joint_names),
            goal_time=0.0,
            blocking=True,
        )

    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        action = self.driver.get_all_positions()
        action_dict = {
            f"{joint_name}.pos": val
            for joint_name, val in zip(
                self.config.joint_names,
                action,
                strict=True,
            )
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Extract efforts from the feedback dictionary
        efforts, effort_modes = [], []
        for joint_name in self.config.joint_names:
            efforts.append(feedback[f"{joint_name}.eff"] if "carriage" in joint_name else 0.0)
            effort_modes.append(trossen_arm.Mode.effort if "carriage" in joint_name else trossen_arm.Mode.external_effort)

        # Send the efforts to the arm
        self.driver.set_joint_external_effort(
            len(self.config.joint_names) - 1,
            -self.config.force_feedback_gain * efforts[-1],
            goal_time=0.0,
            blocking=True,
        )
        logger.debug(f"{self} sent feedback: {efforts}")

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.driver.set_all_modes(trossen_arm.Mode.position)
        # Move the arm to the staged positions before disconnecting
        self.driver.set_all_positions(
            self.config.staged_positions,
            goal_time=2.0,
            blocking=True,
        )
        # Move the arm to the sleep position (all positions to 0.0)
        self.driver.set_all_positions(
            [0.0] * len(self.config.joint_names),
            goal_time=2.0,
            blocking=True,
        )

        self.driver.cleanup()
        logger.info(f"{self} disconnected.")
