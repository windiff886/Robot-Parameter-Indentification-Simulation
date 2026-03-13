from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Callable


def _default_sdk_factory(can_port: str):
    repo_root = Path(__file__).resolve().parents[2]
    sdk_root = repo_root / "piper_sdk"
    if str(sdk_root) not in sys.path:
        sys.path.insert(0, str(sdk_root))
    from piper_sdk import C_PiperInterface_V2

    return C_PiperInterface_V2(can_port)


@dataclass
class RobotState:
    timestamp: float
    position: list[float]
    velocity: list[float]
    effort: list[float]
    arm_status_code: int
    ctrl_mode: int
    mode_feed: int
    has_error: bool


class PiperSdkAdapter:
    def __init__(
        self,
        can_port: str,
        sdk_factory: Callable[[str], object] | None = None,
    ) -> None:
        self._sdk = (sdk_factory or _default_sdk_factory)(can_port)

    def connect(self) -> None:
        self._sdk.ConnectPort()

    def disconnect(self) -> None:
        self._sdk.DisconnectPort()

    def enable(self, timeout_sec: float = 3.0, sleep_fn=time.sleep) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._sdk.EnablePiper():
                return
            sleep_fn(0.05)
        raise TimeoutError("Piper 使能超时")

    def disable(self) -> None:
        if hasattr(self._sdk, "DisablePiper"):
            self._sdk.DisablePiper()

    def emergency_stop(self) -> None:
        if hasattr(self._sdk, "EmergencyStop"):
            self._sdk.EmergencyStop(0x01)

    def read_state(self) -> RobotState:
        joint = self._sdk.GetArmJointMsgs().joint_state
        high = self._sdk.GetArmHighSpdInfoMsgs()
        status = self._sdk.GetArmStatus().arm_status

        position = [
            math.radians(joint.joint_1 * 1e-3),
            math.radians(joint.joint_2 * 1e-3),
            math.radians(joint.joint_3 * 1e-3),
            math.radians(joint.joint_4 * 1e-3),
            math.radians(joint.joint_5 * 1e-3),
            math.radians(joint.joint_6 * 1e-3),
        ]
        velocity = [
            high.motor_1.motor_speed * 1e-3,
            high.motor_2.motor_speed * 1e-3,
            high.motor_3.motor_speed * 1e-3,
            high.motor_4.motor_speed * 1e-3,
            high.motor_5.motor_speed * 1e-3,
            high.motor_6.motor_speed * 1e-3,
        ]
        effort = [
            high.motor_1.effort * 1e-3,
            high.motor_2.effort * 1e-3,
            high.motor_3.effort * 1e-3,
            high.motor_4.effort * 1e-3,
            high.motor_5.effort * 1e-3,
            high.motor_6.effort * 1e-3,
        ]
        err_status = status.err_status
        has_error = any(
            [
                err_status.joint_1_angle_limit,
                err_status.joint_2_angle_limit,
                err_status.joint_3_angle_limit,
                err_status.joint_4_angle_limit,
                err_status.joint_5_angle_limit,
                err_status.joint_6_angle_limit,
                err_status.communication_status_joint_1,
                err_status.communication_status_joint_2,
                err_status.communication_status_joint_3,
                err_status.communication_status_joint_4,
                err_status.communication_status_joint_5,
                err_status.communication_status_joint_6,
            ]
        ) or status.arm_status != 0

        return RobotState(
            timestamp=time.monotonic(),
            position=position,
            velocity=velocity,
            effort=effort,
            arm_status_code=status.arm_status,
            ctrl_mode=status.ctrl_mode,
            mode_feed=status.mode_feed,
            has_error=has_error,
        )

    def send_joint_position(self, positions_rad: list[float], speed_rate: int) -> None:
        if len(positions_rad) != 6:
            raise ValueError("positions_rad 必须包含 6 个关节")
        joints_mdeg = [round(math.degrees(value) * 1000.0) for value in positions_rad]
        self._sdk.MotionCtrl_2(0x01, 0x01, int(speed_rate), 0x00)
        self._sdk.JointCtrl(*joints_mdeg)

    def send_mit_command(
        self,
        positions_rad: list[float],
        velocities_rad_s: list[float],
        kp: list[float],
        kd: list[float],
        feedforward_torque: list[float],
    ) -> None:
        self._sdk.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        for index in range(6):
            self._sdk.JointMitCtrl(
                index + 1,
                float(positions_rad[index]),
                float(velocities_rad_s[index]),
                float(kp[index]),
                float(kd[index]),
                float(feedforward_torque[index]),
            )

