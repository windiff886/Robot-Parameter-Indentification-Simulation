from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

from .config import RealExperimentConfig
from .recorder import CsvRecorder, RealSample
from .sdk_adapter import PiperSdkAdapter, RobotState
from .trajectory import FourierExcitationTrajectory


@dataclass
class CommandFrame:
    position: list[float]
    velocity: list[float]
    torque: list[float]


class RealExperimentRunner:
    def __init__(
        self,
        config: RealExperimentConfig,
        adapter_factory: Callable[[RealExperimentConfig], PiperSdkAdapter] | None = None,
        recorder_factory: Callable[[object, int], CsvRecorder] | None = None,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        self._config = config
        self._adapter_factory = adapter_factory or (
            lambda cfg: PiperSdkAdapter(can_port=cfg.can_port)
        )
        self._recorder_factory = recorder_factory or (
            lambda path, dof: CsvRecorder(path, dof)
        )
        self._sleep_fn = sleep_fn
        self._trajectory = self._build_trajectory()

    def run(self) -> None:
        adapter = self._adapter_factory(self._config)
        recorder = self._recorder_factory(self._config.output_csv, 6)
        start_time = time.monotonic()
        sample_index = 0
        try:
            adapter.connect()
            adapter.enable(self._config.startup_timeout_sec, self._sleep_fn)

            while True:
                now = time.monotonic()
                elapsed = now - start_time
                if elapsed >= self._config.duration:
                    break
                if self._config.max_samples is not None and sample_index >= self._config.max_samples:
                    break

                state = adapter.read_state()
                self._assert_safe(state)
                command = self._build_command(elapsed)

                if self._config.control_mode == "position_validation":
                    adapter.send_joint_position(command.position, self._config.speed_rate)
                elif self._config.control_mode in {
                    "mit_identification",
                    "excitation_trajectory",
                }:
                    adapter.send_mit_command(
                        command.position,
                        command.velocity,
                        self._config.mit_kp,
                        self._config.mit_kd,
                        command.torque,
                    )

                recorder.record(
                    RealSample(
                        timestamp=elapsed,
                        position=state.position,
                        velocity=state.velocity,
                        effort=state.effort,
                        commanded_effort=command.torque,
                    )
                )
                sample_index += 1
                self._sleep_fn(max(0.0, 1.0 / max(self._config.control_rate_hz, 1.0)))
        except Exception:
            adapter.emergency_stop()
            raise
        finally:
            recorder.close()
            adapter.disable()
            adapter.disconnect()

    def _assert_safe(self, state: RobotState) -> None:
        if state.has_error:
            raise RuntimeError(f"机械臂状态异常，错误码: {state.arm_status_code}")
        if self._config.joint_soft_limits:
            for index, (lower, upper) in enumerate(self._config.joint_soft_limits):
                if not lower <= state.position[index] <= upper:
                    raise RuntimeError(f"关节 {index + 1} 超出软限位")

    def _build_command(self, elapsed: float) -> CommandFrame:
        if self._config.control_mode == "state_only":
            return CommandFrame(
                position=list(self._config.home_position),
                velocity=[0.0] * 6,
                torque=[0.0] * 6,
            )
        if self._config.control_mode in {"mit_identification", "excitation_trajectory"}:
            point = self._trajectory.evaluate(elapsed)
            return CommandFrame(
                position=point.position,
                velocity=point.velocity,
                torque=[0.0] * 6,
            )

        position = []
        velocity = []
        for base, amplitude, frequency in zip(
            self._config.home_position,
            self._config.position_amplitude,
            self._config.position_frequency,
        ):
            omega = 2.0 * math.pi * frequency
            position.append(base + amplitude * math.sin(omega * elapsed))
            velocity.append(amplitude * omega * math.cos(omega * elapsed))
        return CommandFrame(position=position, velocity=velocity, torque=[0.0] * 6)

    def _build_trajectory(self) -> FourierExcitationTrajectory:
        return FourierExcitationTrajectory.from_random(
            q0=self._config.home_position,
            period=self._config.trajectory_period,
            n_harmonics=self._config.trajectory_harmonics,
            coefficient_scale=self._config.trajectory_coefficient_scale,
            seed=self._config.trajectory_seed,
        )
