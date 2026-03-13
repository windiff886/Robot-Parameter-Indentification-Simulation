from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import sys
from typing import Callable

from .config import load_real_experiment_config
from .sdk_adapter import PiperSdkAdapter


@dataclass
class BridgeSession:
    adapter: object
    time_step: float = 0.01
    startup_timeout_sec: float = 3.0
    sleep_fn: Callable[[float], None] = lambda _: None
    move_to_home_before_start: bool = True
    home_position: list[float] = field(default_factory=lambda: [0.0] * 6)
    home_position_tolerance: list[float] = field(default_factory=lambda: [0.05] * 6)
    home_speed_rate: int = 15
    home_timeout_sec: float = 8.0
    joint_soft_limits: list[tuple[float, float]] = field(default_factory=list)
    max_command_velocity: list[float] = field(default_factory=lambda: [1.5] * 6)
    max_command_torque: list[float] = field(default_factory=lambda: [8.0] * 6)
    initialized: bool = False

    def handle_command(self, line: str) -> str:
        command = line.strip()
        if not command:
            return "ERROR empty_command"

        if command == "INIT":
            self.adapter.connect()
            self.adapter.enable(self.startup_timeout_sec, self.sleep_fn)
            state = self.adapter.read_state()
            if self.move_to_home_before_start:
                state = self._move_to_home_if_needed(state)
            self.initialized = True
            return self._format_state(state)

        if command == "STOP":
            self.adapter.disable()
            self.adapter.disconnect()
            self.initialized = False
            return "OK"

        if command.startswith("STEP "):
            if not self.initialized:
                return "ERROR not_initialized"
            payload = [float(item) for item in command.split()[1:]]
            if len(payload) != 30:
                return f"ERROR invalid_step_payload:{len(payload)}"
            position = payload[0:6]
            velocity = payload[6:12]
            kp = payload[12:18]
            kd = payload[18:24]
            torque = payload[24:30]
            limit_error = self._validate_command(position, velocity, torque)
            if limit_error:
                self.adapter.emergency_stop()
                return limit_error
            self.adapter.send_mit_command(position, velocity, kp, kd, torque)
            self.sleep_fn(self.time_step)
            return self._format_state(self.adapter.read_state())

        return "ERROR unsupported_command"

    @staticmethod
    def _format_state(state) -> str:
        parts = ["STATE"]
        parts.extend(f"{value:.6f}" for value in state.position)
        parts.extend(f"{value:.6f}" for value in state.velocity)
        parts.extend(f"{value:.6f}" for value in state.effort)
        return " ".join(parts)

    def _validate_command(
        self, position: list[float], velocity: list[float], torque: list[float]
    ) -> str | None:
        for index, value in enumerate(position):
            if self.joint_soft_limits:
                lower, upper = self.joint_soft_limits[index]
                if value < lower or value > upper:
                    return f"ERROR command_position_limit joint={index + 1}"

        for index, value in enumerate(velocity):
            if abs(value) > self.max_command_velocity[index]:
                return f"ERROR command_velocity_limit joint={index + 1}"

        for index, value in enumerate(torque):
            if abs(value) > self.max_command_torque[index]:
                return f"ERROR command_torque_limit joint={index + 1}"

        return None

    def _move_to_home_if_needed(self, state):
        if self._is_within_home_tolerance(state.position):
            return state

        deadline = self.home_timeout_sec
        elapsed = 0.0
        while elapsed < deadline:
            self.adapter.send_joint_position(self.home_position, self.home_speed_rate)
            self.sleep_fn(self.time_step)
            elapsed += self.time_step
            state = self.adapter.read_state()
            if self._is_within_home_tolerance(state.position):
                return state

        self.adapter.emergency_stop()
        raise RuntimeError("机械臂未能在限定时间内回到安全初始位")

    def _is_within_home_tolerance(self, position: list[float]) -> bool:
        return all(
            abs(current - target) <= tolerance
            for current, target, tolerance in zip(
                position, self.home_position, self.home_position_tolerance
            )
        )


def create_default_session(config_path: Path) -> BridgeSession:
    repo_root = Path(__file__).resolve().parents[2]
    config = load_real_experiment_config(config_path, repo_root=repo_root)
    adapter = PiperSdkAdapter(can_port=config.can_port)
    return BridgeSession(
        adapter=adapter,
        time_step=1.0 / max(config.control_rate_hz, 1.0),
        startup_timeout_sec=config.startup_timeout_sec,
        move_to_home_before_start=config.move_to_home_before_start,
        home_position=config.home_position,
        home_position_tolerance=config.home_position_tolerance,
        home_speed_rate=config.home_speed_rate,
        home_timeout_sec=config.home_timeout_sec,
        joint_soft_limits=config.joint_soft_limits,
        max_command_velocity=config.max_command_velocity,
        max_command_torque=config.max_command_torque,
    )


def run_stdio_loop(config_path: Path) -> int:
    session = create_default_session(config_path)
    try:
        for raw_line in sys.stdin:
            response = session.handle_command(raw_line)
            print(response, flush=True)
            if response == "OK" and raw_line.strip() == "STOP":
                break
    except Exception as exc:  # pragma: no cover - 由运行期兜底
        print(f"ERROR {exc}", flush=True)
        return 1
    return 0
