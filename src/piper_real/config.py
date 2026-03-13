from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import ast
from typing import Iterable


@dataclass
class RealExperimentConfig:
    can_port: str = "can0"
    control_mode: str = "state_only"
    control_rate_hz: float = 100.0
    record_rate_hz: float = 100.0
    output_csv: Path = Path("data/real_benchmark_data.csv")
    duration: float = 10.0
    speed_rate: int = 20
    home_position: list[float] = field(default_factory=lambda: [0.0] * 6)
    position_amplitude: list[float] = field(default_factory=lambda: [0.0] * 6)
    position_frequency: list[float] = field(default_factory=lambda: [0.2] * 6)
    mit_kp: list[float] = field(default_factory=lambda: [10.0] * 6)
    mit_kd: list[float] = field(default_factory=lambda: [0.8] * 6)
    trajectory_period: float = 10.0
    trajectory_harmonics: int = 5
    trajectory_coefficient_scale: float = 0.12
    trajectory_seed: int = 42
    max_samples: int | None = None
    joint_soft_limits: list[tuple[float, float]] = field(default_factory=list)
    max_command_velocity: list[float] = field(default_factory=lambda: [1.5] * 6)
    max_command_torque: list[float] = field(default_factory=lambda: [8.0] * 6)
    move_to_home_before_start: bool = True
    home_position_tolerance: list[float] = field(default_factory=lambda: [0.05] * 6)
    home_speed_rate: int = 15
    home_timeout_sec: float = 8.0
    startup_timeout_sec: float = 3.0

    def __post_init__(self) -> None:
        self.home_position = _ensure_length(self.home_position, "home_position")
        self.position_amplitude = _ensure_length(
            self.position_amplitude, "position_amplitude"
        )
        self.position_frequency = _ensure_length(
            self.position_frequency, "position_frequency"
        )
        self.mit_kp = _ensure_length(self.mit_kp, "mit_kp")
        self.mit_kd = _ensure_length(self.mit_kd, "mit_kd")
        self.max_command_velocity = _ensure_length(
            self.max_command_velocity, "max_command_velocity"
        )
        self.max_command_torque = _ensure_length(
            self.max_command_torque, "max_command_torque"
        )
        self.home_position_tolerance = _ensure_length(
            self.home_position_tolerance, "home_position_tolerance"
        )
        if self.joint_soft_limits:
            if len(self.joint_soft_limits) != 6:
                raise ValueError("joint_soft_limits 必须包含 6 个关节范围")
            self.joint_soft_limits = [
                (float(limit[0]), float(limit[1])) for limit in self.joint_soft_limits
            ]
        self.output_csv = Path(self.output_csv)


def load_real_experiment_config(
    config_path: Path, repo_root: Path | None = None
) -> RealExperimentConfig:
    repo_root = repo_root or Path(__file__).resolve().parents[2]
    parsed: dict[str, object] = {}
    for raw_line in Path(config_path).read_text(encoding="utf-8").splitlines():
        line = _strip_comment(raw_line).strip()
        if not line:
            continue
        key, value = _split_key_value(line)
        parsed[key] = _parse_value(value)

    output_csv = Path(parsed.get("output_csv", "data/real_benchmark_data.csv"))
    if not output_csv.is_absolute():
        output_csv = repo_root / output_csv

    return RealExperimentConfig(
        can_port=str(parsed.get("can_port", "can0")),
        control_mode=str(parsed.get("control_mode", "state_only")),
        control_rate_hz=float(parsed.get("control_rate_hz", 100.0)),
        record_rate_hz=float(parsed.get("record_rate_hz", 100.0)),
        output_csv=output_csv,
        duration=float(parsed.get("duration", 10.0)),
        speed_rate=int(parsed.get("speed_rate", 20)),
        home_position=_coerce_float_list(parsed.get("home_position", [0.0] * 6)),
        position_amplitude=_coerce_float_list(
            parsed.get("position_amplitude", [0.0] * 6)
        ),
        position_frequency=_coerce_float_list(
            parsed.get("position_frequency", [0.2] * 6)
        ),
        mit_kp=_coerce_float_list(parsed.get("mit_kp", [10.0] * 6)),
        mit_kd=_coerce_float_list(parsed.get("mit_kd", [0.8] * 6)),
        trajectory_period=float(parsed.get("trajectory_period", 10.0)),
        trajectory_harmonics=int(parsed.get("trajectory_harmonics", 5)),
        trajectory_coefficient_scale=float(
            parsed.get("trajectory_coefficient_scale", 0.12)
        ),
        trajectory_seed=int(parsed.get("trajectory_seed", 42)),
        max_samples=(
            None if parsed.get("max_samples") is None else int(parsed["max_samples"])
        ),
        joint_soft_limits=_coerce_joint_limits(parsed.get("joint_soft_limits", [])),
        max_command_velocity=_coerce_float_list(
            parsed.get("max_command_velocity", [1.5] * 6)
        ),
        max_command_torque=_coerce_float_list(
            parsed.get("max_command_torque", [8.0] * 6)
        ),
        move_to_home_before_start=bool(parsed.get("move_to_home_before_start", True)),
        home_position_tolerance=_coerce_float_list(
            parsed.get("home_position_tolerance", [0.05] * 6)
        ),
        home_speed_rate=int(parsed.get("home_speed_rate", 15)),
        home_timeout_sec=float(parsed.get("home_timeout_sec", 8.0)),
        startup_timeout_sec=float(parsed.get("startup_timeout_sec", 3.0)),
    )


def _strip_comment(line: str) -> str:
    in_quote = False
    quote_char = ""
    for index, char in enumerate(line):
        if char in {"'", '"'}:
            if not in_quote:
                in_quote = True
                quote_char = char
            elif quote_char == char:
                in_quote = False
                quote_char = ""
        elif char == "#" and not in_quote:
            return line[:index]
    return line


def _split_key_value(line: str) -> tuple[str, str]:
    if ":" not in line:
        raise ValueError(f"无法解析配置行: {line}")
    key, value = line.split(":", 1)
    return key.strip(), value.strip()


def _parse_value(value: str):
    if not value:
        return None
    if value[0] in "[{('\"" or value in {"true", "false", "True", "False"}:
        try:
            return ast.literal_eval(value)
        except (ValueError, SyntaxError):
            pass
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        pass
    return value.strip("'\"")


def _coerce_float_list(value: object) -> list[float]:
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    raise ValueError(f"期望数组配置，收到: {value!r}")


def _coerce_joint_limits(value: object) -> list[tuple[float, float]]:
    if value in (None, []):
        return []
    if not isinstance(value, (list, tuple)):
        raise ValueError("joint_soft_limits 必须是二维数组")
    return [(float(item[0]), float(item[1])) for item in value]


def _ensure_length(values: Iterable[float], field_name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != 6:
        raise ValueError(f"{field_name} 必须包含 6 个元素")
    return result
