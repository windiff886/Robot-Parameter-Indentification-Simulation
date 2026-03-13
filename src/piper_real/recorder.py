from __future__ import annotations

from dataclasses import dataclass, field
import csv
from pathlib import Path


@dataclass
class RealSample:
    timestamp: float
    position: list[float]
    velocity: list[float]
    effort: list[float]
    commanded_effort: list[float] = field(default_factory=list)


class CsvRecorder:
    def __init__(self, output_csv: Path, dof: int) -> None:
        self._path = Path(output_csv)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self._path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._dof = dof
        self._last_sample: RealSample | None = None
        self._writer.writerow(self._build_header(dof))

    def record(self, sample: RealSample) -> None:
        qdd = self._estimate_acceleration(sample)
        commanded_effort = sample.commanded_effort or [0.0] * self._dof
        row = [f"{sample.timestamp:.6f}"]
        row.extend(f"{value:.6f}" for value in sample.position[: self._dof])
        row.extend(f"{value:.6f}" for value in sample.velocity[: self._dof])
        row.extend(f"{value:.6f}" for value in qdd)
        row.extend(f"{value:.6f}" for value in sample.effort[: self._dof])
        row.extend(f"{value:.6f}" for value in commanded_effort[: self._dof])
        self._writer.writerow(row)
        self._file.flush()
        self._last_sample = sample

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def _estimate_acceleration(self, sample: RealSample) -> list[float]:
        if self._last_sample is None:
            return [0.0] * self._dof
        dt = sample.timestamp - self._last_sample.timestamp
        if dt <= 0.0:
            return [0.0] * self._dof
        return [
            (sample.velocity[index] - self._last_sample.velocity[index]) / dt
            for index in range(self._dof)
        ]

    @staticmethod
    def _build_header(dof: int) -> list[str]:
        header = ["time"]
        header.extend(f"q{i}" for i in range(dof))
        header.extend(f"qd{i}" for i in range(dof))
        header.extend(f"qdd{i}" for i in range(dof))
        header.extend(f"tau{i}" for i in range(dof))
        header.extend(f"tau_cmd{i}" for i in range(dof))
        return header

