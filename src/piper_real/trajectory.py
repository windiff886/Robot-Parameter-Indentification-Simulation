from __future__ import annotations

from dataclasses import dataclass
import math
import random


@dataclass
class TrajectoryPoint:
    position: list[float]
    velocity: list[float]
    acceleration: list[float]


class FourierExcitationTrajectory:
    def __init__(
        self,
        q0: list[float],
        period: float,
        a_coefficients: list[list[float]],
        b_coefficients: list[list[float]],
    ) -> None:
        self.q0 = [float(value) for value in q0]
        self.period = float(period)
        self.omega = 2.0 * math.pi / self.period
        self.a_coefficients = [list(map(float, row)) for row in a_coefficients]
        self.b_coefficients = [list(map(float, row)) for row in b_coefficients]
        self._validate()
        self._apply_initial_condition_constraints()

    @classmethod
    def from_random(
        cls,
        q0: list[float],
        period: float,
        n_harmonics: int,
        coefficient_scale: float,
        seed: int,
    ) -> "FourierExcitationTrajectory":
        rng = random.Random(seed)
        a_coefficients = []
        b_coefficients = []
        for _ in range(len(q0)):
            a_coefficients.append(
                [rng.uniform(-coefficient_scale, coefficient_scale) for _ in range(n_harmonics)]
            )
            b_coefficients.append(
                [rng.uniform(-coefficient_scale, coefficient_scale) for _ in range(n_harmonics)]
            )
        return cls(q0, period, a_coefficients, b_coefficients)

    def evaluate(self, t: float) -> TrajectoryPoint:
        position = list(self.q0)
        velocity = [0.0] * len(self.q0)
        acceleration = [0.0] * len(self.q0)

        for joint_index in range(len(self.q0)):
            for harmonic_index, (a_ij, b_ij) in enumerate(
                zip(
                    self.a_coefficients[joint_index],
                    self.b_coefficients[joint_index],
                )
            ):
                order = float(harmonic_index + 1)
                omega_j = self.omega * order
                cos_wt = math.cos(omega_j * t)
                sin_wt = math.sin(omega_j * t)
                position[joint_index] += a_ij / omega_j * sin_wt - b_ij / omega_j * cos_wt
                velocity[joint_index] += a_ij * cos_wt + b_ij * sin_wt
                acceleration[joint_index] += self.omega * (
                    b_ij * order * cos_wt - a_ij * order * sin_wt
                )

        return TrajectoryPoint(position=position, velocity=velocity, acceleration=acceleration)

    def _validate(self) -> None:
        n_dof = len(self.q0)
        if n_dof != 6:
            raise ValueError("Fourier 激励轨迹当前仅支持 6 轴 Piper")
        if len(self.a_coefficients) != n_dof or len(self.b_coefficients) != n_dof:
            raise ValueError("Fourier 系数维度必须与关节数一致")
        harmonic_count = len(self.a_coefficients[0])
        if harmonic_count == 0:
            raise ValueError("至少需要一个谐波")
        for row in self.a_coefficients + self.b_coefficients:
            if len(row) != harmonic_count:
                raise ValueError("所有关节的谐波数量必须一致")

    def _apply_initial_condition_constraints(self) -> None:
        for joint_index in range(len(self.q0)):
            sum_a = sum(self.a_coefficients[joint_index][1:])
            self.a_coefficients[joint_index][0] = -sum_a

            if len(self.b_coefficients[joint_index]) >= 2:
                s1 = 0.0
                s2 = 0.0
                for harmonic_index in range(2, len(self.b_coefficients[joint_index])):
                    order = float(harmonic_index + 1)
                    b_ij = self.b_coefficients[joint_index][harmonic_index]
                    s1 += b_ij / (self.omega * order)
                    s2 += b_ij * order

                b1 = (s1 * self.omega - s2) * (2.0 / 3.0)
                b0 = -s1 * self.omega - b1 / 2.0
                self.b_coefficients[joint_index][0] = b0
                self.b_coefficients[joint_index][1] = b1
            else:
                self.b_coefficients[joint_index][0] = 0.0

