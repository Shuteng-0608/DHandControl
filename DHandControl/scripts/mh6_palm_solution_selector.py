"""Stateful, continuity-aware selection of MH6 palm solver branches."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple

from mh6_palm_calibration import (
    PALM_MOTOR_CALIBRATION,
    PALM_MOTOR_SAFE_LIMITS,
    PALM_NEUTRAL_MOTORS,
)


@dataclass
class PalmSelectionResult:
    requested_input: Tuple[float, float, float]
    selected_motor: List[float]
    status: str
    held_previous: bool
    candidate_count: int
    valid_candidate_count: int
    selected_candidate_index: Optional[int]
    normalized_distance: Optional[float]
    normalized_jump: Optional[float]


class PalmSolutionSelector:
    """Select the valid branch with minimum calibrated motion from last output."""

    def __init__(
        self,
        motor_weights: Sequence[float] = (1.0, 1.0, 1.0),
        max_normalized_speed_per_sec: float = 2.0,
        nominal_dt: float = 0.05,
    ) -> None:
        if len(motor_weights) != 3 or any(float(weight) <= 0 for weight in motor_weights):
            raise ValueError("motor_weights must contain three positive values")
        if max_normalized_speed_per_sec <= 0.0:
            raise ValueError("max_normalized_speed_per_sec must be positive")
        if nominal_dt <= 0.0:
            raise ValueError("nominal_dt must be positive")
        self.motor_weights = tuple(float(weight) for weight in motor_weights)
        self.max_normalized_speed_per_sec = float(max_normalized_speed_per_sec)
        self.nominal_dt = float(nominal_dt)
        self.previous_valid_input: Optional[Tuple[float, float, float]] = None
        self.previous_motor: Optional[List[float]] = None
        self.previous_timestamp: Optional[float] = None

    def reset(self) -> None:
        self.previous_valid_input = None
        self.previous_motor = None
        self.previous_timestamp = None

    @staticmethod
    def _motor_to_signed(motor_id: int, value: float) -> float:
        points = PALM_MOTOR_CALIBRATION[motor_id]
        outward = float(points["outward"])
        neutral = float(points["neutral"])
        inward = float(points["inward"])
        value = float(value)

        outward_direction = math.copysign(1.0, outward - neutral)
        on_outward_side = (value - neutral) * outward_direction >= 0.0
        endpoint = outward if on_outward_side else inward
        span = abs(endpoint - neutral)
        if span <= 1e-12:
            return 0.0
        magnitude = min(abs(value - neutral) / span, 1.0)
        return -magnitude if on_outward_side else magnitude

    @classmethod
    def motor_to_signed(cls, solution: Sequence[float]) -> Tuple[float, float, float]:
        if len(solution) != 3:
            raise ValueError("palm motor solution must contain three values")
        return tuple(
            cls._motor_to_signed(motor_id, solution[motor_id - 1])
            for motor_id in (1, 2, 3)
        )

    @staticmethod
    def _is_valid_solution(solution: Sequence[float]) -> bool:
        if len(solution) != 3:
            return False
        for motor_id, value in zip((1, 2, 3), solution):
            try:
                value = float(value)
            except (TypeError, ValueError):
                return False
            low, high = PALM_MOTOR_SAFE_LIMITS[motor_id]
            if not math.isfinite(value) or not low <= value <= high:
                return False
        return True

    def _distance(self, solution: Sequence[float], reference: Sequence[float]) -> float:
        solution_q = self.motor_to_signed(solution)
        reference_q = self.motor_to_signed(reference)
        return math.sqrt(
            sum(
                weight * (current - previous) ** 2
                for weight, current, previous in zip(
                    self.motor_weights,
                    solution_q,
                    reference_q,
                )
            )
        )

    def _max_jump(self, solution: Sequence[float], reference: Sequence[float]) -> float:
        solution_q = self.motor_to_signed(solution)
        reference_q = self.motor_to_signed(reference)
        return max(abs(current - previous) for current, previous in zip(solution_q, reference_q))

    def select(
        self,
        requested_input: Sequence[float],
        solutions: Sequence[Sequence[float]],
        timestamp: Optional[float] = None,
    ) -> PalmSelectionResult:
        requested = tuple(float(value) for value in requested_input)
        if len(requested) != 3 or not all(math.isfinite(value) for value in requested):
            raise ValueError("requested_input must contain three finite values")

        raw_solutions = list(solutions)
        valid = [
            (index, [float(value) for value in solution])
            for index, solution in enumerate(raw_solutions)
            if self._is_valid_solution(solution)
        ]
        reference = self.previous_motor or list(PALM_NEUTRAL_MOTORS)

        if not valid:
            status = "HELD_NO_SOLUTION" if not raw_solutions else "HELD_NO_VALID_SOLUTION"
            if self.previous_motor is None:
                status = status.replace("HELD_", "HELD_NEUTRAL_")
            self.previous_timestamp = timestamp if timestamp is not None else self.previous_timestamp
            return PalmSelectionResult(
                requested, list(reference), status, True, len(raw_solutions), 0,
                None, None, None,
            )

        selected_index, selected = min(
            valid,
            key=lambda item: self._distance(item[1], reference),
        )
        distance = self._distance(selected, reference)
        jump = self._max_jump(selected, reference)

        if self.previous_motor is not None:
            dt = self.nominal_dt
            if timestamp is not None and self.previous_timestamp is not None:
                dt = max(float(timestamp) - self.previous_timestamp, 0.0)
            allowed_jump = self.max_normalized_speed_per_sec * dt
            if jump > allowed_jump + 1e-12:
                self.previous_timestamp = timestamp if timestamp is not None else self.previous_timestamp
                return PalmSelectionResult(
                    requested, list(self.previous_motor), "HELD_JUMP_REJECTED", True,
                    len(raw_solutions), len(valid), selected_index, distance, jump,
                )

        self.previous_valid_input = requested
        self.previous_motor = list(selected)
        self.previous_timestamp = timestamp if timestamp is not None else self.previous_timestamp
        return PalmSelectionResult(
            requested, list(selected), "SELECTED", False, len(raw_solutions),
            len(valid), selected_index, distance, jump,
        )
