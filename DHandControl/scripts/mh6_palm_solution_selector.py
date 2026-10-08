"""Stateful, continuity-aware selection of MH6 palm solver branches."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

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
    solver_diagnostics: Optional[Dict[str, Any]] = None
    candidate_motor_position_delta: Optional[List[int]] = None
    max_motor_step: Optional[List[float]] = None
    jump_rejection_reason: Optional[str] = None
    position_valid_candidate_count: Optional[int] = None
    branch_selection: Optional[Dict[str, Any]] = None


class PalmSolutionSelector:
    """Select bounded branches using calibrated or integer position continuity."""

    def __init__(
        self,
        motor_weights: Sequence[float] = (1.0, 1.0, 1.0),
        max_normalized_speed_per_sec: Optional[float] = 2.0,
        nominal_dt: float = 0.05,
        max_dt: float = 0.10,
        max_motor_step: Optional[Sequence[float]] = None,
        selection_policy: str = "calibrated",
        branch_switch_penalty: float = 0.005,
        allow_unguarded: bool = False,
        fixed_branch_id: Optional[str] = None,
    ) -> None:
        if len(motor_weights) != 3 or any(float(weight) <= 0 for weight in motor_weights):
            raise ValueError("motor_weights must contain three positive values")
        if max_normalized_speed_per_sec is not None and (
            not math.isfinite(float(max_normalized_speed_per_sec))
            or max_normalized_speed_per_sec <= 0.0
        ):
            raise ValueError("max_normalized_speed_per_sec must be positive")
        if max_motor_step is not None:
            if len(max_motor_step) != 3 or any(
                isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0
                for value in max_motor_step
            ):
                raise ValueError("max_motor_step must contain three finite positive values")
            max_motor_step = tuple(float(value) for value in max_motor_step)
        if selection_policy not in ("calibrated", "integer_continuous"):
            raise ValueError("selection_policy must be calibrated or integer_continuous")
        if fixed_branch_id not in (None, "plus_acos", "minus_acos"):
            raise ValueError("fixed_branch_id must be plus_acos, minus_acos or None")
        if not math.isfinite(branch_switch_penalty) or branch_switch_penalty < 0:
            raise ValueError("branch_switch_penalty must be finite and nonnegative")
        if max_normalized_speed_per_sec is None and max_motor_step is None and not allow_unguarded:
            raise ValueError("a position or speed guard must be configured")
        if nominal_dt <= 0.0:
            raise ValueError("nominal_dt must be positive")
        if max_dt <= 0.0:
            raise ValueError("max_dt must be positive")
        self.motor_weights = tuple(float(weight) for weight in motor_weights)
        self.max_normalized_speed_per_sec = (
            float(max_normalized_speed_per_sec)
            if max_normalized_speed_per_sec is not None else None
        )
        self.max_motor_step = max_motor_step
        self.selection_policy = selection_policy
        self.fixed_branch_id = fixed_branch_id
        self.branch_switch_penalty = branch_switch_penalty if selection_policy == "integer_continuous" and fixed_branch_id is None else 0.0
        self.nominal_dt = float(nominal_dt)
        self.max_dt = float(max_dt)
        self.previous_valid_input: Optional[Tuple[float, float, float]] = None
        self.previous_motor: Optional[List[float]] = None
        self.previous_timestamp: Optional[float] = None
        self.previous_branch_id: Optional[str] = None

    def reset(self) -> None:
        self.previous_valid_input = None
        self.previous_motor = None
        self.previous_timestamp = None
        self.previous_branch_id = None

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
        branch_ids: Optional[Sequence[Optional[str]]] = None,
    ) -> PalmSelectionResult:
        requested = tuple(float(value) for value in requested_input)
        if len(requested) != 3 or not all(math.isfinite(value) for value in requested):
            raise ValueError("requested_input must contain three finite values")

        raw_solutions = list(solutions)
        if branch_ids is None:
            branch_ids = [None] * len(raw_solutions)
        if len(branch_ids) != len(raw_solutions) or any(
            branch is not None and (not isinstance(branch, str) or not branch)
            for branch in branch_ids
        ):
            raise ValueError("branch_ids must align with solutions and contain nonempty strings or None")
        valid = [
            (index, [float(value) for value in solution])
            for index, solution in enumerate(raw_solutions)
            if self._is_valid_solution(solution)
        ]
        branch_valid = [item for item in valid if self.fixed_branch_id is None or
                        branch_ids[item[0]] == self.fixed_branch_id]
        reference = self.previous_motor or list(PALM_NEUTRAL_MOTORS)
        first_output = self.previous_motor is None
        reference_integer = [int(round(value)) for value in reference]
        previous_branch = self.previous_branch_id
        scores = []
        for index, solution in enumerate(raw_solutions):
            within_limits = self._is_valid_solution(solution)
            score = {"candidate_index": index, "branch_id": branch_ids[index],
                     "fixed_branch_allowed": self.fixed_branch_id is None or branch_ids[index] == self.fixed_branch_id,
                     "within_motor_limits": within_limits, "position_guard_passed": False,
                     "integer_motor": None, "position_delta": None,
                     "motion_cost": None, "switch_penalty": None, "total_cost": None}
            if within_limits:
                integer = [int(round(value)) for value in solution]
                delta = [a-b for a, b in zip(integer, reference_integer)]
                if self.selection_policy == "integer_continuous":
                    motion = sum(weight * (change / (high-low)) ** 2
                                 for weight, change, (low, high) in zip(
                                     self.motor_weights, delta, PALM_MOTOR_SAFE_LIMITS.values()))
                else:
                    motion = self._distance(solution, reference) ** 2
                switching = previous_branch is not None and branch_ids[index] is not None and branch_ids[index] != previous_branch
                penalty = self.branch_switch_penalty if switching else 0.0
                position_passed = self.max_motor_step is None or all(
                    abs(change) <= cap for change, cap in zip(delta, self.max_motor_step))
                score.update(integer_motor=integer, position_delta=delta, motion_cost=motion,
                             switch_penalty=penalty, total_cost=motion+penalty,
                             position_guard_passed=position_passed)
            scores.append(score)

        def diagnostics(index=None, reason=None, accepted=False):
            candidate_branch = branch_ids[index] if index is not None else None
            return {"policy": self.selection_policy, "switch_penalty": self.branch_switch_penalty,
                    "fixed_branch_id": self.fixed_branch_id,
                    "fixed_branch_valid_candidate_count": len(branch_valid),
                    "reference_motor_integer": reference_integer,
                    "previous_branch_id": previous_branch, "candidate_branch_id": candidate_branch,
                    "selected_branch_id": self.previous_branch_id,
                    "branch_changed": bool(accepted and previous_branch is not None and
                                           candidate_branch is not None and candidate_branch != previous_branch),
                    "proposed_candidate_index": index, "reason": reason,
                    "candidate_scores": scores}

        if not branch_valid:
            reason = "no_valid_candidate"
            if self.fixed_branch_id is not None:
                reason = ("fixed_branch_out_of_limits" if self.fixed_branch_id in branch_ids
                          else "fixed_branch_unavailable")
                status = "HELD_" + reason.upper()
            else:
                status = "HELD_NO_SOLUTION" if not raw_solutions else "HELD_NO_VALID_SOLUTION"
            if self.previous_motor is None:
                status = status.replace("HELD_", "HELD_NEUTRAL_")
            self.previous_timestamp = timestamp if timestamp is not None else self.previous_timestamp
            result = PalmSelectionResult(
                requested, list(reference), status, True, len(raw_solutions), len(valid),
                None, None, None,
            )
            result.max_motor_step = list(self.max_motor_step) if self.max_motor_step is not None else None
            result.position_valid_candidate_count = 0
            result.branch_selection = diagnostics(reason=reason)
            return result

        # Position limits apply to the actual integer command, independently of
        # timestamps. Keep all passing branches before choosing the nearest one.
        def position_delta(solution):
            return [int(round(current))-int(round(previous))
                    for current, previous in zip(solution, reference)]

        position_valid = branch_valid
        if self.max_motor_step is not None:
            position_valid = [item for item in branch_valid if all(
                abs(delta) <= limit for delta, limit in zip(
                    position_delta(item[1]), self.max_motor_step))]
        eligible = position_valid or branch_valid
        def choice_key(item):
            index, solution = item
            if self.selection_policy == "calibrated":
                return (scores[index]["motion_cost"], index)
            return (scores[index]["total_cost"],
                    0 if previous_branch is not None and branch_ids[index] == previous_branch else 1,
                    scores[index]["motion_cost"], branch_ids[index] or "",
                    tuple(scores[index]["integer_motor"]))
        selected_index, selected = min(eligible, key=choice_key)
        distance = math.sqrt(scores[selected_index]["motion_cost"])
        jump = self._max_jump(selected, reference)

        dt = min(self.nominal_dt, self.max_dt)
        if timestamp is not None and self.previous_timestamp is not None:
            dt = min(
                max(float(timestamp) - self.previous_timestamp, 0.0),
                self.max_dt,
            )
        allowed_jump = (
            self.max_normalized_speed_per_sec * dt
            if self.max_normalized_speed_per_sec is not None else None
        )
        position_rejected = not position_valid
        speed_rejected = allowed_jump is not None and jump > allowed_jump + 1e-12
        motor_delta = position_delta(selected)
        if position_rejected or speed_rejected:
            self.previous_timestamp = (
                timestamp if timestamp is not None else self.previous_timestamp
            )
            status = (
                "HELD_JUMP_REJECTED"
                if self.previous_motor is not None
                else "HELD_NEUTRAL_INITIAL_JUMP_REJECTED"
            )
            result = PalmSelectionResult(
                requested,
                list(reference),
                status,
                True,
                len(raw_solutions),
                len(valid),
                selected_index,
                distance,
                jump,
            )
            result.candidate_motor_position_delta = motor_delta
            result.max_motor_step = list(self.max_motor_step) if self.max_motor_step is not None else None
            result.jump_rejection_reason = "position_jump" if position_rejected else "speed_limit"
            result.position_valid_candidate_count = len(position_valid)
            result.branch_selection = diagnostics(selected_index, result.jump_rejection_reason)
            return result

        self.previous_valid_input = requested
        self.previous_motor = (list(scores[selected_index]["integer_motor"])
                               if self.selection_policy == "integer_continuous" else list(selected))
        self.previous_branch_id = branch_ids[selected_index]
        self.previous_timestamp = timestamp if timestamp is not None else self.previous_timestamp
        result = PalmSelectionResult(
            requested, list(self.previous_motor), "SELECTED", False, len(raw_solutions),
            len(valid), selected_index, distance, jump,
        )
        result.candidate_motor_position_delta = motor_delta
        result.max_motor_step = list(self.max_motor_step) if self.max_motor_step is not None else None
        result.position_valid_candidate_count = len(position_valid)
        nearest_motion = min(eligible, key=lambda item: scores[item[0]]["motion_cost"])[0]
        if self.fixed_branch_id is not None:
            reason = "fixed_branch"
        elif first_output:
            reason = "initial_nearest"
        elif previous_branch is None or self.previous_branch_id is None:
            reason = "nearest_without_branch_history"
        elif len(valid) == 1:
            reason = "only_valid_candidate"
        elif (scores[selected_index]["motion_cost"] > scores[nearest_motion]["motion_cost"] + 1e-15
              and branch_ids[selected_index] == previous_branch):
            reason = "stay_branch_hysteresis"
        elif (self.selection_policy == "integer_continuous" and branch_ids[selected_index] == previous_branch
              and any(branch_ids[index] != previous_branch and
                      abs(scores[index]["motion_cost"]-scores[selected_index]["motion_cost"]) <= 1e-15
                      for index, _ in eligible if index != selected_index)):
            reason = "integer_tie_keep_branch"
        elif self.previous_branch_id != previous_branch:
            reason = "nearest_branch_switch"
        else:
            reason = "nearest_same_branch"
        result.branch_selection = diagnostics(selected_index, reason, accepted=True)
        return result


class PalmInputSlewLimiter:
    """Rate-limit signed palm intent before closed-loop solving."""

    def __init__(
        self,
        max_speed_per_sec: Sequence[float] = (2.0, 2.0, 2.0),
        nominal_dt: float = 0.05,
        max_dt: float = 0.10,
    ) -> None:
        if len(max_speed_per_sec) != 3 or any(
            float(speed) <= 0.0 for speed in max_speed_per_sec
        ):
            raise ValueError("max_speed_per_sec must contain three positive values")
        if nominal_dt <= 0.0 or max_dt <= 0.0:
            raise ValueError("nominal_dt and max_dt must be positive")
        self.max_speed_per_sec = tuple(float(speed) for speed in max_speed_per_sec)
        self.nominal_dt = float(nominal_dt)
        self.max_dt = float(max_dt)
        self.applied = [0.0, 0.0, 0.0]
        self.previous_timestamp: Optional[float] = None

    def reset(self, values: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        if len(values) != 3:
            raise ValueError("values must contain three entries")
        self.applied = [min(max(float(value), -1.0), 1.0) for value in values]
        self.previous_timestamp = None

    def hold(self, values: Sequence[float]) -> None:
        """Restore the last feasible applied input without resetting time."""

        if len(values) != 3:
            raise ValueError("values must contain three entries")
        self.applied = [min(max(float(value), -1.0), 1.0) for value in values]

    def apply(
        self,
        target: Sequence[float],
        timestamp: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        if len(target) != 3:
            raise ValueError("target must contain three values")
        target_values = [float(value) for value in target]
        if not all(math.isfinite(value) for value in target_values):
            raise ValueError("target must contain finite values")
        target_values = [min(max(value, -1.0), 1.0) for value in target_values]

        dt = min(self.nominal_dt, self.max_dt)
        if timestamp is not None and self.previous_timestamp is not None:
            dt = min(max(float(timestamp) - self.previous_timestamp, 0.0), self.max_dt)

        for index, (current, requested, speed) in enumerate(
            zip(self.applied, target_values, self.max_speed_per_sec)
        ):
            max_delta = speed * dt
            delta = min(max(requested - current, -max_delta), max_delta)
            self.applied[index] = current + delta

        self.previous_timestamp = timestamp if timestamp is not None else self.previous_timestamp
        return tuple(self.applied)
