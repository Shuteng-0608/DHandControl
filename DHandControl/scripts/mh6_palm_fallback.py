"""Safe degraded palm control for prolonged MH6 solver failures."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

from mh6_palm_calibration import PALM_MOTOR_CALIBRATION


@dataclass(frozen=True)
class PalmFallbackResult:
    mode: str
    status: str
    signed_closure: float
    requested_closure: float
    applied_closure: float
    selected_motor: List[float]
    no_solution_duration: float
    entry_distance: Optional[float]


class PalmFallbackController:
    """Fall back to one calibrated synchronized palm-closure trajectory.

    The controller never treats the three motors as independent channels.  It
    rate-limits a single closure coordinate, then evaluates all three motors on
    the same piecewise-linear calibrated path.
    """

    def __init__(
        self,
        *,
        vertical_weight: float = 0.8,
        activation_delay: float = 0.5,
        max_closure_speed_per_sec: float = 1.0,
        neutral_closure: float = 0.25,
        entry_distance_limit: float = 0.08,
        recovery_frames: int = 5,
        recovery_distance_limit: float = 0.05,
        neutral_recovery_band: float = 0.05,
        nominal_dt: float = 0.05,
        max_dt: float = 0.10,
    ) -> None:
        if not 0.0 <= vertical_weight <= 1.0:
            raise ValueError("vertical_weight must be within [0,1]")
        if activation_delay < 0.0:
            raise ValueError("activation_delay must be non-negative")
        if max_closure_speed_per_sec <= 0.0:
            raise ValueError("max_closure_speed_per_sec must be positive")
        if not 0.0 < neutral_closure < 1.0:
            raise ValueError("neutral_closure must be within (0,1)")
        if entry_distance_limit < 0.0 or recovery_distance_limit < 0.0:
            raise ValueError("distance limits must be non-negative")
        if recovery_frames <= 0:
            raise ValueError("recovery_frames must be positive")
        if neutral_recovery_band < 0.0:
            raise ValueError("neutral_recovery_band must be non-negative")
        if nominal_dt <= 0.0 or max_dt <= 0.0:
            raise ValueError("nominal_dt and max_dt must be positive")

        self.vertical_weight = float(vertical_weight)
        self.lateral_weight = 1.0 - self.vertical_weight
        self.activation_delay = float(activation_delay)
        self.max_closure_speed_per_sec = float(max_closure_speed_per_sec)
        self.neutral_closure = float(neutral_closure)
        self.entry_distance_limit = float(entry_distance_limit)
        self.recovery_frames = int(recovery_frames)
        self.recovery_distance_limit = float(recovery_distance_limit)
        self.neutral_recovery_band = float(neutral_recovery_band)
        self.nominal_dt = float(nominal_dt)
        self.max_dt = float(max_dt)

        self.active = False
        self.applied_closure = self.neutral_closure
        self.no_solution_since: Optional[float] = None
        self.previous_timestamp: Optional[float] = None
        self.recovery_count = 0

    def reset(self) -> None:
        self.active = False
        self.applied_closure = self.neutral_closure
        self.no_solution_since = None
        self.previous_timestamp = None
        self.recovery_count = 0

    def combine_intent(self, vertical: float, lateral: float) -> float:
        values = (float(vertical), float(lateral))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("fallback palm inputs must be finite")
        signed = self.vertical_weight * values[0] + self.lateral_weight * values[1]
        return min(max(signed, -1.0), 1.0)

    def signed_to_closure(self, signed_closure: float) -> float:
        signed = min(max(float(signed_closure), -1.0), 1.0)
        if signed <= 0.0:
            return self.neutral_closure * (signed + 1.0)
        return self.neutral_closure + (1.0 - self.neutral_closure) * signed

    def closure_to_motors(self, closure: float) -> List[float]:
        closure = min(max(float(closure), 0.0), 1.0)
        if closure <= self.neutral_closure:
            ratio = closure / self.neutral_closure
            start_name, end_name = "outward", "neutral"
        else:
            ratio = (closure - self.neutral_closure) / (1.0 - self.neutral_closure)
            start_name, end_name = "neutral", "inward"

        return [
            float(PALM_MOTOR_CALIBRATION[motor_id][start_name])
            + (
                float(PALM_MOTOR_CALIBRATION[motor_id][end_name])
                - float(PALM_MOTOR_CALIBRATION[motor_id][start_name])
            )
            * ratio
            for motor_id in (1, 2, 3)
        ]

    @staticmethod
    def _motor_to_signed(motor_id: int, value: float) -> float:
        points = PALM_MOTOR_CALIBRATION[motor_id]
        outward = float(points["outward"])
        neutral = float(points["neutral"])
        inward = float(points["inward"])
        direction = math.copysign(1.0, outward - neutral)
        on_outward_side = (float(value) - neutral) * direction >= 0.0
        endpoint = outward if on_outward_side else inward
        span = abs(endpoint - neutral)
        if span <= 1e-12:
            return 0.0
        magnitude = min(abs(float(value) - neutral) / span, 1.0)
        return -magnitude if on_outward_side else magnitude

    @classmethod
    def _motor_distance(cls, a: Sequence[float], b: Sequence[float]) -> float:
        if len(a) != 3 or len(b) != 3:
            raise ValueError("palm motor values must contain three entries")
        qa = [cls._motor_to_signed(index, value) for index, value in enumerate(a, 1)]
        qb = [cls._motor_to_signed(index, value) for index, value in enumerate(b, 1)]
        return math.sqrt(sum((left - right) ** 2 for left, right in zip(qa, qb)))

    def nearest_closure(self, motors: Sequence[float]) -> Tuple[float, float]:
        """Project a motor triplet onto the synchronized fallback path."""

        if len(motors) != 3 or not all(math.isfinite(float(value)) for value in motors):
            raise ValueError("palm motor values must contain three finite entries")
        signed = [
            self._motor_to_signed(motor_id, value)
            for motor_id, value in enumerate(motors, 1)
        ]

        # On each calibrated segment every fallback motor has the same signed
        # normalized coordinate.  Project onto the outward and inward diagonal
        # segments, then retain the closer one.
        outward_q = min(max(sum(signed) / 3.0, -1.0), 0.0)
        inward_q = min(max(sum(signed) / 3.0, 0.0), 1.0)
        outward_distance = math.sqrt(
            sum((value - outward_q) ** 2 for value in signed)
        )
        inward_distance = math.sqrt(
            sum((value - inward_q) ** 2 for value in signed)
        )
        if outward_distance <= inward_distance:
            closure = self.neutral_closure * (outward_q + 1.0)
            return closure, outward_distance
        closure = self.neutral_closure + (1.0 - self.neutral_closure) * inward_q
        return closure, inward_distance

    def _dt(self, timestamp: Optional[float]) -> float:
        dt = min(self.nominal_dt, self.max_dt)
        if timestamp is not None and self.previous_timestamp is not None:
            dt = min(max(float(timestamp) - self.previous_timestamp, 0.0), self.max_dt)
        self.previous_timestamp = (
            float(timestamp) if timestamp is not None else self.previous_timestamp
        )
        return dt

    def _advance_closure(self, target: float, dt: float) -> None:
        max_delta = self.max_closure_speed_per_sec * dt
        delta = min(max(target - self.applied_closure, -max_delta), max_delta)
        self.applied_closure += delta

    def update(
        self,
        vertical: float,
        lateral: float,
        solver_selection,
        *,
        timestamp: Optional[float] = None,
    ) -> PalmFallbackResult:
        signed_closure = self.combine_intent(vertical, lateral)
        requested_closure = self.signed_to_closure(signed_closure)
        dt = self._dt(timestamp)
        now = float(timestamp) if timestamp is not None else None
        pure_no_solution = solver_selection.status.endswith("NO_SOLUTION")

        if pure_no_solution:
            if self.no_solution_since is None:
                self.no_solution_since = now
        else:
            self.no_solution_since = None

        no_solution_duration = 0.0
        if self.no_solution_since is not None:
            if now is None:
                no_solution_duration = self.activation_delay
            else:
                no_solution_duration = max(now - self.no_solution_since, 0.0)

        entry_closure, entry_distance = self.nearest_closure(
            solver_selection.selected_motor
        )
        just_entered = False

        if not self.active:
            status = "SOLVER_ACTIVE"
            if pure_no_solution:
                status = "FALLBACK_PENDING"
                if no_solution_duration >= self.activation_delay:
                    if entry_distance <= self.entry_distance_limit:
                        self.active = True
                        self.applied_closure = entry_closure
                        self.recovery_count = 0
                        just_entered = True
                        status = "FALLBACK_ENTERED"
                    else:
                        status = "FALLBACK_ENTRY_UNSAFE"

            if not self.active:
                return PalmFallbackResult(
                    "SOLVER",
                    status,
                    signed_closure,
                    requested_closure,
                    self.applied_closure,
                    list(solver_selection.selected_motor),
                    no_solution_duration,
                    entry_distance,
                )

        # The entry frame only joins the closest synchronized path point.  Motion
        # toward the requested closure starts on the next frame so switching
        # modes cannot itself add a full rate-limited step.
        if not just_entered:
            self._advance_closure(requested_closure, dt)
        fallback_motor = self.closure_to_motors(self.applied_closure)

        near_neutral = (
            abs(requested_closure - self.neutral_closure)
            <= self.neutral_recovery_band
        )
        solver_close = (
            solver_selection.status == "SELECTED"
            and self._motor_distance(
                fallback_motor,
                solver_selection.selected_motor,
            )
            <= self.recovery_distance_limit
        )
        if near_neutral and solver_close:
            self.recovery_count += 1
        else:
            self.recovery_count = 0

        if self.recovery_count >= self.recovery_frames:
            self.active = False
            self.no_solution_since = None
            self.recovery_count = 0
            return PalmFallbackResult(
                "SOLVER",
                "FALLBACK_EXITED",
                signed_closure,
                requested_closure,
                self.applied_closure,
                list(solver_selection.selected_motor),
                no_solution_duration,
                entry_distance,
            )

        return PalmFallbackResult(
            "FALLBACK",
            "FALLBACK_ENTERED" if just_entered else "FALLBACK_ACTIVE",
            signed_closure,
            requested_closure,
            self.applied_closure,
            fallback_motor,
            no_solution_duration,
            entry_distance,
        )
