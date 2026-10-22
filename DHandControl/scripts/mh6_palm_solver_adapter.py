"""Adapt palm intent to the unmodified v2 palm solver.

This branch defaults to inward-only conditional workspace generation, with
zero input at the Solver-calibrated flat pose [247,500,500]. The original
geometry, motor calibration and native normalized API remain unchanged.

This adapter produces candidates and diagnostics without actuator state.
The runtime selector applies fixed-assembly and integer position guards.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from mh6_palm_calibration import PALM_MOTOR_SAFE_LIMITS
from mh6_palm_solver_v2 import MH6PalmSolver
from mh6_inward_workspace import InwardPalmWorkspace, CLOSING_MOTOR_LIMITS


DEFAULT_CALIBRATION_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "mh6_palm_adapter_neutral_to_grasp.json"
)
LEGACY_CALIBRATION_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "mh6_palm_adapter_calibration.json"
)
CONDITIONAL_CALIBRATION_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "mh6_palm_adapter_workspace_conditional.json"
)
MOTOR_RANGE_NEUTRAL = (31.1 / 121.9, 59.0 / 239.0, 8.6 / 32.3)
MAPPING_MODES = ("motor_range", "workspace_conditional", "workspace_independent")


def _triplet(values: Sequence[float], name: str) -> Tuple[float, float, float]:
    if isinstance(values, (str, bytes, dict)):
        raise ValueError(f"{name} must contain three finite numbers")
    try:
        if len(values) != 3 or any(isinstance(value, bool) for value in values):
            raise ValueError
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain three finite numbers") from exc
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{name} must contain three finite numbers")
    return converted


@dataclass(frozen=True)
class PalmAdapterCalibration:
    """Anchors in solver angle order (arpha2, arpha3, theta1), hence (h, r, v)."""

    mapping_mode: str = "motor_range"
    outward: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    neutral: Optional[Tuple[float, float, float]] = None
    inward: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    project_invalid: bool = False
    enforce_margin: bool = False
    target_abs_value: float = 0.999
    max_projection_delta_deg: Optional[Tuple[float, float, float]] = None
    input_domain: str = "signed"
    workspace_profile: str = "native"

    def __post_init__(self) -> None:
        if self.input_domain not in ("signed", "neutral_to_grasp"):
            raise ValueError("input_domain must be signed or neutral_to_grasp")
        if self.mapping_mode not in MAPPING_MODES:
            raise ValueError(f"mapping_mode must be one of {MAPPING_MODES}")
        if self.workspace_profile not in ("native", "closing_only"):
            raise ValueError("workspace_profile must be native or closing_only")
        if self.workspace_profile == "closing_only" and (
                self.mapping_mode != "workspace_conditional" or self.input_domain != "neutral_to_grasp"
                or self.project_invalid or self.enforce_margin):
            raise ValueError("closing_only requires unprojected neutral_to_grasp conditional mapping")
        neutral = self.neutral
        if neutral is None:
            if self.input_domain == "neutral_to_grasp" and self.mapping_mode != "motor_range":
                neutral = (0.0, 0.0, 0.0)
            elif self.mapping_mode != "motor_range":
                raise ValueError("workspace modes require explicit neutral coordinates")
            else:
                neutral = MOTOR_RANGE_NEUTRAL
        for name, values in (
            ("outward", self.outward), ("neutral", neutral), ("inward", self.inward)
        ):
            values = _triplet(values, name)
            if not all(0.0 <= value <= 1.0 for value in values):
                raise ValueError(f"{name} coordinates must lie in [0, 1]")
            object.__setattr__(self, name, values)
        if self.input_domain == "neutral_to_grasp":
            if not all(neutral < inward for neutral, inward in zip(self.neutral, self.inward)):
                raise ValueError("each inward axis must exceed the neutral coordinate")
        elif not all(
            outward < neutral < inward
            for outward, neutral, inward in zip(self.outward, self.neutral, self.inward)
        ):
            raise ValueError("each axis must satisfy outward < neutral < inward")
        if self.workspace_profile == "closing_only" and (
                self.neutral != (0., 0., 0.) or self.inward != (1., 1., 1.)):
            raise ValueError("closing_only requires zero-to-one workspace anchors")
        if not isinstance(self.project_invalid, bool) or not isinstance(self.enforce_margin, bool):
            raise ValueError("project_invalid and enforce_margin must be booleans")
        if self.enforce_margin and not self.project_invalid:
            # The upstream enforce_margin flag can project even when
            # project_invalid=False.  Make that permission explicit here.
            raise ValueError("enforce_margin requires project_invalid=True")
        if isinstance(self.target_abs_value, bool):
            raise ValueError("target_abs_value must lie in (0, 1)")
        try:
            target = float(self.target_abs_value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("target_abs_value must lie in (0, 1)") from exc
        if not math.isfinite(target) or not 0.0 < target < 1.0:
            raise ValueError("target_abs_value must lie in (0, 1)")
        object.__setattr__(self, "target_abs_value", target)
        if self.max_projection_delta_deg is not None:
            limits = _triplet(self.max_projection_delta_deg, "max_projection_delta_deg")
            if not all(value > 0.0 for value in limits):
                raise ValueError("max_projection_delta_deg values must be positive")
            object.__setattr__(self, "max_projection_delta_deg", limits)
        if self.project_invalid and self.max_projection_delta_deg is None:
            raise ValueError("projection requires explicit max_projection_delta_deg")

    @classmethod
    def from_file(cls, path: str) -> "PalmAdapterCalibration":
        data = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("palm adapter calibration must be a JSON object")
        allowed = set(cls.__dataclass_fields__)
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(f"unknown palm adapter calibration fields: {sorted(unknown)}")
        return cls(**data)


class PalmSolverAdapter:
    """Accept semantic h/v/r; the default input domain is [0,1]."""

    def __init__(
        self,
        calibration: Optional[PalmAdapterCalibration] = None,
        solver: Optional[MH6PalmSolver] = None,
        motor_limits: Optional[Dict[int, Tuple[float, float]]] = None,
    ) -> None:
        self.calibration = (calibration if calibration is not None else
                            PalmAdapterCalibration.from_file(str(DEFAULT_CALIBRATION_PATH)))
        self.solver = solver if solver is not None else MH6PalmSolver()
        # Standalone historical callers retain their explicit legacy profile.
        # The teleop entry supplies the unified configuration instead.
        defaults = CLOSING_MOTOR_LIMITS if self.calibration.workspace_profile == "closing_only" else PALM_MOTOR_SAFE_LIMITS
        self.motor_limits = dict(defaults if motor_limits is None else motor_limits)
        if set(self.motor_limits) != {1, 2, 3}:
            raise ValueError("motor_limits requires physical IDs 1, 2, 3")
        for pair in self.motor_limits.values():
            if len(pair) != 2 or not all(math.isfinite(x) for x in pair) or not 0 <= pair[0] < pair[1] <= 1000:
                raise ValueError("motor_limits must lie inside the Solver's [0,1000] domain")
        self.inward_workspace = (InwardPalmWorkspace(self.solver, self.motor_limits)
                                 if self.calibration.workspace_profile == "closing_only" else None)

    def _validate_intent(self, vertical, lateral, thumb_rotation_command):
        values = _triplet((vertical, lateral, thumb_rotation_command), "teleop intent")
        lower = 0.0 if self.calibration.input_domain == "neutral_to_grasp" else -1.0
        if not all(lower <= value <= 1.0 for value in values):
            raise ValueError(f"teleop intent must lie in [{lower:g}, 1]")
        return values

    def map_teleop_to_workspace(
        self, vertical: float, lateral: float, thumb_rotation_command: float
    ) -> Tuple[float, float, float]:
        """Return solver-ordered coordinates for vertical/thumb/lateral intent.

        The public arguments are semantic (h, v, r), while the solver and robot
        calibration use (arpha2, arpha3, theta1), corresponding to (h, r, v).
        The inward profile maps 0/1 to neutral/inward solver coordinates.
        Historical signed profiles map -1/0/+1 to their three anchors.
        """

        h, v, r = self._validate_intent(vertical, lateral, thumb_rotation_command)
        return tuple(
            neutral + (-value) * (outward - neutral)
            if value < 0.0 else neutral + value * (inward - neutral)
            for value, outward, neutral, inward in zip(
                (h, r, v),
                self.calibration.outward,
                self.calibration.neutral,
                self.calibration.inward,
            )
        )

    def _candidate_branch_ids(self, angles, candidates):
        """Identify the +/- acos root, independently of candidate list order.

        The upstream solver sorts roots by tiny rotation residuals, so indices
        are not stable identities. Match its unrounded analytic roots to the
        four-decimal motor targets. Coincident/ambiguous roots stay unidentified
        and incur no switch penalty. This never validates rounded motor poses.
        """
        if not candidates or angles is None:
            return [None] * len(candidates)
        a2, a3, t1 = self.solver.canonicalize_input_angles(*angles)
        vector = self.solver.RyRz(20, a3) @ self.solver.RyRz(225, 0) @ np.array([0, 0, 1])
        phase = math.atan2(vector[1], vector[0])
        roots = []
        raw_roots = self.solver.solve_remaining(a2, a3, t1)
        for _, t3, a1, _, _ in raw_roots:
            sign = math.sin(math.radians(t3) + phase)
            branch = None if abs(sign) < 1e-9 else ("plus_acos" if sign > 0 else "minus_acos")
            if self.inward_workspace is not None and len(raw_roots) == 1:
                # At the flat open pose both analytic roots coincide. Preserve
                # the inward assembly identity, rather than rejecting its origin.
                branch = self.inward_workspace.branch
            motor = [round(247 - (753 / 180) * a3, 4),
                     round(500 - (380 / 90.8) * a2, 4),
                     round(500 + (99 / 23.6) * self.solver._norm_to_180(a1), 4)]
            roots.append((motor, branch))
        identities = []
        for candidate in candidates:
            matches = {branch for motor, branch in roots
                       if max(abs(a-b) for a, b in zip(candidate, motor)) < 1e-6}
            if self.inward_workspace is not None and self.inward_workspace.branch in matches:
                # Distinct analytic roots can also have identical four-decimal
                # API motor outputs near the flat boundary. The command then
                # represents the inward root as well; do not invent a switch.
                identities.append(self.inward_workspace.branch)
            else:
                identities.append(next(iter(matches)) if len(matches) == 1 else None)
        return identities

    def solve_motor_from_teleop(
        self,
        vertical: float,
        lateral: float,
        thumb_rotation_command: float,
        *,
        previous_motor: Optional[Sequence[float]] = None,
    ) -> Dict[str, Any]:
        """Return hardware-ID-ordered candidates and every adaptation stage.

        ``solutions`` satisfies the project's per-motor calibration bounds.
        ``candidates`` retains rejected motor branches for the runtime selector
        and diagnostics.  No motor target is sent or committed by this method.
        """

        signed = self._validate_intent(vertical, lateral, thumb_rotation_command)
        workspace = self.map_teleop_to_workspace(*signed)
        if previous_motor is not None:
            previous_motor = _triplet(previous_motor, "previous_motor")
        calibration = self.calibration
        solve_options = dict(
            previous_motor=previous_motor,
            motor_order="timeseries",
            project_invalid=calibration.project_invalid,
            enforce_margin=calibration.enforce_margin,
            target_abs_value=calibration.target_abs_value,
            max_projection_delta_deg=calibration.max_projection_delta_deg,
        )
        workspace_diagnostic = None
        if self.inward_workspace is not None:
            angles, workspace_diagnostic = self.inward_workspace.map(*workspace)
            core = self.solver.solve_motor_safe(*angles, **solve_options)
            entrypoint = "solve_motor_safe"
        elif calibration.mapping_mode in ("workspace_conditional", "workspace_independent"):
            # Let the normalized API choose angles within the requested workspace.
            # In particular, conditional mapping couples the thumb range to h/v.
            core = self.solver.solve_motor_safe_from_normalized(
                *workspace, mode=calibration.mapping_mode, **solve_options,
            )
            entrypoint = "solve_motor_safe_from_normalized"
        else:
            # Retain the historical flat-neutral comparison, including its
            # sub-1e-10-degree endpoint roundoff correction.
            angles = self.solver.map_normalized(*workspace, mode=calibration.mapping_mode)
            limits = ((-31.1, 90.8), (-180.0, 59.0), (-23.7, 8.6))
            requested = []
            for value, (low, high) in zip(angles, limits):
                if low - 1e-10 <= value < low:
                    value = low
                elif high < value <= high + 1e-10:
                    value = high
                requested.append(value)
            core = self.solver.solve_motor_safe(*requested, **solve_options)
            entrypoint = "solve_motor_safe"
        candidates = [[float(value) for value in row] for row in core["solutions"]]
        # Upstream rejected branches are still in API order [a1, a2, a3].
        candidates.extend(
            self.solver.reorder_motor_solution(row, motor_order="timeseries")
            for row in core["rejected_motor_solutions"]
        )
        branch_ids = self._candidate_branch_ids(core["used_angles"], candidates)
        required_branch = self.inward_workspace.branch if self.inward_workspace is not None else None
        if workspace_diagnostic is not None:
            workspace_diagnostic["branch_coalesced"] = (
                len(candidates) == 1 or len({tuple(row) for row in candidates}) < len(candidates))
        solutions = []
        rejected = []
        for row, branch in zip(candidates, branch_ids):
            violations = [
                motor_id
                for motor_id, value in enumerate(row, start=1)
                if not self.motor_limits[motor_id][0]
                <= value <= self.motor_limits[motor_id][1]
            ]
            if violations or (required_branch is not None and branch != required_branch):
                rejected.append({"motors": row, "limit_violations": violations,
                                 "branch_id": branch,
                                 "branch_rejected": required_branch is not None and branch != required_branch})
            else:
                solutions.append(row)
        success = bool(solutions)
        if success:
            status = "PROJECTED" if core["projected"] else "SOLVED"
            error = None
        elif candidates:
            status = "NO_VALID_MOTOR_SOLUTION"
            error = "calibrated_motor_bounds_rejected_all_solutions"
        else:
            status = "NO_SOLUTION"
            error = core["error"]
        requested_angles = core["requested_angles"]
        used_angles = core["used_angles"]
        canonical = core["canonical_requested_angles"]
        return {
            "success": success,
            "status": status,
            "mapping_mode": calibration.mapping_mode,
            "workspace_profile": calibration.workspace_profile,
            "workspace_diagnostic": workspace_diagnostic,
            "required_branch": required_branch,
            "input_domain": [0, 1] if calibration.input_domain == "neutral_to_grasp" else [-1, 1],
            "solver_entrypoint": entrypoint,
            "semantic_input": dict(zip(
                ("vertical", "lateral", "thumb_rotation_command"), signed
            )),
            "workspace_input": dict(zip(("u1", "u2", "u3"), workspace)),
            "requested_angles": requested_angles,
            "used_angles": used_angles,
            "angle_delta_deg": (
                [used - requested for used, requested in zip(used_angles, canonical)]
                if used_angles is not None else None
            ),
            "angle_order": ["arpha2", "arpha3", "theta1"],
            "angle_input_order": ["vertical", "thumb_rotation_command", "lateral"],
            "motor_order": [1, 2, 3],
            "candidates": candidates,
            "candidate_branch_ids": branch_ids,
            "solutions": solutions,
            # Upstream branch choice obeys its own [0,1000] bounds. It is not
            # necessarily accepted by the narrower per-motor hardware limits.
            "core_selected_motor": core["selected"],
            "core_selected_index": core["selected_index"],
            "rejected_motor_solutions": rejected,
            "projected": core["projected"],
            "projection": core["projection"],
            "projection_within_limits": core["projection_within_limits"],
            "diagnostic": core["diagnostic"],
            "used_diagnostic": core["used_diagnostic"],
            "error": error,
        }
