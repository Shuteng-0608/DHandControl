"""Observe palm solver returns without actuator history or control guards."""

from collections import Counter
import json
import math
from pathlib import Path
import time
from typing import Dict, Optional

from mh6_palm_calibration import PALM_MOTOR_SAFE_LIMITS


def validate_solver_test_config(solver) -> None:
    """Evaluate the requested pose, rather than a projected replacement."""
    calibration = getattr(solver, "calibration", None)
    if calibration is not None and (
        calibration.project_invalid or calibration.enforce_margin
    ):
        raise ValueError("--solver-only requires project_invalid=false and enforce_margin=false")


def evaluate_palm_solver(result: Dict, solver) -> Dict:
    """Return every motor branch, distinguishing geometry from motor limits.

    The adapter retains both accepted and rejected motor branches in candidates.
    Their presence measures geometric solvability; its success flag alone would
    incorrectly classify a solved pose with out-of-range motors as no geometry.
    No previous motor is supplied and no branch is committed or held.
    """
    validate_solver_test_config(solver)
    palm = result["palm_command"]
    inputs = {
        "palm_flexion": float(palm["vertical"]),
        "palm_cross": float(palm["lateral"]),
        "thumb_inward": float(palm["thumb_rotation_command"]),
    }
    started = time.perf_counter()
    teleop_solver = getattr(solver, "solve_motor_from_teleop", None)
    if callable(teleop_solver):
        adapter = teleop_solver(
            vertical=inputs["palm_flexion"],
            lateral=inputs["palm_cross"],
            thumb_rotation_command=inputs["thumb_inward"],
            previous_motor=None,
        )
        angles = adapter["requested_angles"]
        candidates = adapter["candidates"]
    else:
        # Retain the legacy API's positional contract (h, thumb, v).
        values = (inputs["palm_flexion"], inputs["thumb_inward"], inputs["palm_cross"])
        angles = solver.map_normalized(*values)
        candidates = solver.solve_motor_from_normalized(*values)
        adapter = None
    solve_time_ms = (time.perf_counter() - started) * 1000.0
    candidates = [[float(value) for value in row] for row in candidates]
    valid = [
        row for row in candidates
        if len(row) == 3 and all(
            math.isfinite(value) and low <= value <= high
            for value, (low, high) in zip(
                row, (PALM_MOTOR_SAFE_LIMITS[motor_id] for motor_id in (1, 2, 3))
            )
        )
    ]
    observation = {
        "input": inputs,
        "requested_angles": [float(value) for value in angles],
        "angle_order": ["arpha2", "arpha3", "theta1"],
        "motor_order": [1, 2, 3],
        "candidates": candidates,
        "valid_motor_candidates": valid,
        "candidate_count": len(candidates),
        "valid_motor_candidate_count": len(valid),
        "has_solution": bool(candidates),
        "has_valid_motor_solution": bool(valid),
        "status": "SOLVED" if valid else (
            "NO_VALID_MOTOR_SOLUTION" if candidates else "NO_SOLUTION"
        ),
        "solve_time_ms": solve_time_ms,
    }
    if adapter is not None:
        observation["adapter"] = adapter
    return observation


class PalmSolverTestRecorder:
    """Save per-frame observations and frame-based solution rates on exit."""

    def __init__(self, log_path: Optional[str], summary_path: Optional[str] = None):
        self.log_path = Path(log_path).expanduser() if log_path else None
        self.summary_path = (
            Path(summary_path).expanduser() if summary_path else
            self.log_path.with_suffix(".summary.json") if self.log_path else None
        )
        if self.log_path and self.summary_path.resolve() == self.log_path.resolve():
            raise ValueError("solver summary and frame log must use different files")
        self.file = None
        self.metadata = {}
        self.frames = 0
        self.geometry_solved_frames = 0
        self.valid_motor_frames = 0
        self.first_timestamp = None
        self.last_timestamp = None
        self.status_counts = Counter()
        self.solve_time_sum_ms = 0.0
        self.solve_time_max_ms = 0.0

    def start(self, metadata: Dict) -> None:
        self.metadata = metadata
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.file = self.log_path.open("w", encoding="utf-8", buffering=1)

    def write(self, timestamp: float, raw_mapping: Dict, observation: Dict) -> None:
        if self.first_timestamp is None:
            self.first_timestamp = float(timestamp)
        self.last_timestamp = float(timestamp)
        record = {
            "format_version": 1,
            "mode": "solver_only",
            "frame_index": self.frames,
            "timestamp": self.last_timestamp - self.first_timestamp,
            "input_source": self.metadata["input_source"],
            "raw_mapping": raw_mapping,
            "solver": observation,
        }
        if self.file is not None:
            self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.frames += 1
        self.geometry_solved_frames += int(observation["has_solution"])
        self.valid_motor_frames += int(observation["has_valid_motor_solution"])
        self.status_counts[observation["status"]] += 1
        self.solve_time_sum_ms += observation["solve_time_ms"]
        self.solve_time_max_ms = max(self.solve_time_max_ms, observation["solve_time_ms"])

    def summary(self, termination: str = "completed") -> Dict:
        return {
            "format_version": 1,
            "mode": "solver_only",
            "termination": termination,
            "metadata": self.metadata,
            "total_frames": self.frames,
            "geometry_solved_frames": self.geometry_solved_frames,
            "valid_motor_frames": self.valid_motor_frames,
            "no_solution_frames": self.frames - self.geometry_solved_frames,
            "motor_limit_rejected_frames": self.geometry_solved_frames - self.valid_motor_frames,
            "geometry_solution_rate": self.geometry_solved_frames / self.frames if self.frames else None,
            "valid_motor_solution_rate": self.valid_motor_frames / self.frames if self.frames else None,
            "duration_seconds": (
                self.last_timestamp - self.first_timestamp if self.frames else 0.0
            ),
            "status_counts": dict(self.status_counts),
            "mean_solve_time_ms": self.solve_time_sum_ms / self.frames if self.frames else None,
            "max_solve_time_ms": self.solve_time_max_ms if self.frames else None,
        }

    def close(self, termination: str = "completed") -> None:
        if self.file is not None:
            self.file.close()
        summary = self.summary(termination)
        if self.summary_path is not None:
            self.summary_path.parent.mkdir(parents=True, exist_ok=True)
            self.summary_path.write_text(
                json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
                encoding="utf-8",
            )
