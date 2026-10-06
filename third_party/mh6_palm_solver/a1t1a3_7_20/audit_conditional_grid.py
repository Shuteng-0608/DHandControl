"""Reproducible 21^3 acceptance audit for the conditional MH6 mapping."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver


def audit(step=0.05):
    count = round(1.0 / step)
    if not math.isclose(count * step, 1.0, abs_tol=1e-12):
        raise ValueError("step must divide [0,1] exactly")

    solver = MH6PalmSolver()
    grid = np.linspace(0.0, 1.0, count + 1)
    points = 0
    solution_count = 0
    zero_solution_points = 0
    max_rotation_error = 0.0
    max_translation_error_mm = 0.0
    motor_min = [math.inf, math.inf, math.inf]
    motor_max = [-math.inf, -math.inf, -math.inf]
    outside_motor_range = [0, 0, 0]

    for u1 in grid:
        for u2 in grid:
            for u3 in grid:
                points += 1
                angles = solver.map_normalized(u1, u2, u3)
                raw = solver.solve_remaining(*angles)
                motors = solver.solve_motor(*angles)
                if not raw:
                    zero_solution_points += 1
                if len(raw) != len(motors):
                    raise RuntimeError("angle and motor solution counts differ")
                solution_count += len(raw)
                for solution, motor in zip(raw, motors):
                    max_rotation_error = max(max_rotation_error, float(solution[3]))
                    max_translation_error_mm = max(
                        max_translation_error_mm, float(solution[4]))
                    for axis, value in enumerate(motor):
                        motor_min[axis] = min(motor_min[axis], value)
                        motor_max[axis] = max(motor_max[axis], value)
                        outside_motor_range[axis] += not 0.0 <= value <= 1000.0

    result = {
        "step": step,
        "points": points,
        "solution_count": solution_count,
        "zero_solution_points": zero_solution_points,
        "motor_min": motor_min,
        "motor_max": motor_max,
        "outside_motor_range_0_1000": outside_motor_range,
        "max_rotation_error": max_rotation_error,
        "max_translation_error_mm": max_translation_error_mm,
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = audit(args.step)
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")

    expected_points = (round(1.0 / args.step) + 1) ** 3
    accepted = (
        result["points"] == expected_points
        and result["zero_solution_points"] == 0
        and result["solution_count"] == 2 * expected_points
        and result["max_rotation_error"] < 1e-12
        and result["max_translation_error_mm"] < 1e-4
        and result["outside_motor_range_0_1000"] == [0, 0, 0]
    )
    if not accepted:
        raise SystemExit("conditional-grid acceptance thresholds failed")


if __name__ == "__main__":
    main()
