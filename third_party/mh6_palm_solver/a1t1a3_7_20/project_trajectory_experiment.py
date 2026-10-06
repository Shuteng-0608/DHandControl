"""Project an input trajectory into the feasible MH6 workspace.

The projection is an experiment, not a production controller.  It moves only
frames whose closure cosine is below a requested safety target, then selects the
analytical solution branch closest to the previous motor command.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver


ROOT = Path(__file__).resolve().parent
INPUT = ROOT / "right_power_grasp_02_solver_timeseries.csv"
OUTPUT = ROOT / "right_power_grasp_02_projected_experiment.csv"
SUMMARY = ROOT / "trajectory_projection_experiment_results.json"


def stats(values):
    a = np.asarray(values, dtype=float)
    return {
        "min": float(a.min()),
        "median": float(np.median(a)),
        "p95": float(np.quantile(a, 0.95)),
        "max": float(a.max()),
    }


def main():
    rows = list(csv.DictReader(INPUT.open(encoding="utf-8-sig", newline="")))
    solver = MH6PalmSolver()
    target = -0.999
    previous = np.array([500.0, 500.0, 247.0])
    output_rows = []
    projection_distances = []
    degree_deltas = []
    selected_steps = []
    first_branch_steps = []
    previous_first = None
    projected_count = 0
    original_failures = 0

    for row in rows:
        original_angles = np.array([
            float(row["input_arpha2_deg"]),
            float(row["input_arpha3_deg"]),
            float(row["input_theta1_deg"]),
        ])
        result = solver.solve_motor_safe(
            *original_angles,
            previous_motor=previous,
            project_invalid=True,
            enforce_margin=True,
            target_abs_value=abs(target),
        )
        if not result["success"]:
            raise AssertionError((row["frame_index"], result))
        before = result["diagnostic"]["closure_value"]
        if not result["diagnostic"]["valid"]:
            original_failures += 1
        after = result["used_diagnostic"]["closure_value"]
        if abs(after) > abs(target) + 1e-9:
            raise AssertionError(
                f"frame {row['frame_index']} missed closure target: {after}")
        projected_angles = np.asarray(result["used_angles"], dtype=float)
        moved = result["projected"]
        projected_count += int(moved)
        projection_distances.append(
            result["projection"]["normalized_distance"] if result["projection"] else 0.0)
        degree_deltas.append(np.abs(projected_angles - original_angles))

        solutions = result["solutions"]
        if len(solutions) != 2:
            raise AssertionError(
                f"frame {row['frame_index']} returned {len(solutions)} solutions")
        solution_array = np.asarray(solutions, dtype=float)
        selected_index = result["selected_index"]
        selected = np.asarray(result["selected"], dtype=float)
        selected_steps.append(float(np.linalg.norm(selected - previous)))
        previous = selected

        first = solution_array[0]
        if previous_first is not None:
            first_branch_steps.append(float(np.linalg.norm(first - previous_first)))
        previous_first = first

        output_rows.append({
            "frame_index": row["frame_index"],
            "time_s": row["time_s"],
            "projected": int(moved),
            "closure_value_before": f"{before:.9f}",
            "closure_value_after": f"{after:.9f}",
            "normalized_projection_distance": f"{projection_distances[-1]:.9f}",
            "original_arpha2_deg": f"{original_angles[0]:.9f}",
            "original_arpha3_deg": f"{original_angles[1]:.9f}",
            "original_theta1_deg": f"{original_angles[2]:.9f}",
            "projected_arpha2_deg": f"{projected_angles[0]:.9f}",
            "projected_arpha3_deg": f"{projected_angles[1]:.9f}",
            "projected_theta1_deg": f"{projected_angles[2]:.9f}",
            "solution_count": len(solutions),
            "selected_solution": selected_index + 1,
            "selected_api_m1": f"{selected[0]:.4f}",
            "selected_api_m2": f"{selected[1]:.4f}",
            "selected_api_m3": f"{selected[2]:.4f}",
        })

    with OUTPUT.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(output_rows[0]))
        writer.writeheader()
        writer.writerows(output_rows)

    deltas = np.asarray(degree_deltas)
    summary = {
        "frames": len(rows),
        "original_failures": original_failures,
        "target_closure_value": target,
        "projected_frames_including_near_boundary_valid_frames": projected_count,
        "failures_after_projection": 0,
        "normalized_projection_distance_all_frames": stats(projection_distances),
        "absolute_degree_delta_all_frames": {
            "arpha2": stats(deltas[:, 0]),
            "arpha3": stats(deltas[:, 1]),
            "theta1": stats(deltas[:, 2]),
        },
        "motor_step_l2": {
            "continuity_branch_selection": stats(selected_steps[1:]),
            "always_first_solution": stats(first_branch_steps),
        },
        "note": "motor columns use the current solver API order; the source time-series CSV stores m1 and m3 in the opposite order",
    }
    SUMMARY.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"\nWrote {OUTPUT}")
    print(f"Wrote {SUMMARY}")


if __name__ == "__main__":
    main()
