"""Validate the production workspace-aware mapping independently."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver


ROOT = Path(__file__).resolve().parent


def _bisect_reference(solver, a2, t1, left, right, target, iterations=50):
    f_left = target - abs(solver.closure_value(a2, left, t1))
    for _ in range(iterations):
        middle = (left + right) / 2.0
        f_middle = target - abs(solver.closure_value(a2, middle, t1))
        if (f_left >= 0.0) == (f_middle >= 0.0):
            left, f_left = middle, f_middle
        else:
            right = middle
    return (left + right) / 2.0


def numerical_reference_interval(solver, a2, t1, target=0.999, samples=513):
    """Slow scan+bisection reference, independent of the production quadratic."""
    angles = np.linspace(-180.0, 59.0, samples)
    valid = np.abs(solver.closure_value(a2, angles, t1)) <= target
    if not np.any(valid):
        return None
    changes = np.diff(np.pad(valid.astype(np.int8), (1, 1)))
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1) - 1
    intervals = []
    for start, end in zip(starts, ends):
        lower, upper = float(angles[start]), float(angles[end])
        if start > 0:
            lower = _bisect_reference(
                solver, a2, t1, float(angles[start - 1]), lower, target)
        if end + 1 < len(angles):
            upper = _bisect_reference(
                solver, a2, t1, upper, float(angles[end + 1]), target)
        intervals.append((lower, upper))
    return max(intervals, key=lambda pair: pair[1] - pair[0])


def stats(values):
    a = np.asarray(values, dtype=float)
    return {
        "min": float(a.min()),
        "median": float(np.median(a)),
        "p95": float(np.quantile(a, 0.95)),
        "max": float(a.max()),
    }


def main():
    solver = MH6PalmSolver()
    rng = np.random.default_rng(20260718)
    target = 0.999

    # Compare the production quadratic interval with a slow independent method.
    interval_errors = []
    for u1, u3 in rng.random((250, 2)):
        a2 = 5.0 + 80.0 * u1
        t1 = -2.0 - 21.7 * u3
        production = solver.feasible_arpha3_interval(
            a2, t1, target_abs_value=target)
        reference = numerical_reference_interval(solver, a2, t1, target=target)
        interval_errors.append(max(
            abs(production[0] - reference[0]),
            abs(production[1] - reference[1]),
        ))

    random_u = rng.random((5_000, 3))
    mapped = np.array([solver.map_workspace_conditional(*u) for u in random_u])
    values = solver.closure_value(mapped[:, 0], mapped[:, 1], mapped[:, 2])

    solution_counts = []
    rotation_errors = []
    translation_errors = []
    for a2, a3, t1 in mapped[:1_000]:
        solutions = solver.solve_remaining(a2, a3, t1)
        solution_counts.append(len(solutions))
        rotation_errors.extend(s[3] for s in solutions)
        translation_errors.extend(s[4] for s in solutions)

    independent = np.array([
        solver.map_workspace_independent(*u) for u in random_u])
    independent_valid = np.abs(solver.closure_value(
        independent[:, 0], independent[:, 1], independent[:, 2])) <= 1.0

    benchmark_u = rng.random((10_000, 3))
    started = time.perf_counter()
    for u in benchmark_u:
        solver.map_workspace_conditional(*u)
    elapsed = time.perf_counter() - started

    counts = np.asarray(solution_counts)
    results = {
        "analytic_interval_vs_scan_bisection": {
            "comparisons": len(interval_errors),
            "max_endpoint_error_deg": max(interval_errors),
        },
        "conditional_mapping": {
            "random_samples": len(random_u),
            "success": int(np.count_nonzero(np.abs(values) <= target + 1e-12)),
            "coverage": float(np.mean(np.abs(values) <= target + 1e-12)),
            "closure_value": stats(values),
            "production_solver_sample": {
                "samples": len(counts),
                "zero_solution": int(np.count_nonzero(counts == 0)),
                "one_solution": int(np.count_nonzero(counts == 1)),
                "two_solutions": int(np.count_nonzero(counts == 2)),
                "max_rotation_error": float(max(rotation_errors)),
                "max_translation_error_mm": float(max(translation_errors)),
            },
            "performance": {
                "calls": len(benchmark_u),
                "total_seconds": elapsed,
                "microseconds_per_mapping": elapsed / len(benchmark_u) * 1e6,
            },
        },
        "independent_A1_same_random_inputs": {
            "samples": len(random_u),
            "success": int(independent_valid.sum()),
            "coverage": float(independent_valid.mean()),
        },
    }

    accepted = (
        results["analytic_interval_vs_scan_bisection"]["max_endpoint_error_deg"] < 1e-10
        and results["conditional_mapping"]["success"] == len(random_u)
        and results["conditional_mapping"]["production_solver_sample"]["two_solutions"] == len(counts)
        and results["conditional_mapping"]["production_solver_sample"]["zero_solution"] == 0
        and results["conditional_mapping"]["production_solver_sample"]["one_solution"] == 0
        and results["conditional_mapping"]["production_solver_sample"]["max_rotation_error"] < 1e-12
        and results["conditional_mapping"]["production_solver_sample"]["max_translation_error_mm"] < 1e-4
    )
    if not accepted:
        raise SystemExit("workspace-mapping acceptance thresholds failed")

    output = ROOT / "workspace_mapping_experiment_results.json"
    output.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(results, ensure_ascii=False, indent=2))
    print(f"\nWrote {output}")


if __name__ == "__main__":
    main()
