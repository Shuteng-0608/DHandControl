"""Quantify MH6 palm solver failures and compare coverage strategies.

This is an analysis utility.  It does not change the production solver.
Results are written to coverage_analysis_results.json for reproducibility.
"""

from __future__ import annotations

import csv
import json
import math
from collections import deque
from pathlib import Path

import numpy as np

from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver


ROOT = Path(__file__).resolve().parent


def angles_from_u(u1, u2, u3):
    return 121.9 * np.asarray(u1) - 31.1, -239.0 * np.asarray(u2) + 59.0, -32.3 * np.asarray(u3) + 8.6


def u_from_angles(a2, a3, t1):
    return (np.asarray(a2) + 31.1) / 121.9, (59.0 - np.asarray(a3)) / 239.0, (8.6 - np.asarray(t1)) / 32.3


def closure_value(a2_deg, a3_deg, t1_deg):
    """Return the cosine RHS used by solve_remaining.

    A rotational closure exists exactly when abs(value) <= 1 (apart from a
    non-occurring Rv singularity).  This is algebraically equivalent to the
    production solver but avoids constructing a rotation matrix per sample.
    """
    a2 = np.deg2rad(np.asarray(a2_deg, dtype=float))
    a3 = np.deg2rad(np.asarray(a3_deg, dtype=float))
    t1 = np.deg2rad(np.asarray(t1_deg, dtype=float))

    s80 = math.sin(math.radians(80.0))
    c80 = math.cos(math.radians(80.0))
    s35 = math.sin(math.radians(35.0))
    c35 = math.cos(math.radians(35.0))
    s20 = math.sin(math.radians(20.0))
    c20 = math.cos(math.radians(20.0))
    s225 = math.sin(math.radians(225.0))
    c225 = math.cos(math.radians(225.0))

    ca2, sa2 = np.cos(a2), np.sin(a2)
    ct1, st1 = np.cos(t1), np.sin(t1)
    c_zz = s80**2 * (c80 * (ct1 * (1.0 - ca2) + ca2) + sa2 * st1) + c80**3

    ca3, sa3 = np.cos(a3), np.sin(a3)
    vx = c20 * s225 * ca3 + s20 * c225
    vy = s225 * sa3
    vz = -s20 * s225 * ca3 + c20 * c225
    rv = np.hypot(vx, vy)
    return (c35 * vz - c_zz) / (s35 * rv)


def grid_values(u1, u2, u3, angle_map=angles_from_u):
    a2, _, _ = angle_map(np.asarray(u1)[:, None, None], 0.0, 0.0)
    _, a3, _ = angle_map(0.0, np.asarray(u2)[None, :, None], 0.0)
    _, _, t1 = angle_map(0.0, 0.0, np.asarray(u3)[None, None, :])
    return closure_value(a2, a3, t1)


def coverage_for_map(angle_map, n=101):
    u = np.linspace(0.0, 1.0, n)
    values = grid_values(u, u, u, angle_map)
    valid = np.abs(values) <= 1.0
    return {
        "grid_n": n,
        "valid": int(valid.sum()),
        "total": int(valid.size),
        "rate": float(valid.mean()),
        "max_abs_value": float(np.max(np.abs(values))),
    }


def linear_map(a2_lo, a2_hi, a3_lo, a3_hi, t1_lo, t1_hi):
    def mapper(u1, u2, u3):
        return (
            a2_lo + (a2_hi - a2_lo) * np.asarray(u1),
            a3_lo + (a3_hi - a3_lo) * np.asarray(u2),
            t1_lo + (t1_hi - t1_lo) * np.asarray(u3),
        )
    return mapper


def connected_components_6(mask):
    seen = np.zeros(mask.shape, dtype=bool)
    sizes = []
    nx, ny, nz = mask.shape
    for seed in zip(*np.nonzero(mask & ~seen)):
        if seen[seed]:
            continue
        q = deque([seed])
        seen[seed] = True
        size = 0
        while q:
            x, y, z = q.popleft()
            size += 1
            for dx, dy, dz in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
                xx, yy, zz = x + dx, y + dy, z + dz
                if 0 <= xx < nx and 0 <= yy < ny and 0 <= zz < nz and mask[xx, yy, zz] and not seen[xx, yy, zz]:
                    seen[xx, yy, zz] = True
                    q.append((xx, yy, zz))
        sizes.append(size)
    return sorted(sizes, reverse=True)


def valid_runs_1d(row):
    runs = []
    start = None
    for i, ok in enumerate(row):
        if ok and start is None:
            start = i
        if start is not None and (not ok or i == len(row) - 1):
            end = i if ok else i - 1
            runs.append((start, end))
            start = None
    return runs


def run_statistics(mask, axis):
    moved = np.moveaxis(mask, axis, -1)
    counts = []
    nonempty = 0
    for row in moved.reshape(-1, moved.shape[-1]):
        runs = valid_runs_1d(row)
        if runs:
            nonempty += 1
        counts.append(len(runs))
    counts = np.asarray(counts)
    return {
        "cross_sections": int(counts.size),
        "nonempty": int(nonempty),
        "max_intervals": int(counts.max()),
        "multi_interval": int(np.count_nonzero(counts > 1)),
    }


def prefix_sum_3d(mask):
    p = mask.astype(np.int64).cumsum(0).cumsum(1).cumsum(2)
    return np.pad(p, ((1, 0), (1, 0), (1, 0)))


def box_counts(prefix, lo, hi):
    x0, y0, z0 = lo[:, 0], lo[:, 1], lo[:, 2]
    x1, y1, z1 = hi[:, 0] + 1, hi[:, 1] + 1, hi[:, 2] + 1
    return (
        prefix[x1, y1, z1] - prefix[x0, y1, z1] - prefix[x1, y0, z1] - prefix[x1, y1, z0]
        + prefix[x0, y0, z1] + prefix[x0, y1, z0] + prefix[x1, y0, z0] - prefix[x0, y0, z0]
    )


def random_box_search(mask, samples=1_500_000, seed=20260718):
    rng = np.random.default_rng(seed)
    n = mask.shape[0]
    prefix = prefix_sum_3d(mask)
    best = {1.0: None, 0.99: None, 0.95: None, 0.90: None}
    chunk = 100_000
    for _ in range((samples + chunk - 1) // chunk):
        m = min(chunk, samples)
        samples -= m
        endpoints = rng.integers(0, n, size=(m, 2, 3), dtype=np.int16)
        lo = endpoints.min(axis=1).astype(np.int64)
        hi = endpoints.max(axis=1).astype(np.int64)
        shape = hi - lo + 1
        volume_points = shape.prod(axis=1)
        counts = box_counts(prefix, lo, hi)
        rates = counts / volume_points
        physical_volume = ((hi - lo) / (n - 1)).prod(axis=1)
        for threshold in best:
            eligible = rates >= threshold - 1e-15
            if not np.any(eligible):
                continue
            score = np.where(eligible, physical_volume, -1.0)
            idx = int(np.argmax(score))
            candidate = {
                "lo_idx": lo[idx].tolist(),
                "hi_idx": hi[idx].tolist(),
                "old_u_lo": (lo[idx] / (n - 1)).round(6).tolist(),
                "old_u_hi": (hi[idx] / (n - 1)).round(6).tolist(),
                "coverage": float(rates[idx]),
                "old_u_volume": float(physical_volume[idx]),
            }
            if best[threshold] is None or candidate["old_u_volume"] > best[threshold]["old_u_volume"]:
                best[threshold] = candidate
    return {str(k): v for k, v in best.items()}


def validate_box(box, n=121):
    lo = np.asarray(box["old_u_lo"])
    hi = np.asarray(box["old_u_hi"])
    u1 = np.linspace(lo[0], hi[0], n)
    u2 = np.linspace(lo[1], hi[1], n)
    u3 = np.linspace(lo[2], hi[2], n)
    values = grid_values(u1, u2, u3)
    valid = np.abs(values) <= 1.0
    a_lo = angles_from_u(*lo)
    a_hi = angles_from_u(*hi)
    return {
        "dense_n": n,
        "coverage": float(valid.mean()),
        "invalid": int(valid.size - valid.sum()),
        "angles_at_old_u_lo": [float(x) for x in a_lo],
        "angles_at_old_u_hi": [float(x) for x in a_hi],
    }


def nearest_valid(points, valid_points, chunk=20_000):
    best_d2 = np.full(len(points), np.inf)
    best_idx = np.full(len(points), -1, dtype=int)
    for start in range(0, len(valid_points), chunk):
        vp = valid_points[start:start + chunk]
        d2 = ((points[:, None, :] - vp[None, :, :]) ** 2).sum(axis=2)
        idx = np.argmin(d2, axis=1)
        val = d2[np.arange(len(points)), idx]
        improve = val < best_d2
        best_d2[improve] = val[improve]
        best_idx[improve] = start + idx[improve]
    return valid_points[best_idx], np.sqrt(best_d2)


def axis_repair(point_u, axis, n=10001):
    candidates = np.linspace(0.0, 1.0, n)
    pts = np.repeat(np.asarray(point_u, dtype=float)[None, :], n, axis=0)
    pts[:, axis] = candidates
    a2, a3, t1 = angles_from_u(pts[:, 0], pts[:, 1], pts[:, 2])
    valid = np.abs(closure_value(a2, a3, t1)) <= 1.0
    if not np.any(valid):
        return None
    valid_candidates = candidates[valid]
    chosen = valid_candidates[np.argmin(np.abs(valid_candidates - point_u[axis]))]
    out = np.asarray(point_u, dtype=float).copy()
    out[axis] = chosen
    return out


def quantiles(values):
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    return {k: float(np.quantile(a, q)) for k, q in (("min", 0), ("median", 0.5), ("p90", 0.9), ("p95", 0.95), ("max", 1))}


def analyze_timeseries(valid_points):
    rows = list(csv.DictReader((ROOT / "right_power_grasp_02_solver_timeseries.csv").open(encoding="utf-8-sig", newline="")))
    angles = np.array([[float(r["input_arpha2_deg"]), float(r["input_arpha3_deg"]), float(r["input_theta1_deg"])] for r in rows])
    points = np.column_stack(u_from_angles(angles[:, 0], angles[:, 1], angles[:, 2]))
    values = closure_value(angles[:, 0], angles[:, 1], angles[:, 2])
    valid = np.abs(values) <= 1.0
    failed_points = points[~valid]
    projected, distances = nearest_valid(failed_points, valid_points)
    p_angles = np.column_stack(angles_from_u(projected[:, 0], projected[:, 1], projected[:, 2]))
    original_failed_angles = angles[~valid]
    angle_delta = np.abs(p_angles - original_failed_angles)

    repairs = {}
    for axis, name in enumerate(("arpha2_only", "arpha3_only", "theta1_only")):
        fixed = []
        deltas = []
        for p in failed_points:
            q = axis_repair(p, axis)
            if q is not None:
                fixed.append(q)
                deltas.append(abs(q[axis] - p[axis]))
        full_range = (121.9, 239.0, 32.3)[axis]
        repairs[name] = {
            "repairable": len(fixed),
            "failed": len(failed_points),
            "rate": len(fixed) / len(failed_points),
            "normalized_delta": quantiles(deltas),
            "degree_delta": quantiles([d * full_range for d in deltas]),
        }

    return {
        "frames": len(rows),
        "success": int(valid.sum()),
        "failure": int((~valid).sum()),
        "success_rate": float(valid.mean()),
        "failed_margin_abs_value_minus_1": quantiles((np.abs(values[~valid]) - 1.0).tolist()),
        "nearest_3d_projection": {
            "normalized_distance": quantiles(distances.tolist()),
            "abs_degree_delta_arpha2": quantiles(angle_delta[:, 0].tolist()),
            "abs_degree_delta_arpha3": quantiles(angle_delta[:, 1].tolist()),
            "abs_degree_delta_theta1": quantiles(angle_delta[:, 2].tolist()),
        },
        "single_axis_repairs": repairs,
        "series": {
            "time_s": [float(r["time_s"]) for r in rows],
            "closure_value": values.round(6).tolist(),
            "success": valid.astype(int).tolist(),
        },
    }


def production_parity_check(samples=5000, seed=7):
    rng = np.random.default_rng(seed)
    pts = rng.random((samples, 3))
    a2, a3, t1 = angles_from_u(pts[:, 0], pts[:, 1], pts[:, 2])
    predicted = np.abs(closure_value(a2, a3, t1)) <= 1.0 + 1e-12
    solver = MH6PalmSolver()
    actual = np.array([bool(solver.solve_remaining(x, y, z)) for x, y, z in zip(a2, a3, t1)])
    return {"samples": samples, "mismatches": int(np.count_nonzero(predicted != actual))}


def main():
    results = {"production_parity": production_parity_check()}

    u101 = np.linspace(0.0, 1.0, 101)
    values101 = grid_values(u101, u101, u101)
    valid101 = np.abs(values101) <= 1.0
    abs_excess = np.abs(values101[~valid101]) - 1.0
    results["current_mapping"] = {
        "grid_n": 101,
        "valid": int(valid101.sum()),
        "total": int(valid101.size),
        "rate": float(valid101.mean()),
        "failure_sides": {
            "value_below_minus_1": int(np.count_nonzero(values101 < -1.0)),
            "value_above_plus_1": int(np.count_nonzero(values101 > 1.0)),
        },
        "invalid_excess_quantiles": quantiles(abs_excess.tolist()),
        "recovered_by_relaxed_tolerance": {
            str(t): int(np.count_nonzero(abs_excess <= t)) for t in (1e-12, 1e-9, 1e-6, 1e-3, 1e-2, 5e-2, 1e-1)
        },
    }

    u41 = np.linspace(0.0, 1.0, 41)
    mask41 = np.abs(grid_values(u41, u41, u41)) <= 1.0
    components = connected_components_6(mask41)
    results["geometry"] = {
        "grid_n": 41,
        "components_6_neighbor": len(components),
        "largest_component_fraction": components[0] / mask41.sum(),
        "intervals_along_u1": run_statistics(mask41, 0),
        "intervals_along_u2": run_statistics(mask41, 1),
        "intervals_along_u3": run_statistics(mask41, 2),
    }

    strategies = {
        "current": angles_from_u,
        "A1_written": linear_map(5.0, 85.0, -5.0, -35.0, -2.0, -24.0),
        "A1_capped": linear_map(5.0, 85.0, -5.0, -35.0, -2.0, -23.7),
        "stable_core_claim": linear_map(3.0, 88.0, 40.0, -37.0, -1.0, -23.7),
    }
    results["strategy_coverage"] = {name: coverage_for_map(fn, 101) for name, fn in strategies.items()}

    boxes = random_box_search(mask41)
    for box in boxes.values():
        if box is not None:
            box["dense_validation"] = validate_box(box)
    results["independent_box_search"] = boxes

    valid_idx = np.column_stack(np.nonzero(valid101))
    valid_points = valid_idx / 100.0
    results["timeseries"] = analyze_timeseries(valid_points)

    output = ROOT / "coverage_analysis_results.json"
    output.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(results, ensure_ascii=False, indent=2))
    print(f"\nWrote {output}")


if __name__ == "__main__":
    main()
