#!/usr/bin/env python3
"""Offline MH6 trajectory audit; never constructs a real serial client.

Uses an existing solver-only JSONL, the production preview functions, and a
memory-only Modbus client. Threshold sweeps are isolated from production config.
Example:
  python DHandControl/scripts/mh6_trajectory_audit.py
Outputs summary.json, frames.jsonl, interpolation.jsonl, threshold_sweep.png,
motor_trajectories.png and gesture_tracking.png in results/trajectory_audit/.
Numerical FK and finite interpolation samples do not certify physical safety.
Gesture phase windows are specific to teleop_01 and its six-fist/four-opposition
sequence; adapt phase_masks() before using phase statistics for another session.
"""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import redirect_stdout
import hashlib
import io
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
from scipy.optimize import least_squares

from mh6_palm_calibration import PALM_MOTOR_SAFE_LIMITS, PALM_NEUTRAL_MOTORS
from mh6_palm_fallback import PalmFallbackController
from mh6_palm_solution_selector import PalmInputSlewLimiter, PalmSolutionSelector
from mh6_palm_solver_adapter import PalmAdapterCalibration, PalmSolverAdapter
from mh6_palm_solver_v2 import MH6PalmSolver
from mh6_teleop_run import (
    CommandLowPassFilter, HardwareSender, select_palm_control_preview,
)
from modbus_dev import DexHandControl


ROOT = Path(__file__).resolve().parents[2]
AXES = ("vertical", "thumb_rotation_command", "lateral")  # h, r, v
ROT_TOL = 1e-10  # numerical model check, not a measured hardware tolerance
TRANS_TOL_MM = 1e-4  # delivery model audit tolerance


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]


def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False) + "\n", encoding="utf-8")


def dump_rows(path, rows):
    with Path(path).open("w", encoding="utf-8") as file:
        for row in rows:
            file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def motor_valid(motor):
    return len(motor) == 3 and all(
        math.isfinite(float(x)) and lo <= x <= hi
        for x, (lo, hi) in zip(motor, PALM_MOTOR_SAFE_LIMITS.values())
    )


def motor_to_angles(motor):
    """Physical ID order M1/M2/M3 -> alpha2/alpha3/alpha1 in degrees."""
    m1, m2, m3 = map(float, motor)
    return np.array([(500.0-m2)*90.8/380.0,
                     (247.0-m1)*180.0/753.0,
                     (m3-500.0)*23.6/99.0])


def rotation_loop(solver, active, passive):
    a2, a3, a1 = active
    t1, t2, t3 = passive
    return (solver.RyRz(80, a2) @ solver.RyRz(-80, t1)
            @ solver.RyRz(80, t2) @ solver.RyRz(35, t3)
            @ solver.RyRz(20, a3) @ solver.RyRz(225, a1))


def pose_residual(solver, active, passive):
    a2, a3, a1 = active
    t1, t2, t3 = passive
    return {
        "rotation_max_abs": float(np.abs(rotation_loop(solver, active, passive)
                                           - np.eye(3)).max()),
        "translation_mm": float(solver.compute_translation_error(
            a2, a3, a1, t1, t2, t3)),
    }


def passive_candidates(solver, active):
    """Analytic FK for fixed alpha1/alpha2/alpha3, independent of teleop IK.

    From R1 R2 R3 R4 R5 R6=I, define
    D=Ry(80) R1.T (R5 R6).T=Rz(t1) Ry(80) Rz(t2) Ry(35) Rz(t3).
    Then Dzz=cos80 cos35-sin80 sin35 cos(t2). Two t2 branches
    determine t1 from column 3 and t3 from the remaining rotation.
    """
    a2, a3, a1 = active
    r1 = solver.RyRz(80, a2)
    r56 = solver.RyRz(20, a3) @ solver.RyRz(225, a1)
    d = solver.RyRz(80, 0) @ r1.T @ r56.T
    value = float((solver.C80*solver.C35-d[2, 2])/(solver.S80*solver.S35))
    if abs(value) > 1+1e-12:
        return [], value
    magnitude = math.degrees(math.acos(np.clip(value, -1, 1)))
    solutions = []
    for t2 in (magnitude, -magnitude):
        w = solver.RyRz(80, t2) @ solver.RyRz(35, 0) @ np.array([0, 0, 1])
        t1 = math.degrees(math.atan2(d[1, 2], d[0, 2])-math.atan2(w[1], w[0]))
        t1 = (t1+180) % 360-180
        base = solver.RyRz(0, t1) @ solver.RyRz(80, t2) @ solver.RyRz(35, 0)
        r3 = base.T @ d
        t3 = math.degrees(math.atan2(r3[1, 0], r3[0, 0]))
        solutions.append([t1, t2, t3])
    return solutions, value


def fit_passive(solver, motor, seed):
    """Audit-only FK with analytic existence check, then closest passive branch.

    Least squares is used only to quantify residuals when the analytic FK has no
    real root. Other passive-joint bounds/collision geometry remain unavailable.
    """
    active = motor_to_angles(motor)
    seed = np.asarray(seed, dtype=float).copy()
    candidates, value = passive_candidates(solver, active)
    valid = [p for p in candidates if -23.7-1e-9 <= p[0] <= 8.6+1e-9]
    if candidates:
        chosen = min(valid or candidates, key=lambda p: np.linalg.norm(
            (np.array(p)-seed+180) % 360-180))
        residual = pose_residual(solver, active, chosen)
        return {"passive_theta1_theta2_theta3_deg": chosen, **residual,
                "numerically_closed": bool(valid and residual["rotation_max_abs"] <= ROT_TOL
                                           and residual["translation_mm"] <= TRANS_TOL_MM),
                "theta1_at_bound": bool(min(abs(chosen[0]+23.7), abs(8.6-chosen[0])) < 1e-5),
                "analytic_fk_cos_theta2": value,
                "reason": "ok" if valid else "theta1_outside_mechanical_range",
                "method": "analytic_fk", "evaluations": 0}
    seed[0] = np.clip(seed[0], -23.7+1e-9, 8.6-1e-9)
    result = least_squares(
        lambda p: (rotation_loop(solver, active, p)-np.eye(3)).ravel(), seed,
        bounds=([-23.7, -np.inf, -np.inf], [8.6, np.inf, np.inf]),
        ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=100,
    )
    residual = pose_residual(solver, active, result.x)
    return {
        "passive_theta1_theta2_theta3_deg": result.x.tolist(),
        **residual,
        "numerically_closed": False,
        "theta1_at_bound": bool(min(result.x[0]+23.7, 8.6-result.x[0]) < 1e-5),
        "analytic_fk_cos_theta2": value,
        "reason": "no_real_passive_solution_in_rigid_model",
        "method": "least_squares_residual_after_failed_analytic_existence_check",
        "evaluations": result.nfev,
    }


def candidate_details(solver, angles):
    a2, a3, t1 = angles
    details = []
    # solve_motor() and solve_remaining() use the same deterministic branch sort.
    raw = solver.solve_remaining(*angles)
    motors = solver.solve_motor(*angles)
    for (t2, t3, a1, rot, trans), motor in zip(raw, motors):
        ordered = solver.reorder_motor_solution(motor, "timeseries")
        v = solver.RyRz(20, 0) @ solver.RyRz(0, a3) @ solver.RyRz(225, 0) @ np.array([0, 0, 1])
        branch = "plus_acos" if math.sin(math.radians(t3)+math.atan2(v[1], v[0])) >= 0 else "minus_acos"
        details.append({"motor": ordered, "within_actual_limits": motor_valid(ordered),
                        "passive_theta1_theta2_theta3_deg": [t1, t2, t3],
                        "active_alpha2_alpha3_alpha1_deg": [a2, a3, a1],
                        "branch": branch, "rotation_max_abs": float(rot),
                        "translation_mm": float(trans)})
        fk_roots, fk_value = passive_candidates(solver, [a2, a3, a1])
        if not fk_roots or min(abs(p[0]-t1) for p in fk_roots) > 1e-6:
            raise AssertionError("Actuator FK does not reproduce the exact IK pose")
        details[-1]["actuator_fk_cos_theta2"] = fk_value
    return details


def choose_detail(details, motor):
    return min(details, key=lambda d: np.linalg.norm(np.array(d["motor"])-motor))


def phase_masks(rows):
    t = np.array([r["timestamp"] for r in rows])
    raw = [r["raw_mapping"] for r in rows]
    curl = np.array([[r["curl_norm"][f] for f in ("index", "middle", "ring", "little")] for r in raw]).mean(axis=1)
    dist = np.array([[r["opposition_distances"][f] for f in ("index", "middle", "ring", "little")] for r in raw])
    fist_phase = (t < 9.5) | ((t >= 17.5) & (t < 25.5))
    masks = {"strong_fist_proxy": fist_phase & (curl >= .8),
             "open_release_proxy": fist_phase & (curl <= -.3)}
    for j, (name, (lo, hi)) in enumerate(zip(("index", "middle", "ring", "little"),
                                            ((9.7, 11), (11.5, 12.7), (13, 14.4), (15.2, 16.5)))):
        masks[name+"_opposition_core"] = ((t >= lo) & (t <= hi) & (dist[:, j] < .02)
                                              & (dist.argmin(axis=1) == j))
    masks["all_opposition_cores"] = np.logical_or.reduce(
        [v for k, v in masks.items() if k.endswith("opposition_core")])
    ids = np.flatnonzero(masks["strong_fist_proxy"])
    if len(ids):
        for j, group in enumerate(np.split(ids, np.flatnonzero(np.diff(ids) > 1)+1), 1):
            mask = np.zeros(len(t), dtype=bool); mask[group] = True
            masks[f"fist_core_{j}"] = mask
    return masks, curl, dist


def command_metrics(t, motors, include_neutral_start=True):
    m = np.asarray(motors, dtype=float)
    if include_neutral_start:
        d = np.diff(np.vstack([PALM_NEUTRAL_MOTORS, m]), axis=0)
        dt = np.r_[.05, np.diff(t)]
        signed = np.array([PalmSolutionSelector.motor_to_signed(x)
                           for x in [PALM_NEUTRAL_MOTORS, *m.tolist()]])
    else:
        d = np.diff(m, axis=0)
        dt = np.diff(t)
        signed = np.array([PalmSolutionSelector.motor_to_signed(x) for x in m])
    speed = d / dt[:, None]
    accel = np.diff(speed, axis=0)/((dt[1:]+dt[:-1])/2)[:, None]
    return {"max_abs_delta_motor": np.abs(d).max(axis=0).tolist(),
            "max_abs_command_velocity_units_per_second": np.abs(speed).max(axis=0).tolist(),
            "max_abs_command_acceleration_units_per_second_squared": np.abs(accel).max(axis=0).tolist(),
            "max_normalized_command_speed_per_second": float((np.abs(np.diff(signed, axis=0))/dt[:, None]).max()),
            "changing_output_frames": int(np.any(np.abs(d) > 1e-8, axis=1).sum()),
            "includes_start_from_assumed_calibrated_neutral": include_neutral_start,
            "first_dt_assumed_seconds": .05 if include_neutral_start else None}


def preview_run(rows, adapter, jump_speed, seed_first=False):
    filt = CommandLowPassFilter(.24)
    selector = PalmSolutionSelector(max_normalized_speed_per_sec=jump_speed)
    limiter = PalmInputSlewLimiter()
    fallback = PalmFallbackController()
    output = []
    for i, r in enumerate(rows):
        filtered = filt.apply(r["raw_mapping"], r["timestamp"])
        # Only this diagnostic assumes the robot already occupies the first
        # filtered feasible pose. It skips the unvalidated startup movement.
        if i == 0 and seed_first:
            p = filtered["palm_command"]
            values = tuple(np.clip([p[a] for a in AXES], -.1, .1))
            diag = adapter.solve_motor_from_teleop(values[0], values[2], values[1])
            valid = diag["solutions"]
            if not valid:
                raise ValueError("No valid first pose for seeded diagnostic")
            selector.previous_motor = min(valid, key=lambda x: selector._distance(x, PALM_NEUTRAL_MOTORS))
            selector.previous_valid_input = values
            selector.previous_timestamp = r["timestamp"]
            limiter.hold(values)
            limiter.previous_timestamp = r["timestamp"]
        preview, fb = select_palm_control_preview(
            filtered, adapter, selector, fallback, limiter, r["timestamp"])
        requested, applied, candidates, sel = preview
        output.append({"timestamp": r["timestamp"], "solver_status": sel.status,
                       "held_previous": sel.held_previous,
                       "requested_h_r_v": [requested[k] for k in ("palm_flexion", "thumb_inward", "palm_cross")],
                       "applied_h_r_v": [applied[k] for k in ("palm_flexion", "thumb_inward", "palm_cross")],
                       "angles_alpha2_alpha3_theta1_deg": sel.solver_diagnostics["used_angles"],
                       "candidates": candidates, "selected_motor": sel.selected_motor,
                       "valid_candidate_count": sel.valid_candidate_count,
                       "normalized_jump": sel.normalized_jump,
                       "control_mode": fb.mode, "control_status": fb.status,
                       "control_motor": fb.selected_motor,
                       "filtered_mapping": filtered})
    return output


class MemoryModbusClient:
    """Inert sink. Any attempt to connect or read device state fails loudly."""
    connected = True

    def __init__(self):
        self.writes = []
        self.fail_write = False

    def connect(self):
        raise AssertionError("Offline audit must never connect to a device")

    def write_registers(self, address, values, **kwargs):
        self.writes.append({"address": address, "values": list(values), **kwargs})
        return SimpleNamespace(isError=lambda: self.fail_write)

    def write_register(self, address, value, **kwargs):
        self.writes.append({"address": address, "value": value, **kwargs})
        return SimpleNamespace(isError=lambda: self.fail_write)

    def read_holding_registers(self, **kwargs):
        raise AssertionError("Audit cannot read real device feedback")


def memory_sender():
    # Deliberately bypass __init__, which constructs the real serial client.
    hand = DexHandControl.__new__(DexHandControl)
    hand.client = MemoryModbusClient()
    hand.persistent_connection = True
    hand.transaction_lock = threading.Lock()
    hand.palm_safe_limits = PALM_MOTOR_SAFE_LIMITS.copy()
    hand.finger_limit = {i: (20, 1950) for i in range(1, 6)}
    sender = HardwareSender("offline-memory-only", 115200)
    sender.hand = hand
    return sender


def audit_sender(run):
    sender = memory_sender()
    payloads = []
    for frame in run:
        start = len(sender.hand.client.writes)
        ok = sender.send(frame["filtered_mapping"], SimpleNamespace(selected_motor=frame["selected_motor"]))
        assert ok
        writes = sender.hand.client.writes[start:]
        assert len(writes) == 2 and writes[0]["address"] == 20
        block = writes[0]["values"]
        assert len(block) == 27 and writes[1]["address"] == 0 and writes[1]["value"] == 4
        palm = [block[13+i*3] for i in range(3)]
        assert palm == [int(round(x)) for x in frame["selected_motor"]]
        payloads.append({"timestamp": frame["timestamp"], "register_block_20_46": block,
                         "palm_integer_motor": palm, "trigger_register_0": 4})
    before = len(sender.hand.client.writes)
    try:
        sender.send(run[0]["filtered_mapping"], SimpleNamespace(selected_motor=[247, 500, 537]))
    except ValueError:
        pass
    else:
        raise AssertionError("Out-of-range motor was not rejected")
    assert len(sender.hand.client.writes) == before
    sender.hand.client.fail_write = True
    with redirect_stdout(io.StringIO()) as failure_log:
        failed_return = sender.send(run[0]["filtered_mapping"], SimpleNamespace(selected_motor=PALM_NEUTRAL_MOTORS))
    assert failed_return is False
    return {"memory_only": True, "packed_frames": len(payloads),
            "out_of_range_rejected_before_write": True,
            "injected_modbus_error_returned_false": True,
            "wait_status": False, "palm_times_ms": [80, 80, 80],
            "real_device_feedback_obtained": False,
            "failure_debug_log": failure_log.getvalue()}, payloads


def audit_preview_poses(solver, run, cache):
    """Evaluate actual selected commands, including held poses and int rounding."""
    seeds = []; last_branch = None; branch_changes = 0
    seed = fit_passive(solver, PALM_NEUTRAL_MOTORS, [0, 0, 0])["passive_theta1_theta2_theta3_deg"]
    for frame in run:
        if frame["solver_status"] == "SELECTED":
            ds = candidate_details(solver, frame["angles_alpha2_alpha3_theta1_deg"])
            d = choose_detail(ds, frame["selected_motor"])
            seed = d["passive_theta1_theta2_theta3_deg"]
            changed = last_branch is not None and last_branch != d["branch"]
            branch_changes += int(changed)
            last_branch = d["branch"]
        else:
            changed = False
        integer = [int(round(x)) for x in frame["selected_motor"]]
        key = (tuple(integer), tuple(np.round(seed, 6)))
        if key not in cache:
            cache[key] = fit_passive(solver, integer, seed)
        frame["selected_branch"] = last_branch
        frame["branch_changed"] = changed
        frame["integer_motor"] = integer
        frame["integer_motor_fk"] = cache[key]
        seeds.append(seed)
    return {"selected_branch_changes": branch_changes,
            "integer_rigid_model_closed_output_frames": sum(r["integer_motor_fk"]["numerically_closed"] for r in run),
            "integer_rigid_model_closed_selected_frames": sum(r["solver_status"] == "SELECTED" and r["integer_motor_fk"]["numerically_closed"] for r in run),
            "integer_rigid_model_failure_counts_including_held_frames": dict(Counter(
                r["integer_motor_fk"]["reason"] for r in run if not r["integer_motor_fk"]["numerically_closed"])),
            "failed_integer_frame_indices": [i for i, r in enumerate(run) if not r["integer_motor_fk"]["numerically_closed"]]}, seeds


def phase_summary(masks, accepted, motors, raw_motor):
    out = {}
    for name, mask in masks.items():
        out[name] = {"frames": int(mask.sum()), "selected_frames": int(accepted[mask].sum()),
                     "median_motor": np.median(motors[mask], axis=0).tolist() if mask.any() else None,
                     "median_command_active_angles_alpha2_alpha3_alpha1_deg": motor_to_angles(
                         np.median(motors[mask], axis=0)).tolist() if mask.any() else None,
                     "mean_abs_motor_difference_from_raw_bound_valid_reference":
                         np.abs(motors[mask]-raw_motor[mask]).mean(axis=0).tolist() if mask.any() else None}
    return out


def interpolation_audit(solver, t, motors, seeds, samples):
    rows = []
    for i in range(1, len(t)):
        if np.max(np.abs(motors[i]-motors[i-1])) < 1e-8:
            continue
        seed = seeds[i-1]
        for fraction in np.linspace(0, 1, samples+2)[1:-1]:
            motor = motors[i-1]+fraction*(motors[i]-motors[i-1])
            fit = fit_passive(solver, motor, seed)
            rows.append({"from_frame": i-1, "to_frame": i,
                         "timestamp": float(t[i-1]+fraction*(t[i]-t[i-1])),
                         "fraction": float(fraction), "motor": motor.tolist(), **fit})
            if fit["numerically_closed"]:
                seed = fit["passive_theta1_theta2_theta3_deg"]
    return rows


def make_plots(out, t, raw_motor, runs, summary, raw_angles, masks, curl):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
    if font.exists():
        plt.rcParams["font.family"] = FontProperties(fname=str(font)).get_name()
        from matplotlib import font_manager
        font_manager.fontManager.addfont(str(font))
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10})
    vals = summary["threshold_sweep"]
    x = np.array([v["jump_speed_per_second"] for v in vals])
    fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    ax[0].plot(x, [v["selected_frames"] for v in vals], "o-", label="接受目标")
    ax[0].plot(x, [v["status_counts"].get("HELD_NO_VALID_SOLUTION", 0) for v in vals], "o-", label="电机限位拒绝")
    ax[0].plot(x, [sum(n for s, n in v["status_counts"].items() if "JUMP_REJECTED" in s) for v in vals], "o-", label="跳变拒绝")
    ax[0].set_ylabel("帧数 / 599"); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].plot(x, [v["metrics"]["max_normalized_command_speed_per_second"] for v in vals], "o-", label="含启动的最大输出变化率")
    ax[1].plot(x, x, "--", color="gray", label="设置阈值")
    ax[1].set_xlabel("测试跳变保护阈值（归一化速度 / 秒）"); ax[1].set_ylabel("归一化速度 / 秒")
    ax[1].legend(); ax[1].grid(alpha=.3)
    fig.suptitle("阈值离线实验：放宽阈值不保证接受帧数持续增加")
    fig.tight_layout(); fig.savefig(out/"threshold_sweep.png", dpi=160); plt.close(fig)

    shown = [key for key in ("speed_2", "speed_6.5", "speed_40", "seeded_speed_2") if key in runs]
    colors = ["#88929c", "#1976d2", "#d36b2b", "#379577"]
    labels = {"speed_2": "默认阈值 2", "speed_6.5": "测试阈值 6.5",
              "speed_40": "测试阈值 40", "seeded_speed_2": "假设起始已对齐，阈值 2"}
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    for j in range(3):
        ax = axes[j]
        ax.plot(t, raw_motor[:, j], color="#202d3b", lw=1, alpha=.65, label="原始目标：限位内最近分支，无跳变保护")
        for k, color in zip(shown, colors):
            ax.plot(t, [r["selected_motor"][j] for r in runs[k]], color=color, lw=1.2, label=labels[k])
        lo, hi = PALM_MOTOR_SAFE_LIMITS[j+1]
        ax.axhline(lo, color="red", ls="--", lw=.8); ax.axhline(hi, color="red", ls="--", lw=.8)
        ax.set_ylabel(f"M{j+1} 位置单位"); ax.grid(alpha=.25)
    axes[0].legend(loc="upper right", fontsize=8, ncol=2)
    for k, color in zip(shown, colors):
        axes[3].step(t, [int(r["solver_status"] == "SELECTED") for r in runs[k]],
                     where="post", color=color, alpha=.8, label=labels[k])
    axes[3].set_ylabel("接受目标 = 1"); axes[3].set_xlabel("录制时间（秒）"); axes[3].grid(alpha=.25)
    fig.suptitle("同一段人手轨迹：实际控制预览输出与原始可用目标")
    fig.tight_layout(); fig.savefig(out/"motor_trajectories.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    for j, label in enumerate(("上下翻折 α2（度）", "大拇指 α3（度）", "左右翻折 θ1（度）")):
        axes[j].plot(t, raw_angles[:, j], color="#202d3b", lw=1, label="原始人手输入生成的目标角度")
        for k, color in zip(shown[:3], colors[:3]):
            axes[j].plot(t, [r["angles_alpha2_alpha3_theta1_deg"][j] for r in runs[k]],
                         color=color, lw=1, label=labels[k]+"：本帧尝试求解角度")
        axes[j].set_ylabel(label); axes[j].grid(alpha=.25)
        for name, mask in masks.items():
            if name.endswith("opposition_core") and mask.any():
                ids = np.flatnonzero(mask); axes[j].axvspan(t[ids[0]], t[ids[-1]], color="#bda7df", alpha=.17)
    axes[0].legend(fontsize=8, ncol=2)
    axes[3].plot(t, curl, color="#202d3b", label="人手四长指平均弯曲")
    for k, color in zip(shown[:3], colors[:3]):
        axes[3].plot(t, [r["applied_h_r_v"][0] for r in runs[k]], color=color, label=labels[k]+"：实际应用 h")
    axes[3].set_xlabel("录制时间（秒）"); axes[3].set_ylabel("人手弯曲 / 应用 h"); axes[3].grid(alpha=.25)
    fig.suptitle("动作意图诊断：尝试求解角度与最终电机输出需同时阅读")
    fig.tight_layout(); fig.savefig(out/"gesture_tracking.png", dpi=160); plt.close(fig)

    names = [k for k in masks if k.startswith("fist_core_")]+[
        k for k in masks if k.endswith("opposition_core")]
    phase_labels = [("握拳 "+k.rsplit("_", 1)[-1]) if k.startswith("fist_core_") else
                    {"index_opposition_core": "食指对指", "middle_opposition_core": "中指对指",
                     "ring_opposition_core": "无名指对指", "little_opposition_core": "小指对指"}[k] for k in names]
    x = np.arange(len(names))
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    raw_counts = [summary["gesture_intent"]["phases"][k]["raw_valid_frames"] for k in names]
    axes[0].bar(x-.25, raw_counts, width=.25, color="#84919e", label="原始目标：存在限位内候选")
    for offset, key, color in [(0, "speed_6.5", "#1976d2"), (.25, "speed_40", "#d36b2b")]:
        if key not in runs:
            continue
        counts = [sum(r["solver_status"] == "SELECTED" for r, b in zip(runs[key], masks[k]) if b) for k in names]
        integers = [sum(r["solver_status"] == "SELECTED" and r["integer_motor_fk"]["numerically_closed"]
                        for r, b in zip(runs[key], masks[k]) if b) for k in names]
        axes[0].bar(x+offset, counts, width=.25, color=color, label=labels[key])
        axes[1].bar(x+(offset-.125), integers, width=.25, color=color, label=labels[key]+"：接受且取整后模型通过")
    for ax in axes:
        ax.plot(x, [int(masks[k].sum()) for k in names], "k_", markersize=25, label="人手动作核心帧数")
        ax.set_ylabel("帧数"); ax.grid(axis="y", alpha=.25); ax.set_axisbelow(True)
        ax.set_ylim(0, max(int(masks[k].sum()) for k in names)+6)
        ax.legend(fontsize=8, loc="upper right")
    axes[1].set_xticks(x, phase_labels)
    axes[0].set_title("浮点目标的可用性与控制链接受情况")
    axes[1].set_title("已接受目标，再检查实际整数指令的严格模型闭合性")
    fig.suptitle("按用户动作序列检验：六次握拳与四种对指")
    fig.tight_layout(); fig.savefig(out/"gesture_phase_acceptance.png", dpi=160); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--solver-log", type=Path, default=ROOT/"results/solver_only/teleop_01_workspace_conditional.jsonl")
    parser.add_argument("--adapter-config", type=Path, default=ROOT/"DHandControl/config/mh6_palm_adapter_workspace_conditional.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT/"results/trajectory_audit/teleop_01_workspace_conditional")
    parser.add_argument("--jump-speeds", type=float, nargs="+", default=[2, 4, 6, 6.5, 8, 10, 12, 20, 24, 28, 32, 36, 40])
    parser.add_argument("--interpolation-samples", type=int, default=3)
    args = parser.parse_args(argv)
    if args.interpolation_samples < 1 or any(not math.isfinite(s) or s <= 0 for s in args.jump_speeds):
        parser.error("Positive finite speeds and at least one interpolation sample required")
    out = args.output_dir; out.mkdir(parents=True, exist_ok=True)
    rows = read_rows(args.solver_log)
    t = np.array([r["timestamp"] for r in rows])
    if len(t) < 3 or not np.all(np.isfinite(t)) or not np.all(np.diff(t) > 0):
        raise ValueError("At least three strictly increasing finite timestamps required")
    adapter = PalmSolverAdapter(PalmAdapterCalibration.from_file(args.adapter_config))
    solver = adapter.solver
    masks, curl, distances = phase_masks(rows)
    raw_angles = np.array([r["solver"]["requested_angles"] for r in rows])
    frames = []
    details = [candidate_details(solver, angles) for angles in raw_angles]
    nearest = PalmSolutionSelector(max_normalized_speed_per_sec=1e9)
    raw_motor = []; passive_seeds = []; branch_ids = []
    previous_seed = None
    for i, (r, branch) in enumerate(zip(rows, details)):
        assert np.allclose([d["motor"] for d in branch], r["solver"]["candidates"], atol=1e-8, rtol=0)
        intent = [r["raw_mapping"]["palm_command"][a] for a in AXES]
        selected = nearest.select(intent, [d["motor"] for d in branch], t[i])
        raw_motor.append(selected.selected_motor)
        if not selected.held_previous:
            current = choose_detail(branch, selected.selected_motor)
            previous_seed = current["passive_theta1_theta2_theta3_deg"]
            branch_ids.append(current["branch"])
        else:
            branch_ids.append(branch_ids[-1] if branch_ids else None)
        if previous_seed is None:
            raise ValueError("First frame has no bounded seed")
        passive_seeds.append(previous_seed)
        for d in branch:
            if d["within_actual_limits"]:
                integer = [int(round(x)) for x in d["motor"]]
                fixed = pose_residual(solver, motor_to_angles(integer), d["passive_theta1_theta2_theta3_deg"])
                d["integer_motor"] = integer
                d["integer_rounding_angle_error_deg"] = (motor_to_angles(integer)-d["active_alpha2_alpha3_alpha1_deg"]).tolist()
                d["integer_fixed_original_passive_residual"] = fixed
                d["integer_adjusted_passive_fit"] = fit_passive(solver, integer, d["passive_theta1_theta2_theta3_deg"])
        frames.append({"frame_index": i, "timestamp": float(t[i]), "raw_h_r_v": intent,
                       "raw_angles_alpha2_alpha3_theta1_deg": raw_angles[i].tolist(),
                       "closure_margin": r["solver"]["adapter"]["used_diagnostic"]["closure_margin"],
                       "candidates": branch, "bound_valid_reference_motor": selected.selected_motor,
                       "bound_valid_reference_held": selected.held_previous, "bound_valid_reference_branch": branch_ids[-1]})
    raw_motor = np.array(raw_motor)
    runs = {}; sweep = []; fk_cache = {}; run_seeds = {}
    for speed in args.jump_speeds:
        key = f"speed_{speed:g}"
        run = preview_run(rows, adapter, speed)
        runs[key] = run
        pose_report, run_seeds[key] = audit_preview_poses(solver, run, fk_cache)
        accepted = np.array([r["solver_status"] == "SELECTED" for r in run])
        motors = np.array([r["selected_motor"] for r in run])
        sweep.append({"key": key, "jump_speed_per_second": speed,
                      "nominal_allowed_step": speed*.05,
                      "selected_frames": int(accepted.sum()), "selected_rate": float(accepted.mean()),
                      "first_selected_frame": int(np.flatnonzero(accepted)[0]) if accepted.any() else None,
                      "status_counts": dict(Counter(r["solver_status"] for r in run)),
                      "control_status_counts": dict(Counter(r["control_status"] for r in run)),
                      "fallback_frames": sum(r["control_mode"] == "FALLBACK" for r in run),
                      "output_within_actual_limits_frames": sum(motor_valid(m) for m in motors),
                      "command_pose_audit": pose_report,
                      "metrics": command_metrics(t, motors),
                      "tracking_mae_vs_raw_bound_valid_motor": np.abs(motors-raw_motor).mean(axis=0).tolist(),
                      "phases": phase_summary(masks, accepted, motors, raw_motor)})
        for name, mask in masks.items():
            sweep[-1]["phases"][name]["status_counts"] = dict(Counter(
                run[i]["solver_status"] for i in np.flatnonzero(mask)))
            sweep[-1]["phases"][name]["accepted_and_integer_rigid_model_closed_frames"] = sum(
                run[i]["solver_status"] == "SELECTED" and run[i]["integer_motor_fk"]["numerically_closed"]
                for i in np.flatnonzero(mask))
    runs["seeded_speed_2"] = preview_run(rows, adapter, 2, seed_first=True)
    seeded = runs["seeded_speed_2"]
    seeded_pose_report, run_seeds["seeded_speed_2"] = audit_preview_poses(solver, seeded, fk_cache)
    seeded_motors = np.array([r["selected_motor"] for r in seeded])
    seeded_accept = np.array([r["solver_status"] == "SELECTED" for r in seeded])
    sender_key = "speed_6.5" if "speed_6.5" in runs else next(iter(runs))
    sender_report, payloads = audit_sender(runs[sender_key])
    dump_rows(out/"memory_sender_payloads.jsonl", payloads)
    interpolation = interpolation_audit(solver, t, raw_motor, passive_seeds, args.interpolation_samples)
    dump_rows(out/"interpolation.jsonl", interpolation)
    output_interpolations = {}
    for key in (sender_key, "speed_40"):
        if key not in runs or key in output_interpolations:
            continue
        sampled = interpolation_audit(solver, t, np.array([r["integer_motor"] for r in runs[key]]),
                                      run_seeds[key], args.interpolation_samples)
        dump_rows(out/f"interpolation_{key}.jsonl", sampled)
        both_closed = [r for r in sampled if all(runs[key][j]["integer_motor_fk"]["numerically_closed"]
                                                for j in (r["from_frame"], r["to_frame"]))]
        output_interpolations[key] = {"sample_count": len(sampled),
            "rigid_model_closed_samples": sum(r["numerically_closed"] for r in sampled),
            "failure_counts": dict(Counter(r["reason"] for r in sampled if not r["numerically_closed"])),
            "failed_intervals": sorted(set(r["to_frame"] for r in sampled if not r["numerically_closed"])),
            "samples_between_two_model_closed_integer_endpoints": len(both_closed),
            "failed_samples_between_two_model_closed_integer_endpoints": sum(not r["numerically_closed"] for r in both_closed),
            "includes_startup_transition": False}
    all_candidates = [d for ds in details for d in ds]
    valid_candidates = [d for d in all_candidates if d["within_actual_limits"]]
    fits = [d["integer_adjusted_passive_fit"] for d in valid_candidates]
    physical = {"n_candidates": len(all_candidates), "model_numerical_checks": {
        "rotation_tol": ROT_TOL, "translation_tol_mm": TRANS_TOL_MM,
        "rotation_max_abs": max(d["rotation_max_abs"] for d in all_candidates),
        "translation_max_mm": max(d["translation_mm"] for d in all_candidates),
        "passing_candidates": sum(d["rotation_max_abs"] <= ROT_TOL and d["translation_mm"] <= TRANS_TOL_MM for d in all_candidates)},
        "within_actual_motor_limits_candidates": len(valid_candidates),
        "independent_actuator_fk_reproduced_exact_ik_candidates": len(all_candidates),
        "minimum_actuator_fk_cos_theta2_margin": min(1-abs(d["actuator_fk_cos_theta2"]) for d in valid_candidates),
        "frames_with_any_actual_limit_candidate": sum(any(d["within_actual_limits"] for d in ds) for ds in details),
        "limit_violations_per_motor_candidates": {str(j+1): sum(not lo <= d["motor"][j] <= hi for d in all_candidates) for j, (lo, hi) in enumerate(PALM_MOTOR_SAFE_LIMITS.values())},
        "integer_quantization": {"checked_candidates": len(fits),
            "max_abs_active_angle_error_deg": np.max(np.abs([d["integer_rounding_angle_error_deg"] for d in valid_candidates]), axis=0).tolist(),
            "fixed_original_passive_max_rotation_residual": max(d["integer_fixed_original_passive_residual"]["rotation_max_abs"] for d in valid_candidates),
            "fixed_original_passive_max_translation_mm": max(d["integer_fixed_original_passive_residual"]["translation_mm"] for d in valid_candidates),
            "adjusted_passive_closed_candidates": sum(f["numerically_closed"] for f in fits),
            "adjusted_passive_unconfirmed_candidates": sum(not f["numerically_closed"] for f in fits),
            "frames_with_any_integer_rigid_model_closed_candidate": sum(any(
                d["within_actual_limits"] and d["integer_adjusted_passive_fit"]["numerically_closed"] for d in ds) for ds in details),
            "unconfirmed_frame_indices": [i for i, ds in enumerate(details) if any(d["within_actual_limits"] and not d["integer_adjusted_passive_fit"]["numerically_closed"] for d in ds)],
            "failure_reason_counts": dict(Counter(f["reason"] for f in fits if not f["numerically_closed"])),
            "interpretation": "Fixed-passive residual measures deviation from original target. Analytic actuator-to-passive FK checks rigid-model feasibility after rounding; neither is physical feedback."}}
    phase_output = {}
    raw_any = np.array([any(d["within_actual_limits"] for d in ds) for ds in details])
    for name, mask in masks.items():
        phase_output[name] = {"frames": int(mask.sum()), "raw_valid_frames": int(raw_any[mask].sum()),
                              "raw_target_angle_median": np.median(raw_angles[mask], axis=0).tolist() if mask.any() else None}
    data_quality = {}
    session_path = ROOT/"recordings/teleop_01.npz"
    if session_path.exists() and args.solver_log.name == "teleop_01_workspace_conditional.jsonl":
        with np.load(session_path, allow_pickle=False) as session:
            transforms = session["transforms"]
            source_times = session["timestamps"]-session["timestamps"][0]
            if len(transforms) != len(rows) or not np.allclose(source_times, t, atol=1e-8, rtol=0):
                raise AssertionError("Solver log does not match source recording timing")
            tail_start = len(transforms)-1
            while tail_start and np.array_equal(transforms[tail_start-1], transforms[-1]):
                tail_start -= 1
            data_quality = {"source_session": str(session_path), "source_sha256": sha(session_path),
                "timestamps_match": True, "exactly_repeated_tail_start_frame": tail_start,
                "exactly_repeated_tail_frames": len(transforms)-tail_start,
                "exactly_repeated_tail_time_seconds": [float(t[tail_start]), float(t[-1])],
                "interpretation": "Identical transforms with increasing timestamps; may be held pose or frozen tracking. No tracking-validity ground truth is present."}
    cli_check = None
    cli_log = ROOT/"results/solver_only/teleop_01_workspace_conditional_control.jsonl"
    if cli_log.exists() and "speed_2" in runs and args.solver_log.name == "teleop_01_workspace_conditional.jsonl":
        cli_rows = read_rows(cli_log)
        assert len(cli_rows) == len(rows)
        assert [r["solver"]["status"] for r in cli_rows] == [r["solver_status"] for r in runs["speed_2"]]
        assert np.allclose([r["solver"]["selected_motor"] for r in cli_rows],
                           [r["selected_motor"] for r in runs["speed_2"]], atol=1e-8, rtol=0)
        cli_check = {"log": str(cli_log), "sha256": sha(cli_log),
                     "all_default_statuses_and_outputs_match": True}
    summary = {"scope": "Offline ordered validation of a fixed recorded hand trajectory; no real serial client or hardware action. Production defaults unchanged.",
        "source": {"solver_log": str(args.solver_log.resolve()), "sha256": sha(args.solver_log),
                   "adapter_config": str(args.adapter_config.resolve()), "adapter_sha256": sha(args.adapter_config),
                   "solver_sha256": sha(ROOT/"DHandControl/scripts/mh6_palm_solver_v2.py"),
                   "runtime_modules_sha256": {name: sha(ROOT/"DHandControl/scripts"/name) for name in
                       ("mh6_trajectory_audit.py", "mh6_palm_solver_adapter.py", "mh6_teleop_run.py",
                        "mh6_palm_solution_selector.py", "mh6_palm_fallback.py", "modbus_dev.py")}},
        "frames": len(rows), "duration_seconds": float(t[-1]-t[0]),
        "source_data_quality": data_quality,
        "production_cli_crosscheck": cli_check,
        "production_preview_defaults": {"filter_tau": .24, "input_signed_speed_per_second": 2,
                                        "jump_normalized_speed_per_second": 2, "max_dt_seconds": .1},
        "validity": physical,
        "raw_bound_valid_reference": {"selection": "Nearest physically bounded branch in calibrated normalized motor distance; no jump guard; hold when none valid.",
            "metrics": command_metrics(t, raw_motor), "branch_changes": sum(a != b for a, b in zip(branch_ids, branch_ids[1:])),
            "interpolation": {"interior_samples_per_changing_interval": args.interpolation_samples,
                "sample_count": len(interpolation), "numerically_closed_samples": sum(r["numerically_closed"] for r in interpolation),
                "unconfirmed_samples": sum(not r["numerically_closed"] for r in interpolation),
                "unconfirmed_intervals": sorted(set(r["to_frame"] for r in interpolation if not r["numerically_closed"])),
                "failure_reason_counts": dict(Counter(r["reason"] for r in interpolation if not r["numerically_closed"])),
                "assumption": "Synchronized straight-line motor interpolation, analytic passive FK choosing the closest branch with theta1 bounds. Finite samples are not proof of whole-path feasibility or actual firmware interpolation."}},
        "threshold_sweep": sweep,
        "integer_output_interpolation": output_interpolations,
        "seeded_start_diagnostic": {"assumption": "Robot already aligned to first filtered feasible pose. Startup transition deliberately not simulated.",
            "selected_frames": int(seeded_accept.sum()), "status_counts": dict(Counter(r["solver_status"] for r in seeded)),
            "command_pose_audit": seeded_pose_report,
            "metrics_excluding_unvalidated_startup": command_metrics(t, seeded_motors, include_neutral_start=False),
            "phases": phase_summary(masks, seeded_accept, seeded_motors, raw_motor)},
        "sender": {**sender_report, "tested_preview": sender_key,
            "firmware_command_duration_ms": 80, "nominal_frame_period_ms": 50,
            "code_review_findings": ["Runner --enable-hardware returns 2 before opening a stream or serial connection.",
                "HardwareSender routes solver selection, not fallback control output.",
                "Modbus write responses are checked; wait_status=False does not read actuator tracking state.",
                "Main loop logs command failure and continues; selector history was already advanced before send success.",
                "Fallback activates only for geometric NO_SOLUTION; actual-limit rejection and jump rejection hold without triggering fallback in this experiment.",
                "Tracking timeout pauses fallback timing; resumed filter/selector use capped dt, not measured robot pose.",
                "Stop closes persistent connection; no device stop command is issued by HardwareSender.stop."]},
        "gesture_intent": {"criteria_are_proxies": True, "phases": phase_output,
            "fist_cores_detected": sum(k.startswith("fist_core_") for k in masks),
            "robot_fingertip_task_ground_truth_available": False,
            "limitations": ["Recorded gesture windows and fingertip-distance thresholds are human completion proxies.",
                "Filtered input/hold modifies subsequent solver targets and solution rate.",
                "Accepted frames can still lag or differ from raw motion; held output within limits is not gesture success.",
                "Robot finger/palm combined kinematics and contact/collision models are not available for validating actual opposition."]},
        "hardware_readiness": {"ready": False,
            "conclusion": "Numerical endpoint validity is good, but default controller does not start; threshold changes expose limits and tracking tradeoffs. Physical dynamic/path/collision and feedback checks remain open.",
            "not_certified": ["Measured starting pose and transition to conditional workspace", "Physical motor speed/acceleration/load limits",
                "Passive-joint bounds beyond theta1 and real assembly branch", "Collision and task contact", "Firmware interpolation under overlapping 80 ms targets",
                "Tracking feedback, command failure recovery, device-side stop behavior"]}}
    dump(out/"summary.json", summary)
    for i, frame in enumerate(frames):
        frame["preview_runs"] = {key: {k: v for k, v in run[i].items() if k != "filtered_mapping"} for key, run in runs.items()}
    dump_rows(out/"frames.jsonl", frames)
    make_plots(out, t, raw_motor, runs, summary, raw_angles, masks, curl)
    print(json.dumps({"summary": str((out/"summary.json").resolve()), "frames": len(rows),
                      "selected_frames_by_threshold": {v["key"]: v["selected_frames"] for v in sweep}}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
