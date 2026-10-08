#!/usr/bin/env python3
"""Verify and plot fixed-session branch selection replays; never access hardware."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from mh6_palm_calibration import PALM_MOTOR_SAFE_LIMITS, PALM_NEUTRAL_MOTORS
from mh6_palm_solution_selector import PalmSolutionSelector

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIR = ROOT / "results/solver_only/branch_selection"


def read(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def metrics(rows):
    motors = np.rint([r["solver"]["selected_motor"] for r in rows]).astype(int)
    steps = np.diff(np.vstack((PALM_NEUTRAL_MOTORS, motors)), axis=0)
    spans = np.array([hi-lo for lo, hi in PALM_MOTOR_SAFE_LIMITS.values()])
    return {"frames": len(rows), "status_counts": dict(Counter(r["solver"]["status"] for r in rows)),
            "branch_changes": sum(r["solver"]["branch_selection"]["branch_changed"] for r in rows),
            "max_abs_integer_step": np.abs(steps).max(axis=0).tolist(),
            "sum_squared_normalized_motion": float(((steps/spans)**2).sum()),
            "M3_step_over_20_count": int((np.abs(steps[:, 2]) > 20).sum()),
            "selection_reason_counts": dict(Counter(r["solver"]["branch_selection"]["reason"] for r in rows))}


def replay(rows, penalty, reverse=False, time_scale=1):
    selector = PalmSolutionSelector(selection_policy="integer_continuous",
                                    branch_switch_penalty=penalty,
                                    max_normalized_speed_per_sec=None, allow_unguarded=True)
    result = []
    for row in rows:
        observation = row["solver"]
        candidates = observation["candidates"]
        identities = observation["adapter"]["candidate_branch_ids"]
        if reverse:
            candidates, identities = candidates[::-1], identities[::-1]
        inputs = observation["applied"]
        selected = selector.select((inputs["palm_flexion"], inputs["thumb_inward"], inputs["palm_cross"]),
                                   candidates, timestamp=row["timestamp"] * time_scale, branch_ids=identities)
        result.append({"timestamp": row["timestamp"], "solver": {
            "status": selected.status, "selected_motor": selected.selected_motor,
            "branch_selection": selected.branch_selection}})
    return result


def compare_outputs(a, b):
    return all(x["solver"]["status"] == y["solver"]["status"]
               and np.array_equal(np.rint(x["solver"]["selected_motor"]), np.rint(y["solver"]["selected_motor"]))
               and x["solver"]["branch_selection"]["selected_branch_id"] == y["solver"]["branch_selection"]["selected_branch_id"]
               for x, y in zip(a, b)) and len(a) == len(b)


def plot(out, baseline, current, different):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties, fontManager
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    font = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
    if font.exists():
        fontManager.addfont(str(font))
        plt.rcParams["font.family"] = FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({"font.size": 11, "axes.unicode_minus": False,
                         "axes.spines.top": False, "axes.spines.right": False, "svg.fonttype": "none"})
    blue, gray, red, purple = "#1466B8", "#75818C", "#C9475B", "#8463B5"
    t = np.array([r["timestamp"] for r in current])
    before = np.rint([r["solver"]["selected_motor"] for r in baseline]).astype(int)
    after = np.rint([r["solver"]["selected_motor"] for r in current]).astype(int)
    held = np.array([r["solver"]["held_previous"] for r in current])
    edges = np.r_[t, t[-1]+np.median(np.diff(t))]
    labels = ["M1 · 大拇指驱动 α3", "M2 · 上下翻折驱动 α2", "M3 · 闭环驱动 α1"]
    penalty = current[0]["solver"]["branch_selection"]["switch_penalty"]

    def shade(ax, mask, color, alpha):
        boundaries = np.r_[0, np.flatnonzero(np.diff(mask.astype(int)))+1, len(mask)]
        for start, end in zip(boundaries[:-1], boundaries[1:]):
            if mask[start]:
                ax.axvspan(edges[start], edges[end], color=color, alpha=alpha, lw=0)

    def branch_values(rows):
        return [1 if r["solver"]["branch_selection"]["selected_branch_id"] == "plus_acos" else -1 for r in rows]

    fig, axes = plt.subplots(4, 1, figsize=(15, 11), sharex=True,
                             gridspec_kw={"height_ratios": [3, 3, 3, 1.2]})
    for j, ax in enumerate(axes[:3]):
        shade(ax, held, red, .12)
        shade(ax, different, purple, .13)
        ax.step(t, before[:, j], where="post", color=gray, ls="--", lw=1.1)
        ax.step(t, after[:, j], where="post", color=blue, lw=1.4)
        lo, hi = PALM_MOTOR_SAFE_LIMITS[j+1]
        ax.axhline(lo, color=red, ls=":", lw=.8)
        ax.axhline(hi, color=red, ls=":", lw=.8)
        ax.set_ylim(lo-(hi-lo)*.07, hi+(hi-lo)*.10)
        ax.set_ylabel(labels[j]+"\n电机位置单位")
    axes[3].step(t, branch_values(baseline), where="post", color=gray, ls="--", lw=1.1)
    axes[3].step(t, branch_values(current), where="post", color=blue, lw=1.4)
    axes[3].set_yticks([-1, 1], ["−acos", "+acos"])
    axes[3].set_ylim(-1.6, 1.6)
    axes[3].set_ylabel("选中分支")
    axes[3].text(.99, .85, f"旧策略 {metrics(baseline)['branch_changes']} 次切换 → 新策略 {metrics(current)['branch_changes']} 次切换",
                 ha="right", transform=axes[3].transAxes, fontsize=10,
                 bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
    for ax in axes:
        ax.set_xlim(0, t[-1]); ax.set_xticks(np.arange(0, t[-1]+1, 2)); ax.grid(alpha=.18)
        for boundary in (9.5, 17.5, 25.5):
            ax.axvline(boundary, color=gray, ls=":", lw=.8)
    for start, end, label in [(0, 9.5, "前三次握拳"), (9.5, 17.5, "四种对指"),
                              (17.5, 25.5, "后三次握拳"), (25.5, t[-1], "收尾")]:
        axes[3].text((start+end)/2, -.52, label, ha="center", fontsize=10, transform=axes[3].get_xaxis_transform())
    axes[3].set_xlabel("录制时间（秒）", labelpad=29)
    fig.suptitle("解分支选择对照：上一有效整数位置 + 根切换惩罚", fontsize=17, y=.988)
    fig.text(.5, .952, f"599 帧固定录制 · λ={penalty:g} · 同一 Mapping / 目标角度 · 无滤波、输入限速和跳变门限",
             ha="center", fontsize=10.5, color="#53606D")
    fig.legend(handles=[Line2D([0], [0], color=blue, label="新：整数位置连续选解 + 切换惩罚"),
                        Line2D([0], [0], color=gray, ls="--", label="旧：标定距离最近分支"),
                        Patch(facecolor=purple, alpha=.25, label="两种策略输出不同"),
                        Patch(facecolor=red, alpha=.25, label="无限位内候选：保持上一位置")],
               loc="upper center", bbox_to_anchor=(.5, .936), ncol=2, frameon=False, fontsize=10)
    fig.text(.5, .015, "重合的曲线表示位置一致；M3 是 α1 电机驱动量，θ1 由闭环关系决定。分支标识按解析根判定。动作分段为近似窗口。",
             ha="center", fontsize=9.5, color="#53606D")
    fig.subplots_adjust(top=.865, bottom=.125, left=.12, right=.98, hspace=.24)
    for suffix in ("png", "svg"):
        fig.savefig(out/f"motor_trajectories.{suffix}", dpi=180, facecolor="white")
    plt.close(fig)

    ids = np.flatnonzero(different)
    window = (max(0, t[ids[0]]-.45), min(t[-1], t[ids[-1]]+.5)) if len(ids) else (0, min(6, t[-1]))
    colors = {"plus_acos": "#268B70", "minus_acos": "#DD862A"}
    fig, axes = plt.subplots(3, 1, figsize=(14, 8.5), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1.2, 2]})
    for branch, color in colors.items():
        candidate = []; costs = []
        for row in current:
            valid = [s for s in row["solver"]["branch_selection"]["candidate_scores"]
                     if s["branch_id"] == branch and s["within_motor_limits"]]
            candidate.append(valid[0]["integer_motor"][2] if valid else np.nan)
            costs.append(valid[0]["total_cost"] if valid else np.nan)
        axes[0].plot(t, candidate, color=color, ls=":", lw=1.1, label=branch+" 限位内候选")
        axes[2].plot(t, costs, color=color, lw=1.2, label=branch+" 总代价")
    axes[0].step(t, before[:, 2], where="post", color=gray, ls="--", lw=1.4, label="旧策略输出")
    axes[0].step(t, after[:, 2], where="post", color=blue, lw=1.7, label="新策略输出")
    axes[0].set_ylabel("M3 整数位置")
    axes[0].legend(loc="upper right", ncol=2, fontsize=9)
    axes[1].step(t, branch_values(baseline), where="post", color=gray, ls="--", lw=1.4)
    axes[1].step(t, branch_values(current), where="post", color=blue, lw=1.7)
    axes[1].set_yticks([-1, 1], ["−acos", "+acos"]); axes[1].set_ylim(-1.4, 1.4)
    axes[1].set_ylabel("选中根")
    axes[2].set_ylabel("运动代价 + 切换惩罚\n低代价区间放大")
    axes[2].set_yscale("symlog", linthresh=.002)
    axes[2].set_ylim(bottom=0)
    axes[2].legend(loc="upper right", fontsize=9)
    axes[2].set_xlabel("录制时间（秒）")
    hysteresis = np.array([r["solver"]["branch_selection"]["reason"] == "stay_branch_hysteresis" for r in current])
    for ax in axes:
        shade(ax, hysteresis, purple, .15)
        ax.set_xlim(*window); ax.grid(alpha=.2)
    fig.suptitle("M3 局部对照：淡紫色帧因切换惩罚保留原分支", fontsize=16)
    fig.text(.5, .02, "候选曲线缺口表示该根不满足电机限位；总代价以新策略上一有效整数输出为参考，纵轴采用对称对数刻度。", ha="center", fontsize=9.5)
    fig.tight_layout(rect=[0, .045, 1, .96])
    for suffix in ("png", "svg"):
        fig.savefig(out/f"m3_branch_detail.{suffix}", dpi=180, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_DIR)
    args = parser.parse_args()
    out = args.results_dir
    paths = {name: out/f"teleop_01_{name}.jsonl" for name in ("calibrated", "integer_nearest", "integer_hysteresis")}
    runs = {name: read(path) for name, path in paths.items()}
    baseline, current = runs["calibrated"], runs["integer_hysteresis"]
    assert len(baseline) == len(current) == len(runs["integer_nearest"]) == 599
    source_path = ROOT/"results/solver_only/teleop_01_workspace_conditional.jsonl"
    reference_path = ROOT/"results/solver_only/teleop_01_workspace_conditional_rounded_no_jump.jsonl"
    source = read(source_path) if source_path.exists() else None
    reference = read(reference_path) if reference_path.exists() else None
    if source is not None:
        assert len(source) == len(current)
    if reference is not None:
        assert len(reference) == len(current)
    for i, row in enumerate(current):
        for name, rows in runs.items():
            candidate = rows[i]
            assert candidate["frame_index"] == i and candidate["timestamp"] == baseline[i]["timestamp"]
            assert candidate["solver"]["requested"] == candidate["solver"]["applied"] == baseline[i]["solver"]["requested"]
            adapter = candidate["solver"]["adapter"]
            assert adapter["requested_angles"] == adapter["used_angles"] == baseline[i]["solver"]["adapter"]["requested_angles"]
            assert adapter["candidates"] == baseline[i]["solver"]["candidates"]
            assert not adapter["projected"] and None not in adapter["candidate_branch_ids"]
            if source is not None:
                assert candidate["timestamp"] == source[i]["timestamp"]
                assert candidate["solver"]["requested"] == source[i]["solver"]["input"]
                assert adapter["requested_angles"] == source[i]["solver"]["requested_angles"]
                assert adapter["candidates"] == source[i]["solver"]["candidates"]
        if reference is not None:
            assert np.array_equal(np.rint(baseline[i]["solver"]["selected_motor"]), reference[i]["rounded_motor"])
        assert all(float(v).is_integer() and lo <= v <= hi for v, (lo, hi) in zip(
            row["solver"]["selected_motor"], PALM_MOTOR_SAFE_LIMITS.values()))
        assert row["solver"]["selected_motor"] == row["control"]["selected_motor"]
    penalty = current[0]["solver"]["branch_selection"]["switch_penalty"]
    assert compare_outputs(current, replay(baseline, penalty))
    assert compare_outputs(current, replay(baseline, penalty, reverse=True))
    assert compare_outputs(current, replay(baseline, penalty, time_scale=.5))
    assert compare_outputs(current, replay(baseline, penalty, time_scale=2))
    before = np.rint([r["solver"]["selected_motor"] for r in baseline])
    after = np.array([r["solver"]["selected_motor"] for r in current])
    different = np.any(before != after, axis=1)
    sensitivity = []
    for value in (0, .0001, .001, .0025, .005, .01):
        data = replay(baseline, value)
        sensitivity.append({"switch_penalty": value, **metrics(data)})
    summary = {"frames": len(current), "duration_seconds": current[-1]["timestamp"],
               "configuration": {"mapping_mode": "workspace_conditional", "input": "raw",
                    "motor_weights": [1, 1, 1], "motor_spans": [1000, 510, 135],
                    "branch_switch_penalty": penalty, "filter_tau": 0,
                    "input_slew_enabled": False, "position_jump_guard_enabled": False,
                    "speed_guard_enabled": False, "rounded_model_gate_enabled": False},
               "metrics": {name: metrics(rows) for name, rows in runs.items()},
               "changed_output_frame_indices": np.flatnonzero(different).tolist(),
               "motor_output_unchanged": np.all(before == after, axis=0).tolist(),
               "penalty_sensitivity": sensitivity,
               "checks": {"matches_actual_runner": True, "candidate_order_independent": True,
                    "timestamp_scale_independent": True, "baseline_mapping_and_angles_unchanged": True,
                    "original_solver_log_crosschecked": source is not None,
                    "baseline_matches_previous_rounded_plot": True if reference is not None else None,
                    "all_candidate_root_ids_identified": True},
               "source_logs": {name: {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                               for name, path in paths.items()},
               "solver_core_sha256": hashlib.sha256((ROOT/"DHandControl/scripts/mh6_palm_solver_v2.py").read_bytes()).hexdigest(),
               "note": "20 position units is only a diagnostic marker, not a rejection threshold. Phase windows describe teleop_01 approximately. No hardware action."}
    (out/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    plot(out, baseline, current, different)
    print(json.dumps({"summary": str((out/"summary.json").resolve()), "metrics": summary["metrics"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
