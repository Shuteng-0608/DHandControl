#!/usr/bin/env python3
"""Verify and plot two fixed-root teleop_01 replays, without accessing hardware."""

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from mh6_branch_selection_report import read, metrics
from mh6_palm_calibration import PALM_MOTOR_SAFE_LIMITS, PALM_NEUTRAL_MOTORS
from mh6_palm_solution_selector import PalmSolutionSelector

ROOT = Path(__file__).resolve().parents[2]
BRANCHES = ("plus_acos", "minus_acos")


def verify(rows, branch):
    target = []; valid = []
    previous = list(PALM_NEUTRAL_MOTORS)
    selector = PalmSolutionSelector(selection_policy="integer_continuous", fixed_branch_id=branch,
                                    max_normalized_speed_per_sec=None, allow_unguarded=True)
    for i, row in enumerate(rows):
        solver = row["solver"]
        ids = solver["adapter"]["candidate_branch_ids"]
        index = ids.index(branch)
        candidate = solver["candidates"][index]
        target.append([int(round(x)) for x in candidate])
        within_limits = PalmSolutionSelector._is_valid_solution(candidate)
        valid.append(within_limits)
        assert row["frame_index"] == i
        assert solver["branch_selection"]["fixed_branch_id"] == branch
        assert solver["status"].endswith("FIXED_BRANCH_OUT_OF_LIMITS") or solver["status"] == "SELECTED"
        assert (solver["status"] == "SELECTED") == within_limits
        if within_limits:
            previous = target[-1]
            assert solver["branch_selection"]["candidate_branch_id"] == branch
            assert solver["branch_selection"]["selected_branch_id"] == branch
        assert solver["selected_motor"] == previous == row["control"]["selected_motor"]
        assert not solver["branch_selection"]["branch_changed"]
        inputs = solver["applied"]
        # Reversed candidate order proves fixed root selection never uses an index.
        replay = selector.select((inputs["palm_flexion"], inputs["thumb_inward"], inputs["palm_cross"]),
                                 solver["candidates"][::-1], branch_ids=ids[::-1], timestamp=row["timestamp"])
        assert replay.selected_motor == solver["selected_motor"] and replay.status == solver["status"]
    return np.array(target), np.array(valid)


def plot(out, rows, targets, valid):
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
    plt.rcParams.update({"font.size": 11, "axes.unicode_minus": False, "svg.fonttype": "none",
                         "axes.spines.top": False, "axes.spines.right": False})
    t = np.array([r["timestamp"] for r in rows[BRANCHES[0]]])
    colors = {"plus_acos": "#1466B8", "minus_acos": "#DF852A"}
    names = {"plus_acos": "+acos", "minus_acos": "−acos"}
    labels = ["M1 · 大拇指驱动 α3", "M2 · 上下翻折驱动 α2", "M3 · 闭环驱动 α1"]
    for kind in ("raw", "bounded"):
        fig, axes = plt.subplots(3, 1, figsize=(15, 9.5), sharex=True)
        for j, ax in enumerate(axes):
            data = []
            for branch in BRANCHES:
                values = targets[branch] if kind == "raw" else np.rint([r["solver"]["selected_motor"] for r in rows[branch]])
                data.append(values[:, j])
                ax.step(t, values[:, j], where="post", color=colors[branch], lw=1.35,
                        ls="-" if branch == "plus_acos" else "--")
            lo, hi = PALM_MOTOR_SAFE_LIMITS[j+1]
            low = min(lo, *(float(x.min()) for x in data))
            high = max(hi, *(float(x.max()) for x in data))
            margin = (high-low)*.07
            ax.set_ylim(low-margin, high+margin)
            ax.axhline(lo, color="#C9475B", ls=":", lw=1)
            ax.axhline(hi, color="#C9475B", ls=":", lw=1)
            ax.axhspan(low-margin, lo, color="#C9475B", alpha=.055)
            ax.axhspan(hi, high+margin, color="#C9475B", alpha=.055)
            ax.text(.99, .91, f"实际限位 [{lo}, {hi}]", transform=ax.transAxes, ha="right", fontsize=10)
            ax.set_ylabel(labels[j]+"\n电机位置单位")
            ax.set_xlim(0, t[-1]); ax.set_xticks(np.arange(0, t[-1]+1, 2)); ax.grid(alpha=.18)
            for boundary in (9.5, 17.5, 25.5):
                ax.axvline(boundary, color="#75818C", ls=":", lw=.8)
        for a, b, name in [(0, 9.5, "前三次握拳"), (9.5, 17.5, "四种对指"),
                            (17.5, 25.5, "后三次握拳"), (25.5, t[-1], "收尾")]:
            axes[-1].text((a+b)/2, -.19, name, ha="center", transform=axes[-1].get_xaxis_transform(), fontsize=10)
        axes[-1].set_xlabel("录制时间（秒）", labelpad=30)
        title = "固定解分支：原始整数解轨迹（包含越界解）" if kind == "raw" else "固定解分支：限位内更新、越界保持后的整数输出"
        fig.suptitle(title, fontsize=17, y=.985)
        fig.text(.5, .948, "599 帧同一录制 · workspace_conditional · 无低通、输入限速和跳变门限 · 全程不切换解析根",
                 ha="center", fontsize=10.5, color="#53606D")
        fig.legend(handles=[Line2D([0], [0], color=colors[b], ls="-" if b == "plus_acos" else "--",
                                 label=f"固定 {names[b]} 分支 · {int(valid[b].sum())}/599 帧满足限位") for b in BRANCHES] +
                           [Patch(facecolor="#C9475B", alpha=.15, label="限位之外的区域")],
                   loc="upper center", bbox_to_anchor=(.5, .925), ncol=3, frameon=False, fontsize=10)
        note = ("M1、M2 的两条原始轨迹完全相同，曲线重合；红色区域内的解只作数学诊断。M3 为 α1 驱动量，θ1 由闭环关系决定。"
                if kind == "raw" else "固定根越界时保持三个电机的上一次有效输出，初始参考 [247,500,500]；不借用另一根，不独立截断电机位置。")
        fig.text(.5, .014, note+" 动作分段为近似窗口。", ha="center", fontsize=9.5, color="#53606D")
        fig.subplots_adjust(top=.85, bottom=.13, left=.12, right=.98, hspace=.23)
        for suffix in ("png", "svg"):
            fig.savefig(out/f"fixed_branch_{kind}_trajectories.{suffix}", dpi=180, facecolor="white")
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=ROOT/"results/solver_only/fixed_branch")
    args = parser.parse_args()
    out = args.results_dir
    rows = {b: read(out/f"teleop_01_{b}.jsonl") for b in BRANCHES}
    assert all(len(data) == 599 for data in rows.values())
    targets = {}; valid = {}
    for branch in BRANCHES:
        targets[branch], valid[branch] = verify(rows[branch], branch)
    for p, m in zip(rows["plus_acos"], rows["minus_acos"]):
        assert p["timestamp"] == m["timestamp"]
        assert p["solver"]["requested"] == p["solver"]["applied"] == m["solver"]["requested"] == m["solver"]["applied"]
        assert p["solver"]["adapter"]["requested_angles"] == m["solver"]["adapter"]["requested_angles"]
        assert p["solver"]["candidates"] == m["solver"]["candidates"]
    assert np.array_equal(targets["plus_acos"][:, :2], targets["minus_acos"][:, :2])
    summary = {"frames": 599, "duration_seconds": rows["plus_acos"][-1]["timestamp"],
               "configuration": {"fixed_root_replays": list(BRANCHES), "filter_tau": 0,
                    "position_jump_guard": False, "speed_guard": False, "input_slew": False,
                    "integer_model_gate": False, "actual_motor_limits": dict(PALM_MOTOR_SAFE_LIMITS)},
               "branches": {}, "checks": {"fixed_root_never_switched": True,
                    "reversed_candidate_order_matches_actual_runner": True,
                    "same_mapping_and_requested_angles": True, "raw_M1_M2_identical_between_roots": True}}
    for branch in BRANCHES:
        raw_steps = np.diff(targets[branch], axis=0)
        summary["branches"][branch] = {"geometry_frames": len(targets[branch]),
            "actual_limit_valid_frames": int(valid[branch].sum()),
            "actual_limit_valid_fraction": float(valid[branch].mean()),
            "raw_integer_min": targets[branch].min(axis=0).tolist(),
            "raw_integer_max": targets[branch].max(axis=0).tolist(),
            "raw_max_abs_step_excluding_startup": np.abs(raw_steps).max(axis=0).tolist(),
            "bounded_output": metrics(rows[branch])}
    source_path = ROOT/"results/solver_only/teleop_01_workspace_conditional.jsonl"
    if source_path.exists():
        from mh6_trajectory_audit import phase_masks
        source = read(source_path)
        masks, _, _ = phase_masks(source)
        for branch in BRANCHES:
            for actual, original in zip(rows[branch], source):
                assert actual["timestamp"] == original["timestamp"]
                assert actual["solver"]["requested"] == original["solver"]["input"]
                assert actual["solver"]["candidates"] == original["solver"]["candidates"]
            summary["branches"][branch]["gesture_core_proxies"] = {
                name: {"frames": int(mask.sum()), "accepted": int(valid[branch][mask].sum())}
                for name, mask in masks.items()}
    with (out/"fixed_branch_targets.jsonl").open("w") as file:
        for i in range(599):
            record = {"frame_index": i, "timestamp": rows["plus_acos"][i]["timestamp"],
                      "solver_input": rows["plus_acos"][i]["solver"]["requested"],
                      "requested_angles": rows["plus_acos"][i]["solver"]["adapter"]["requested_angles"],
                      "branches": {branch: {"integer_target": targets[branch][i].tolist(),
                          "within_actual_limits": bool(valid[branch][i]),
                          "bounded_output": rows[branch][i]["solver"]["selected_motor"],
                          "status": rows[branch][i]["solver"]["status"]} for branch in BRANCHES}}
            file.write(json.dumps(record, ensure_ascii=False)+"\n")
    (out/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
    plot(out, rows, targets, valid)
    print(json.dumps({"summary": str((out/"summary.json").resolve()), "branches": summary["branches"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
