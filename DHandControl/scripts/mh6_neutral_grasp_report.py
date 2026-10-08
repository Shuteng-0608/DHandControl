"""Check and plot three actual natural-to-grasp replay logs (offline only)."""

import argparse
from collections import Counter
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

from mh6_palm_calibration import PALM_MOTOR_SAFE_LIMITS, PALM_NEUTRAL_MOTORS
from mh6_palm_solver_v2 import MH6PalmSolver


def read(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path(__file__).resolve().parents[2]
                        / "results/solver_only/neutral_to_grasp")
    args = parser.parse_args()
    out = args.directory
    runs = {name: read(out / f"teleop_01_{name}.jsonl")
            for name in ("solver", "guarded", "unguarded")}
    raw = runs["solver"]
    if len(raw) != 599 or any(len(rows) != len(raw) for rows in runs.values()):
        raise ValueError("Expected all 599 teleop_01 frames in each actual runner log")
    names = ("vertical", "lateral", "thumb_rotation_command")
    inputs = np.array([[row["raw_mapping"]["palm_command"][name] for name in names] for row in raw])
    times = np.array([row["timestamp"] for row in raw])
    assert np.all((inputs >= 0.) & (inputs <= 1.))
    for rows in runs.values():
        np.testing.assert_array_equal(times, [row["timestamp"] for row in rows])
        np.testing.assert_array_equal(inputs, [[row["raw_mapping"]["palm_command"][n] for n in names]
                                              for row in rows])
    core = MH6PalmSolver()
    zero = core.solve_motor_safe_from_normalized(0., 0., 0., mode="workspace_conditional",
                                               motor_order="timeseries", project_invalid=False)
    for row in raw:
        stage = row["solver"]["adapter"]
        assert stage["mapping_mode"] == "workspace_conditional"
        h, v, r = [row["raw_mapping"]["palm_command"][name] for name in names]
        np.testing.assert_array_equal([h, r, v], list(stage["workspace_input"].values()))
        np.testing.assert_array_equal(stage["requested_angles"],
                                      core.map_normalized(h, r, v, mode="workspace_conditional"))
    outputs = {}
    stats = {}
    for name in ("guarded", "unguarded"):
        rows = runs[name]
        motors = np.array([row["solver"]["selected_motor"] for row in rows])
        outputs[name] = motors
        assert np.all(motors == np.round(motors))
        for axis, (low, high) in enumerate(PALM_MOTOR_SAFE_LIMITS.values()):
            assert np.all((motors[:, axis] >= low) & (motors[:, axis] <= high))
        changes = np.diff(np.vstack([PALM_NEUTRAL_MOTORS, motors]), axis=0)
        stats[name] = {"status_counts": dict(Counter(row["solver"]["status"] for row in rows)),
                       "max_abs_step_including_startup": np.max(np.abs(changes), axis=0).tolist(),
                       "motor_min": motors.min(axis=0).tolist(), "motor_max": motors.max(axis=0).tolist()}
    summary = {"frames": len(raw), "duration_seconds": float(times[-1]),
               "mapping_range": {name: [float(values.min()), float(values.max())]
                                 for name, values in zip(names, inputs.T)},
               "geometric_solution_frames": sum(row["solver"]["has_solution"] for row in raw),
               "valid_motor_solution_frames": sum(row["solver"]["has_valid_motor_solution"] for row in raw),
               "actual_motor_limits": PALM_MOTOR_SAFE_LIMITS,
               "solver_zero_input": {"angles": zero["requested_angles"], "motor_candidates": zero["solutions"],
                                     "selected_motor": zero["selected"]},
               "controller_start_reference": PALM_NEUTRAL_MOTORS, "previews": stats,
               "checks": {"same_mapping_all_replays": True, "no_negative_palm_input": True,
                          "solver_angles_match_original_native_map": True},
               "scope": "Offline previews only; no serial connection or hardware command. Geometry rate is not update rate or physical validation."}
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    font = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
    fp = FontProperties(fname=str(font)) if font.exists() else FontProperties()
    fig, axes = plt.subplots(6, 1, sharex=True, figsize=(12, 13))
    labels = ("h：上下翻折", "v：左右翻折", "r：大拇指旋转")
    for axis, values, label in zip(axes[:3], inputs.T, labels):
        axis.plot(times, values, color="#1673c9", linewidth=1.4)
        axis.set_ylim(-.05, 1.05); axis.set_ylabel(label, fontproperties=fp)
    for i, axis in enumerate(axes[3:]):
        axis.plot(times, outputs["unguarded"][:, i], label="关闭跳变保护的选解", color="#1673c9", linewidth=1.2)
        axis.plot(times, outputs["guarded"][:, i], label="默认保护后的输出", color="#ef8b28", linewidth=1.2)
        low, high = PALM_MOTOR_SAFE_LIMITS[i+1]
        axis.axhline(PALM_NEUTRAL_MOTORS[i], color="gray", linestyle="--", linewidth=.8,
                     label="控制器现有初始参考")
        axis.axhline(zero["selected"][i], color="#8846b0", linestyle="-.", linewidth=.8,
                     label="Solver 原生零输入选解")
        axis.axhline(low, color="red", linestyle=":", linewidth=.8)
        axis.axhline(high, color="red", linestyle=":", linewidth=.8)
        axis.set_ylim(low-.04*(high-low), high+.04*(high-low))
        axis.set_ylabel(f"M{i+1} 位置", fontproperties=fp)
    axes[3].legend(prop=fp, loc="best")
    for axis in axes:
        axis.grid(alpha=.25)
    axes[-1].set_xlabel("录制时间（秒）", fontproperties=fp)
    fig.suptitle("自然状态 → 向内抓：三分量映射与整数电机输出", fontproperties=fp, fontsize=18)
    fig.tight_layout(rect=(0, 0, 1, .97))
    for suffix in ("png", "svg"):
        fig.savefig(out / f"mapping_and_motor_trajectories.{suffix}", dpi=170)
    plt.close(fig)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
