"""Inspect three raw recordings and their solver-only logs; no hardware access.

First generate the logs with mh6_teleop_run.py --solver-only, fixed
recordings/mapping_guided_01.json, and --replay-no-wait, then run this script.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "results/solver_only/guided_actions_01"
NAMES = ("grasp_01", "opposition_01", "tripod_pinch_01")
TITLES = ("Power grasp", "Four finger oppositions", "Tripod pinch")
LIMITS = ((0, 1000), (120, 630), (401, 536))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def windows(mask, t):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return [{"start_s": float(t[a]), "end_s": float(t[b-1]), "frames": int(b-a)}
            for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))]


def shade(ax, mask, t):
    for w in windows(mask, t):
        ax.axvspan(w["start_s"], w["end_s"], color="red", alpha=.12)


def style(ax, t, ylabel):
    ax.set_xlim(0, t[-1])
    ax.set_ylabel(ylabel)
    ax.grid(alpha=.25)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    report = {"calibration": str(ROOT / "recordings/mapping_guided_01.json"),
              "calibration_sha256": sha(ROOT / "recordings/mapping_guided_01.json"),
              "method": "Raw mapping, native workspace_conditional, project_invalid=False; all branches; no filtering, selection, jump guards or fallback. Limits are current production motor limits. No post-rounding model validation.",
              "sessions": {}}
    overview, axes = plt.subplots(3, 3, figsize=(17, 10), constrained_layout=True)
    for col, (name, title) in enumerate(zip(NAMES, TITLES)):
        source = ROOT / f"recordings/{name}.npz"
        with np.load(source, allow_pickle=False) as z:
            t, raw = z["timestamps"], z["transforms"]
            phases, hands = z["phases"], z["hands"]
            assert raw.shape == (len(t), 27, 4, 4) and np.isfinite(raw).all()
            assert (np.diff(t) > 0).all()
            assert (phases == "teleop").all() and (hands == "right").all()
        log = OUT / f"{name}.jsonl"
        rows = [json.loads(line) for line in log.read_text().splitlines() if line]
        assert len(rows) == len(t)
        assert np.allclose([r["timestamp"] for r in rows], t, atol=1e-6)
        mapping = np.array([[r["solver"]["input"][k] for k in
                            ("palm_flexion", "palm_cross", "thumb_inward")] for r in rows])
        assert np.isfinite(mapping).all() and ((mapping >= 0) & (mapping <= 1)).all()
        bad = np.array([not r["solver"]["has_valid_motor_solution"] for r in rows])
        angles = np.array([r["solver"]["requested_angles"] for r in rows])
        branches = {}
        violations = Counter()
        for i, r in enumerate(rows):
            solver = r["solver"]
            assert not solver["adapter"]["projected"]
            for branch, motor in zip(solver["adapter"]["candidate_branch_ids"], solver["candidates"]):
                branches.setdefault(branch, np.full((len(t), 3), np.nan))[i] = motor
                if bad[i]:
                    for j, (value, (lo, hi)) in enumerate(zip(motor, LIMITS)):
                        if value < lo or value > hi:
                            violations[f"M{j+1}_{'below' if value < lo else 'above'}_limit"] += 1
        stats = json.loads((OUT / f"{name}.summary.json").read_text())
        stats.update({"source": str(source), "source_sha256": sha(source),
                      "frame_log_sha256": sha(log),
                      "sampling_dt_percentiles_s": dict(zip(("min", "median", "p95", "max"), np.percentile(np.diff(t), [0, 50, 95, 100]).tolist())),
                      "exact_duplicate_adjacent_frames": int(np.all(raw[1:] == raw[:-1], axis=(1, 2, 3)).sum()),
                      "mapping_order": ["h_vertical", "v_lateral", "r_thumb"],
                      "mapping_min": mapping.min(0).tolist(), "mapping_max": mapping.max(0).tolist(),
                      "first_3s_mapping_median": np.median(mapping[t < 3], 0).tolist(),
                      "last_3s_mapping_median": np.median(mapping[t > t[-1]-3], 0).tolist(),
                      "no_motor_feasible_windows": windows(bad, t),
                      "candidate_violations_during_infeasible_frames": dict(violations),
                      "high_fold_h_gt_0_7": {"frames": int((mapping[:, 0] > .7).sum()),
                          "valid_frames": int(((mapping[:, 0] > .7) & ~bad).sum())}})
        report["sessions"][name] = stats

        fig, detail = plt.subplots(6, 1, figsize=(12, 15), sharex=True, constrained_layout=True)
        for ax in (axes[0, col], detail[0]):
            for j, label in enumerate(("h: vertical fold", "v: lateral fold", "r: thumb rotation")):
                ax.plot(t, mapping[:, j], label=label, lw=1.1)
            ax.set_ylim(-.03, 1.05)
            ax.legend(fontsize=8, loc="upper right")
            style(ax, t, "Normalized input")
        axes[0, col].set_title(f"{title}\nGeometry {stats['geometry_solution_rate']:.1%}; motor feasible {stats['valid_motor_solution_rate']:.2%}")
        detail[0].set_title(f"{title}: raw input and ALL motor branches; red = no motor-feasible branch")
        for j, label in enumerate(("alpha2", "alpha3", "theta1")):
            axes[1, col].plot(t, angles[:, j], label=label, lw=1.1)
        axes[1, col].legend(fontsize=8)
        style(axes[1, col], t, "Target angle (deg)")
        for finger in ("thumb", "index", "middle", "ring", "little"):
            detail[1].plot(t, [r["raw_mapping"]["curl_norm"][finger] for r in rows], label=finger, lw=1)
        for finger in ("index", "middle", "ring", "little"):
            detail[2].plot(t, [r["raw_mapping"]["opposition_distances"][finger]*1000 for r in rows], label=finger, lw=1)
        for ax, label in ((detail[1], "Finger curl [0,1]"), (detail[2], "Thumb-tip distance (mm)")):
            ax.legend(ncol=5, fontsize=8)
            style(ax, t, label)
        for j, (lo, hi) in enumerate(LIMITS):
            targets = [detail[3+j]] + ([axes[2, col]] if j == 2 else [])
            for ax in targets:
                for branch in ("minus_acos", "plus_acos"):
                    if branch not in branches:
                        continue
                    motors = branches[branch]
                    ax.plot(t, motors[:, j], label=branch, lw=1,
                            color={"minus_acos": "tab:blue", "plus_acos": "tab:orange"}[branch])
                ax.axhline(lo, color="black", ls="--", lw=.8)
                ax.axhline(hi, color="black", ls="--", lw=.8)
                # M1's very wide limit otherwise hides its motion.
                if j == 0:
                    values = np.concatenate([b[:, j] for b in branches.values()])
                    ax.set_ylim(np.nanmin(values)-15, np.nanmax(values)+15)
                ax.legend(fontsize=8)
                style(ax, t, f"M{j+1} candidate position")
        for ax in list(detail) + list(axes[:, col]):
            shade(ax, bad, t)
        detail[-1].set_xlabel("Time (s)")
        axes[-1, col].set_xlabel("Time (s)")
        fig.savefig(OUT / f"{name}_details.png", dpi=140)
        plt.close(fig)
    overview.suptitle("New guided calibration: mapping, native workspace targets and M3 branches\nRed shading: all branches outside production motor limits; dashed lines: M3 [401,536]")
    overview.savefig(OUT / "overview.png", dpi=150)
    overview.savefig(OUT / "overview.svg")
    plt.close(overview)
    (OUT / "inspection.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    for name, s in report["sessions"].items():
        print(name, "bad windows:", s["no_motor_feasible_windows"], "high fold:", s["high_fold_h_gt_0_7"])


if __name__ == "__main__":
    main()
