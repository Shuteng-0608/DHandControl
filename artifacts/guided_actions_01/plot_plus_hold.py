"""Plot a fixed plus branch, holding its last accepted M3 position offline."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "results/solver_only/guided_actions_01"
LIMITS = ((0, 1000), (120, 630), (401, 536))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--include-all-motors", action="store_true",
                        help="Add raw plus-branch M1/M2; only M3 uses hold.")
    args = parser.parse_args()
    fig, axes = plt.subplots(3, 3, figsize=(17, 10), constrained_layout=True)
    summary = {"policy": "Fixed plus_acos; accept only when the full candidate passes the existing project motor bounds; otherwise hold last accepted M3. No jump guard or filtering; no integer rounding.",
               "motor_bounds_hardware_order": LIMITS, "sessions": {}}
    for col, (name, title) in enumerate(zip(
            ("grasp_01", "opposition_01", "tripod_pinch_01"),
            ("Power grasp", "Four finger oppositions", "Tripod pinch"))):
        rows = [json.loads(line) for line in (OUT / f"{name}.jsonl").read_text().splitlines() if line]
        t = np.array([row["timestamp"] for row in rows])
        mapping = np.array([[row["solver"]["input"][key] for key in
                            ("palm_flexion", "palm_cross", "thumb_inward")] for row in rows])
        angles = np.array([row["solver"]["requested_angles"] for row in rows])
        last_m3 = None
        saved = []
        for row in rows:
            solver = row["solver"]
            plus = next((candidate for branch, candidate in zip(
                solver["adapter"]["candidate_branch_ids"], solver["candidates"])
                if branch == "plus_acos"), None)
            accepted = plus is not None and all(
                np.isfinite(x) and lo <= x <= hi for x, (lo, hi) in zip(plus, LIMITS))
            if accepted:
                last_m3 = plus[2]
            # No initial artificial motor position if the first branch is invalid.
            saved.append({"frame_index": row["frame_index"], "timestamp": row["timestamp"],
                          "branch": "plus_acos", "candidate": plus,
                          "accepted": bool(accepted), "held": not accepted and last_m3 is not None,
                          "m3_position": last_m3})
        accepted_count = sum(row["accepted"] for row in saved)
        for j, label in enumerate(("h: vertical fold", "v: lateral fold", "r: thumb rotation")):
            axes[0, col].plot(t, mapping[:, j], label=label, lw=1.1)
        axes[0, col].set_ylim(-.03, 1.05)
        axes[0, col].set_ylabel("Normalized input")
        axes[0, col].set_title(title)
        for j, label in enumerate(("alpha2", "alpha3", "theta1")):
            axes[1, col].plot(t, angles[:, j], label=label, lw=1.1)
        axes[1, col].set_ylabel("Target angle (deg)")
        positions = np.array([row["m3_position"] if row["m3_position"] is not None else np.nan for row in saved])
        if args.include_all_motors:
            for j, color in ((0, "tab:blue"), (1, "tab:orange")):
                raw_motor = [row["candidate"][j] if row["candidate"] is not None else np.nan for row in saved]
                axes[2, col].plot(t, raw_motor, color=color, label=f"M{j+1} (plus, raw)", lw=1.1)
        axes[2, col].plot(t, positions, color="tab:green" if args.include_all_motors else "tab:orange",
                          label="M3 (plus, hold if rejected)", lw=1.2)
        for value in LIMITS[2]:
            axes[2, col].axhline(value, color="black", ls="--", lw=.8)
        axes[2, col].set_ylabel("Motor position" if args.include_all_motors else "M3 position")
        if not args.include_all_motors:
            axes[2, col].set_ylim(390, 545)
        axes[2, col].set_xlabel("Time (s)")
        for ax in axes[:, col]:
            ax.set_xlim(0, t[-1])
            ax.grid(alpha=.25)
            ax.legend(fontsize=8, loc="upper right")
        path = OUT / f"{name}_plus_hold.jsonl"
        path.write_text("".join(json.dumps(row, allow_nan=False) + "\n" for row in saved))
        summary["sessions"][name] = {"frames": len(rows), "accepted": accepted_count,
                                     "held": sum(row["held"] for row in saved),
                                     "uninitialized": sum(row["m3_position"] is None for row in saved),
                                     "frame_log": str(path)}
    label = "M1/M2 raw, M3 hold" if args.include_all_motors else "M3 hold"
    fig.suptitle(f"Guided calibration: original mapping and target angles; fixed plus branch ({label})\nOnly M3 holds on rejection; dashed lines: existing project M3 bounds [401,536]")
    stem = "overview_plus_hold_all_motors" if args.include_all_motors else "overview_plus_hold"
    fig.savefig(OUT / f"{stem}.png", dpi=150)
    fig.savefig(OUT / f"{stem}.svg")
    plt.close(fig)
    (OUT / "plus_hold.summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary["sessions"], indent=2))


if __name__ == "__main__":
    main()
