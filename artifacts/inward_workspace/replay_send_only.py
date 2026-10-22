#!/usr/bin/env python3
"""Offline-only counterfactual replay of a send-only JSONL log. No device I/O."""
import argparse
from collections import Counter
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "DHandControl/scripts"))
from mh6_actuator_preview import NineChannelPreview, FINGER_KEYS, prepare_packet
from mh6_hardware_config import HardwareConfig, FINGERS, PALM
from mh6_palm_solver_adapter import PalmSolverAdapter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "results/hardware_send_only/live_20261022_200151_822363.jsonl")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results/inward_workspace/live_20261022_200151")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(line) for line in args.input.read_text().splitlines() if line.strip()]
    config = HardwareConfig(rows[0]["hardware_config"])
    adapter = PalmSolverAdapter(motor_limits=config.palm_diagnostic_limits())
    planner = NineChannelPreview(config)
    source_hash = hashlib.sha256(args.input.read_bytes()).hexdigest()
    zero = adapter.solve_motor_from_teleop(0., 0., 0.)
    assert zero["solutions"] == [[247., 500., 500.]]
    output = []
    times = []
    rotation_errors = []
    translation_errors = []
    for old in rows:
        semantic = old["solver"]["semantic_input"]
        started = time.perf_counter()
        solved = adapter.solve_motor_from_teleop(**semantic)
        times.append(1000*(time.perf_counter()-started))
        # Reuse the exact already-filtered inputs logged by the live run.
        # Do not filter them a second time or change finger/M4 calibration.
        mapping = copy.deepcopy(old["raw_mapping"])
        for name, key in zip(FINGERS, FINGER_KEYS):
            mapping["low_dim"][key] = old["control"]["channels"][name]["normalized_input"]
        mapping["low_dim"]["u_thumb_abduction"] = old["control"]["m4_mapping"]["normalized"]
        mapping["m4_mapping"]["calibrated"] = old["control"]["m4_mapping"]["calibrated"]
        proposed = planner.propose(mapping, solved)
        packet = prepare_packet(proposed)
        # Retain the original acknowledgement schedule, including the failed
        # final write. These are simulated references, never a device receipt.
        source_ack = old["execution"]["write_acknowledged"]
        if source_ack:
            planner.commit_preview(proposed)
        for name in PALM:
            target = proposed["positions"][name]
            assert config.actuators[name]["min"] <= target <= config.actuators[name]["max"]
        assert config.fixed_branch == "plus_acos"
        for axis in (*FINGERS, "M4"):
            assert proposed["positions"][axis] == old["control"]["positions"][axis]
        roots = adapter.solver.solve_remaining(*solved["used_angles"])
        for candidate, identity in zip(solved["candidates"], solved["candidate_branch_ids"]):
            if identity != "plus_acos":
                continue
            for root in roots:
                if abs(500.+99./23.6*root[2]-candidate[2]) < .000051:
                    rotation_errors.append(root[3])
                    translation_errors.append(root[4])
        output.append({"format_version": 3, "mode": "OFFLINE_RECOMPUTE_NOT_SENT",
                       "frame_index": old["frame_index"], "timestamp": old["timestamp"],
                       "raw_mapping": old["raw_mapping"], "solver": solved,
                       "control": proposed, "packet_preview": packet,
                       "source_write_acknowledged": source_ack,
                       "simulated_history_committed": source_ack})
    path = args.output_dir / "recomputed.jsonl"
    with path.open("w") as log:
        for row in output:
            log.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+"\n")
    old_counts = Counter(r["control"]["palm"]["reason"] or "ACCEPTED" for r in rows)
    new_counts = Counter(r["control"]["palm"]["reason"] or "ACCEPTED" for r in output)
    summary = {"mode": "OFFLINE_RECOMPUTE_NOT_SENT", "source": str(args.input.resolve()),
               "source_sha256": source_hash, "frames": len(rows),
               "workspace_calibration": asdict(adapter.calibration),
               "rigid_solver_sha256": hashlib.sha256((ROOT / "DHandControl/scripts/mh6_palm_solver_v2.py").read_bytes()).hexdigest(),
               "input": "exact logged filtered semantic h/v/r; original finger/M4 inputs and acknowledgement schedule",
               "zero_input_motor": zero["solutions"], "hardware_config": config.to_dict(),
               "source_palm_status": dict(old_counts), "recomputed_palm_status": dict(new_counts),
               "geometry_and_plus_bounds_passed": sum(bool(r["solver"]["solutions"]) for r in output),
               "max_rotation_residual": max(rotation_errors),
               "max_translation_residual_mm": max(translation_errors),
               "compute_ms": {"mean": float(np.mean(times)), "p95": float(np.percentile(times,95)), "max": max(times)},
               "motor_ranges": {}, "finger_and_m4_targets_unchanged": True,
               "actual_hardware_sends": 0,
               "source_unchanged": source_hash == hashlib.sha256(args.input.read_bytes()).hexdigest()}
    for i, name in enumerate(PALM):
        selected = [r["control"]["positions"][name] for r in output]
        requested = [r["control"]["palm"]["requested_motor"][i] for r in output]
        deltas = np.diff([config.actuators[name]["initial_position"], *selected])
        summary["motor_ranges"][name] = {"requested": [min(requested), max(requested)],
                                         "selected": [min(selected), max(selected)],
                                         "maximum_selected_step": int(max(abs(deltas)))}
    (args.output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.array([r["execution"]["started_monotonic"] for r in rows])
    t -= t[0]
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True, layout="constrained")
    fig.suptitle("Offline recomputation: inward-only workspace, fixed plus assembly\n"
                 "Same recorded inputs and guards; no hardware commands sent")
    for key, label in (("vertical", "h: vertical fold"), ("lateral", "v: lateral fold"),
                       ("thumb_rotation_command", "r: thumb rotation")):
        axes[0].plot(t, [r["solver"]["semantic_input"][key] for r in rows], label=label)
    axes[0].set_ylabel("Filtered intention"); axes[0].legend()
    for i, (name, ax) in enumerate(zip(PALM, axes[1:])):
        ax.plot(t, [r["control"]["palm"]["requested_motor"][i] for r in output], label="New requested solution")
        ax.step(t, [r["control"]["positions"][name] for r in output], where="post", label="New guarded target")
        ax.plot(t, [r["control"]["positions"][name] for r in rows], ls="--", color="gray", label="Original target")
        ax.set_ylabel(name+" position"); ax.legend(loc="best", fontsize=8)
    for ax in axes:
        ax.grid(alpha=.2)
    axes[-1].set_xlabel("Recorded host time (s)")
    fig.savefig(args.output_dir / "comparison.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
