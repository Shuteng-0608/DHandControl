#!/usr/bin/env python3
"""Offline paired intent/solver comparison on the same recorded hand frames."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"DHandControl/scripts"))
from mh6_hand_session import ReplayHandStream
from mh6_mapping import MH6HandMapper,MappingCalibration
from mh6_intent_fusion import ThumbParticipationFusion
from mh6_teleop_run import CommandLowPassFilter
from mh6_palm_solver_adapter import PalmSolverAdapter
from mh6_hardware_config import HardwareConfig


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping",type=Path,default=ROOT/"recordings/mapping_guided_02.json")
    parser.add_argument("--output-dir",type=Path,default=ROOT/"results/thumb_participation")
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    calibration=MappingCalibration.load_json(args.mapping)
    fusion=ThumbParticipationFusion.load()
    hardware=HardwareConfig.load()
    report={"mode":"OFFLINE_NOT_SENT","mapping":str(args.mapping.resolve()),
            "mapping_sha256":hashlib.sha256(args.mapping.read_bytes()).hexdigest(),
            "fusion":fusion.to_dict(),"filter_tau_seconds":.24,"nominal_rate_hz":30,
            "rigid_solver_sha256":hashlib.sha256((ROOT/"DHandControl/scripts/mh6_palm_solver_v2.py").read_bytes()).hexdigest(),
            "actual_hardware_sends":0,"datasets":{},
            "note":"Paired raw skeletons and human calibration. Solver rates precede position guards. Known workspace exceptions are recorded per frame for diagnosis; the live runner still exits on that exception."}
    collections=[]
    for name,title in (("grasp_01","Power grasp"),("opposition_01","Four finger oppositions"),("tripod_pinch_01","Tripod pinch motion")):
        source=ROOT/"recordings"/(name+".npz")
        stream=ReplayHandStream(source,no_wait=True);stream.start()
        mappers=[MH6HandMapper(calibration),MH6HandMapper(calibration,intent_fusion=fusion)]
        filters=[CommandLowPassFilter(.24,nominal_dt=1/30) for _ in mappers]
        solvers=[PalmSolverAdapter(motor_limits=hardware.palm_diagnostic_limits()) for _ in mappers]
        records=[];counts=[Counter(),Counter()];examples=[[],[]]
        for i in np.flatnonzero(stream.phases=="teleop"):
            points=stream.transforms[i,:,:3,3];now=float(stream.timestamps[i])
            raw=[mapper.step(points) for mapper in mappers]
            filtered=[filt.apply(row,now) for filt,row in zip(filters,raw)]
            assert raw[0]["finger_bending"]==raw[1]["finger_bending"]
            assert raw[0]["m4_mapping"]==raw[1]["m4_mapping"]
            for key in ("power_grasp","opposition_cross"):
                assert raw[0]["grasp_intent"][key]==raw[1]["grasp_intent"][key]
            for key in ("u_thumb","u_index","u_middle","u_ring","u_little","u_thumb_abduction"):
                assert filtered[0]["low_dim"][key]==filtered[1]["low_dim"][key]
            assert "tripod_precision" not in raw[1]["intent"]
            observations=[]
            for profile,(row,solver) in enumerate(zip(filtered,solvers)):
                palm=row["palm_command"]
                try:
                    solution=solver.solve_motor_from_teleop(palm["vertical"],palm["lateral"],palm["thumb_rotation_command"])
                    status="SOLVED" if solution["success"] else solution["status"]
                    observations.append({"status":status,"has_geometry":bool(solution["candidates"]),
                                         "valid_plus":solution["success"],"solutions":solution["solutions"]})
                except ValueError as exc:
                    status="WORKSPACE_GENERATION_ERROR"
                    observations.append({"status":status,"error":str(exc),"has_geometry":False,"valid_plus":False})
                    if len(examples[profile])<5:examples[profile].append({"frame":int(i),"time":now,"error":str(exc)})
                counts[profile][status]+=1
            old,new=raw
            records.append({"timestamp":now,"legacy_intents":old["grasp_intent"],"new_intents":new["grasp_intent"],
                            "thumb_components":{"rotation":new["low_dim"]["u_thumb_rotation"],"abduction":new["low_dim"]["u_thumb_abduction"]},
                            "new_contributions":new["palm_command"],
                            "legacy_filtered":filtered[0]["palm_command"],"new_filtered":filtered[1]["palm_command"],
                            "solver":dict(zip(("legacy","thumb_participation"),observations))})
        stream.stop()
        n=len(records);assert n
        summary={"source":str(source.resolve()),"source_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),"frames":n,
                 "direct_finger_and_m4_commands_unchanged":True,"power_and_opposition_intents_unchanged":True,
                 "solver_statuses":dict(zip(("legacy","thumb_participation"),[dict(c) for c in counts])),
                 "workspace_exception_examples":dict(zip(("legacy","thumb_participation"),examples)),
                 "valid_plus_rate":{p:sum(r["solver"][p]["valid_plus"] for r in records)/n for p in ("legacy","thumb_participation")},
                 "folds":{},"old_tripod_overlap":{}}
        active=[r for r in records if r["legacy_intents"]["tripod_precision"]>.05]
        summary["old_tripod_overlap"]={"active_frames":len(active),
            "also_power_above_005":sum(r["legacy_intents"]["power_grasp"]>.05 for r in active),
            "also_opposition_above_005":sum(r["legacy_intents"]["opposition_cross"]>.05 for r in active)}
        for key in ("vertical","lateral","thumb_rotation_command"):
            old_values=np.array([r["legacy_filtered"][key] for r in records]);new_values=np.array([r["new_filtered"][key] for r in records])
            summary["folds"][key]={"legacy_range":[float(min(old_values)),float(max(old_values))],
                                   "new_range":[float(min(new_values)),float(max(new_values))],
                                   "mean_absolute_change":float(np.mean(abs(new_values-old_values)))}
        summary["preclip_saturation_frames"]={
            "legacy":{
                "vertical":sum(calibration.vertical_power_gain*r["legacy_intents"]["power_grasp"]+
                               calibration.vertical_tripod_gain*r["legacy_intents"]["tripod_precision"]>1 for r in records),
                "lateral":sum(calibration.lateral_power_gain*r["legacy_intents"]["power_grasp"]+
                              calibration.lateral_opposition_gain*r["legacy_intents"]["opposition_cross"]>1 for r in records)},
            "thumb_participation":{key:sum(r["new_contributions"][key+"_before_clip"]>1 for r in records)
                                   for key in ("vertical","lateral")}}
        summary["thumb_input_saturation_frames"]={key:sum(r["thumb_components"][key]>=1-1e-9 for r in records)
                                                  for key in ("rotation","abduction")}
        report["datasets"][name]=summary
        with (args.output_dir/(name+".jsonl")).open("w") as f:
            for record in records:f.write(json.dumps(record,allow_nan=False)+"\n")
        collections.append((title,records))
        print(name,json.dumps(summary["solver_statuses"]),flush=True)
    (args.output_dir/"summary.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,3,figsize=(17,12),layout="constrained")
    fig.suptitle("Thumb participation vs. legacy intent fusion\nSame hand recordings, calibration and filter; no hardware commands sent")
    for col,(title,rows) in enumerate(collections):
        t=np.array([r["timestamp"] for r in rows]);t-=t[0]
        axes[0,col].set_title(title)
        for key,label in (("power_grasp","Power grasp"),("opposition_cross","Finger opposition"),("thumb_participation","Thumb participation")):
            axes[0,col].plot(t,[r["new_intents"][key] for r in rows],label=label)
        axes[0,col].plot(t,[r["legacy_intents"]["tripod_precision"] for r in rows],ls=":",color="gray",label="Removed tripod (reference)")
        axes[0,col].set_ylabel("Raw intention");axes[0,col].legend(fontsize=8)
        for row,key,label in ((1,"vertical","h: vertical fold"),(2,"lateral","v: lateral fold"),(3,"thumb_rotation_command","r: solver thumb")):
            axes[row,col].plot(t,[r["legacy_filtered"][key] for r in rows],color="gray",ls="--",label="Legacy")
            axes[row,col].plot(t,[r["new_filtered"][key] for r in rows],label="Thumb participation")
            axes[row,col].set_ylabel(label);axes[row,col].legend(fontsize=8)
        for ax in axes[:,col]:ax.grid(alpha=.2);ax.set_ylim(-.03,1.05)
        axes[-1,col].set_xlabel("Time (s)")
    fig.savefig(args.output_dir/"comparison.png",dpi=150);plt.close(fig)


if __name__=="__main__":main()
