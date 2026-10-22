#!/usr/bin/env python3
"""Compare host streaming timings. No imports of serial or hardware drivers."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics


def stats(values):
    values = sorted(values)
    if not values:
        return None
    def percentile(q):
        index = (len(values)-1)*q
        lower = int(index); upper = min(lower+1,len(values)-1)
        return values[lower]+(index-lower)*(values[upper]-values[lower])
    return {"mean":statistics.mean(values),"median":statistics.median(values),
            "p95":percentile(.95),"max":max(values)}


def summarize(path):
    lines = Path(path).read_text().splitlines()
    rows = []
    for i,line in enumerate(lines):
        if not line.strip(): continue
        try: rows.append(json.loads(line))
        except json.JSONDecodeError:
            if i != len(lines)-1: raise
            # A currently running logger may have a partial final line.
    if not rows:
        raise ValueError(f"empty log: {path}")
    completed = [r for r in rows if (r.get("execution") or {}).get("write_acknowledged")
                 or (r.get("execution") or {}).get("status") == "HOST_WRITTEN_UNCONFIRMED"]
    starts = [r["execution"]["started_monotonic"] for r in completed]
    intervals = [1000*(b-a) for a,b in zip(starts,starts[1:])]
    stage_names = sorted({key for r in completed for key in
                          ((r["execution"].get("driver_result") or {}).get("timings_ms") or {})})
    runtime = rows[0].get("runtime_config") or {}
    transport = runtime.get("control_transport",(rows[-1].get("execution") or {}).get("transport_mode","legacy"))
    return {"path":str(Path(path).resolve()),"transport_mode":transport,"runtime_config":runtime,
            "frames":len(rows),"successful_host_sends":len(completed),
            "device_acknowledged":sum(bool((r.get("execution") or {}).get("write_acknowledged")) for r in rows),
            "host_only_sends":sum((r.get("execution") or {}).get("status")=="HOST_WRITTEN_UNCONFIRMED" for r in rows),
            "host_send_hz":(len(starts)-1)/(starts[-1]-starts[0]) if len(starts)>1 and starts[-1]>starts[0] else None,
            "interval_ms":stats(intervals),
            "send_duration_ms":stats([r["execution"]["duration_ms"] for r in completed]),
            "stage_ms":{key:stats([r["execution"]["driver_result"]["timings_ms"][key] for r in completed
                                    if key in ((r["execution"].get("driver_result") or {}).get("timings_ms") or {})])
                        for key in stage_names},
            "execution_statuses":dict(Counter((r.get("execution") or {}).get("status","UNKNOWN") for r in rows)),
            "position_limit_counts":dict(Counter(name for r in rows for name,ch in r["control"]["channels"].items()
                                                 if ch.get("position_limited"))),
            "palm_statuses":dict(Counter(r["control"]["palm"]["reason"] or "ACCEPTED" for r in rows)),
            "measurement_note":"Host command timing only; one-way sends have no measured device delivery or motor arrival time."}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs",type=Path,nargs="+")
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    data=[summarize(path) for path in args.logs]
    rendered=json.dumps(data,ensure_ascii=False,indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(rendered+"\n")
    print(rendered)


if __name__ == "__main__": main()
