#!/usr/bin/env python3
"""Read the ESP32 cached guard state only: no targets, clears or force-zero writes."""
import argparse
import json
import math
from pathlib import Path
import time

STATES=("WAITING","FOLLOWING","HOLDING","OPENING","STALE","FAULT")
REASONS=("NONE","UNCONFIGURED","FEEDBACK_MISSING","COMMAND_EXPIRED","OVER_FORCE",
         "OVER_CURRENT","OVER_TEMPERATURE","DRIVER_FAULT","COMMUNICATION","POSITION_INVALID")


def signed(value):
    return value-65536 if value>=32768 else value


def decode(registers):
    if len(registers)!=86 or registers[:2]!=[0x4647,1] or registers[3]!=0x0204:
        raise ValueError("requires matching V2.0.4 finger-guard ESP32 firmware")
    result={"armed":bool(registers[2]&1),"configured":bool(registers[2]&2),
            "enabled_finger_mask":registers[2]>>8,
            "holding_count":registers[4],"fault_or_stale_mask":registers[5],"fingers":{}}
    for i in range(5):
        r=registers[6+16*i:22+16*i]
        result["fingers"][f"F{i+1}"]={
            "state":STATES[r[0]] if r[0]<len(STATES) else "UNKNOWN",
            "reason":REASONS[r[1]] if r[1]<len(REASONS) else "UNKNOWN",
            "latest_desired_position":r[2],"planned_position":r[3],
            "actual_position":signed(r[4]),"force_g":signed(r[5]),"current_ma":r[6],
            "temperature_c":signed(r[7]),"feedback_age_ms":r[8],"error_flags":r[9],
            "valid_reply_count":(r[10]<<16)|r[11],"last_planned_native_mode":r[12],
            "stroke_upper":r[13],"command_age_ms":r[14],"feedback_fresh":bool(r[15])}
    return result


class FeedbackSummary:
    """Summarize distinct, fresh, fault-free cached replies; never choose loads."""

    def __init__(self):
        self.rows={f"F{i}":{"observations":0,"stale_observations":0,
            "fault_observations":0,"unique_valid_samples":0} for i in range(1,6)}
        self.last_reply={}
        self.force_sum={name:0 for name in self.rows}

    def observe(self,data):
        for name,feedback in data["fingers"].items():
            row=self.rows[name]
            row["observations"]+=1
            if not feedback["feedback_fresh"] or feedback["valid_reply_count"]==0:
                row["stale_observations"]+=1
                continue
            if feedback["error_flags"] or feedback["state"]=="FAULT":
                row["fault_observations"]+=1
                continue
            count=feedback["valid_reply_count"]
            if self.last_reply.get(name)==count:
                continue
            self.last_reply[name]=count
            row["unique_valid_samples"]+=1
            self.force_sum[name]+=feedback["force_g"]
            for field in ("force_g","current_ma","temperature_c","actual_position"):
                value=feedback[field]
                row[field+"_min"]=min(row.get(field+"_min",value),value)
                row[field+"_max"]=max(row.get(field+"_max",value),value)

    def result(self):
        rows={name:dict(row) for name,row in self.rows.items()}
        for name,row in rows.items():
            count=row["unique_valid_samples"]
            row["force_g_mean"]=self.force_sum[name]/count if count else None
        return {"motion_commands_sent":False,"load_thresholds_inferred":False,
            "force_sign_inferred":False,"fingers":rows}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",default="/dev/ttyUSB0")
    parser.add_argument("--baudrate",type=int,default=115200)
    parser.add_argument("--watch",action="store_true",help="Read repeatedly at 5 Hz; Ctrl+C exits")
    parser.add_argument("--duration",type=float,help="Read at 5 Hz for this many seconds, then exit")
    parser.add_argument("--output",type=Path,help="Optional JSONL (new file; never overwrite)")
    parser.add_argument("--summary-output",type=Path,help="Optional force/current/temperature summary JSON (new file)")
    args=parser.parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration<=0):
        parser.error("--duration must be a positive finite number")
    from modbus_dev import DexHandControl
    device=DexHandControl(port=args.port,baudrate=args.baudrate,timeout=.15,enable_motion=False)
    output=None; summary_output=None; summary=FeedbackSummary(); observations=0
    try:
        if args.output:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            output=args.output.open("x",encoding="utf-8")
        if args.summary_output:
            args.summary_output.parent.mkdir(parents=True,exist_ok=True)
            summary_output=args.summary_output.open("x",encoding="utf-8")
        if not device.start_persistent_connection(): raise RuntimeError("serial connection failed")
        deadline=time.monotonic()+args.duration if args.duration is not None else None
        while True:
            with device.transaction_lock:
                response=device.client.read_holding_registers(address=80,count=86,device_id=1)
                device._ensure_ok(response,"read cached ESP32 finger guard")
            data=decode(list(response.registers)); data["host_time_unix"]=time.time()
            summary.observe(data); observations+=1
            line=json.dumps(data,ensure_ascii=False)
            print(line,flush=True)
            if output: output.write(line+"\n"); output.flush()
            if not args.watch and deadline is None: break
            if deadline is not None:
                remaining=deadline-time.monotonic()
                if remaining<=0: break
                time.sleep(min(.2,remaining))
                if time.monotonic()>=deadline: break
            else:
                time.sleep(.2)
        return 0
    except KeyboardInterrupt: return 130
    except (OSError,RuntimeError,ValueError) as exc:
        print(f"ERROR: {exc}"); return 1
    finally:
        device.stop_persistent_connection()
        if output: output.close()
        if summary_output:
            if observations:
                json.dump(summary.result(),summary_output,ensure_ascii=False,indent=2)
                summary_output.write("\n")
            summary_output.close()


if __name__=="__main__": raise SystemExit(main())
