#!/usr/bin/env python3
"""Read actuator feedback only. This entry point has no motion sender or mapper."""
import argparse
import json
import math
from pathlib import Path
import time

from mh6_actuator_io import FeedbackPoller
from modbus_dev import DexHandControl


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",required=True)
    parser.add_argument("--baudrate",type=int,default=115200)
    parser.add_argument("--feedback-rate",type=float,default=10.,help="Total queries/s, shared by nine channels")
    parser.add_argument("--serial-timeout",type=float,default=.15)
    parser.add_argument("--duration",type=float,help="Seconds to read; default: until Ctrl+C")
    parser.add_argument("--output",required=True,help="New feedback JSONL file; existing files are protected")
    args = parser.parse_args(argv)
    numbers = [args.feedback_rate,args.serial_timeout]+([] if args.duration is None else [args.duration])
    if any(not math.isfinite(x) or x <= 0 for x in numbers):
        parser.error("rates, timeout and duration must be positive and finite")
    path = Path(args.output).expanduser()
    path.parent.mkdir(parents=True,exist_ok=True)
    driver = None
    try:
        with path.open("x",encoding="utf-8",buffering=1) as log:
            driver = DexHandControl(port=args.port,baudrate=args.baudrate,timeout=args.serial_timeout)
            if not driver.start_persistent_connection():
                raise RuntimeError("feedback-only Modbus connection failed")
            poller = FeedbackPoller(driver,args.feedback_rate)
            started = time.monotonic()
            print("只读反馈已开启，不发送运动指令。Ctrl+C 结束。",flush=True)
            while args.duration is None or time.monotonic()-started < args.duration:
                sample = poller.poll()
                if sample is not None:
                    log.write(json.dumps({"format_version":1,"mode":"feedback_only",
                        "elapsed_seconds":time.monotonic()-started,"sample":sample,
                        "feedback":poller.snapshot()},ensure_ascii=False,allow_nan=False)+"\n")
                time.sleep(min(.01,max(0.,poller.next_query-time.monotonic())))
    except KeyboardInterrupt:
        pass
    except (RuntimeError,OSError,ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    finally:
        if driver is not None:
            driver.stop_persistent_connection()
    print(f"反馈已保存：{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
