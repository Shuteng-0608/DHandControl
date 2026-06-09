#!/usr/bin/env python3
"""Benchmark fast normalized MH6 hand control over persistent Modbus."""

from __future__ import annotations

import argparse
import math
import time
from typing import Optional, Sequence

from modbus_dev import DexHandControl


def clip(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def signal_value(mode: str, elapsed: float, duration: float) -> float:
    phase = elapsed % 1.0
    if mode == "sine":
        return math.sin(2.0 * math.pi * elapsed)
    if mode == "ramp":
        return 2.0 * phase - 1.0
    if mode == "open_close":
        return -1.0 if phase < 0.5 else 1.0
    raise ValueError(f"unsupported mode: {mode}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark fast normalized MH6 control")
    parser.add_argument("--port", required=True, help="Modbus serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--amplitude", type=float, default=0.3)
    parser.add_argument("--center", type=float, default=0.3)
    parser.add_argument("--mode", choices=("ramp", "sine", "open_close"), default="sine")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.rate <= 0.0:
        print("ERROR: --rate must be greater than 0")
        return 2
    if args.duration <= 0.0:
        print("ERROR: --duration must be greater than 0")
        return 2
    if args.amplitude < 0.0:
        print("ERROR: --amplitude must be non-negative")
        return 2

    low = clip(args.center - args.amplitude, 0.0, 1.0)
    high = clip(args.center + args.amplitude, 0.0, 1.0)
    print("WARNING: HARDWARE MOTION TEST ENABLED.")
    print(
        f"port={args.port} rate={args.rate:.1f}Hz duration={args.duration:.1f}s "
        f"mode={args.mode} normalized_range=[{low:.2f}, {high:.2f}]"
    )

    hand = DexHandControl(port=args.port, baudrate=args.baudrate)
    attempted = 0
    successful = 0
    failures = 0
    send_times = []
    period = 1.0 / args.rate
    started_at = time.monotonic()
    next_frame_at = started_at

    try:
        if not hand.start_persistent_connection():
            print("ERROR: failed to start persistent Modbus connection")
            return 1

        while True:
            now = time.monotonic()
            elapsed = now - started_at
            if elapsed >= args.duration:
                break

            u = clip(
                args.center + args.amplitude * signal_value(args.mode, elapsed, args.duration),
                0.0,
                1.0,
            )
            finger_values = {
                "u_thumb": u,
                "u_index": u,
                "u_middle": u,
                "u_ring": u,
                "u_little": u,
            }
            palm_values = {
                "thumbSide": u,
                "littleSide": u,
                "UL": u,
                "UR": u,
                "LL": u,
                "LR": u,
            }

            attempted += 1
            send_started = time.monotonic()
            ok = hand.move_hand_normalized(
                finger_values,
                palm_values,
                wait_status=False,
            )
            send_times.append(time.monotonic() - send_started)
            if ok:
                successful += 1
            else:
                failures += 1

            next_frame_at += period
            sleep_time = next_frame_at - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif -sleep_time > period:
                next_frame_at = time.monotonic()
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping fast control test")
    finally:
        hand.stop_persistent_connection()

    total_time = time.monotonic() - started_at
    average_send = sum(send_times) / len(send_times) if send_times else 0.0
    max_send = max(send_times) if send_times else 0.0
    effective_hz = attempted / total_time if total_time > 0.0 else 0.0
    print(
        f"attempted={attempted} successful={successful} failures={failures} "
        f"avg_send_ms={average_send * 1000.0:.3f} "
        f"max_send_ms={max_send * 1000.0:.3f} effective_hz={effective_hz:.2f}"
    )
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
