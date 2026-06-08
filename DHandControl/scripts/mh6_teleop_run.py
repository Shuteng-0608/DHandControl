#!/usr/bin/env python3
"""
Plain Vision Pro to MH6 mapping runner.

Flow:
VisionProHandStream -> open-hand calibration -> MH6HandMapper -> printed intent.

This script does not control hardware. The hardware output function is a
placeholder for future DexHandControl.move_hand(...) integration.
"""

from __future__ import annotations

import argparse
import time
from typing import Dict, List, Optional, Sequence

import numpy as np

from mh6_mapping import MH6HandMapper
from visionpro_stream import VisionProHandStream


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vision Pro to MH6 mapping runner")
    parser.add_argument("--avp-ip", required=True, help="Vision Pro IP address or room code")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--origin", choices=("avp", "sim"), default="avp")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--calibrate-seconds", type=float, default=2.0)
    return parser.parse_args(argv)


def send_to_hardware_placeholder(result: Dict[str, Dict[str, float]]) -> None:
    """Future hardware output hook. Intentionally no-op for this safe runner."""
    _ = result


def collect_open_hand_samples(
    stream: VisionProHandStream,
    duration: float,
    rate_hz: float,
) -> List[np.ndarray]:
    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration
    samples: List[np.ndarray] = []

    while time.monotonic() < deadline:
        loop_start = time.monotonic()
        frame = stream.get_latest_frame()
        if frame is not None:
            samples.append(frame.points)

        sleep_time = period - (time.monotonic() - loop_start)
        if sleep_time > 0.0:
            time.sleep(sleep_time)

    return samples


def print_mapping_line(result: Dict[str, Dict[str, float]]) -> None:
    low_dim = result["low_dim"]
    palm = result["palm"]
    intent = result["intent"]
    fingers = (
        f"T={low_dim['u_thumb']:.2f} "
        f"I={low_dim['u_index']:.2f} "
        f"M={low_dim['u_middle']:.2f} "
        f"R={low_dim['u_ring']:.2f} "
        f"L={low_dim['u_little']:.2f}"
    )
    palm_text = (
        f"u_h={low_dim['u_h']:.2f} "
        f"u_v={low_dim['u_v']:.2f} "
        f"thumbSide={palm['thumbSide']:.2f} "
        f"littleSide={palm['littleSide']:.2f}"
    )
    intent_text = (
        f"P_opp={intent['P_opp']:.2f} "
        f"g={intent['g']:.2f} "
        f"t={intent['t']:.2f}"
    )
    print(f"fingers: {fingers} | palm: {palm_text} | intent: {intent_text}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.rate <= 0.0:
        print("ERROR: --rate must be greater than 0")
        return 2
    if args.calibrate_seconds <= 0.0:
        print("ERROR: --calibrate-seconds must be greater than 0")
        return 2

    stream = VisionProHandStream(
        avp_ip=args.avp_ip,
        hand=args.hand,
        origin=args.origin,
    )
    mapper = MH6HandMapper()
    period = 1.0 / args.rate
    next_print = 0.0

    try:
        stream.start()

        print("Please keep your hand open for calibration...")
        samples = collect_open_hand_samples(stream, args.calibrate_seconds, args.rate)
        if not samples:
            print("ERROR: no valid Vision Pro hand samples collected during calibration")
            return 1

        mapper.calibrate_open(samples)
        print(f"Collected {len(samples)} open-hand calibration samples")
        print("calibrated curl_open:", mapper.calibration.curl_open)
        print("calibrated opposition_open_dist:", mapper.calibration.opposition_open_dist)
        print("Entering mapping loop. Press Ctrl-C to stop.")

        while True:
            loop_start = time.monotonic()
            frame = stream.get_latest_frame()
            if frame is not None:
                result = mapper.step(frame.points)
                if loop_start >= next_print:
                    print_mapping_line(result)
                    next_print = loop_start + 0.2
                send_to_hardware_placeholder(result)

            sleep_time = period - (time.monotonic() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping MH6 teleop mapping runner")
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        return 2
    finally:
        stream.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
