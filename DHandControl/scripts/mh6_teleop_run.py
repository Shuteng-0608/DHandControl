#!/usr/bin/env python3
"""
Plain Vision Pro to MH6 mapping runner.

Flow:
VisionProHandStream -> open-hand calibration -> MH6HandMapper -> printed intent.

Hardware output is disabled by default and requires --enable-hardware.
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
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--port", help="Modbus serial port, required with --enable-hardware")
    parser.add_argument("--baudrate", type=int, default=115200)
    return parser.parse_args(argv)


def send_to_hardware_placeholder(result: Dict[str, Dict[str, float]]) -> None:
    """Intentionally no-op when hardware output is disabled."""
    _ = result


class HardwareSender:
    """Persistent normalized-command output to the MH6 hardware driver."""

    def __init__(self, port: str, baudrate: int) -> None:
        self.port = port
        self.baudrate = baudrate
        self.hand = None

    def start(self) -> None:
        try:
            from modbus_dev import DexHandControl
        except ImportError as exc:
            raise RuntimeError(
                "DexHandControl could not be imported. Install the Modbus dependencies "
                "and ensure modbus_dev.py is available."
            ) from exc

        self.hand = DexHandControl(port=self.port, baudrate=self.baudrate)
        if not self.hand.start_persistent_connection():
            self.hand = None
            raise RuntimeError("failed to start persistent Modbus connection")

    def stop(self) -> None:
        if self.hand is not None:
            self.hand.stop_persistent_connection()
            self.hand = None

    def send(self, result: Dict[str, Dict[str, float]]) -> bool:
        if self.hand is None:
            return False
        return self.hand.move_hand_normalized(
            finger_values=result["low_dim"],
            palm_values=result["palm"],
            wait_status=False,
        )


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
    if args.enable_hardware and not args.port:
        print("ERROR: --port is required with --enable-hardware")
        return 2

    stream = VisionProHandStream(
        avp_ip=args.avp_ip,
        hand=args.hand,
        origin=args.origin,
    )
    mapper = MH6HandMapper()
    period = 1.0 / args.rate
    next_print = 0.0
    hardware_sender = (
        HardwareSender(port=args.port, baudrate=args.baudrate)
        if args.enable_hardware
        else None
    )

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

        if hardware_sender is not None:
            print("WARNING: HARDWARE OUTPUT ENABLED. The MH6 hand will move.")
            hardware_sender.start()

        print("Entering mapping loop. Press Ctrl-C to stop.")

        while True:
            loop_start = time.monotonic()
            frame = stream.get_latest_frame()
            if frame is not None:
                result = mapper.step(frame.points)
                if loop_start >= next_print:
                    print_mapping_line(result)
                    next_print = loop_start + 0.2
                if hardware_sender is not None:
                    if not hardware_sender.send(result):
                        print("WARNING: hardware command failed")
                else:
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
        if hardware_sender is not None:
            hardware_sender.stop()
        stream.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
