#!/usr/bin/env python3
"""
Manual MH6 gesture demo runner.

This script intentionally runs hardware gestures only when explicitly executed
with a serial port and gesture name.
"""

from __future__ import annotations

import argparse
import time
from typing import Optional, Sequence


try:
    from modbus_dev import DexHandControl
except ImportError as exc:
    DexHandControl = None
    MODBUS_IMPORT_ERROR = exc
else:
    MODBUS_IMPORT_ERROR = None


def palm_free(hand) -> None:
    hand.move_palms([1, 2, 3], [753, 500, 500], [1000, 1000, 1000])


def finger_free(hand) -> None:
    hand.move_fingers([1, 2, 3, 4, 5], [20, 20, 20, 20, 20])


def free(hand) -> None:
    palm_free(hand)
    finger_free(hand)


def thumb_index(hand) -> None:
    hand.move_fingers([1, 2, 3, 4, 5], [640, 1200, 20, 20, 20])


def thumb_mid(hand) -> None:
    hand.move_fingers([1, 2, 3, 4, 5], [1200, 20, 1750, 20, 20])
    hand.move_palms([1, 2, 3], [700, 600, 520], [1000, 1000, 1000])


def rock(hand) -> None:
    hand.move_fingers([1, 2, 3, 4, 5], [1200, 20, 1750, 1600, 20])
    hand.move_palms([1, 2, 3], [700, 600, 520], [1000, 1000, 1000])


def boxing(hand) -> None:
    hand.move_fingers([1, 2, 3, 4, 5], [20, 1950, 1950, 1950, 1950])
    time.sleep(0.5)
    hand.move_fingers([1], [600])


def one(hand) -> None:
    hand.move_fingers([1, 2, 3, 4, 5], [1000, 20, 1950, 1950, 1950])


def two(hand) -> None:
    hand.move_fingers([1, 2, 3, 4, 5], [1200, 20, 20, 1950, 1950])


GESTURES = {
    "free": free,
    "palm_free": palm_free,
    "finger_free": finger_free,
    "thumb_index": thumb_index,
    "thumb_mid": thumb_mid,
    "rock": rock,
    "boxing": boxing,
    "one": one,
    "two": two,
}

SEQUENCE = (
    "free",
    "one",
    "two",
    "rock",
    "boxing",
    "thumb_index",
    "thumb_mid",
    "free",
)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run manual MH6 gesture demos")
    parser.add_argument("--port", required=True, help="Modbus serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--gesture", choices=tuple(GESTURES.keys()) + ("sequence",), required=True)
    parser.add_argument("--delay", type=float, default=1.0)
    return parser.parse_args(argv)


def run_gesture(hand, gesture: str) -> None:
    print(f"Running gesture: {gesture}")
    GESTURES[gesture](hand)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    if DexHandControl is None:
        print(f"Cannot import DexHandControl from modbus_dev: {MODBUS_IMPORT_ERROR}")
        return 2

    try:
        hand = DexHandControl(port=args.port, baudrate=args.baudrate)

        if args.gesture == "sequence":
            for gesture in SEQUENCE:
                run_gesture(hand, gesture)
                time.sleep(args.delay)
        else:
            run_gesture(hand, args.gesture)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping gesture demo")
    except Exception as exc:
        print(f"Gesture demo failed: {exc}")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
