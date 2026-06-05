#!/usr/bin/env python3
"""
Small Vision Pro hand tracking stream wrapper.

This module intentionally has no ROS or hardware-control dependencies. It wraps
the current avp_stream API and returns hand transforms/points only.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np


@dataclass
class VisionProHandFrame:
    points: np.ndarray
    transforms: np.ndarray
    timestamp: float
    hand: str


class VisionProHandStream:
    def __init__(
        self,
        avp_ip: str,
        hand: str = "right",
        origin: str = "avp",
    ) -> None:
        if hand not in ("left", "right"):
            raise ValueError("hand must be 'left' or 'right'")
        if origin not in ("avp", "sim"):
            raise ValueError("origin must be 'avp' or 'sim'")
        self.avp_ip = avp_ip
        self.hand = hand
        self.origin = origin
        self.streamer: Optional[Any] = None

    def start(self) -> None:
        try:
            from avp_stream import VisionProStreamer
        except ImportError as exc:
            raise RuntimeError(
                "avp_stream is required. Install with: "
                "pip install --upgrade 'avp_stream>2.50.0'"
            ) from exc

        try:
            self.streamer = VisionProStreamer(ip=self.avp_ip, origin=self.origin)
        except TypeError:
            print(
                "WARNING: installed avp_stream does not support origin=; "
                "falling back to VisionProStreamer(ip=...)."
            )
            self.streamer = VisionProStreamer(ip=self.avp_ip)

    def stop(self) -> None:
        if self.streamer is None:
            return
        for method_name in ("stop", "close"):
            method = getattr(self.streamer, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception as exc:
                    print(f"WARNING: VisionProHandStream.{method_name}() failed: {exc}")
                break
        self.streamer = None

    def get_latest_raw(self) -> Optional[Any]:
        if self.streamer is None:
            return None
        data = self.streamer.get_latest()
        return data

    def get_latest_transforms(self) -> Optional[np.ndarray]:
        data = self.get_latest_raw()
        if data is None:
            return None

        hand_data = getattr(data, self.hand, None)
        if hand_data is None:
            return None

        transforms = np.asarray(hand_data, dtype=float)
        if transforms.ndim != 3 or transforms.shape[0] < 27 or transforms.shape[1:] != (4, 4):
            return None
        if not np.all(np.isfinite(transforms)):
            return None
        return transforms[:27].copy()

    def get_latest_points(self) -> Optional[np.ndarray]:
        transforms = self.get_latest_transforms()
        if transforms is None:
            return None

        points = transforms[:, :3, 3]
        if points.shape != (27, 3):
            return None
        if not np.all(np.isfinite(points)):
            return None
        if np.allclose(points, 0.0):
            return None
        if not self._passes_hand_scale_check(points):
            return None
        return points.copy()

    def get_latest_frame(self) -> Optional[VisionProHandFrame]:
        transforms = self.get_latest_transforms()
        if transforms is None:
            return None

        points = transforms[:, :3, 3]
        if points.shape != (27, 3):
            return None
        if not np.all(np.isfinite(points)):
            return None
        if np.allclose(points, 0.0):
            return None
        if not self._passes_hand_scale_check(points):
            return None

        return VisionProHandFrame(
            points=points.copy(),
            transforms=transforms,
            timestamp=time.monotonic(),
            hand=self.hand,
        )

    @staticmethod
    def _passes_hand_scale_check(points: np.ndarray) -> bool:
        wrist = points[0]
        fingertip_indices = [4, 9, 14, 19, 24]
        fingertip_distances = np.linalg.norm(points[fingertip_indices] - wrist, axis=1)
        max_distance = float(np.max(fingertip_distances))
        return 0.02 <= max_distance <= 0.50


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vision Pro hand stream smoke test")
    parser.add_argument("--avp-ip", required=True, help="Vision Pro IP address or room code")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--origin", choices=("avp", "sim"), default="avp")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=None)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    stream = VisionProHandStream(
        avp_ip=args.avp_ip,
        hand=args.hand,
        origin=args.origin,
    )
    period = 1.0 / args.rate if args.rate > 0.0 else 0.05
    start_time = time.monotonic()
    frame_count = 0

    try:
        stream.start()
        while True:
            now = time.monotonic()
            if args.duration is not None and (now - start_time) >= args.duration:
                break

            frame = stream.get_latest_frame()
            if frame is not None:
                frame_count += 1
                wrist = frame.points[0]
                thumb_tip = frame.points[4]
                index_tip = frame.points[9]
                print(
                    f"frames={frame_count}",
                    f"hand={frame.hand}",
                    f"wrist={np.array2string(wrist, precision=3)}",
                    f"thumb_tip={np.array2string(thumb_tip, precision=3)}",
                    f"index_tip={np.array2string(index_tip, precision=3)}",
                )

            elapsed = time.monotonic() - now
            sleep_time = period - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping Vision Pro hand stream")
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        return 2
    finally:
        stream.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
