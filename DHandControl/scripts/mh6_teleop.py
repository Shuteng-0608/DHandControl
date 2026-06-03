#!/usr/bin/env python3
"""
Plain Python MH6 teleoperation framework.

This module is intentionally ROS-independent and VisionProTeleop-independent.
It implements the neutral data structures, numpy-based hand geometry math,
low-dimensional hand-intention mapping, actuator conversion, and a conservative
runtime shell that can stream through DexHandControl.move_hand(...,
wait_status=False).
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

try:
    from modbus_dev import DexHandControl
except ImportError as exc:
    DexHandControl = None
    DEX_HAND_IMPORT_ERROR = exc
else:
    DEX_HAND_IMPORT_ERROR = None


FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")
LONG_FINGER_NAMES = ("index", "middle", "ring", "little")
PALM_BLOCK_NAMES = ("UL", "UR", "LL", "LR")

DEFAULT_FINGER_KEYPOINTS = {
    "thumb": ("thumb_base", "thumb_mcp", "thumb_ip", "thumb_tip"),
    "index": ("index_base", "index_mcp", "index_pip", "index_dip", "index_tip"),
    "middle": ("middle_base", "middle_mcp", "middle_pip", "middle_dip", "middle_tip"),
    "ring": ("ring_base", "ring_mcp", "ring_pip", "ring_dip", "ring_tip"),
    "little": ("little_base", "little_mcp", "little_pip", "little_dip", "little_tip"),
}

KEYPOINT_INDEX_NAMES = {
    0: "wrist",
    1: "thumb_base",
    2: "thumb_mcp",
    3: "thumb_ip",
    4: "thumb_tip",
    5: "index_base",
    6: "index_mcp",
    7: "index_pip",
    8: "index_dip",
    9: "index_tip",
    10: "middle_base",
    11: "middle_mcp",
    12: "middle_pip",
    13: "middle_dip",
    14: "middle_tip",
    15: "ring_base",
    16: "ring_mcp",
    17: "ring_pip",
    18: "ring_dip",
    19: "ring_tip",
    20: "little_base",
    21: "little_mcp",
    22: "little_pip",
    23: "little_dip",
    24: "little_tip",
}


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    @classmethod
    def from_array(cls, value: Sequence[float]) -> "Vec3":
        arr = np.asarray(value, dtype=float)
        if arr.shape != (3,):
            raise ValueError(f"Vec3 requires shape (3,), got {arr.shape}")
        return cls(float(arr[0]), float(arr[1]), float(arr[2]))

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)


VectorInput = Union[Vec3, Sequence[float], np.ndarray]


@dataclass
class HandSkeleton:
    keypoints: Dict[str, Vec3] = field(default_factory=dict)
    timestamp: float = 0.0
    valid: bool = True
    finger_keypoints: Dict[str, Tuple[str, ...]] = field(
        default_factory=lambda: dict(DEFAULT_FINGER_KEYPOINTS)
    )

    def point(self, name: str) -> Vec3:
        try:
            return self.keypoints[name]
        except KeyError as exc:
            raise KeyError(f"Missing hand keypoint: {name}") from exc

    def finger_points(self, finger_name: str) -> List[Vec3]:
        if finger_name not in self.finger_keypoints:
            raise KeyError(f"Unknown finger name: {finger_name}")
        return [self.point(name) for name in self.finger_keypoints[finger_name]]

    @classmethod
    def from_array(
        cls,
        points: Union[np.ndarray, Sequence[Sequence[float]]],
        timestamp: Optional[float] = None,
        valid: bool = True,
    ) -> "HandSkeleton":
        arr = np.asarray(points, dtype=float)
        if arr.shape != (27, 3):
            raise ValueError(f"HandSkeleton.from_array requires shape (27, 3), got {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError("HandSkeleton.from_array received non-finite keypoint values")

        keypoints = {
            name: Vec3.from_array(arr[index])
            for index, name in KEYPOINT_INDEX_NAMES.items()
        }
        return cls(
            keypoints=keypoints,
            timestamp=time.monotonic() if timestamp is None else timestamp,
            valid=valid,
        )


@dataclass
class TeleopCalibration:
    curl_open: Dict[str, float] = field(default_factory=lambda: {
        "thumb": 0.10,
        "index": 0.20,
        "middle": 0.20,
        "ring": 0.20,
        "little": 0.20,
    })
    curl_closed: Dict[str, float] = field(default_factory=lambda: {
        "thumb": 1.40,
        "index": 2.50,
        "middle": 2.60,
        "ring": 2.55,
        "little": 2.40,
    })
    opposition_open_dist: Dict[str, float] = field(default_factory=lambda: {
        "index": 0.095,
        "middle": 0.105,
        "ring": 0.120,
        "little": 0.135,
    })
    opposition_closed_dist: Dict[str, float] = field(default_factory=lambda: {
        "index": 0.018,
        "middle": 0.020,
        "ring": 0.025,
        "little": 0.030,
    })
    opposition_threshold: float = 0.35
    thumb_curl_gain: float = 0.70
    grasp_weights: Dict[str, float] = field(default_factory=lambda: {
        "index": 0.20,
        "middle": 0.30,
        "ring": 0.30,
        "little": 0.20,
    })
    opposition_horizontal_weights: Dict[str, float] = field(default_factory=lambda: {
        "index": 0.20,
        "middle": 0.35,
        "ring": 0.70,
        "little": 1.00,
    })
    opposition_vertical_weights: Dict[str, float] = field(default_factory=lambda: {
        "index": -0.25,
        "middle": -0.45,
        "ring": 0.75,
        "little": 1.00,
    })
    horizontal_from_grasp: float = 0.55
    horizontal_from_tripod: float = 0.20
    horizontal_from_opposition: float = 0.35
    vertical_from_finger_bias: float = 0.30
    vertical_from_opposition: float = 0.70
    finger_ids: List[int] = field(default_factory=lambda: [1, 2, 3, 4, 5])
    palm_ids: List[int] = field(default_factory=lambda: [1, 2, 3])
    finger_open_positions: Dict[str, int] = field(default_factory=lambda: {
        "thumb": 20,
        "index": 20,
        "middle": 20,
        "ring": 20,
        "little": 20,
    })
    finger_closed_positions: Dict[str, int] = field(default_factory=lambda: {
        "thumb": 1200,
        "index": 1950,
        "middle": 1950,
        "ring": 1950,
        "little": 1950,
    })
    palm_open_positions: Dict[str, int] = field(default_factory=lambda: {
        "palm_1": 500,
        "palm_2": 500,
        "palm_3": 500,
    })
    palm_closed_positions: Dict[str, int] = field(default_factory=lambda: {
        "palm_1": 650,
        "palm_2": 600,
        "palm_3": 560,
    })
    palm_block_weights: Dict[str, Dict[str, float]] = field(default_factory=lambda: {
        "palm_1": {"UL": 1.0, "LL": 1.0},
        "palm_2": {"UR": 0.5, "LR": 0.5},
        "palm_3": {"UR": 1.0, "LR": 1.0},
    })
    palm_times: List[int] = field(default_factory=lambda: [80, 80, 80])
    max_finger_delta_per_sec: float = 800.0
    max_palm_delta_per_sec: float = 400.0


@dataclass
class LowDimHandCommand:
    u_thumb: float = 0.0
    u_index: float = 0.0
    u_middle: float = 0.0
    u_ring: float = 0.0
    u_little: float = 0.0
    u_h: float = 0.0
    u_v: float = 0.0


@dataclass
class ActuatorCommand:
    finger_ids: List[int] = field(default_factory=list)
    finger_positions: List[int] = field(default_factory=list)
    palm_ids: List[int] = field(default_factory=list)
    palm_positions: List[int] = field(default_factory=list)
    palm_times: List[int] = field(default_factory=list)


@dataclass
class TeleopStats:
    started_at: float = 0.0
    frames_received: int = 0
    frames_sent: int = 0
    frames_dropped: int = 0
    loop_iterations: int = 0
    send_failures: int = 0
    last_send_time: float = 0.0
    last_loop_hz: float = 0.0
    dry_run: bool = True


@dataclass
class MappingDebug:
    curl_raw: Dict[str, float] = field(default_factory=dict)
    curl_norm: Dict[str, float] = field(default_factory=dict)
    opposition_raw: Dict[str, float] = field(default_factory=dict)
    opposition: Dict[str, float] = field(default_factory=dict)
    p_opp: float = 0.0
    g: float = 0.0
    t: float = 0.0
    o_h: float = 0.0
    b_f: float = 0.0
    o_v: float = 0.0
    palm_blocks: Dict[str, float] = field(default_factory=dict)


def clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def as_array(value: VectorInput) -> np.ndarray:
    if isinstance(value, Vec3):
        return value.as_array()
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"Expected vector shape (3,), got {arr.shape}")
    return arr


def subtract(a: VectorInput, b: VectorInput) -> np.ndarray:
    return as_array(a) - as_array(b)


def dot(a: VectorInput, b: VectorInput) -> float:
    return float(np.dot(as_array(a), as_array(b)))


def norm(v: VectorInput) -> float:
    return float(np.linalg.norm(as_array(v)))


def distance(a: VectorInput, b: VectorInput) -> float:
    return norm(subtract(a, b))


def angle_between(a: VectorInput, b: VectorInput) -> float:
    denom = norm(a) * norm(b)
    if denom <= 1e-12:
        return 0.0
    cos_value = clip(dot(a, b) / denom, -1.0, 1.0)
    return float(math.acos(cos_value))


def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    if abs(in_max - in_min) <= 1e-12:
        return out_min
    ratio = (value - in_min) / (in_max - in_min)
    return out_min + ratio * (out_max - out_min)


def clamp_to_range(value: float, endpoint_a: float, endpoint_b: float) -> int:
    return int(round(clip(value, min(endpoint_a, endpoint_b), max(endpoint_a, endpoint_b))))


def normalized_threshold(value: float, threshold: float) -> float:
    if threshold >= 1.0:
        return 0.0
    return clip((value - threshold) / (1.0 - threshold), 0.0, 1.0)


def rate_limit_value(current: int, target: int, max_delta_per_sec: float, dt: float) -> int:
    if dt <= 0.0 or max_delta_per_sec <= 0.0:
        return target
    max_delta = max_delta_per_sec * dt
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return int(round(current + math.copysign(max_delta, delta)))


def compute_finger_curl(points: Sequence[Vec3]) -> float:
    if len(points) < 3:
        return 0.0

    vectors = [subtract(points[i + 1], points[i]) for i in range(len(points) - 1)]
    return sum(angle_between(vectors[i], vectors[i + 1]) for i in range(len(vectors) - 1))


def normalize_bending(curl: float, finger: str, calibration: TeleopCalibration) -> float:
    open_value = calibration.curl_open[finger]
    closed_value = calibration.curl_closed[finger]
    denom = closed_value - open_value
    if abs(denom) <= 1e-12:
        return 0.0
    return clip((curl - open_value) / denom, 0.0, 1.0)


def opposition_strength(raw_distance: float, finger: str, calibration: TeleopCalibration) -> float:
    open_dist = calibration.opposition_open_dist[finger]
    closed_dist = calibration.opposition_closed_dist[finger]
    denom = open_dist - closed_dist
    if abs(denom) <= 1e-12:
        return 0.0
    return clip((open_dist - raw_distance) / denom, 0.0, 1.0)


def map_skeleton_to_low_dim(
    skeleton: HandSkeleton,
    calibration: TeleopCalibration,
) -> Tuple[LowDimHandCommand, MappingDebug]:
    if not skeleton.valid:
        raise ValueError("HandSkeleton is marked invalid")

    curl_raw: Dict[str, float] = {}
    curl_norm: Dict[str, float] = {}
    for finger in FINGER_NAMES:
        c_i = compute_finger_curl(skeleton.finger_points(finger))
        curl_raw[finger] = c_i
        curl_norm[finger] = normalize_bending(c_i, finger, calibration)

    thumb_tip = skeleton.point("thumb_tip")
    opposition_raw: Dict[str, float] = {}
    opposition: Dict[str, float] = {}
    for finger in LONG_FINGER_NAMES:
        d = distance(thumb_tip, skeleton.point(f"{finger}_tip"))
        p_raw = opposition_strength(d, finger, calibration)
        opposition_raw[finger] = p_raw
        opposition[finger] = normalized_threshold(p_raw, calibration.opposition_threshold)

    p_i = opposition["index"]
    p_m = opposition["middle"]
    p_r = opposition["ring"]
    p_l = opposition["little"]
    p_opp = max(p_i, p_m, p_r, p_l)

    u_thumb = max(calibration.thumb_curl_gain * curl_norm["thumb"], p_opp)
    u_index = max(curl_norm["index"], p_i)
    u_middle = max(curl_norm["middle"], p_m)
    u_ring = max(curl_norm["ring"], p_r)
    u_little = max(curl_norm["little"], p_l)

    u_thumb = clip(u_thumb, 0.0, 1.0)
    u_index = clip(u_index, 0.0, 1.0)
    u_middle = clip(u_middle, 0.0, 1.0)
    u_ring = clip(u_ring, 0.0, 1.0)
    u_little = clip(u_little, 0.0, 1.0)

    g = (
        calibration.grasp_weights["index"] * u_index
        + calibration.grasp_weights["middle"] * u_middle
        + calibration.grasp_weights["ring"] * u_ring
        + calibration.grasp_weights["little"] * u_little
    )
    g = clip(g, 0.0, 1.0)
    t = min(u_thumb, u_index, u_middle)
    o_h = clip(
        calibration.opposition_horizontal_weights["index"] * p_i
        + calibration.opposition_horizontal_weights["middle"] * p_m
        + calibration.opposition_horizontal_weights["ring"] * p_r
        + calibration.opposition_horizontal_weights["little"] * p_l,
        0.0,
        1.0,
    )
    u_h = clip(
        calibration.horizontal_from_grasp * g
        + calibration.horizontal_from_tripod * t
        + calibration.horizontal_from_opposition * o_h,
        0.0,
        1.0,
    )
    b_f = 0.5 * (u_ring + u_little) - 0.5 * (u_index + u_middle)
    o_v = clip(
        calibration.opposition_vertical_weights["index"] * p_i
        + calibration.opposition_vertical_weights["middle"] * p_m
        + calibration.opposition_vertical_weights["ring"] * p_r
        + calibration.opposition_vertical_weights["little"] * p_l,
        -1.0,
        1.0,
    )
    u_v = clip(
        calibration.vertical_from_finger_bias * b_f
        + calibration.vertical_from_opposition * o_v,
        -1.0,
        1.0,
    )

    command = LowDimHandCommand(
        u_thumb=u_thumb,
        u_index=u_index,
        u_middle=u_middle,
        u_ring=u_ring,
        u_little=u_little,
        u_h=u_h,
        u_v=u_v,
    )
    palm_blocks = expand_palm_blocks(command)
    debug = MappingDebug(
        curl_raw=curl_raw,
        curl_norm=curl_norm,
        opposition_raw=opposition_raw,
        opposition=opposition,
        p_opp=p_opp,
        g=g,
        t=t,
        o_h=o_h,
        b_f=b_f,
        o_v=o_v,
        palm_blocks=palm_blocks,
    )
    return command, debug


def expand_palm_blocks(command: LowDimHandCommand) -> Dict[str, float]:
    thumb_side = clip(command.u_h - command.u_v, 0.0, 1.0)
    little_side = clip(command.u_h + command.u_v, 0.0, 1.0)
    return {
        "UL": thumb_side,
        "LL": thumb_side,
        "UR": little_side,
        "LR": little_side,
    }


def low_dim_to_actuator_command(
    command: LowDimHandCommand,
    calibration: TeleopCalibration,
) -> ActuatorCommand:
    normalized = {
        "thumb": clip(command.u_thumb, 0.0, 1.0),
        "index": clip(command.u_index, 0.0, 1.0),
        "middle": clip(command.u_middle, 0.0, 1.0),
        "ring": clip(command.u_ring, 0.0, 1.0),
        "little": clip(command.u_little, 0.0, 1.0),
    }

    finger_positions: List[int] = []
    for finger in FINGER_NAMES:
        open_pos = calibration.finger_open_positions[finger]
        closed_pos = calibration.finger_closed_positions[finger]
        mapped = map_range(normalized[finger], 0.0, 1.0, open_pos, closed_pos)
        finger_positions.append(clamp_to_range(mapped, open_pos, closed_pos))

    palm_blocks = expand_palm_blocks(command)
    palm_names = list(calibration.palm_open_positions.keys())[:len(calibration.palm_ids)]
    palm_positions: List[int] = []
    for palm_name in palm_names:
        weights = calibration.palm_block_weights.get(palm_name, {})
        if weights:
            total_weight = sum(abs(weight) for weight in weights.values())
            if total_weight <= 1e-12:
                u_palm = 0.0
            else:
                u_palm = sum(palm_blocks[block] * weight for block, weight in weights.items()) / total_weight
        else:
            u_palm = 0.0
        u_palm = clip(u_palm, 0.0, 1.0)
        open_pos = calibration.palm_open_positions[palm_name]
        closed_pos = calibration.palm_closed_positions[palm_name]
        mapped = map_range(u_palm, 0.0, 1.0, open_pos, closed_pos)
        palm_positions.append(clamp_to_range(mapped, open_pos, closed_pos))

    palm_times = [
        int(clip(value, 0, 65535))
        for value in calibration.palm_times[:len(palm_positions)]
    ]
    while len(palm_times) < len(palm_positions):
        palm_times.append(80)

    return ActuatorCommand(
        finger_ids=list(calibration.finger_ids[:len(finger_positions)]),
        finger_positions=finger_positions,
        palm_ids=list(calibration.palm_ids[:len(palm_positions)]),
        palm_positions=palm_positions,
        palm_times=palm_times,
    )


class MH6TeleopController:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        rate_hz: float = 20.0,
        safe_mode: bool = True,
        dry_run: bool = True,
        calibration: Optional[TeleopCalibration] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.rate_hz = rate_hz
        self.safe_mode = safe_mode
        self.dry_run = dry_run
        self.calibration = calibration if calibration is not None else TeleopCalibration()
        self.hand: Optional[DexHandControl] = None
        self.running = False
        self._target_lock = threading.Lock()
        self._latest_target: Optional[ActuatorCommand] = None
        self._last_command: Optional[ActuatorCommand] = None
        self._last_command_time = 0.0
        self.stats = TeleopStats(dry_run=dry_run)
        self.last_mapping_debug: Optional[MappingDebug] = None

    def start(self) -> None:
        if self.running:
            return

        if not self.dry_run:
            if not self.port:
                raise ValueError("--port is required when hardware output is enabled")
            if DexHandControl is None:
                raise RuntimeError(
                    "Unable to import DexHandControl. Install pymodbus and run from "
                    "the repository root or from DHandControl/scripts."
                ) from DEX_HAND_IMPORT_ERROR
            print("WARNING: hardware motion is enabled. MH6 hand may move.")
            self.hand = DexHandControl(port=self.port, baudrate=self.baudrate)
            if not self.hand.start_persistent_connection():
                raise RuntimeError("Failed to open persistent Modbus connection")

        self.running = True
        self.stats.started_at = time.monotonic()

    def stop(self) -> None:
        self.running = False
        if self.hand is not None:
            self.hand.stop_persistent_connection()
            self.hand = None

    def update_skeleton(self, skeleton: HandSkeleton) -> None:
        low_dim, debug = map_skeleton_to_low_dim(skeleton, self.calibration)
        self.last_mapping_debug = debug
        self.update_low_dim_command(low_dim)

    def update_low_dim_command(self, command: LowDimHandCommand) -> None:
        self.update_actuator_command(low_dim_to_actuator_command(command, self.calibration))

    def update_actuator_command(self, command: ActuatorCommand) -> None:
        with self._target_lock:
            if self._latest_target is not None:
                self.stats.frames_dropped += 1
            self._latest_target = command
            self.stats.frames_received += 1

    def run_forever(self, duration: Optional[float] = None) -> None:
        self.start()
        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.05
        deadline = None if duration is None else time.monotonic() + duration

        try:
            while self.running:
                loop_start = time.monotonic()
                if deadline is not None and loop_start >= deadline:
                    break

                target = self._take_latest_target()
                if target is not None:
                    command = self._prepare_command(target, loop_start)
                    self._send_or_print(command)

                self.stats.loop_iterations += 1
                elapsed = time.monotonic() - loop_start
                self.stats.last_loop_hz = 1.0 / elapsed if elapsed > 0.0 else 0.0
                sleep_time = period - elapsed
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("KeyboardInterrupt: stopping MH6 teleop controller")
        finally:
            self.stop()

    def get_stats(self) -> TeleopStats:
        return TeleopStats(**self.stats.__dict__)

    def _take_latest_target(self) -> Optional[ActuatorCommand]:
        with self._target_lock:
            target = self._latest_target
            self._latest_target = None
            return target

    def _prepare_command(self, target: ActuatorCommand, now: float) -> ActuatorCommand:
        clamped = self._clamp_command(target)
        if not self.safe_mode:
            self._last_command = clamped
            self._last_command_time = now
            return clamped

        dt = now - self._last_command_time if self._last_command_time else 0.0
        if self._last_command is None:
            limited = clamped
        else:
            limited = self._rate_limit_command(self._last_command, clamped, dt)

        self._last_command = limited
        self._last_command_time = now
        return limited

    def _clamp_command(self, command: ActuatorCommand) -> ActuatorCommand:
        finger_positions: List[int] = []
        for idx, position in enumerate(command.finger_positions[:len(self.calibration.finger_ids)]):
            finger = FINGER_NAMES[idx]
            open_pos = self.calibration.finger_open_positions[finger]
            closed_pos = self.calibration.finger_closed_positions[finger]
            finger_positions.append(clamp_to_range(position, open_pos, closed_pos))

        palm_positions: List[int] = []
        palm_names = list(self.calibration.palm_open_positions.keys())
        for idx, position in enumerate(command.palm_positions[:len(self.calibration.palm_ids)]):
            palm_name = palm_names[idx]
            open_pos = self.calibration.palm_open_positions[palm_name]
            closed_pos = self.calibration.palm_closed_positions[palm_name]
            palm_positions.append(clamp_to_range(position, open_pos, closed_pos))

        palm_times = [
            int(clip(value, 0, 65535))
            for value in command.palm_times[:len(palm_positions)]
        ]
        while len(palm_times) < len(palm_positions):
            palm_times.append(80)

        return ActuatorCommand(
            finger_ids=list(command.finger_ids[:len(finger_positions)]),
            finger_positions=finger_positions,
            palm_ids=list(command.palm_ids[:len(palm_positions)]),
            palm_positions=palm_positions,
            palm_times=palm_times,
        )

    def _rate_limit_command(self, previous: ActuatorCommand, target: ActuatorCommand, dt: float) -> ActuatorCommand:
        finger_positions = [
            rate_limit_value(prev, cur, self.calibration.max_finger_delta_per_sec, dt)
            for prev, cur in zip(previous.finger_positions, target.finger_positions)
        ]
        palm_positions = [
            rate_limit_value(prev, cur, self.calibration.max_palm_delta_per_sec, dt)
            for prev, cur in zip(previous.palm_positions, target.palm_positions)
        ]

        return ActuatorCommand(
            finger_ids=list(target.finger_ids),
            finger_positions=finger_positions,
            palm_ids=list(target.palm_ids),
            palm_positions=palm_positions,
            palm_times=list(target.palm_times),
        )

    def _send_or_print(self, command: ActuatorCommand) -> None:
        if self.dry_run:
            print(
                "dry-run command:",
                f"finger_ids={command.finger_ids}",
                f"finger_positions={command.finger_positions}",
                f"palm_ids={command.palm_ids}",
                f"palm_positions={command.palm_positions}",
                f"palm_times={command.palm_times}",
            )
            self.stats.frames_sent += 1
            self.stats.last_send_time = time.monotonic()
            return

        if self.hand is None:
            self.stats.send_failures += 1
            raise RuntimeError("Hardware output requested before Modbus connection was opened")

        ok = self.hand.move_hand(
            finger_ids=command.finger_ids,
            finger_positions=command.finger_positions,
            palm_ids=command.palm_ids,
            palm_positions=command.palm_positions,
            palm_times=command.palm_times,
            wait_status=False,
        )
        if ok:
            self.stats.frames_sent += 1
            self.stats.last_send_time = time.monotonic()
        else:
            self.stats.send_failures += 1


def make_demo_skeleton(mode: str = "open") -> HandSkeleton:
    points = np.zeros((27, 3), dtype=float)
    name_to_index = {name: index for index, name in KEYPOINT_INDEX_NAMES.items()}
    bases = {
        "thumb": np.array([-0.035, 0.015, 0.000], dtype=float),
        "index": np.array([-0.020, 0.055, 0.000], dtype=float),
        "middle": np.array([0.000, 0.060, 0.000], dtype=float),
        "ring": np.array([0.020, 0.055, 0.000], dtype=float),
        "little": np.array([0.040, 0.045, 0.000], dtype=float),
    }
    segment_lengths = {
        "thumb": (0.025, 0.022, 0.020),
        "index": (0.030, 0.025, 0.020, 0.015),
        "middle": (0.035, 0.030, 0.022, 0.015),
        "ring": (0.032, 0.027, 0.020, 0.014),
        "little": (0.025, 0.020, 0.016, 0.012),
    }
    bends_by_mode = {
        "open": {"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0, "little": 0.0},
        "fist": {"thumb": 0.7, "index": 0.8, "middle": 0.9, "ring": 0.9, "little": 0.8},
        "thumb-index": {"thumb": 0.4, "index": 0.3, "middle": 0.0, "ring": 0.0, "little": 0.0},
        "thumb-little": {"thumb": 0.5, "index": 0.0, "middle": 0.1, "ring": 0.4, "little": 0.5},
    }
    if mode not in bends_by_mode:
        raise ValueError(f"Unknown demo skeleton mode: {mode}")

    for finger, bend in bends_by_mode[mode].items():
        names = DEFAULT_FINGER_KEYPOINTS[finger]
        p = bases[finger]
        points[name_to_index[names[0]]] = p
        for idx, name in enumerate(names[1:]):
            length = segment_lengths[finger][idx]
            y = 1.0 - bend
            z = -bend
            direction = np.array([0.0, y, z], dtype=float)
            direction_norm = np.linalg.norm(direction)
            if direction_norm <= 1e-12:
                direction = np.array([0.0, 1.0, 0.0], dtype=float)
            else:
                direction = direction / direction_norm
            p = p + direction * length
            points[name_to_index[name]] = p

    if mode == "thumb-index":
        index_tip = points[name_to_index["index_tip"]]
        points[name_to_index["thumb_tip"]] = index_tip + np.array([-0.005, 0.0, 0.0], dtype=float)
    elif mode == "thumb-little":
        little_tip = points[name_to_index["little_tip"]]
        points[name_to_index["thumb_tip"]] = little_tip + np.array([-0.005, 0.0, 0.0], dtype=float)

    return HandSkeleton.from_array(points, timestamp=time.monotonic(), valid=True)


def run_demo(controller: MH6TeleopController, duration: Optional[float]) -> None:
    controller.start()
    period = 1.0 / controller.rate_hz if controller.rate_hz > 0.0 else 0.05
    start_time = time.monotonic()
    next_print = start_time
    modes = ("open", "fist", "thumb-index", "thumb-little")

    try:
        while controller.running:
            now = time.monotonic()
            if duration is not None and (now - start_time) >= duration:
                break

            mode_idx = int((now - start_time) / 1.5) % len(modes)
            controller.update_skeleton(make_demo_skeleton(modes[mode_idx]))
            target = controller._take_latest_target()
            if target is not None:
                command = controller._prepare_command(target, now)
                controller._send_or_print(command)

            if now >= next_print:
                stats = controller.get_stats()
                debug = controller.last_mapping_debug
                if debug is not None:
                    print(
                        "mapping:",
                        f"mode={modes[mode_idx]}",
                        f"g={debug.g:.2f}",
                        f"t={debug.t:.2f}",
                        f"o_h={debug.o_h:.2f}",
                        f"u_blocks={{{', '.join(f'{k}:{v:.2f}' for k, v in debug.palm_blocks.items())}}}",
                    )
                print(
                    "stats:",
                    f"received={stats.frames_received}",
                    f"sent={stats.frames_sent}",
                    f"dropped={stats.frames_dropped}",
                    f"failures={stats.send_failures}",
                    f"dry_run={stats.dry_run}",
                )
                next_print = now + 1.0

            elapsed = time.monotonic() - now
            sleep_time = period - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping demo")
    finally:
        controller.stop()


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plain Python MH6 teleoperation framework")
    parser.add_argument("--demo", action="store_true", help="Run conservative internal demo skeletons")
    parser.add_argument("--duration", type=float, default=None, help="Optional run duration in seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="Control loop rate in Hz")
    parser.add_argument("--port", default=None, help="Modbus serial port, required with --enable-hardware")
    parser.add_argument("--enable-hardware", action="store_true", help="Actually send commands to MH6 hardware")
    parser.add_argument("--dry-run", action="store_true", help="Force dry-run output even if hardware is disabled")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    dry_run = True
    if args.enable_hardware:
        dry_run = False
    if args.dry_run:
        dry_run = True

    if args.enable_hardware and args.dry_run:
        print("ERROR: --enable-hardware and --dry-run cannot be used together")
        return 2

    if args.enable_hardware and not args.port:
        print("ERROR: --port is required when --enable-hardware is used")
        return 2

    if not args.demo:
        print("Safety note: no demo/input source selected, so no motion commands will be generated.")
        print("Run with --demo for a conservative dry-run demo.")
        print("Hardware output additionally requires --enable-hardware and --port.")
        return 0

    controller = MH6TeleopController(
        port=args.port,
        rate_hz=args.rate,
        dry_run=dry_run,
    )

    if dry_run:
        print("Dry-run mode: commands will be printed, not sent to hardware.")
    else:
        print("WARNING: --enable-hardware is set. Commands will be sent to the MH6 hand.")
        print("Keep emergency stop available and verify the workspace is clear.")

    run_demo(controller, args.duration)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
