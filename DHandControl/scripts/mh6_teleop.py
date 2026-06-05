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
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

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
PALM_SOURCE_NAMES = ("thumbSide", "littleSide", "UL", "UR", "LL", "LR")
CALIBRATION_EPSILON = 1e-8

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


@dataclass
class HandSkeleton:
    points: np.ndarray = field(default_factory=lambda: np.zeros((27, 3), dtype=float))
    timestamp: float = 0.0
    valid: bool = True
    finger_keypoints: Dict[str, Tuple[str, ...]] = field(
        default_factory=lambda: dict(DEFAULT_FINGER_KEYPOINTS)
    )

    def __post_init__(self) -> None:
        arr = np.asarray(self.points, dtype=float)
        if arr.shape != (27, 3):
            raise ValueError(f"HandSkeleton points require shape (27, 3), got {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError("HandSkeleton received non-finite keypoint values")
        self.points = arr
        self._name_to_index = {name: index for index, name in KEYPOINT_INDEX_NAMES.items()}

    def point(self, name: str) -> np.ndarray:
        try:
            return self.points[self._name_to_index[name]]
        except KeyError as exc:
            raise KeyError(f"Missing hand keypoint: {name}") from exc

    def finger_points(self, finger_name: str) -> List[np.ndarray]:
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

        return cls(
            points=arr.copy(),
            timestamp=time.monotonic() if timestamp is None else timestamp,
            valid=valid,
        )


@dataclass
class PalmServoConfig:
    id: int
    name: str
    open_position: int
    closed_position: int
    time: int
    source: Optional[str] = None
    weights: Dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        data: Dict[str, object] = {
            "id": self.id,
            "name": self.name,
            "open_position": self.open_position,
            "closed_position": self.closed_position,
            "time": self.time,
        }
        if self.weights:
            data["weights"] = dict(self.weights)
        else:
            data["source"] = self.source
        return data


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
    palm_servos: List[PalmServoConfig] = field(default_factory=lambda: [
        PalmServoConfig(
            id=1,
            name="palm_1",
            open_position=500,
            closed_position=650,
            time=80,
            source="thumbSide",
        ),
        PalmServoConfig(
            id=2,
            name="palm_2",
            open_position=500,
            closed_position=600,
            time=80,
            weights={"UR": 0.5, "LR": 0.5},
        ),
        PalmServoConfig(
            id=3,
            name="palm_3",
            open_position=500,
            closed_position=560,
            time=80,
            source="littleSide",
        ),
    ])
    max_finger_delta_per_sec: float = 800.0
    max_palm_delta_per_sec: float = 400.0

    def to_dict(self) -> Dict[str, object]:
        return {
            "curl_open": dict(self.curl_open),
            "curl_closed": dict(self.curl_closed),
            "opposition_open_dist": dict(self.opposition_open_dist),
            "opposition_closed_dist": dict(self.opposition_closed_dist),
            "opposition_threshold": self.opposition_threshold,
            "thumb_curl_gain": self.thumb_curl_gain,
            "grasp_weights": dict(self.grasp_weights),
            "opposition_horizontal_weights": dict(self.opposition_horizontal_weights),
            "opposition_vertical_weights": dict(self.opposition_vertical_weights),
            "horizontal_from_grasp": self.horizontal_from_grasp,
            "horizontal_from_tripod": self.horizontal_from_tripod,
            "horizontal_from_opposition": self.horizontal_from_opposition,
            "vertical_from_finger_bias": self.vertical_from_finger_bias,
            "vertical_from_opposition": self.vertical_from_opposition,
            "finger_ids": list(self.finger_ids),
            "finger_open_positions": dict(self.finger_open_positions),
            "finger_closed_positions": dict(self.finger_closed_positions),
            "palm_servos": [servo.to_dict() for servo in self.palm_servos],
            "max_finger_delta_per_sec": self.max_finger_delta_per_sec,
            "max_palm_delta_per_sec": self.max_palm_delta_per_sec,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "TeleopCalibration":
        if not isinstance(data, dict):
            raise ValueError("Calibration JSON root must be an object")

        required_keys = set(cls().to_dict().keys())
        missing = sorted(required_keys - set(data.keys()))
        if missing:
            raise ValueError(f"Calibration missing required keys: {', '.join(missing)}")

        calibration = cls(
            curl_open=_require_float_dict(data, "curl_open", FINGER_NAMES),
            curl_closed=_require_float_dict(data, "curl_closed", FINGER_NAMES),
            opposition_open_dist=_require_float_dict(data, "opposition_open_dist", LONG_FINGER_NAMES),
            opposition_closed_dist=_require_float_dict(data, "opposition_closed_dist", LONG_FINGER_NAMES),
            opposition_threshold=_require_float(data, "opposition_threshold"),
            thumb_curl_gain=_require_float(data, "thumb_curl_gain"),
            grasp_weights=_require_float_dict(data, "grasp_weights", LONG_FINGER_NAMES),
            opposition_horizontal_weights=_require_float_dict(
                data, "opposition_horizontal_weights", LONG_FINGER_NAMES
            ),
            opposition_vertical_weights=_require_float_dict(
                data, "opposition_vertical_weights", LONG_FINGER_NAMES
            ),
            horizontal_from_grasp=_require_float(data, "horizontal_from_grasp"),
            horizontal_from_tripod=_require_float(data, "horizontal_from_tripod"),
            horizontal_from_opposition=_require_float(data, "horizontal_from_opposition"),
            vertical_from_finger_bias=_require_float(data, "vertical_from_finger_bias"),
            vertical_from_opposition=_require_float(data, "vertical_from_opposition"),
            finger_ids=_require_int_list(data, "finger_ids"),
            finger_open_positions=_require_int_dict(data, "finger_open_positions", FINGER_NAMES),
            finger_closed_positions=_require_int_dict(data, "finger_closed_positions", FINGER_NAMES),
            palm_servos=_require_palm_servos(data, "palm_servos"),
            max_finger_delta_per_sec=_require_float(data, "max_finger_delta_per_sec"),
            max_palm_delta_per_sec=_require_float(data, "max_palm_delta_per_sec"),
        )
        calibration.validate()
        return calibration

    @classmethod
    def load_json(cls, path: Union[str, Path]) -> "TeleopCalibration":
        json_path = Path(path)
        with json_path.open("r", encoding="utf-8") as fp:
            data = json.load(fp)
        return cls.from_dict(data)

    def save_json(self, path: Union[str, Path]) -> None:
        json_path = Path(path)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        with json_path.open("w", encoding="utf-8") as fp:
            json.dump(self.to_dict(), fp, indent=2, sort_keys=True)
            fp.write("\n")

    def validate(self) -> None:
        _validate_keys(self.curl_open, FINGER_NAMES, "curl_open")
        _validate_keys(self.curl_closed, FINGER_NAMES, "curl_closed")
        _validate_keys(self.opposition_open_dist, LONG_FINGER_NAMES, "opposition_open_dist")
        _validate_keys(self.opposition_closed_dist, LONG_FINGER_NAMES, "opposition_closed_dist")
        _validate_keys(self.grasp_weights, LONG_FINGER_NAMES, "grasp_weights")
        _validate_keys(self.opposition_horizontal_weights, LONG_FINGER_NAMES, "opposition_horizontal_weights")
        _validate_keys(self.opposition_vertical_weights, LONG_FINGER_NAMES, "opposition_vertical_weights")
        _validate_keys(self.finger_open_positions, FINGER_NAMES, "finger_open_positions")
        _validate_keys(self.finger_closed_positions, FINGER_NAMES, "finger_closed_positions")

        if len(self.finger_ids) != len(FINGER_NAMES):
            raise ValueError("finger_ids must contain exactly five IDs")
        if not self.palm_servos:
            raise ValueError("palm_servos must contain at least one servo")

        for field_name, values in (
            ("curl_open", self.curl_open),
            ("curl_closed", self.curl_closed),
            ("opposition_open_dist", self.opposition_open_dist),
            ("opposition_closed_dist", self.opposition_closed_dist),
            ("grasp_weights", self.grasp_weights),
            ("opposition_horizontal_weights", self.opposition_horizontal_weights),
            ("opposition_vertical_weights", self.opposition_vertical_weights),
        ):
            _validate_finite_values(values.values(), field_name)

        _validate_finite_values([
            self.opposition_threshold,
            self.thumb_curl_gain,
            self.horizontal_from_grasp,
            self.horizontal_from_tripod,
            self.horizontal_from_opposition,
            self.vertical_from_finger_bias,
            self.vertical_from_opposition,
            self.max_finger_delta_per_sec,
            self.max_palm_delta_per_sec,
        ], "scalar calibration values")

        if not 0.0 <= self.opposition_threshold < 1.0:
            raise ValueError("opposition_threshold must be in the range [0, 1)")
        if self.max_finger_delta_per_sec < 0.0 or self.max_palm_delta_per_sec < 0.0:
            raise ValueError("rate limits must be non-negative")

        for finger in FINGER_NAMES:
            if abs(self.curl_closed[finger] - self.curl_open[finger]) <= CALIBRATION_EPSILON:
                raise ValueError(f"curl_open and curl_closed are degenerate for finger: {finger}")

        for finger in LONG_FINGER_NAMES:
            delta = self.opposition_open_dist[finger] - self.opposition_closed_dist[finger]
            if abs(delta) <= CALIBRATION_EPSILON:
                raise ValueError(
                    "opposition_open_dist and opposition_closed_dist are degenerate "
                    f"for thumb-{finger}"
                )

        for device_id in self.finger_ids:
            if not 0 <= device_id <= 255:
                raise ValueError("finger_ids values must be in 0..255")

        seen_palm_names = set()
        for servo in self.palm_servos:
            _validate_palm_servo(servo)
            if servo.name in seen_palm_names:
                raise ValueError(f"Duplicate palm servo name: {servo.name}")
            seen_palm_names.add(servo.name)

    @property
    def palm_ids(self) -> List[int]:
        return [servo.id for servo in self.palm_servos]


def _validate_keys(values: Dict[str, object], required_keys: Sequence[str], name: str) -> None:
    missing = [key for key in required_keys if key not in values]
    if missing:
        raise ValueError(f"{name} missing required keys: {', '.join(missing)}")


def _validate_finite_values(values: Sequence[float], name: str) -> None:
    for value in values:
        if not math.isfinite(float(value)):
            raise ValueError(f"{name} must contain finite numbers")


def _require_mapping(data: Dict[str, object], key: str) -> Dict[str, object]:
    value = data[key]
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be an object")
    return value


def _require_float(data: Dict[str, object], key: str) -> float:
    value = data[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{key} must be finite")
    return result


def _require_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    return int(value)


def _require_float_dict(
    data: Dict[str, object],
    key: str,
    required_keys: Optional[Sequence[str]] = None,
) -> Dict[str, float]:
    source = _require_mapping(data, key)
    if required_keys is not None:
        _validate_keys(source, required_keys, key)
    result: Dict[str, float] = {}
    for item_key, value in source.items():
        if not isinstance(item_key, str):
            raise ValueError(f"{key} keys must be strings")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{key}.{item_key} must be a number")
        result[item_key] = float(value)
    _validate_finite_values(list(result.values()), key)
    return result


def _require_int_dict(
    data: Dict[str, object],
    key: str,
    required_keys: Optional[Sequence[str]] = None,
) -> Dict[str, int]:
    source = _require_mapping(data, key)
    if required_keys is not None:
        _validate_keys(source, required_keys, key)
    result: Dict[str, int] = {}
    for item_key, value in source.items():
        if not isinstance(item_key, str):
            raise ValueError(f"{key} keys must be strings")
        result[item_key] = _require_int(value, f"{key}.{item_key}")
    return result


def _require_int_list(data: Dict[str, object], key: str) -> List[int]:
    value = data[key]
    if not isinstance(value, list):
        raise ValueError(f"{key} must be a list")
    return [_require_int(item, f"{key}[{idx}]") for idx, item in enumerate(value)]


def _require_palm_servos(data: Dict[str, object], key: str) -> List[PalmServoConfig]:
    value = data[key]
    if not isinstance(value, list):
        raise ValueError(f"{key} must be a list")

    servos: List[PalmServoConfig] = []
    for idx, item in enumerate(value):
        if not isinstance(item, dict):
            raise ValueError(f"{key}[{idx}] must be an object")

        has_source = "source" in item
        has_weights = "weights" in item
        if has_source == has_weights:
            raise ValueError(f"{key}[{idx}] must contain exactly one of source or weights")

        name = item.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError(f"{key}[{idx}].name must be a non-empty string")

        source: Optional[str] = None
        weights: Dict[str, float] = {}
        if has_source:
            source_value = item["source"]
            if not isinstance(source_value, str):
                raise ValueError(f"{key}[{idx}].source must be a string")
            source = source_value
        else:
            weights_value = {"weights": item["weights"]}
            weights = _require_float_dict(weights_value, "weights")

        servo = PalmServoConfig(
            id=_require_int(item.get("id"), f"{key}[{idx}].id"),
            name=name,
            open_position=_require_int(item.get("open_position"), f"{key}[{idx}].open_position"),
            closed_position=_require_int(item.get("closed_position"), f"{key}[{idx}].closed_position"),
            time=_require_int(item.get("time"), f"{key}[{idx}].time"),
            source=source,
            weights=weights,
        )
        _validate_palm_servo(servo)
        servos.append(servo)
    return servos


def _validate_palm_servo(servo: PalmServoConfig) -> None:
    if not 0 <= servo.id <= 255:
        raise ValueError(f"palm servo {servo.name} id must be in 0..255")
    if not 0 <= servo.time <= 65535:
        raise ValueError(f"palm servo {servo.name} time must be in 0..65535")
    _validate_finite_values(
        [servo.open_position, servo.closed_position],
        f"palm servo {servo.name} actuator positions",
    )

    if servo.weights:
        for source_name, weight in servo.weights.items():
            if source_name not in PALM_SOURCE_NAMES:
                raise ValueError(
                    f"palm servo {servo.name} weights contain invalid source: {source_name}"
                )
            if not math.isfinite(weight):
                raise ValueError(f"palm servo {servo.name} weights must be finite")
    else:
        if servo.source not in PALM_SOURCE_NAMES:
            raise ValueError(f"palm servo {servo.name} source is invalid: {servo.source}")


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


def clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    denom = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denom <= 1e-12:
        return 0.0
    cos_value = clip(float(np.dot(a, b)) / denom, -1.0, 1.0)
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


def compute_finger_curl(points: Sequence[np.ndarray]) -> float:
    if len(points) < 3:
        return 0.0

    vectors = [points[i + 1] - points[i] for i in range(len(points) - 1)]
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
) -> LowDimHandCommand:
    if not skeleton.valid:
        raise ValueError("HandSkeleton is marked invalid")

    curl_norm: Dict[str, float] = {}
    for finger in FINGER_NAMES:
        c_i = compute_finger_curl(skeleton.finger_points(finger))
        curl_norm[finger] = normalize_bending(c_i, finger, calibration)

    thumb_tip = skeleton.point("thumb_tip")
    opposition: Dict[str, float] = {}
    for finger in LONG_FINGER_NAMES:
        d = float(np.linalg.norm(thumb_tip - skeleton.point(f"{finger}_tip")))
        p_raw = opposition_strength(d, finger, calibration)
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

    return LowDimHandCommand(
        u_thumb=u_thumb,
        u_index=u_index,
        u_middle=u_middle,
        u_ring=u_ring,
        u_little=u_little,
        u_h=u_h,
        u_v=u_v,
    )


def expand_palm_blocks(command: LowDimHandCommand) -> Dict[str, float]:
    thumb_side = clip(command.u_h - command.u_v, 0.0, 1.0)
    little_side = clip(command.u_h + command.u_v, 0.0, 1.0)
    return {
        "UL": thumb_side,
        "LL": thumb_side,
        "UR": little_side,
        "LR": little_side,
    }


def palm_source_value(source: str, palm_blocks: Dict[str, float]) -> float:
    if source == "thumbSide":
        return palm_blocks["UL"]
    if source == "littleSide":
        return palm_blocks["UR"]
    return palm_blocks[source]


def palm_servo_command_value(servo: PalmServoConfig, palm_blocks: Dict[str, float]) -> float:
    if servo.weights:
        total_weight = sum(abs(weight) for weight in servo.weights.values())
        if total_weight <= 1e-12:
            return 0.0
        return sum(
            palm_source_value(source, palm_blocks) * weight
            for source, weight in servo.weights.items()
        ) / total_weight
    if servo.source is None:
        return 0.0
    return palm_source_value(servo.source, palm_blocks)


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
    palm_positions: List[int] = []
    for servo in calibration.palm_servos:
        u_palm = palm_servo_command_value(servo, palm_blocks)
        u_palm = clip(u_palm, 0.0, 1.0)
        open_pos = servo.open_position
        closed_pos = servo.closed_position
        mapped = map_range(u_palm, 0.0, 1.0, open_pos, closed_pos)
        palm_positions.append(clamp_to_range(mapped, open_pos, closed_pos))

    return ActuatorCommand(
        finger_ids=list(calibration.finger_ids[:len(finger_positions)]),
        finger_positions=finger_positions,
        palm_ids=[servo.id for servo in calibration.palm_servos],
        palm_positions=palm_positions,
        palm_times=[servo.time for servo in calibration.palm_servos],
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
        self.stats = {
            "started_at": 0.0,
            "received": 0,
            "sent": 0,
            "dropped": 0,
            "failures": 0,
            "dry_run": dry_run,
        }

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
        self.stats["started_at"] = time.monotonic()

    def stop(self) -> None:
        self.running = False
        if self.hand is not None:
            self.hand.stop_persistent_connection()
            self.hand = None

    def update_skeleton(self, skeleton: HandSkeleton) -> None:
        low_dim = map_skeleton_to_low_dim(skeleton, self.calibration)
        actuator = low_dim_to_actuator_command(low_dim, self.calibration)
        self.update_actuator_command(actuator)

    def update_actuator_command(self, command: ActuatorCommand) -> None:
        with self._target_lock:
            if self._latest_target is not None:
                self.stats["dropped"] += 1
            self._latest_target = command
            self.stats["received"] += 1

    def get_stats(self) -> Dict[str, object]:
        return dict(self.stats)

    def tick(self) -> bool:
        target = self._take_latest_target()
        if target is None:
            return False
        command = self._prepare_command(target, time.monotonic())
        self._send_or_print(command)
        return True

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
        for idx, position in enumerate(command.palm_positions[:len(self.calibration.palm_servos)]):
            servo = self.calibration.palm_servos[idx]
            open_pos = servo.open_position
            closed_pos = servo.closed_position
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
            self.stats["sent"] += 1
            return

        if self.hand is None:
            self.stats["failures"] += 1
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
            self.stats["sent"] += 1
        else:
            self.stats["failures"] += 1


class VisionProTeleopAdapter:
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
                "avp_stream is required for --use-avp. Install it with "
                "`pip install --upgrade 'avp_stream>=2.50.0'`."
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
                    print(f"WARNING: VisionProTeleopAdapter.{method_name}() failed: {exc}")
                break
        self.streamer = None

    def get_skeleton(self) -> Optional[HandSkeleton]:
        if self.streamer is None:
            raise RuntimeError("VisionProTeleopAdapter.start() must be called before get_skeleton()")

        data = self.streamer.get_latest()
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

        points = transforms[:27, :3, 3]
        if points.shape != (27, 3):
            return None
        if not np.all(np.isfinite(points)):
            return None
        if np.allclose(points, 0.0):
            return None
        if not self._passes_hand_scale_check(points):
            return None

        return HandSkeleton.from_array(points, timestamp=time.monotonic(), valid=True)

    @staticmethod
    def _passes_hand_scale_check(points: np.ndarray) -> bool:
        wrist = points[0]
        fingertip_indices = [4, 9, 14, 19, 24]
        fingertip_distances = np.linalg.norm(points[fingertip_indices] - wrist, axis=1)
        max_distance = float(np.max(fingertip_distances))
        point_span = float(np.max(np.ptp(points, axis=0)))
        return 0.02 <= max_distance <= 0.50 and 0.02 <= point_span <= 0.70


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


def run_loop(
    controller: MH6TeleopController,
    get_skeleton_fn,
    duration: Optional[float],
    label: str,
) -> None:
    controller.start()
    period = 1.0 / controller.rate_hz if controller.rate_hz > 0.0 else 0.05
    start_time = time.monotonic()
    next_print = start_time

    try:
        while controller.running:
            now = time.monotonic()
            if duration is not None and (now - start_time) >= duration:
                break

            skeleton = get_skeleton_fn()
            if skeleton is not None:
                controller.update_skeleton(skeleton)
            controller.tick()

            if now >= next_print:
                stats = controller.get_stats()
                print(
                    label,
                    f"received={stats['received']}",
                    f"sent={stats['sent']}",
                    f"dropped={stats['dropped']}",
                    f"failures={stats['failures']}",
                    f"dry_run={stats['dry_run']}",
                )
                next_print = now + 1.0

            elapsed = time.monotonic() - now
            sleep_time = period - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print(f"KeyboardInterrupt: stopping {label}")
    finally:
        controller.stop()


def run_demo(controller: MH6TeleopController, duration: Optional[float]) -> None:
    modes = ("open", "fist", "thumb-index", "thumb-little")
    start_time = time.monotonic()

    def get_demo_skeleton() -> HandSkeleton:
        mode_idx = int((time.monotonic() - start_time) / 1.5) % len(modes)
        return make_demo_skeleton(modes[mode_idx])

    run_loop(controller, get_demo_skeleton, duration, "stats:")


def run_avp(
    controller: MH6TeleopController,
    adapter: VisionProTeleopAdapter,
    duration: Optional[float],
) -> None:
    try:
        adapter.start()
        run_loop(controller, adapter.get_skeleton, duration, "avp stats:")
    finally:
        adapter.stop()


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plain Python MH6 teleoperation framework")
    parser.add_argument("--demo", action="store_true", help="Run conservative internal demo skeletons")
    parser.add_argument("--duration", type=float, default=None, help="Optional run duration in seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="Control loop rate in Hz")
    parser.add_argument("--port", default=None, help="Modbus serial port, required with --enable-hardware")
    parser.add_argument("--enable-hardware", action="store_true", help="Actually send commands to MH6 hardware")
    parser.add_argument("--dry-run", action="store_true", help="Force dry-run output even if hardware is disabled")
    parser.add_argument("--calibration", default=None, help="Load teleop calibration from JSON")
    parser.add_argument("--use-avp", action="store_true", help="Use VisionProTeleop / avp_stream hand tracking")
    parser.add_argument("--avp-ip", default=None, help="Vision Pro IP address or room code for avp_stream")
    parser.add_argument("--hand", choices=("left", "right"), default="right", help="Tracked hand to use")
    parser.add_argument("--avp-origin", choices=("avp", "sim"), default="avp", help="avp_stream origin frame")
    parser.add_argument(
        "--save-default-calibration",
        default=None,
        help="Write the built-in default teleop calibration JSON to PATH and exit",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    if args.save_default_calibration:
        TeleopCalibration().save_json(args.save_default_calibration)
        print(f"Saved default calibration to {args.save_default_calibration}")
        return 0

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

    if args.demo and args.use_avp:
        print("ERROR: --demo and --use-avp cannot be used together")
        return 2

    if args.use_avp and not args.avp_ip:
        print("ERROR: --avp-ip is required when --use-avp is used")
        return 2

    if not args.demo and not args.use_avp:
        print("Safety note: no demo/input source selected, so no motion commands will be generated.")
        print("Run with --demo for a conservative dry-run demo.")
        print("Run with --use-avp --avp-ip <ip_or_room_code> for Vision Pro dry-run teleop.")
        print("Hardware output additionally requires --enable-hardware and --port.")
        return 0

    try:
        calibration = (
            TeleopCalibration.load_json(args.calibration)
            if args.calibration
            else TeleopCalibration()
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"ERROR: failed to load calibration: {exc}")
        return 2

    controller = MH6TeleopController(
        port=args.port,
        rate_hz=args.rate,
        dry_run=dry_run,
        calibration=calibration,
    )

    if dry_run:
        print("Dry-run mode: commands will be printed, not sent to hardware.")
    else:
        print("WARNING: --enable-hardware is set. Commands will be sent to the MH6 hand.")
        print("Keep emergency stop available and verify the workspace is clear.")

    if args.use_avp:
        adapter = VisionProTeleopAdapter(
            avp_ip=args.avp_ip,
            hand=args.hand,
            origin=args.avp_origin,
        )
        try:
            run_avp(controller, adapter, args.duration)
        except RuntimeError as exc:
            print(f"ERROR: {exc}")
            return 2
    else:
        run_demo(controller, args.duration)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
