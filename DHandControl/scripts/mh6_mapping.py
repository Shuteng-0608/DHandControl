#!/usr/bin/env python3
"""
Pure MH6 hand teleoperation mapping.

This module maps 27x3 Vision Pro hand keypoints into MH6 low-dimensional hand
intentions. It intentionally contains no AVP streaming, ROS, Modbus, or hardware
control code.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
import json
from numbers import Real
import os
from pathlib import Path
import tempfile
from typing import Dict, Iterable, List, Optional

import numpy as np


FINGER_JOINTS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8, 9),
    "middle": (10, 11, 12, 13, 14),
    "ring": (15, 16, 17, 18, 19),
    "little": (20, 21, 22, 23, 24),
}
TIP_INDICES = {
    "thumb": 4,
    "index": 9,
    "middle": 14,
    "ring": 19,
    "little": 24,
}
FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")
LONG_FINGERS = ("index", "middle", "ring", "little")
WRIST_INDEX = 0
THUMB_BASE_INDEX = 1
THUMB_PROXIMAL_INDEX = 2
INDEX_BASE_INDEX = 5
MIDDLE_BASE_INDEX = 10
LITTLE_BASE_INDEX = 20


@dataclass
class MappingCalibration:
    """Calibration for normalized hand intention mapping.

    The natural pose is the 0 reference. Outward and inward motion boundaries
    map independently to -1 and +1.
    """

    curl_outward: Dict[str, float] = field(
        default_factory=lambda: {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
            "little": 0.0,
        }
    )
    # Compatibility name: curl_open now means the calibrated natural pose.
    curl_open: Dict[str, float] = field(
        default_factory=lambda: {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
            "little": 0.0,
        }
    )
    curl_closed: Dict[str, float] = field(
        default_factory=lambda: {
            "thumb": 1.40,
            "index": 2.40,
            "middle": 2.60,
            "ring": 2.60,
            "little": 2.40,
        }
    )
    opposition_open_dist: Dict[str, float] = field(
        default_factory=lambda: {
            "index": 0.090,
            "middle": 0.105,
            "ring": 0.120,
            "little": 0.135,
        }
    )
    opposition_outward_dist: Dict[str, float] = field(
        default_factory=lambda: {
            "index": 0.090,
            "middle": 0.105,
            "ring": 0.120,
            "little": 0.135,
        }
    )
    opposition_closed_dist: Dict[str, float] = field(
        default_factory=lambda: {
            "index": 0.018,
            "middle": 0.022,
            "ring": 0.026,
            "little": 0.030,
        }
    )
    opposition_threshold: float = 0.35
    thumb_rotation_outward: float = 0.0
    thumb_rotation_open: float = 0.0
    thumb_rotation_closed: float = float(np.pi / 2.0)
    power_grasp_weights: Dict[str, float] = field(
        default_factory=lambda: {
            "thumb": 0.15,
            "index": 0.20,
            "middle": 0.25,
            "ring": 0.22,
            "little": 0.18,
        }
    )
    opposition_cross_weights: Dict[str, float] = field(
        default_factory=lambda: {
            "index": 0.15,
            "middle": 0.30,
            "ring": 0.75,
            "little": 1.00,
        }
    )
    vertical_power_gain: float = 1.0
    vertical_tripod_gain: float = 0.35
    lateral_power_gain: float = 0.10
    lateral_opposition_gain: float = 1.0
    lateral_outward_distance_gain: float = 0.35
    thumb_tripod_compensation_gain: float = 0.35
    motion_range_low_percentile: float = 5.0
    motion_range_high_percentile: float = 95.0

    def validate(self) -> None:
        """Validate a complete fixed mapping calibration before file I/O."""
        defaults = MappingCalibration()
        for definition in fields(self):
            name = definition.name
            value = getattr(self, name)
            expected = getattr(defaults, name)
            if isinstance(expected, dict):
                if not isinstance(value, dict) or set(value) != set(expected):
                    raise ValueError(f"{name} must contain exactly: {', '.join(expected)}")
                numbers = value.values()
            else:
                numbers = (value,)
            for number in numbers:
                if isinstance(number, bool) or not isinstance(number, Real) or not np.isfinite(number):
                    raise ValueError(f"{name} must contain only finite numbers")
        for finger in FINGER_NAMES:
            if not 0 <= self.curl_outward[finger] <= self.curl_open[finger] <= self.curl_closed[finger]:
                raise ValueError(f"invalid outward/neutral/inward curl range for {finger}")
        for finger in LONG_FINGERS:
            if not 0 <= self.opposition_closed_dist[finger] <= self.opposition_open_dist[finger] <= self.opposition_outward_dist[finger]:
                raise ValueError(f"invalid inward/neutral/outward distance range for {finger}")
        if not self.thumb_rotation_outward <= self.thumb_rotation_open <= self.thumb_rotation_closed:
            raise ValueError("invalid outward/neutral/inward thumb rotation range")
        if not 0 <= self.opposition_threshold < 1:
            raise ValueError("opposition_threshold must be within [0,1)")
        if not 0 <= self.motion_range_low_percentile < self.motion_range_high_percentile <= 100:
            raise ValueError("motion range percentiles must satisfy 0 <= low < high <= 100")
        if sum(max(value, 0) for value in self.power_grasp_weights.values()) <= 0:
            raise ValueError("power_grasp_weights must contain a positive weight")

    def to_dict(self) -> Dict:
        self.validate()
        return {"format_version": 1, "hand": "right", "mapping": asdict(self)}

    @classmethod
    def from_dict(cls, data: Dict) -> "MappingCalibration":
        if not isinstance(data, dict) or data.get("format_version") != 1 or data.get("hand") != "right":
            raise ValueError("expected a version-1 right-hand MappingCalibration JSON")
        mapping = data.get("mapping")
        expected_keys = {definition.name for definition in fields(cls)}
        if not isinstance(mapping, dict) or set(mapping) != expected_keys:
            raise ValueError("mapping must contain every MappingCalibration field and no unknown fields")
        calibration = cls(**mapping)
        calibration.validate()
        return calibration

    @classmethod
    def load_json(cls, path: str) -> "MappingCalibration":
        with Path(path).expanduser().open(encoding="utf-8") as source:
            return cls.from_dict(json.load(source))

    def save_json(self, path: str) -> None:
        data = self.to_dict()
        destination = Path(path).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=destination.parent,
                prefix=f".{destination.name}.", suffix=".tmp", delete=False,
            ) as temporary:
                temporary_path = Path(temporary.name)
                json.dump(data, temporary, indent=2, sort_keys=True, allow_nan=False)
                temporary.write("\n")
            os.replace(temporary_path, destination)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()


def validate_points(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.shape != (27, 3):
        raise ValueError(f"points must have shape (27, 3), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError("points must contain only finite values")
    return arr


def clip(x: float, lo: float, hi: float) -> float:
    return float(min(max(float(x), lo), hi))


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))
    if n1 <= 1e-12 or n2 <= 1e-12:
        return 0.0
    cos_theta = clip(float(np.dot(v1, v2)) / (n1 * n2), -1.0, 1.0)
    return float(np.arccos(cos_theta))


def finger_curl(points: np.ndarray, joint_indices: Iterable[int]) -> float:
    joint_points = points[list(joint_indices)]
    if len(joint_points) < 3:
        return 0.0
    vectors = np.diff(joint_points, axis=0)
    return float(
        sum(angle_between(vectors[i], vectors[i + 1]) for i in range(len(vectors) - 1))
    )


def distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def normalize(value: float, open_value: float, closed_value: float) -> float:
    denom = float(closed_value) - float(open_value)
    if abs(denom) <= 1e-12:
        raise ValueError("open and closed calibration values must not be equal")
    return clip((float(value) - float(open_value)) / denom, 0.0, 1.0)


def normalize_signed(
    value: float,
    outward_value: float,
    neutral_value: float,
    inward_value: float,
) -> float:
    """Map outward/neutral/inward calibration points to -1/0/+1.

    Each half is scaled independently. A missing or degenerate half-range is
    treated as zero on that side so calibration noise cannot cause division by
    zero or an artificial saturated command.
    """

    value = float(value)
    outward_value = float(outward_value)
    neutral_value = float(neutral_value)
    inward_value = float(inward_value)
    if value < neutral_value:
        span = neutral_value - outward_value
        if span <= 1e-12:
            return 0.0
        return clip((value - neutral_value) / span, -1.0, 0.0)

    span = inward_value - neutral_value
    if span <= 1e-12:
        return 0.0
    return clip((value - neutral_value) / span, 0.0, 1.0)


def normalize_signed_distance(
    value: float,
    outward_value: float,
    neutral_value: float,
    inward_value: float,
) -> float:
    """Map a distance to -1/0/+1, with larger=outward and smaller=inward."""

    value = float(value)
    outward_value = float(outward_value)
    neutral_value = float(neutral_value)
    inward_value = float(inward_value)
    if value > neutral_value:
        span = outward_value - neutral_value
        if span <= 1e-12:
            return 0.0
        return clip(-(value - neutral_value) / span, -1.0, 0.0)

    span = neutral_value - inward_value
    if span <= 1e-12:
        return 0.0
    return clip((neutral_value - value) / span, 0.0, 1.0)


def dominant_direction(negative: float, positive: float) -> float:
    """Choose the stronger directional candidate instead of cancelling them."""

    negative = clip(negative, -1.0, 0.0)
    positive = clip(positive, 0.0, 1.0)
    return negative if abs(negative) > positive else positive


def threshold_strength(value: float, threshold: float) -> float:
    threshold = clip(threshold, 0.0, 0.999999)
    value = clip(value, 0.0, 1.0)
    if value <= threshold:
        return 0.0
    return clip((value - threshold) / (1.0 - threshold), 0.0, 1.0)


def thumb_rotation_angle(points: np.ndarray) -> float:
    """Return right-thumb in-plane rotation from abduction toward opposition.

    The palm-local frame uses little-base -> index-base as the thumb-side axis
    and wrist -> middle-base as the forward axis. The thumb proximal bone is
    projected into that plane, making the angle invariant to global hand pose.
    """

    points = validate_points(points)
    thumb_side = points[INDEX_BASE_INDEX] - points[LITTLE_BASE_INDEX]
    thumb_side_norm = float(np.linalg.norm(thumb_side))
    if thumb_side_norm <= 1e-12:
        return 0.0
    thumb_side = thumb_side / thumb_side_norm

    palm_forward = points[MIDDLE_BASE_INDEX] - points[WRIST_INDEX]
    palm_forward = palm_forward - float(np.dot(palm_forward, thumb_side)) * thumb_side
    palm_forward_norm = float(np.linalg.norm(palm_forward))
    if palm_forward_norm <= 1e-12:
        return 0.0
    palm_forward = palm_forward / palm_forward_norm

    thumb_proximal = points[THUMB_PROXIMAL_INDEX] - points[THUMB_BASE_INDEX]
    thumb_side_component = float(np.dot(thumb_proximal, thumb_side))
    palm_forward_component = float(np.dot(thumb_proximal, palm_forward))
    if abs(thumb_side_component) <= 1e-12 and abs(palm_forward_component) <= 1e-12:
        return 0.0

    return float(np.arctan2(palm_forward_component, thumb_side_component))


class MH6HandMapper:
    def __init__(self, calibration: Optional[MappingCalibration] = None) -> None:
        self.calibration = calibration if calibration is not None else MappingCalibration()

    def calibrate_neutral(self, samples: List[np.ndarray]) -> None:
        if not samples:
            raise ValueError("calibrate_neutral requires at least one sample")

        curls = {finger: [] for finger in FINGER_NAMES}
        opposition_distances = {finger: [] for finger in LONG_FINGERS}
        thumb_rotations = []
        for sample in samples:
            points = validate_points(sample)
            raw_curls = self.compute_finger_curls(points)
            for finger in FINGER_NAMES:
                curls[finger].append(raw_curls[finger])

            raw_distances = self.compute_opposition_distances(points)
            for finger in LONG_FINGERS:
                opposition_distances[finger].append(raw_distances[finger])
            thumb_rotations.append(thumb_rotation_angle(points))

        self.calibration.curl_open = {
            finger: float(np.median(values)) for finger, values in curls.items()
        }
        self.calibration.opposition_open_dist = {
            finger: float(np.median(values))
            for finger, values in opposition_distances.items()
        }
        self.calibration.thumb_rotation_open = float(np.median(thumb_rotations))

    def calibrate_open(self, samples: List[np.ndarray]) -> None:
        """Compatibility alias for the natural-pose calibration."""

        self.calibrate_neutral(samples)

    def calibrate_motion_range(self, samples: List[np.ndarray]) -> None:
        """Capture robust outward/inward boundaries from two motion cycles."""

        if not samples:
            raise ValueError("calibrate_motion_range requires at least one sample")

        low_q = self.calibration.motion_range_low_percentile
        high_q = self.calibration.motion_range_high_percentile
        if not 0.0 <= low_q < high_q <= 100.0:
            raise ValueError("motion range percentiles must satisfy 0 <= low < high <= 100")

        curls = {finger: [] for finger in FINGER_NAMES}
        opposition_distances = {finger: [] for finger in LONG_FINGERS}
        thumb_rotations = []
        for sample in samples:
            points = validate_points(sample)
            raw_curls = self.compute_finger_curls(points)
            raw_distances = self.compute_opposition_distances(points)
            for finger in FINGER_NAMES:
                curls[finger].append(raw_curls[finger])
            for finger in LONG_FINGERS:
                opposition_distances[finger].append(raw_distances[finger])
            thumb_rotations.append(thumb_rotation_angle(points))

        for finger in FINGER_NAMES:
            outward = float(np.percentile(curls[finger], low_q))
            inward = float(np.percentile(curls[finger], high_q))
            neutral = self.calibration.curl_open[finger]
            self.calibration.curl_outward[finger] = min(outward, neutral)
            if inward > neutral + 1e-6:
                self.calibration.curl_closed[finger] = inward

        for finger in LONG_FINGERS:
            inward = float(np.percentile(opposition_distances[finger], low_q))
            outward = float(np.percentile(opposition_distances[finger], high_q))
            neutral = self.calibration.opposition_open_dist[finger]
            self.calibration.opposition_outward_dist[finger] = max(outward, neutral)
            if inward < neutral - 1e-6:
                self.calibration.opposition_closed_dist[finger] = inward

        outward_rotation = float(np.percentile(thumb_rotations, low_q))
        inward_rotation = float(np.percentile(thumb_rotations, high_q))
        neutral_rotation = self.calibration.thumb_rotation_open
        self.calibration.thumb_rotation_outward = min(
            outward_rotation,
            neutral_rotation,
        )
        if inward_rotation > neutral_rotation + 1e-6:
            self.calibration.thumb_rotation_closed = inward_rotation

    def compute_finger_curls(self, points: np.ndarray) -> Dict[str, float]:
        points = validate_points(points)
        return {
            finger: finger_curl(points, FINGER_JOINTS[finger])
            for finger in FINGER_NAMES
        }

    def compute_opposition_distances(self, points: np.ndarray) -> Dict[str, float]:
        """Return raw thumb-to-fingertip distances without intent mapping."""

        points = validate_points(points)
        thumb_tip = points[TIP_INDICES["thumb"]]
        return {
            finger: distance(thumb_tip, points[TIP_INDICES[finger]])
            for finger in LONG_FINGERS
        }

    def compute_normalized_curls(self, points: np.ndarray) -> Dict[str, float]:
        raw = self.compute_finger_curls(points)
        return self.normalize_finger_curls(raw)

    def normalize_finger_curls(
        self,
        raw: Dict[str, float],
    ) -> Dict[str, float]:
        """Normalize already-extracted curls without recomputing geometry."""

        return {
            finger: normalize_signed(
                raw[finger],
                self.calibration.curl_outward[finger],
                self.calibration.curl_open[finger],
                self.calibration.curl_closed[finger],
            )
            for finger in FINGER_NAMES
        }

    def compute_opposition_proximity(self, points: np.ndarray) -> Dict[str, float]:
        """Return continuous 0..1 inward fingertip proximity."""

        raw_distances = self.compute_opposition_distances(points)
        return self.normalize_opposition_distances(raw_distances)

    def normalize_opposition_distances(
        self,
        raw_distances: Dict[str, float],
    ) -> Dict[str, float]:
        """Return the positive half of signed fingertip-distance motion."""

        return {
            key: max(value, 0.0)
            for key, value in self.normalize_signed_opposition_distances(
                raw_distances
            ).items()
        }

    def normalize_signed_opposition_distances(
        self,
        raw_distances: Dict[str, float],
    ) -> Dict[str, float]:
        """Map larger/neutral/smaller tip distances to -1/0/+1."""

        signed = {}
        for finger, key in (
            ("index", "p_I"),
            ("middle", "p_M"),
            ("ring", "p_R"),
            ("little", "p_L"),
        ):
            signed[key] = normalize_signed_distance(
                raw_distances[finger],
                self.calibration.opposition_outward_dist[finger],
                self.calibration.opposition_open_dist[finger],
                self.calibration.opposition_closed_dist[finger],
            )
        return signed

    def compute_opposition(self, points: np.ndarray) -> Dict[str, float]:
        """Return thresholded per-finger opposition intents."""

        proximity = self.compute_opposition_proximity(points)
        return self.threshold_opposition(proximity)

    def threshold_opposition(
        self,
        proximity: Dict[str, float],
    ) -> Dict[str, float]:
        """Apply the configured dead zone to continuous proximity features."""

        return {
            key: threshold_strength(value, self.calibration.opposition_threshold)
            for key, value in proximity.items()
        }

    def compute_thumb_rotation(self, points: np.ndarray) -> float:
        """Return signed right-thumb rotation: -1=outward, +1=opposed."""

        raw_angle = thumb_rotation_angle(points)
        return self.normalize_thumb_rotation(raw_angle)

    def normalize_thumb_rotation(self, raw_angle: float) -> float:
        """Normalize an already-extracted thumb rotation angle."""

        return normalize_signed(
            raw_angle,
            self.calibration.thumb_rotation_outward,
            self.calibration.thumb_rotation_open,
            self.calibration.thumb_rotation_closed,
        )

    @staticmethod
    def compute_finger_bending_commands(
        normalized_curls: Dict[str, float],
    ) -> Dict[str, float]:
        """Expose pure finger bending without mixing fingertip opposition."""

        return {
            f"u_{finger}": clip(normalized_curls[finger], -1.0, 1.0)
            for finger in FINGER_NAMES
        }

    def compute_grasp_intents(
        self,
        normalized_curls: Dict[str, float],
        opposition: Dict[str, float],
        opposition_proximity: Optional[Dict[str, float]] = None,
    ) -> Dict[str, float]:
        """Infer static power, tripod-precision, and opposition grasp intents."""

        c_thumb = clip(normalized_curls["thumb"], 0.0, 1.0)
        c_index = clip(normalized_curls["index"], 0.0, 1.0)
        c_middle = clip(normalized_curls["middle"], 0.0, 1.0)
        c_ring = clip(normalized_curls["ring"], 0.0, 1.0)
        c_little = clip(normalized_curls["little"], 0.0, 1.0)

        p_i = clip(opposition["p_I"], 0.0, 1.0)
        p_m = clip(opposition["p_M"], 0.0, 1.0)
        p_r = clip(opposition["p_R"], 0.0, 1.0)
        p_l = clip(opposition["p_L"], 0.0, 1.0)
        proximity = opposition if opposition_proximity is None else opposition_proximity
        near_i = clip(proximity["p_I"], 0.0, 1.0)
        near_m = clip(proximity["p_M"], 0.0, 1.0)

        curls = {
            "thumb": c_thumb,
            "index": c_index,
            "middle": c_middle,
            "ring": c_ring,
            "little": c_little,
        }
        power_weights = self.calibration.power_grasp_weights
        power_weight_sum = sum(
            max(float(power_weights[finger]), 0.0)
            for finger in FINGER_NAMES
        )
        if power_weight_sum <= 1e-12:
            raise ValueError("power grasp weights must contain a positive weight")
        power_grasp = sum(
            max(float(power_weights[finger]), 0.0) * curls[finger]
            for finger in FINGER_NAMES
        ) / power_weight_sum

        tripod_flexion = min(c_thumb, c_index, c_middle)
        tripod_proximity = min(near_i, near_m)
        tripod_precision = min(tripod_flexion, tripod_proximity)

        cross_weights = self.calibration.opposition_cross_weights
        opposition_cross = max(
            max(float(cross_weights["index"]), 0.0) * p_i,
            max(float(cross_weights["middle"]), 0.0) * p_m,
            max(float(cross_weights["ring"]), 0.0) * p_r,
            max(float(cross_weights["little"]), 0.0) * p_l,
        )
        thumb_opposition = max(p_i, p_m, p_r, p_l)

        return {
            "power_grasp": power_grasp,
            "tripod_flexion": tripod_flexion,
            "tripod_proximity": tripod_proximity,
            "tripod_precision": tripod_precision,
            "opposition_cross": opposition_cross,
            "thumb_opposition": thumb_opposition,
            "pinch_index": p_i,
            "pinch_middle": p_m,
            "pinch_ring": p_r,
            "pinch_little": p_l,
            # Compatibility aliases for existing logs and analysis scripts.
            "P_opp": thumb_opposition,
            "finger_enclosure": power_grasp,
            "g": power_grasp,
            "t": tripod_precision,
        }

    def compute_palm_commands(
        self,
        grasp_intent: Dict[str, float],
        thumb_rotation_measured: float,
        signed_curls: Optional[Dict[str, float]] = None,
        signed_opposition: Optional[Dict[str, float]] = None,
    ) -> Dict[str, float]:
        """Combine signed outward motion with positive grasp assistance."""

        vertical_positive = clip(
            self.calibration.vertical_power_gain * grasp_intent["power_grasp"]
            + self.calibration.vertical_tripod_gain
            * grasp_intent["tripod_precision"],
            0.0,
            1.0,
        )
        lateral_positive = clip(
            self.calibration.lateral_power_gain * grasp_intent["power_grasp"]
            + self.calibration.lateral_opposition_gain
            * grasp_intent["opposition_cross"],
            0.0,
            1.0,
        )
        signed_curls = signed_curls or {}
        power_weights = self.calibration.power_grasp_weights
        weight_sum = sum(
            max(float(power_weights.get(finger, 0.0)), 0.0)
            for finger in FINGER_NAMES
        )
        vertical_negative = 0.0
        if weight_sum > 1e-12:
            vertical_negative = sum(
                max(float(power_weights.get(finger, 0.0)), 0.0)
                * min(float(signed_curls.get(finger, 0.0)), 0.0)
                for finger in FINGER_NAMES
            ) / weight_sum
        vertical_fold = dominant_direction(vertical_negative, vertical_positive)

        signed_opposition = signed_opposition or {}
        cross_weights = self.calibration.opposition_cross_weights
        outward_distance = min(
            min(float(signed_opposition.get(key, 0.0)), 0.0)
            * max(float(cross_weights[finger]), 0.0)
            for finger, key in (
                ("index", "p_I"),
                ("middle", "p_M"),
                ("ring", "p_R"),
                ("little", "p_L"),
            )
        )
        thumb_rotation_measured = clip(thumb_rotation_measured, -1.0, 1.0)
        lateral_negative = min(
            min(thumb_rotation_measured, 0.0),
            self.calibration.lateral_outward_distance_gain * outward_distance,
        )
        lateral_fold = dominant_direction(lateral_negative, lateral_positive)

        thumb_rotation_compensation = clip(
            self.calibration.thumb_tripod_compensation_gain
            * grasp_intent["tripod_precision"]
            * (1.0 - max(thumb_rotation_measured, 0.0)),
            0.0,
            1.0 - thumb_rotation_measured,
        )
        thumb_rotation_command = clip(
            thumb_rotation_measured + thumb_rotation_compensation,
            -1.0,
            1.0,
        )
        return {
            "vertical": vertical_fold,
            "lateral": lateral_fold,
            "thumb_rotation_measured": thumb_rotation_measured,
            "thumb_rotation_compensation": thumb_rotation_compensation,
            "thumb_rotation_command": thumb_rotation_command,
            # Compatibility aliases while downstream names are migrated.
            "common": vertical_fold,
            "lateral_unit": lateral_fold,
        }

    def compute_palm_folds(
        self,
        grasp_intent: Dict[str, float],
        thumb_rotation_measured: float = 0.0,
    ) -> Dict[str, float]:
        """Compatibility alias for compute_palm_commands()."""

        return self.compute_palm_commands(grasp_intent, thumb_rotation_measured)

    def compute_low_dim(self, points: np.ndarray) -> Dict[str, float]:
        """Return the 8D normalized command.

        Fingers are -1=outward, 0=natural, and 1=curled. u_h is the vertical
        fold from finger extension toward flexion. u_v is the lateral fold
        from the thumb side toward the little-finger side. Both are signed.
        """

        return self.step(points)["low_dim"]

    def step(self, points: np.ndarray) -> Dict[str, Dict[str, float]]:
        """Return debug-friendly mapping outputs with normalized conventions.

        Directional motion uses -1=outward, 0=natural, +1=inward. Grasp
        intentions remain non-negative strengths in 0..1.
        """

        points = validate_points(points)
        curl_raw = self.compute_finger_curls(points)
        curl_norm = self.normalize_finger_curls(curl_raw)
        opposition_distances = self.compute_opposition_distances(points)
        opposition_signed = self.normalize_signed_opposition_distances(
            opposition_distances
        )
        opposition_proximity = {
            key: max(value, 0.0) for key, value in opposition_signed.items()
        }
        opposition = self.threshold_opposition(opposition_proximity)
        thumb_rotation_raw = thumb_rotation_angle(points)
        u_thumb_rotation = self.normalize_thumb_rotation(thumb_rotation_raw)
        finger_bending = self.compute_finger_bending_commands(curl_norm)
        grasp_intent = self.compute_grasp_intents(
            curl_norm,
            opposition,
            opposition_proximity,
        )
        palm_command = self.compute_palm_commands(
            grasp_intent,
            u_thumb_rotation,
            curl_norm,
            opposition_signed,
        )

        u_h = palm_command["vertical"]
        u_v = palm_command["lateral"]

        return {
            "curl_raw": curl_raw,
            "curl_norm": curl_norm,
            "opposition_distances": opposition_distances,
            "opposition_signed": opposition_signed,
            "opposition_proximity": opposition_proximity,
            "opposition": opposition,
            "finger_bending": finger_bending,
            "grasp_intent": grasp_intent,
            "intent": {
                **grasp_intent,
                "thumb_rotation_raw": thumb_rotation_raw,
            },
            "palm_fold": palm_command,
            "palm_command": palm_command,
            "low_dim": {
                "u_thumb": finger_bending["u_thumb"],
                "u_thumb_rotation": u_thumb_rotation,
                "u_index": finger_bending["u_index"],
                "u_middle": finger_bending["u_middle"],
                "u_ring": finger_bending["u_ring"],
                "u_little": finger_bending["u_little"],
                "u_h": u_h,
                "u_v": u_v,
            },
            "palm": palm_command.copy(),
        }
