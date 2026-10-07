import copy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_mapping import (
    MappingCalibration,
    MH6HandMapper,
    normalize_signed,
    normalize_signed_distance,
    thumb_rotation_angle,
)


class FixedMappingCalibrationTest(unittest.TestCase):
    def test_json_round_trip_preserves_every_parameter_and_mapping_output(self):
        calibration = MappingCalibration(
            thumb_rotation_outward=-0.4,
            thumb_rotation_open=0.3,
            thumb_rotation_closed=1.2,
            vertical_tripod_gain=0.7,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested" / "mapping.json"
            calibration.save_json(str(path))
            loaded = MappingCalibration.load_json(str(path))
        self.assertEqual(loaded.to_dict(), calibration.to_dict())
        points = make_right_hand(0.6)
        self.assertEqual(MH6HandMapper(loaded).step(points), MH6HandMapper(calibration).step(points))

    def test_bad_calibrations_are_rejected_before_use(self):
        valid = MappingCalibration().to_dict()
        changes = (
            ("format_version", 2), ("hand", "left"), ("mapping", {}),
        )
        for key, value in changes:
            data = copy.deepcopy(valid)
            data[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                MappingCalibration.from_dict(data)
        changes = (
            ("opposition_threshold", float("nan")), ("vertical_power_gain", True),
            ("curl_closed", {"index": 1}), ("thumb_rotation_closed", -1),
            ("motion_range_high_percentile", 101),
            ("power_grasp_weights", dict.fromkeys(valid["mapping"]["power_grasp_weights"], 0)),
        )
        for key, value in changes:
            data = copy.deepcopy(valid)
            data["mapping"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                MappingCalibration.from_dict(data)

    def test_old_teleop_configuration_is_not_silently_accepted(self):
        legacy = SCRIPTS_DIR.parent / "config" / "mh6_teleop_default_calibration.json"
        with self.assertRaises(ValueError):
            MappingCalibration.load_json(str(legacy))


class SignedTwoSegmentNormalizationTest(unittest.TestCase):
    def test_motion_hits_outward_neutral_and_inward_anchors(self) -> None:
        self.assertEqual(normalize_signed(2.0, 2.0, 4.0, 10.0), -1.0)
        self.assertEqual(normalize_signed(4.0, 2.0, 4.0, 10.0), 0.0)
        self.assertEqual(normalize_signed(10.0, 2.0, 4.0, 10.0), 1.0)
        self.assertEqual(normalize_signed(3.0, 2.0, 4.0, 10.0), -0.5)
        self.assertEqual(normalize_signed(7.0, 2.0, 4.0, 10.0), 0.5)

    def test_distance_uses_larger_outward_and_smaller_inward(self) -> None:
        self.assertEqual(normalize_signed_distance(12.0, 12.0, 10.0, 4.0), -1.0)
        self.assertEqual(normalize_signed_distance(10.0, 12.0, 10.0, 4.0), 0.0)
        self.assertEqual(normalize_signed_distance(4.0, 12.0, 10.0, 4.0), 1.0)

    def test_degenerate_half_range_returns_zero_instead_of_dividing(self) -> None:
        self.assertEqual(normalize_signed(-1.0, 0.0, 0.0, 1.0), 0.0)


def make_right_hand(thumb_angle: float) -> np.ndarray:
    points = np.zeros((27, 3), dtype=float)
    points[0] = [0.0, 0.0, 0.0]
    points[5] = [1.0, 2.0, 0.0]
    points[10] = [0.0, 2.0, 0.0]
    points[20] = [-1.0, 2.0, 0.0]

    points[1] = [1.2, 0.5, 0.0]
    thumb_direction = np.array([np.cos(thumb_angle), np.sin(thumb_angle), 0.0])
    points[2] = points[1] + thumb_direction
    points[3] = points[2] + thumb_direction
    points[4] = points[3] + thumb_direction

    for start in (5, 10, 15, 20):
        for offset in range(1, 5):
            points[start + offset] = points[start] + [0.0, 0.5 * offset, 0.0]
    return points


class ThumbRotationMappingTest(unittest.TestCase):
    def test_raw_angle_is_invariant_to_global_rotation(self) -> None:
        points = make_right_hand(np.pi / 3.0)
        rotation = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )

        self.assertAlmostEqual(thumb_rotation_angle(points), np.pi / 3.0)
        self.assertAlmostEqual(thumb_rotation_angle(points @ rotation.T), np.pi / 3.0)

    def test_low_dim_rotation_runs_from_minus_one_to_one(self) -> None:
        calibration = MappingCalibration(
            thumb_rotation_outward=-float(np.pi / 2.0),
            thumb_rotation_open=0.0,
            thumb_rotation_closed=float(np.pi / 2.0),
        )
        mapper = MH6HandMapper(calibration)

        outward_result = mapper.step(make_right_hand(-np.pi / 2.0))
        open_result = mapper.step(make_right_hand(0.0))
        opposed_result = mapper.step(make_right_hand(np.pi / 2.0))

        self.assertEqual(outward_result["low_dim"]["u_thumb_rotation"], -1.0)
        self.assertEqual(open_result["low_dim"]["u_thumb_rotation"], 0.0)
        self.assertEqual(opposed_result["low_dim"]["u_thumb_rotation"], 1.0)

    def test_open_calibration_sets_rotation_zero(self) -> None:
        sample = make_right_hand(0.35)
        mapper = MH6HandMapper()

        mapper.calibrate_open([sample, sample.copy()])

        self.assertAlmostEqual(mapper.calibration.thumb_rotation_open, 0.35)
        self.assertEqual(mapper.step(sample)["low_dim"]["u_thumb_rotation"], 0.0)


class LateralPalmMappingTest(unittest.TestCase):
    def test_neutral_lateral_fold_is_zero(self) -> None:
        sample = make_right_hand(0.35)
        mapper = MH6HandMapper()
        mapper.calibrate_open([sample])

        result = mapper.step(sample)

        self.assertAlmostEqual(result["palm_command"]["lateral"], 0.0)
        self.assertAlmostEqual(result["low_dim"]["u_v"], 0.0)

    def test_u_v_uses_signed_range(self) -> None:
        mapper = MH6HandMapper(
            MappingCalibration(
                thumb_rotation_outward=-1.0,
                thumb_rotation_open=0.0,
                thumb_rotation_closed=1.0,
            )
        )

        outputs = []
        for angle in np.linspace(-1.0, 1.0, 9):
            u_v = mapper.step(make_right_hand(float(angle)))["low_dim"]["u_v"]
            self.assertGreaterEqual(u_v, -1.0)
            self.assertLessEqual(u_v, 1.0)
            outputs.append(u_v)
        self.assertLess(min(outputs), 0.0)


class SeparatedMappingLayersTest(unittest.TestCase):
    def setUp(self) -> None:
        self.mapper = MH6HandMapper()
        self.curls = {
            "thumb": 0.4,
            "index": 0.2,
            "middle": 0.6,
            "ring": 0.8,
            "little": 1.0,
        }
        self.opposition = {
            "p_I": 0.9,
            "p_M": 0.1,
            "p_R": 0.0,
            "p_L": 0.3,
        }

    def test_finger_bending_does_not_include_opposition(self) -> None:
        finger_bending = self.mapper.compute_finger_bending_commands(self.curls)

        self.assertEqual(
            finger_bending,
            {
                "u_thumb": 0.4,
                "u_index": 0.2,
                "u_middle": 0.6,
                "u_ring": 0.8,
                "u_little": 1.0,
            },
        )
        self.assertNotEqual(finger_bending["u_index"], self.opposition["p_I"])

    def test_step_keeps_high_pinch_out_of_finger_low_dim(self) -> None:
        self.mapper.normalize_finger_curls = lambda _raw: self.curls.copy()
        self.mapper.normalize_opposition_distances = (
            lambda _raw: self.opposition.copy()
        )
        self.mapper.threshold_opposition = lambda _proximity: self.opposition.copy()

        result = self.mapper.step(make_right_hand(0.35))

        self.assertEqual(result["low_dim"]["u_index"], 0.2)
        self.assertEqual(result["opposition"]["p_I"], 0.9)
        self.assertEqual(result["grasp_intent"]["pinch_index"], 0.9)

    def test_grasp_intent_keeps_bending_and_opposition_contributions_named(self) -> None:
        intent = self.mapper.compute_grasp_intents(self.curls, self.opposition)

        self.assertAlmostEqual(intent["power_grasp"], 0.606)
        self.assertAlmostEqual(intent["tripod_flexion"], 0.2)
        self.assertAlmostEqual(intent["tripod_proximity"], 0.1)
        self.assertAlmostEqual(intent["tripod_precision"], 0.1)
        self.assertAlmostEqual(intent["thumb_opposition"], 0.9)
        self.assertAlmostEqual(intent["pinch_index"], 0.9)
        self.assertAlmostEqual(intent["opposition_cross"], 0.3)

    def test_palm_folds_are_computed_only_after_grasp_intents(self) -> None:
        intent = self.mapper.compute_grasp_intents(self.curls, self.opposition)
        folds = self.mapper.compute_palm_folds(intent)

        self.assertAlmostEqual(folds["vertical"], 0.641)
        self.assertAlmostEqual(folds["lateral"], 0.3606)
        self.assertAlmostEqual(folds["thumb_rotation_compensation"], 0.035)
        self.assertAlmostEqual(folds["thumb_rotation_command"], 0.035)


class GraspScenarioTest(unittest.TestCase):
    def setUp(self) -> None:
        self.mapper = MH6HandMapper()
        self.zero_opposition = {"p_I": 0.0, "p_M": 0.0, "p_R": 0.0, "p_L": 0.0}

    def test_power_grasp_drives_vertical_fold_more_than_lateral_fold(self) -> None:
        curls = {finger: 1.0 for finger in ("thumb", "index", "middle", "ring", "little")}
        intent = self.mapper.compute_grasp_intents(curls, self.zero_opposition)
        command = self.mapper.compute_palm_commands(intent, thumb_rotation_measured=0.0)

        self.assertAlmostEqual(intent["power_grasp"], 1.0)
        self.assertAlmostEqual(intent["tripod_precision"], 0.0)
        self.assertAlmostEqual(command["vertical"], 1.0)
        self.assertAlmostEqual(command["lateral"], 0.1)

    def test_tripod_precision_requires_curl_and_both_tip_distances(self) -> None:
        curls = {
            "thumb": 1.0,
            "index": 1.0,
            "middle": 1.0,
            "ring": 0.0,
            "little": 0.0,
        }
        both_close = {"p_I": 1.0, "p_M": 1.0, "p_R": 0.0, "p_L": 0.0}
        only_index_close = {"p_I": 1.0, "p_M": 0.0, "p_R": 0.0, "p_L": 0.0}

        tripod = self.mapper.compute_grasp_intents(curls, both_close)
        missing_middle = self.mapper.compute_grasp_intents(curls, only_index_close)
        not_curled = self.mapper.compute_grasp_intents(
            {finger: 0.0 for finger in curls},
            both_close,
        )

        self.assertEqual(tripod["tripod_precision"], 1.0)
        self.assertEqual(missing_middle["tripod_precision"], 0.0)
        self.assertEqual(not_curled["tripod_precision"], 0.0)

        command = self.mapper.compute_palm_commands(tripod, thumb_rotation_measured=0.0)
        self.assertGreater(command["vertical"], tripod["power_grasp"])
        self.assertGreater(command["thumb_rotation_compensation"], 0.0)

    def test_tripod_uses_continuous_distance_reduction_before_opposition_threshold(self) -> None:
        curls = {
            "thumb": 1.0,
            "index": 1.0,
            "middle": 1.0,
            "ring": 0.0,
            "little": 0.0,
        }
        thresholded_opposition = {
            "p_I": 0.0,
            "p_M": 0.0,
            "p_R": 0.0,
            "p_L": 0.0,
        }
        continuous_proximity = {
            "p_I": 0.2,
            "p_M": 0.2,
            "p_R": 0.0,
            "p_L": 0.0,
        }

        intent = self.mapper.compute_grasp_intents(
            curls,
            thresholded_opposition,
            continuous_proximity,
        )

        self.assertEqual(intent["tripod_proximity"], 0.2)
        self.assertEqual(intent["tripod_precision"], 0.2)
        self.assertEqual(intent["opposition_cross"], 0.0)

    def test_ring_and_little_opposition_drive_more_lateral_fold(self) -> None:
        curls = {finger: 0.0 for finger in ("thumb", "index", "middle", "ring", "little")}
        lateral_outputs = []
        for key in ("p_I", "p_M", "p_R", "p_L"):
            opposition = {"p_I": 0.0, "p_M": 0.0, "p_R": 0.0, "p_L": 0.0}
            opposition[key] = 1.0
            intent = self.mapper.compute_grasp_intents(curls, opposition)
            command = self.mapper.compute_palm_commands(intent, thumb_rotation_measured=0.0)
            lateral_outputs.append(command["lateral"])

        self.assertEqual(lateral_outputs, [0.15, 0.30, 0.75, 1.0])

    def test_negative_motion_does_not_create_negative_grasp_intent(self) -> None:
        curls = {finger: -1.0 for finger in ("thumb", "index", "middle", "ring", "little")}
        outward = {"p_I": -1.0, "p_M": -1.0, "p_R": -1.0, "p_L": -1.0}

        intent = self.mapper.compute_grasp_intents(curls, outward, outward)

        self.assertEqual(intent["power_grasp"], 0.0)
        self.assertEqual(intent["tripod_precision"], 0.0)
        self.assertEqual(intent["opposition_cross"], 0.0)

    def test_outward_curl_and_thumb_rotation_drive_negative_palm_commands(self) -> None:
        intent = self.mapper.compute_grasp_intents(
            {finger: -1.0 for finger in ("thumb", "index", "middle", "ring", "little")},
            self.zero_opposition,
        )
        command = self.mapper.compute_palm_commands(
            intent,
            thumb_rotation_measured=-0.8,
            signed_curls={finger: -1.0 for finger in ("thumb", "index", "middle", "ring", "little")},
            signed_opposition={"p_I": -1.0, "p_M": -1.0, "p_R": -1.0, "p_L": -1.0},
        )

        self.assertEqual(command["vertical"], -1.0)
        self.assertEqual(command["lateral"], -0.8)
        self.assertEqual(command["thumb_rotation_command"], -0.8)


class MotionRangeCalibrationTest(unittest.TestCase):
    def test_natural_pose_and_two_cycles_capture_three_boundaries(self) -> None:
        mapper = MH6HandMapper()
        mapper.compute_finger_curls = lambda points: {
            finger: float(points[0, 0])
            for finger in ("thumb", "index", "middle", "ring", "little")
        }
        mapper.compute_opposition_distances = lambda points: {
            finger: 1.0 - float(points[0, 0])
            for finger in ("index", "middle", "ring", "little")
        }

        def sample(value: float) -> np.ndarray:
            points = np.zeros((27, 3), dtype=float)
            points[0, 0] = value
            return points

        neutral_samples = [sample(0.5) for _ in range(10)]
        range_samples = [sample(0.0) for _ in range(10)] + [
            sample(1.0) for _ in range(10)
        ]

        with patch("mh6_mapping.thumb_rotation_angle", side_effect=lambda p: float(p[0, 0])):
            mapper.calibrate_neutral(neutral_samples)
            mapper.calibrate_motion_range(range_samples)

        for finger in ("thumb", "index", "middle", "ring", "little"):
            self.assertLess(mapper.calibration.curl_outward[finger], 0.5)
            self.assertEqual(mapper.calibration.curl_open[finger], 0.5)
            self.assertGreater(mapper.calibration.curl_closed[finger], 0.5)
        for finger in ("index", "middle", "ring", "little"):
            self.assertGreater(mapper.calibration.opposition_outward_dist[finger], 0.5)
            self.assertEqual(mapper.calibration.opposition_open_dist[finger], 0.5)
            self.assertLess(mapper.calibration.opposition_closed_dist[finger], 0.5)
        self.assertLess(mapper.calibration.thumb_rotation_outward, 0.5)
        self.assertEqual(mapper.calibration.thumb_rotation_open, 0.5)
        self.assertGreater(mapper.calibration.thumb_rotation_closed, 0.5)


if __name__ == "__main__":
    unittest.main()
