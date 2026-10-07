import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_hand_session import HandSessionRecorder, ReplayHandStream
from visionpro_stream import VisionProHandFrame


def make_frame(timestamp, marker, hand="right"):
    transforms = np.repeat(np.eye(4)[None, :, :], 27, axis=0)
    transforms[:, 0, 3] = marker
    transforms[:, 1, 3] = np.linspace(0.0, 0.13, 27)
    return VisionProHandFrame(
        points=transforms[:, :3, 3].copy(),
        transforms=transforms,
        timestamp=timestamp,
        hand=hand,
    )


class HandSessionRoundTripTest(unittest.TestCase):
    def test_realtime_loop_waits_for_the_recorded_boundary_interval(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "loop-timing.npz"
            recorder = HandSessionRecorder(str(path))
            for index in range(3):
                recorder.record(make_frame(index * 0.1, index), "teleop")
            recorder.save()
            with patch("mh6_hand_session.time.monotonic", return_value=100.0) as clock:
                replay = ReplayHandStream(str(path), loop=True)
                replay.start()
                replay.set_phase("teleop")
                replay.get_latest_frame()
                clock.return_value = 100.11
                replay.get_latest_frame()
                clock.return_value = 100.21
                last = replay.get_latest_frame()
                self.assertTrue(replay.phase_finished)
                self.assertIsNone(replay.get_latest_frame())
                clock.return_value = 100.31
                first = replay.get_latest_frame()
                self.assertAlmostEqual(first.timestamp - last.timestamp, 0.1)
                self.assertEqual(first.points[0, 0], 0)

    def test_speed_and_recorded_gaps_control_wall_clock_delivery(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timed.npz"
            recorder = HandSessionRecorder(str(path))
            for timestamp in (10.0, 10.2, 11.0):
                recorder.record(make_frame(timestamp, timestamp), "teleop")
            recorder.save()
            with patch("mh6_hand_session.time.monotonic", return_value=100.0) as clock:
                replay = ReplayHandStream(str(path), speed=2.0)
                replay.start()
                replay.set_phase("teleop")
                first = replay.get_latest_frame()
                clock.return_value = 100.05
                self.assertIsNone(replay.get_latest_frame())
                clock.return_value = 100.11
                second = replay.get_latest_frame()
                self.assertAlmostEqual(second.timestamp - first.timestamp, 0.1)
                clock.return_value = 100.4
                self.assertIsNone(replay.get_latest_frame())
                clock.return_value = 100.5
                third = replay.get_latest_frame()
                self.assertAlmostEqual(third.timestamp - first.timestamp, 0.5)
                self.assertTrue(replay.phase_finished)

    def test_invalid_speed_and_wrong_hand_are_rejected(self):
        for speed in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                ReplayHandStream("unused.npz", speed=speed)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "left.npz"
            recorder = HandSessionRecorder(str(path))
            recorder.record(make_frame(1, 0.1, hand="left"), "teleop")
            recorder.save()
            with self.assertRaisesRegex(RuntimeError, "only 'right'"):
                ReplayHandStream(str(path), hand="right").start()

    def test_malformed_archive_dimensions_and_labels_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "malformed.npz"
            recorder = HandSessionRecorder(str(path))
            recorder.record(make_frame(1, 0.1), "teleop")
            recorder.save()
            with np.load(path, allow_pickle=False) as archive:
                valid = {key: archive[key] for key in archive.files}
            for key, bad_value in (
                ("timestamps", np.asarray(1.0)),
                ("hands", np.asarray([["right"]])),
                ("phases", np.asarray(["unknown"])),
                ("hands", np.asarray(["unknown"])),
                ("metadata_json", np.asarray("[]")),
            ):
                with self.subTest(key=key, bad_value=bad_value):
                    np.savez_compressed(path, **{**valid, key: bad_value})
                    with self.assertRaises(RuntimeError):
                        ReplayHandStream(str(path)).start()

    def test_points_and_transforms_readers_return_independent_arrays(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "readers.npz"
            recorder = HandSessionRecorder(str(path))
            recorder.record(make_frame(1, 0.1), "teleop")
            recorder.record(make_frame(1.05, 0.2), "teleop")
            recorder.save()
            replay = ReplayHandStream(str(path), no_wait=True)
            replay.start()
            replay.set_phase("teleop")
            points = replay.get_latest_points()
            transforms = replay.get_latest_transforms()
            self.assertEqual(points.shape, (27, 3))
            self.assertEqual(transforms.shape, (27, 4, 4))
            points[:] = 99
            transforms[:] = 99
            replay.set_phase("teleop")
            self.assertAlmostEqual(replay.get_latest_points()[0, 0], 0.1)
            self.assertAlmostEqual(replay.get_latest_transforms()[0, 0, 3], 0.2)
            self.assertIsNone(replay.get_latest_frame())

    def test_invalid_archive_is_reported_as_runtime_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.npz"
            np.savez_compressed(path, unexpected=np.asarray([1]))

            with self.assertRaisesRegex(RuntimeError, "failed to load replay session"):
                ReplayHandStream(str(path)).start()

    def test_recording_round_trip_preserves_phases_transforms_and_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.npz"
            recorder = HandSessionRecorder(str(path), {"rate": 20.0})
            recorder.record(make_frame(10.0, 0.1), "neutral")
            recorder.record(make_frame(10.1, 0.2), "range")
            recorder.record(make_frame(10.2, 0.3), "teleop")
            self.assertEqual(recorder.save(), path)

            replay = ReplayHandStream(str(path), no_wait=True)
            replay.start()
            self.assertEqual(replay.metadata["rate"], 20.0)

            expected = {"neutral": 0.1, "range": 0.2, "teleop": 0.3}
            for phase, marker in expected.items():
                replay.set_phase(phase)
                frame = replay.get_latest_frame()
                self.assertIsNotNone(frame)
                self.assertTrue(np.allclose(frame.transforms[:, 0, 3], marker))
                self.assertTrue(replay.phase_finished)
            replay.stop()

    def test_teleop_loop_keeps_replayed_timestamps_strictly_increasing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "loop.npz"
            recorder = HandSessionRecorder(str(path))
            for index in range(3):
                recorder.record(make_frame(5.0 + index * 0.05, index), "teleop")
            recorder.save()

            replay = ReplayHandStream(str(path), no_wait=True, loop=True)
            replay.start()
            replay.set_phase("teleop")
            timestamps = [replay.get_latest_frame().timestamp for _ in range(8)]

            self.assertTrue(
                all(right > left for left, right in zip(timestamps, timestamps[1:]))
            )

    def test_saved_session_uses_json_metadata_without_pickle(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "safe.npz"
            recorder = HandSessionRecorder(str(path), {"note": "测试"})
            recorder.record(make_frame(1.0, 0.0), "neutral")
            recorder.save()

            with np.load(path, allow_pickle=False) as archive:
                metadata = json.loads(str(archive["metadata_json"].item()))
                self.assertEqual(metadata["note"], "测试")
                self.assertEqual(archive["transforms"].shape, (1, 27, 4, 4))


if __name__ == "__main__":
    unittest.main()
