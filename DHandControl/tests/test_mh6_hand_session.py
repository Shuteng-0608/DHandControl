import json
import sys
import tempfile
import unittest
from pathlib import Path

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
