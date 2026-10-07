from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_hand_session import HandSessionRecorder, ReplayHandStream
from mh6_mapping import MappingCalibration
from mh6_mapping_calibration import calibration_from_session
from mh6_teleop_run import main as teleop_main
from visionpro_session import main, parse_args
from visionpro_stream import VisionProHandFrame


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def make_frame(timestamp, amount=0.2, hand="right"):
    points = np.zeros((27, 3))
    for start, x in ((5, 0.025), (10, 0), (15, -0.012), (20, -0.025)):
        points[start] = (x, 0.04, 0)
        for offset in range(1, 5):
            angle = (offset - 1) * amount * 0.8
            points[start + offset] = points[start + offset - 1] + (
                0, 0.018 * np.cos(angle), -0.018 * np.sin(angle)
            )
    points[1] = (0.025, 0.015, 0)
    for offset in range(1, 4):
        angle = 0.3 + amount * 0.85 + (offset - 1) * amount * 0.3
        points[offset + 1] = points[offset] + (
            0.02 * np.cos(angle), 0.02 * np.sin(angle), 0
        )
    transforms = np.repeat(np.eye(4)[None], 27, axis=0)
    transforms[:, :3, 3] = points
    return VisionProHandFrame(points, transforms, timestamp, hand)


class FakeHandStream:
    def __init__(self, clock, *, ready_after=0.0, interrupt_after=None):
        self.clock = clock
        self.ready_at = clock.now + ready_after
        self.interrupt_after = interrupt_after
        self.reads = 0
        self.started = False
        self.stopped = False
        self.frames = []

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def get_latest_frame(self):
        self.reads += 1
        if self.interrupt_after is not None and self.reads > self.interrupt_after:
            raise KeyboardInterrupt
        if self.clock.now < self.ready_at:
            return None
        frame = make_frame(self.clock.now, (self.reads % 20) / 19)
        self.frames.append(frame)
        return frame


class StandaloneSessionTest(unittest.TestCase):
    def capture(self, path, stream, clock, extra=()):
        with (
            patch("visionpro_session.VisionProHandStream", return_value=stream),
            patch("visionpro_session.time.monotonic", side_effect=clock.monotonic),
            patch("visionpro_session.time.sleep", side_effect=clock.sleep),
            redirect_stdout(io.StringIO()) as output,
        ):
            status = main([
                "record", "--avp-ip", "test-avp", "--output", str(path),
                "--mode", "full",
                "--prepare-seconds", "0", "--calibrate-seconds", "0.2",
                "--range-calibrate-seconds", "0.3", "--duration", "0.2",
                *extra,
            ])
        return status, output.getvalue()

    def test_standalone_capture_replays_through_the_full_mh6_pipeline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.npz"
            clock = FakeClock()
            stream = FakeHandStream(clock, ready_after=0.15)
            status, _ = self.capture(path, stream, clock)
            self.assertEqual(status, 0)
            self.assertTrue(stream.started and stream.stopped)
            with np.load(path, allow_pickle=False) as archive:
                metadata = json.loads(archive["metadata_json"].item())
                self.assertEqual(metadata["source"], "visionpro_session")
                self.assertEqual(metadata["hand"], "right")
                self.assertEqual(set(archive["phases"]), {"neutral", "range", "teleop"})
                expected_count = int(np.sum(archive["phases"] == "teleop"))
                np.testing.assert_array_equal(
                    archive["transforms"], np.stack([frame.transforms for frame in stream.frames[1:]])
                )
                np.testing.assert_allclose(
                    archive["timestamps"],
                    [frame.timestamp - stream.frames[1].timestamp for frame in stream.frames[1:]],
                )

            # Repeated offline experiments must retain all frames and match exactly.
            logs = []
            for index in range(2):
                log = Path(directory) / f"replay-{index}.jsonl"
                with redirect_stdout(io.StringIO()):
                    status = teleop_main([
                        "--replay-session", str(path), "--replay-no-wait", "--debug-log", str(log),
                    ])
                self.assertEqual(status, 0)
                records = [json.loads(line) for line in log.read_text().splitlines()]
                self.assertEqual(len(records), expected_count)
                logs.append(records)
            self.assertEqual(logs[0], logs[1])

    def test_default_recording_mode_contains_only_teleop(self):
        self.assertEqual(parse_args([
            "record", "--avp-ip", "test", "--output", "test.npz",
        ]).mode, "teleop")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleop.npz"
            clock = FakeClock()
            status, _ = self.capture(path, FakeHandStream(clock), clock, ["--mode", "teleop"])
            self.assertEqual(status, 0)
            with np.load(path, allow_pickle=False) as archive:
                self.assertEqual(set(archive["phases"]), {"teleop"})
            with (
                patch("mh6_teleop_run.MH6HandMapper.calibrate_neutral") as neutral,
                patch("mh6_teleop_run.MH6HandMapper.calibrate_motion_range") as motion_range,
                redirect_stdout(io.StringIO()),
            ):
                status = teleop_main([
                    "--replay-session", str(path), "--replay-no-wait",
                    "--use-default-calibration",
                ])
            self.assertEqual(status, 0)
            neutral.assert_not_called()
            motion_range.assert_not_called()

    def test_separate_calibration_npz_and_fixed_json_match_full_session_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            full = Path(directory) / "full.npz"
            calibration_path = Path(directory) / "calibration.npz"
            teleop_path = Path(directory) / "teleop.npz"
            calibration_json = Path(directory) / "mapping.json"
            clock = FakeClock()
            status, _ = self.capture(full, FakeHandStream(clock), clock)
            self.assertEqual(status, 0)
            source = ReplayHandStream(str(full), no_wait=True)
            source.start()
            calibration_recorder = HandSessionRecorder(str(calibration_path))
            teleop_recorder = HandSessionRecorder(str(teleop_path))
            for index, phase in enumerate(source.phases):
                transforms = source.transforms[index].copy()
                frame = VisionProHandFrame(
                    transforms[:, :3, 3].copy(), transforms,
                    float(source.timestamps[index]), str(source.hands[index]),
                )
                target = teleop_recorder if phase == "teleop" else calibration_recorder
                target.record(frame, str(phase))
            calibration_recorder.save()
            teleop_recorder.save()
            with redirect_stdout(io.StringIO()):
                status = main([
                    "calibrate", "--input", str(calibration_path), "--output", str(calibration_json),
                ])
            self.assertEqual(status, 0)
            self.assertEqual(
                MappingCalibration.load_json(str(calibration_json)).to_dict(),
                calibration_from_session(str(full)).to_dict(),
            )
            variants = (
                ["--replay-session", str(full)],
                ["--replay-session", str(teleop_path), "--calibration-session", str(calibration_path)],
                ["--replay-session", str(teleop_path), "--mapping-calibration", str(calibration_json)],
            )
            logs = []
            for index, options in enumerate(variants):
                log = Path(directory) / f"split-{index}.jsonl"
                with redirect_stdout(io.StringIO()):
                    status = teleop_main([*options, "--replay-no-wait", "--debug-log", str(log)])
                self.assertEqual(status, 0)
                logs.append([json.loads(line) for line in log.read_text().splitlines()])
            self.assertEqual(logs[0], logs[1])
            self.assertEqual(logs[0], logs[2])

    def test_calibration_only_capture_saves_two_phases_and_exports_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.npz"
            output = Path(directory) / "calibration.json"
            clock = FakeClock()
            stream = FakeHandStream(clock)
            status, _ = self.capture(path, stream, clock, ["--mode", "calibration"])
            self.assertEqual(status, 0)
            with np.load(path, allow_pickle=False) as archive:
                self.assertEqual(set(archive["phases"]), {"neutral", "range"})
            with redirect_stdout(io.StringIO()):
                status = main(["calibrate", "--input", str(path), "--output", str(output)])
            self.assertEqual(status, 0)
            MappingCalibration.load_json(str(output))

    def test_teleop_only_requires_an_explicit_calibration_source(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleop.npz"
            recorder = HandSessionRecorder(str(path))
            recorder.record(make_frame(1), "teleop")
            recorder.save()
            with redirect_stdout(io.StringIO()) as output:
                status = teleop_main(["--replay-session", str(path), "--replay-no-wait"])
            self.assertEqual(status, 2)
            self.assertIn("--use-default-calibration", output.getvalue())

    def test_runner_exports_fixed_parameters_and_rejects_invalid_json_before_connecting(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleop.npz"
            calibration = Path(directory) / "defaults.json"
            recorder = HandSessionRecorder(str(path))
            recorder.record(make_frame(1), "teleop")
            recorder.save()
            with redirect_stdout(io.StringIO()):
                status = teleop_main([
                    "--replay-session", str(path), "--use-default-calibration",
                    "--replay-no-wait", "--save-mapping-calibration", str(calibration),
                ])
            self.assertEqual(status, 0)
            self.assertEqual(MappingCalibration.load_json(str(calibration)), MappingCalibration())
            calibration.write_text('{"mapping": {}}')
            with patch("mh6_teleop_run.VisionProHandStream") as stream, redirect_stdout(io.StringIO()):
                status = teleop_main(["--mapping-calibration", str(calibration)])
            self.assertEqual(status, 2)
            stream.assert_not_called()

    def test_exporting_calibration_from_teleop_only_does_not_create_a_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleop.npz"
            output = Path(directory) / "invalid.json"
            recorder = HandSessionRecorder(str(path))
            recorder.record(make_frame(1), "teleop")
            recorder.save()
            with redirect_stdout(io.StringIO()):
                status = main(["calibrate", "--input", str(path), "--output", str(output)])
            self.assertEqual(status, 2)
            self.assertFalse(output.exists())

    def test_ctrl_c_saves_and_closes_the_source(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "interrupted.npz"
            clock = FakeClock()
            stream = FakeHandStream(clock, interrupt_after=20)
            status, _ = self.capture(path, stream, clock, ["--duration", "30"])
            self.assertEqual(status, 0)
            self.assertTrue(stream.stopped)
            replay = ReplayHandStream(str(path), no_wait=True)
            replay.start()
            self.assertTrue(replay.metadata["interrupted"])
            self.assertEqual(set(replay.phases), {"neutral", "range", "teleop"})

    def test_partial_capture_is_saved_but_reported_as_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "partial.npz"
            clock = FakeClock()
            stream = FakeHandStream(clock, interrupt_after=2)
            status, output = self.capture(path, stream, clock)
            self.assertEqual(status, 1)
            self.assertTrue(stream.stopped)
            self.assertIn("Incomplete session", output)
            with np.load(path, allow_pickle=False) as archive:
                self.assertEqual(list(archive["phases"]), ["neutral"])

    def test_missing_tracking_times_out_without_an_empty_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.npz"
            clock = FakeClock()
            stream = FakeHandStream(clock, ready_after=100)
            status, output = self.capture(path, stream, clock, ["--connect-timeout", "0.2"])
            self.assertEqual(status, 2)
            self.assertTrue(stream.stopped)
            self.assertFalse(path.exists())
            self.assertIn("connect-timeout", output)

    def test_existing_recording_is_preserved_unless_overwrite_is_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "keep.npz"
            path.write_bytes(b"existing recording")
            clock = FakeClock()
            stream = FakeHandStream(clock)
            status, _ = self.capture(path, stream, clock)
            self.assertEqual(status, 2)
            self.assertFalse(stream.started)
            self.assertEqual(path.read_bytes(), b"existing recording")
            status, _ = self.capture(path, stream, clock, ["--overwrite"])
            self.assertEqual(status, 0)
            ReplayHandStream(str(path)).start()

    def test_save_error_still_closes_the_source(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = FakeClock()
            stream = FakeHandStream(clock)
            with patch("visionpro_session.HandSessionRecorder.save", side_effect=OSError("disk full")):
                status, output = self.capture(Path(directory) / "failed.npz", stream, clock)
            self.assertEqual(status, 2)
            self.assertTrue(stream.stopped)
            self.assertIn("disk full", output)

    def test_info_and_preview_work_offline_for_a_selected_phase(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "offline.npz"
            recorder = HandSessionRecorder(str(path))
            for phase in ("neutral", "range", "teleop"):
                for index in range(3):
                    recorder.record(make_frame(1 + len(recorder.timestamps) * 0.05, index / 3), phase)
            recorder.save()
            for phase in ("neutral", "range", "teleop"):
                with redirect_stdout(io.StringIO()) as output:
                    status = main(["replay", "--input", str(path), "--phase", phase, "--no-wait"])
                self.assertEqual(status, 0)
                self.assertIn("Replay completed: 3 frames", output.getvalue())
            with redirect_stdout(io.StringIO()) as output:
                status = main(["info", "--input", str(path)])
            self.assertEqual(status, 0)
            self.assertIn("Frames: 9", output.getvalue())

    def test_invalid_timing_options_are_rejected(self):
        for command, flag in (("record", "--rate"), ("record", "--duration"),
                              ("record", "--prepare-seconds"), ("replay", "--speed")):
            for value in ("nan", "inf", "-1"):
                with self.subTest(command=command, flag=flag, value=value):
                    required = ["--avp-ip", "test", "--output", "test.npz"] if command == "record" else ["--input", "test.npz"]
                    with redirect_stdout(io.StringIO()), patch("sys.stderr", new=io.StringIO()):
                        with self.assertRaises(SystemExit):
                            parse_args([command, *required, flag, value])


if __name__ == "__main__":
    unittest.main()
