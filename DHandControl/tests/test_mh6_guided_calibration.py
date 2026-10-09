"""Timing, stage isolation, quality failures, and portable voiced calibration."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import wave

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mh6_calibration_voice import CalibrationVoice, VOICE_DIRECTORY, CUES
from mh6_guided_calibration import (apply_stage, run_guided_calibration, calibration_from_staged_stream,
                                    metrics, verify_neutral, PROTOCOL, finish_neutral_verification,
                                    staged_endpoint_mapper, load_pending_calibration,
                                    NeutralVerificationError)
from mh6_hand_session import HandSessionRecorder, ReplayHandStream, STAGED_CALIBRATION_PHASES
from mh6_mapping import MH6HandMapper, TIP_INDICES
from mh6_mapping_calibration import calibration_from_session
from mh6_teleop_run import main as teleop_main
from visionpro_session import main as session_main
from visionpro_stream import VisionProHandFrame
from test_visionpro_session import FakeClock, make_frame


def pose(cue):
    points = make_frame(0, 0).points.copy()
    if cue in ("grasp", "thumb_rotate"):
        return make_frame(0, 1).points.copy()
    if cue.startswith("opp_"):
        points[TIP_INDICES[cue[4:]]] = points[4]+(.001, 0, 0)
    return points


class Operator:
    def __init__(self, clock):
        self.clock, self.cue, self.calls = clock, "neutral", []
        self.playback_end = clock.now
        self.playback_ends = []
        self.sample_times = []
        self.stopped = False

    def play(self, cue):
        self.calls.append(cue)
        self.clock.sleep(.7)  # Simulate actual playback time.
        self.playback_end = self.clock.now
        self.playback_ends.append(self.playback_end)
        self.cue = cue

    def start(self): pass
    def stop(self): self.stopped = True

    def get_latest_frame(self):
        points = pose(self.cue)
        transforms = np.tile(np.eye(4), (27, 1, 1)); transforms[:, :3, 3] = points
        self.sample_times.append((self.cue, self.clock.now-self.playback_end))
        return VisionProHandFrame(points, transforms, self.clock.now, "right")


class VoiceTest(unittest.TestCase):
    def test_all_user_recordings_have_valid_portable_pcm_copies(self):
        manifest = json.loads((VOICE_DIRECTORY / "manifest.json").read_text())
        self.assertEqual(set(manifest), set(CUES))
        import hashlib
        for cue in CUES:
            with wave.open(str(VOICE_DIRECTORY / f"{cue}.wav")) as audio:
                self.assertEqual((audio.getnchannels(), audio.getsampwidth(), audio.getframerate()), (1, 2, 24000))
                self.assertGreater(audio.getnframes()/audio.getframerate(), 1)
            for ext in ("m4a", "wav"):
                self.assertEqual(hashlib.sha256((VOICE_DIRECTORY / f"{cue}.{ext}").read_bytes()).hexdigest(),
                                 manifest[cue][f"{ext}_sha256"])

    def test_player_is_synchronous_and_failure_falls_back_once(self):
        with patch("mh6_calibration_voice.shutil.which", return_value="/usr/bin/paplay"), \
                patch("mh6_calibration_voice.subprocess.run") as run, redirect_stdout(io.StringIO()):
            voice = CalibrationVoice()
            voice.play("grasp")
            run.assert_called_once()
            run.side_effect = subprocess.CalledProcessError(1, "paplay")
            voice.play("neutral")
            voice.play("grasp")
            self.assertEqual(run.call_count, 2)

    def test_muted_mode_never_launches_a_player_or_requires_assets(self):
        with patch("mh6_calibration_voice.subprocess.run") as run, redirect_stdout(io.StringIO()):
            CalibrationVoice("/does/not/exist", enabled=False).play("neutral")
            run.assert_not_called()

    def test_keyboard_interrupt_is_not_swallowed(self):
        with patch("mh6_calibration_voice.shutil.which", return_value="paplay"), \
                patch("mh6_calibration_voice.subprocess.run", side_effect=KeyboardInterrupt), \
                redirect_stdout(io.StringIO()):
            with self.assertRaises(KeyboardInterrupt): CalibrationVoice().play("neutral")


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.mapper = MH6HandMapper()
        self.neutral = apply_stage(self.mapper, "neutral", [[pose("neutral")]*10])

    def test_each_gesture_updates_only_its_own_endpoints(self):
        before = self.mapper.calibration.to_dict()
        apply_stage(self.mapper, "grasp", [[pose("grasp")]*10]*2, self.neutral)
        self.assertEqual(self.mapper.calibration.opposition_closed_dist, before["mapping"]["opposition_closed_dist"])
        self.assertEqual(self.mapper.calibration.thumb_rotation_closed, before["mapping"]["thumb_rotation_closed"])
        old_curls = self.mapper.calibration.curl_closed.copy()
        apply_stage(self.mapper, "thumb_rotate", [[pose("thumb_rotate")]*10]*2, self.neutral)
        for finger in ("index", "middle", "ring", "little"):
            old = self.mapper.calibration.opposition_closed_dist.copy()
            apply_stage(self.mapper, f"opp_{finger}", [[pose(f"opp_{finger}")]*10], self.neutral)
            for other in old:
                if other != finger: self.assertEqual(self.mapper.calibration.opposition_closed_dist[other], old[other])
        self.assertEqual(self.mapper.calibration.curl_closed, old_curls)
        self.mapper.calibration.validate()
        np.testing.assert_allclose(verify_neutral(self.mapper, [pose("neutral")]*10)["median_h_v_r"], [0,0,0])

    def test_inadequate_movement_does_not_silently_keep_defaults(self):
        before = self.mapper.calibration.to_dict()
        with self.assertRaisesRegex(ValueError, "范围不足"):
            apply_stage(self.mapper, "grasp", [[pose("neutral")]*10]*2, self.neutral)
        self.assertEqual(before, self.mapper.calibration.to_dict())

    def test_noise_and_inconsistent_repetitions_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "不一致"):
            apply_stage(self.mapper, "grasp", [[make_frame(0, .5).points]*10,
                                                [pose("grasp")]*10], self.neutral)
        with self.assertRaisesRegex(ValueError, "不稳定|不足"):
            apply_stage(self.mapper, "grasp", [[pose("neutral"), pose("grasp")]*10]*2, self.neutral)
        with self.assertRaisesRegex(ValueError, "采样不足"): metrics([pose("neutral")]*4)

    def capture(self, folder):
        clock = FakeClock(); operator = Operator(clock)
        recorder = HandSessionRecorder(str(Path(folder)/"staged.npz"))
        with patch("mh6_guided_calibration.time.monotonic", side_effect=clock.monotonic), \
                patch("mh6_guided_calibration.time.sleep", side_effect=clock.sleep), redirect_stdout(io.StringIO()):
            calibration, report = run_guided_calibration(operator, operator, recorder=recorder,
                                                        neutral_seconds=.3, hold_seconds=.3, move_seconds=.2)
        recorder.save()
        return recorder, calibration, report, operator

    def test_audio_finishes_before_movement_and_stable_windows_and_roundtrip(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder, calibration, report, operator = self.capture(folder)
            self.assertTrue(report["passed"])
            self.assertEqual(operator.calls.count("grasp"), 2)
            self.assertEqual(operator.calls.count("thumb_rotate"), 2)
            self.assertNotIn("complete", operator.calls)  # Caller plays it only after successful saving.
            self.assertTrue(all(relative >= 0 for _, relative in operator.sample_times[1:]))
            for segment in recorder.metadata["calibration_segments"]:
                self.assertGreaterEqual(segment["end"]-segment["start"], 5)
                # Last voice playback ends 0.2 seconds before each stable window.
                first_time = recorder.timestamps[segment["start"]]+recorder._first_timestamp
                previous_voice = max(end for end in operator.playback_ends if end <= first_time)
                self.assertGreaterEqual(first_time-previous_voice, .2-1e-8)
            self.assertEqual(calibration_from_session(str(recorder.path)).to_dict(), calibration.to_dict())
            self.assertEqual(set(recorder.phases)-{"transition", "verify_neutral"}, set(STAGED_CALIBRATION_PHASES))

    def test_incomplete_or_mislabeled_saved_windows_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder, _, _, _ = self.capture(folder)
            stream = ReplayHandStream(str(recorder.path)); stream.start()
            stream.metadata["calibration_complete"] = False
            with self.assertRaisesRegex(ValueError, "incomplete"): calibration_from_staged_stream(stream)
            stream.metadata["calibration_complete"] = True
            stream.metadata["calibration_segments"][0]["end"] += 1
            with self.assertRaisesRegex(ValueError, "segment"): calibration_from_staged_stream(stream)

    def test_staged_record_export_and_full_replay_never_play_voices_offline(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder, calibration, _, _ = self.capture(folder)
            # Add teleop frames to the completed calibration session.
            full = HandSessionRecorder(str(Path(folder)/"full.npz"), metadata=recorder.metadata)
            source = ReplayHandStream(str(recorder.path)); source.start()
            for index, phase in enumerate(source.phases):
                t = source.transforms[index]
                full.record(VisionProHandFrame(t[:,:3,3],t,float(source.timestamps[index]),"right"),str(phase))
            for i in range(3): full.record(make_frame(float(source.timestamps[-1])+1+i*.05, .2),"teleop")
            full.save()
            output = Path(folder)/"mapping.json"; log = Path(folder)/"solver.jsonl"
            with redirect_stdout(io.StringIO()), patch("mh6_calibration_voice.subprocess.run", side_effect=AssertionError("audio during replay")):
                self.assertEqual(session_main(["calibrate","--input",str(full.path),"--output",str(output)]),0)
                self.assertEqual(teleop_main(["--solver-only","--replay-session",str(full.path),"--replay-no-wait","--debug-log",str(log)]),0)
            self.assertTrue(output.with_suffix(".report.json").exists())
            self.assertEqual(len(log.read_text().splitlines()),3)
            self.assertEqual(json.loads(output.read_text()),calibration.to_dict())

    def test_standalone_cli_uses_staged_voice_and_saves_before_completion(self):
        with tempfile.TemporaryDirectory() as folder:
            clock=FakeClock(); operator=Operator(clock); path=Path(folder)/"calibration.npz"
            mapping=Path(folder)/"mapping.json"
            original_play=operator.play
            def play(cue):
                if cue == "complete":
                    self.assertTrue(path.exists() and mapping.exists())
                    self.assertTrue(mapping.with_suffix(".report.json").exists())
                original_play(cue)
            with patch("visionpro_session.VisionProHandStream",return_value=operator), \
                    patch("visionpro_session.CalibrationVoice",return_value=operator), \
                    patch.object(operator,"play",side_effect=play), \
                    patch("mh6_guided_calibration.time.monotonic",side_effect=clock.monotonic), \
                    patch("mh6_guided_calibration.time.sleep",side_effect=clock.sleep), redirect_stdout(io.StringIO()):
                status=session_main(["record","--avp-ip","mock","--output",str(path),"--mode","calibration",
                                     "--mapping-output",str(mapping),
                                     "--calibrate-seconds",".3","--calibration-hold-seconds",".3","--calibration-move-seconds",".2"])
            self.assertEqual(status,0); self.assertTrue(operator.stopped)
            self.assertEqual(operator.calls[-1],"complete")
            calibration_from_session(str(path))

    def test_failed_stage_retries_only_that_stage_and_cannot_report_completion(self):
        with tempfile.TemporaryDirectory() as folder:
            clock=FakeClock(); operator=Operator(clock)
            recorder=HandSessionRecorder(str(Path(folder)/"failed.npz"))
            original_play=operator.play
            def no_grasp(cue):
                original_play(cue)
                if cue == "grasp": operator.cue="neutral"
            with patch.object(operator,"play",side_effect=no_grasp), \
                    patch("mh6_guided_calibration.time.monotonic",side_effect=clock.monotonic), \
                    patch("mh6_guided_calibration.time.sleep",side_effect=clock.sleep), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError,"范围不足"):
                    run_guided_calibration(operator,operator,recorder=recorder,
                                           neutral_seconds=.3,hold_seconds=.3,move_seconds=.2)
            self.assertEqual(operator.calls.count("grasp"),4)
            self.assertNotIn("thumb_rotate",operator.calls)
            self.assertFalse(recorder.metadata["calibration_complete"])
            self.assertNotIn("complete",operator.calls)
            recorder.save()
            with self.assertRaisesRegex(ValueError,"incomplete"):
                calibration_from_session(str(recorder.path))

    def test_live_runner_checkpoints_before_completion_and_then_records_teleop(self):
        with tempfile.TemporaryDirectory() as folder:
            clock=FakeClock(); operator=Operator(clock)
            session=Path(folder)/"live.npz"; mapping=Path(folder)/"mapping.json"
            original_play=operator.play; original_frame=operator.get_latest_frame
            after_completion=[]
            def play(cue):
                if cue=="complete":
                    self.assertTrue(session.exists() and mapping.exists())
                    self.assertTrue(mapping.with_suffix(".report.json").exists())
                original_play(cue)
            def frame():
                if operator.cue=="complete":
                    if after_completion: raise KeyboardInterrupt
                    after_completion.append(True)
                    operator.cue="neutral"
                    result=original_frame()
                    operator.cue="complete"
                    return result
                return original_frame()
            with patch("mh6_teleop_run.VisionProHandStream",return_value=operator), \
                    patch("mh6_teleop_run.CalibrationVoice",return_value=operator), \
                    patch.object(operator,"play",side_effect=play), \
                    patch.object(operator,"get_latest_frame",side_effect=frame), \
                    patch("mh6_guided_calibration.time.monotonic",side_effect=clock.monotonic), \
                    patch("mh6_guided_calibration.time.sleep",side_effect=clock.sleep), redirect_stdout(io.StringIO()):
                status=teleop_main(["--solver-only","--avp-ip","mock","--record-session",str(session),
                                   "--save-mapping-calibration",str(mapping),"--debug-log",str(Path(folder)/"solver.jsonl"),
                                   "--calibrate-seconds",".3","--calibration-hold-seconds",".3","--calibration-move-seconds",".2"])
            self.assertEqual(status,0); self.assertTrue(operator.stopped)
            replay=ReplayHandStream(str(session)); replay.start()
            self.assertEqual(int(np.sum(replay.phases=="teleop")),1)
            self.assertTrue(replay.metadata["calibration_complete"])

    def test_full_capture_announces_completed_calibration_before_teleop(self):
        with tempfile.TemporaryDirectory() as folder:
            clock=FakeClock(); operator=Operator(clock); path=Path(folder)/"full.npz"
            original_play=operator.play; original_frame=operator.get_latest_frame
            teleop_started=[]
            def play(cue):
                if cue=="complete":
                    self.assertTrue(path.exists())
                    self.assertFalse(teleop_started)
                original_play(cue)
            def frame():
                if operator.cue=="complete":
                    teleop_started.append(True)
                    operator.cue="neutral"
                    result=original_frame()
                    operator.cue="complete"
                    return result
                return original_frame()
            with patch("visionpro_session.VisionProHandStream",return_value=operator), \
                    patch("visionpro_session.CalibrationVoice",return_value=operator), \
                    patch.object(operator,"play",side_effect=play), \
                    patch.object(operator,"get_latest_frame",side_effect=frame), \
                    patch("mh6_guided_calibration.time.monotonic",side_effect=clock.monotonic), \
                    patch("mh6_guided_calibration.time.sleep",side_effect=clock.sleep), redirect_stdout(io.StringIO()):
                status=session_main(["record","--avp-ip","mock","--output",str(path),"--mode","full",
                                     "--duration",".2","--prepare-seconds","0", "--calibrate-seconds",".3",
                                     "--calibration-hold-seconds",".3","--calibration-move-seconds",".2"])
            self.assertEqual(status,0)
            self.assertEqual(operator.calls.count("complete"),1)
            replay=ReplayHandStream(str(path)); replay.start()
            self.assertGreater(int(np.sum(replay.phases=="teleop")),0)
            calibration_from_session(str(path))

    def test_final_neutral_failure_records_diagnostics_and_retries_without_changing_endpoints(self):
        with tempfile.TemporaryDirectory() as folder:
            original, calibration, _, _ = self.capture(folder)
            source=ReplayHandStream(str(original.path)); source.start()
            mapper, _=staged_endpoint_mapper(source)
            before=mapper.calibration.to_dict()
            clock=FakeClock(); operator=Operator(clock)
            recorder=HandSessionRecorder(str(Path(folder)/"verify.npz"),metadata={
                "calibration_segments":[],"calibration_report":{"passed":False},"calibration_complete":False})
            original_play=operator.play; calls=[]
            def play(cue):
                original_play(cue); calls.append(cue)
                operator.cue="thumb_rotate" if len(calls)==1 else "neutral"
            with patch.object(operator,"play",side_effect=play), \
                    patch("mh6_guided_calibration.time.monotonic",side_effect=clock.monotonic), \
                    patch("mh6_guided_calibration.time.sleep",side_effect=clock.sleep),redirect_stdout(io.StringIO()):
                finish_neutral_verification(operator,operator,mapper,recorder,rate=20.,move_seconds=.2,neutral_seconds=.3)
            segments=recorder.metadata["calibration_segments"]
            self.assertEqual([s["accepted"] for s in segments],[False,True])
            failures=recorder.metadata["calibration_report"]["verification_attempts"]
            self.assertIn("r",failures[0]["failed_components"])
            self.assertFalse(failures[0]["passed"])
            self.assertEqual(mapper.calibration.to_dict(),before)
            self.assertEqual(operator.calls,["neutral","neutral"])

    def test_resume_keeps_source_frames_and_endpoints_and_only_prompts_for_neutral(self):
        with tempfile.TemporaryDirectory() as folder:
            original, calibration, _, _ = self.capture(folder)
            # Emulate the first released file's missing failed verification label.
            with np.load(original.path,allow_pickle=False) as archive:
                values={k:archive[k] for k in archive.files}
            metadata=json.loads(values["metadata_json"].item())
            metadata["calibration_complete"]=False
            metadata["calibration_segments"]=[s for s in metadata["calibration_segments"] if s["phase"]!="verify_neutral"]
            metadata["calibration_report"]["passed"]=False
            values["metadata_json"]=np.asarray(json.dumps(metadata))
            pending=Path(folder)/"pending.npz"; np.savez_compressed(pending,**values)
            before=pending.read_bytes()
            clock=FakeClock(); operator=Operator(clock); new=Path(folder)/"recovered.npz"; mapping=Path(folder)/"mapping.json"
            with patch("visionpro_session.VisionProHandStream",return_value=operator), \
                    patch("visionpro_session.CalibrationVoice",return_value=operator), \
                    patch("mh6_guided_calibration.time.monotonic",side_effect=clock.monotonic), \
                    patch("mh6_guided_calibration.time.sleep",side_effect=clock.sleep),redirect_stdout(io.StringIO()):
                status=session_main(["record","--avp-ip","mock","--mode","calibration","--resume-calibration",str(pending),
                                     "--output",str(new),"--mapping-output",str(mapping),"--calibrate-seconds",".3",
                                     "--calibration-hold-seconds",".3","--calibration-move-seconds",".2"])
            self.assertEqual(status,0)
            self.assertEqual(operator.calls,["neutral","complete"])
            self.assertEqual(pending.read_bytes(),before)
            resumed=ReplayHandStream(str(new)); resumed.start()
            np.testing.assert_array_equal(resumed.transforms[:len(values["transforms"])],values["transforms"])
            np.testing.assert_allclose(resumed.timestamps[:len(values["timestamps"])],values["timestamps"],atol=1e-9)
            self.assertEqual(calibration_from_session(str(new)).to_dict(),calibration.to_dict())
            self.assertEqual(json.loads(mapping.read_text()),calibration.to_dict())
            with redirect_stdout(io.StringIO()),patch("visionpro_session.VisionProHandStream") as stream:
                self.assertEqual(session_main(["record","--avp-ip","mock","--mode","calibration",
                                               "--resume-calibration",str(pending),"--output",str(pending),"--overwrite"]),2)
                stream.assert_not_called()

    def test_complete_and_incomplete_gesture_sources_cannot_be_resumed(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder, _, _, _ = self.capture(folder)
            with self.assertRaisesRegex(ValueError,"pending"): load_pending_calibration(str(recorder.path))
            with np.load(recorder.path,allow_pickle=False) as archive:
                values={k:archive[k] for k in archive.files}
            metadata=json.loads(values["metadata_json"].item()); metadata["calibration_complete"]=False
            metadata["calibration_segments"]=[s for s in metadata["calibration_segments"] if s["phase"]!="opp_little"]
            values["metadata_json"]=np.asarray(json.dumps(metadata))
            pending=Path(folder)/"missing.npz"; np.savez_compressed(pending,**values)
            with self.assertRaisesRegex(ValueError,"opp_little"): load_pending_calibration(str(pending))


if __name__ == "__main__": unittest.main()
