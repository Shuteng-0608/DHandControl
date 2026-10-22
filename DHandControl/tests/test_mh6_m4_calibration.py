"""M4 human calibration stays in Mapping; hardware settings stay motor-only."""
from contextlib import redirect_stdout
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mh6_mapping import MappingCalibration, MH6HandMapper
from mh6_guided_calibration import (apply_stage, calibration_from_staged_stream, verify_neutral,
                                    finish_neutral_verification, PROTOCOL, NeutralVerificationError)
from mh6_hand_session import HandSessionRecorder, ReplayHandStream
from mh6_actuator_preview import NineChannelPreview, prepare_packet
from mh6_hardware_config import HardwareConfig
from mh6_calibration_voice import CalibrationVoice
from mh6_teleop_run import main as teleop_main
from visionpro_stream import VisionProHandFrame
import test_mh6_guided_calibration as fixtures
import test_mh6_actuator_preview as output_fixtures


def abduct(degrees):
    points = fixtures.pose("neutral")
    length = np.linalg.norm(points[2, :2]-points[1, :2])
    points[2, 2] = length*np.tan(np.radians(degrees))
    return points


class M4CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.mapper = MH6HandMapper()
        self.zero = apply_stage(self.mapper,"neutral",[[fixtures.pose("neutral")]*10])

    def capture(self, folder):
        return fixtures.CalibrationTest.capture(self, folder)

    def test_natural_reference_and_each_gesture_are_isolated(self):
        self.assertAlmostEqual(self.mapper.calibration.thumb_abduction_neutral_deg,0.)
        self.assertIsNone(self.mapper.calibration.thumb_abduction_endpoint_deg)
        before = self.mapper.calibration.to_dict()
        apply_stage(self.mapper,"thumb_abduct",[[abduct(30)]*10]*2,self.zero)
        after = self.mapper.calibration.to_dict()
        self.assertAlmostEqual(after["mapping"]["thumb_abduction_endpoint_deg"],30.)
        before["mapping"]["thumb_abduction_endpoint_deg"] = after["mapping"]["thumb_abduction_endpoint_deg"]
        self.assertEqual(before,after)
        endpoint = self.mapper.calibration.thumb_abduction_endpoint_deg
        apply_stage(self.mapper,"grasp",[[fixtures.pose("grasp")]*10]*2,self.zero)
        self.assertEqual(self.mapper.calibration.thumb_abduction_endpoint_deg,endpoint)

    def test_both_directions_normalize_natural_midpoint_and_endpoint(self):
        for endpoint in (30.,-30.):
            apply_stage(self.mapper,"thumb_abduct",[[abduct(endpoint)]*10]*2,self.zero)
            for angle, expected in ((0,0),(endpoint/2,.5),(endpoint,1),(-endpoint,0),(endpoint*1.5,1)):
                result = self.mapper.step(abduct(angle))
                self.assertAlmostEqual(result["low_dim"]["u_thumb_abduction"],expected)
                self.assertTrue(result["m4_mapping"]["calibrated"])

    def test_combined_thumb_stage_updates_both_or_neither(self):
        before = self.mapper.calibration.to_dict()
        with self.assertRaisesRegex(ValueError,"M4偏转"):
            apply_stage(self.mapper,"thumb_rotate",[[fixtures.pose("thumb_rotate_flat")]*10]*2,
                        self.zero,combine_thumb=True)
        self.assertEqual(self.mapper.calibration.to_dict(),before)
        report = apply_stage(self.mapper,"thumb_rotate",[[fixtures.pose("thumb_rotate")]*10]*2,
                             self.zero,combine_thumb=True)
        self.assertEqual(set(report),{"rotation:thumb","abduction:thumb"})
        self.assertAlmostEqual(self.mapper.calibration.thumb_rotation_closed,
                               report["rotation:thumb"]["endpoint"])
        self.assertAlmostEqual(self.mapper.calibration.thumb_abduction_endpoint_deg,45.)

    def test_neutral_recalibration_clears_old_m4_endpoint(self):
        apply_stage(self.mapper,"thumb_abduct",[[abduct(30)]*10]*2,self.zero)
        self.mapper.calibrate_neutral([abduct(5)]*10,include_thumb_abduction=True)
        self.assertAlmostEqual(self.mapper.calibration.thumb_abduction_neutral_deg,5)
        self.assertFalse(self.mapper.calibration.has_thumb_abduction_calibration)
        self.assertIsNone(self.mapper.step(abduct(5))["low_dim"]["u_thumb_abduction"])

    def test_insufficient_noisy_and_inconsistent_endpoints_do_not_update(self):
        cases = ([[abduct(2)]*10]*2,
                 [[abduct(30)]*10,[abduct(10)]*10],
                 [[abduct(0),abduct(30)]*10]*2)
        for rounds in cases:
            before = self.mapper.calibration.to_dict()
            with self.assertRaises(ValueError):
                apply_stage(self.mapper,"thumb_abduct",rounds,self.zero)
            self.assertEqual(before,self.mapper.calibration.to_dict())

    def test_final_verification_detects_wrong_side_even_if_normalized_value_is_zero(self):
        apply_stage(self.mapper,"thumb_abduct",[[abduct(30)]*10]*2,self.zero)
        self.assertEqual(self.mapper.step(abduct(-10))["low_dim"]["u_thumb_abduction"],0)
        with self.assertRaises(NeutralVerificationError) as caught:
            verify_neutral(self.mapper,[abduct(-10)]*10)
        self.assertIn("m4",caught.exception.diagnostics["failed_components"])
        self.assertAlmostEqual(caught.exception.diagnostics["m4_return_error"],1/3)
        self.assertAlmostEqual(verify_neutral(self.mapper,[abduct(0)]*10)["m4_return_error"],0)

    def test_version_two_json_roundtrip_and_missing_endpoint_cannot_drive_m4(self):
        apply_stage(self.mapper,"thumb_abduct",[[abduct(30)]*10]*2,self.zero)
        data = self.mapper.calibration.to_dict()
        self.assertEqual(data["format_version"],2)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"mapping.json"
            self.mapper.calibration.save_json(path)
            loaded = MappingCalibration.load_json(path)
            self.assertEqual(loaded.to_dict(),data)
            self.assertAlmostEqual(MH6HandMapper(loaded).step(abduct(15))["low_dim"]["u_thumb_abduction"],.5)
        old = MappingCalibration.from_dict(MappingCalibration().to_dict())
        self.assertFalse(old.has_thumb_abduction_calibration)
        output = MH6HandMapper(old).step(abduct(15))
        config = HardwareConfig(output_fixtures.configured())
        proposal = NineChannelPreview(config).propose(output,output_fixtures.roots())
        self.assertEqual(proposal["channels"]["M4"]["status"],"UNCONFIGURED")
        self.assertIsNone(prepare_packet(proposal))

    def test_invalid_human_references_and_unknown_json_fields_are_rejected(self):
        apply_stage(self.mapper,"thumb_abduct",[[abduct(30)]*10]*2,self.zero)
        data = self.mapper.calibration.to_dict()
        for key,value in (("thumb_abduction_neutral_deg",True),
                          ("thumb_abduction_endpoint_deg",float("nan")),
                          ("thumb_abduction_endpoint_deg",91),
                          ("thumb_abduction_endpoint_deg",0),
                          ("thumb_abduction_neutral_deg",None)):
            changed = copy.deepcopy(data); changed["mapping"][key] = value
            with self.assertRaises(ValueError): MappingCalibration.from_dict(changed)
        changed = copy.deepcopy(data); changed["mapping"]["invented_field"] = 1
        with self.assertRaises(ValueError): MappingCalibration.from_dict(changed)

    def test_new_capture_uses_same_two_thumb_windows_for_both_components(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder,calibration,report,operator = self.capture(folder)
            self.assertEqual(recorder.metadata["calibration_protocol"],"staged_v3")
            self.assertEqual(operator.calls.count("thumb_rotate"),2)
            self.assertNotIn("thumb_abduct",operator.calls)
            self.assertEqual(set(report["stages"]["thumb_rotate"]),{"rotation:thumb","abduction:thumb"})
            self.assertTrue(calibration.has_thumb_abduction_calibration)
            self.assertAlmostEqual(calibration.thumb_abduction_endpoint_deg,45)
            self.assertEqual(report["verification"]["m4_return_error"],0)
            stream = ReplayHandStream(recorder.path); stream.start()
            self.assertEqual(calibration_from_staged_stream(stream).to_dict(),calibration.to_dict())
            stream.metadata["calibration_segments"] = [s for s in stream.metadata["calibration_segments"] if s["phase"]!="thumb_rotate"]
            with self.assertRaisesRegex(ValueError,"thumb_rotate"):
                calibration_from_staged_stream(stream)

    def test_legacy_staged_reconstruction_never_invents_m4_endpoints(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder,_,_,_ = self.capture(folder)
            stream = ReplayHandStream(recorder.path); stream.start()
            stream.metadata["calibration_protocol"] = "staged_v1"
            stream.metadata["calibration_segments"] = [s for s in stream.metadata["calibration_segments"] if s["phase"]!="thumb_abduct"]
            calibration = calibration_from_staged_stream(stream)
            self.assertFalse(calibration.has_thumb_abduction_calibration)
            self.assertEqual(calibration.to_dict()["format_version"],1)

    def test_version_two_separate_stage_still_reconstructs(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder,expected,_,_ = self.capture(folder)
            stream = ReplayHandStream(recorder.path); stream.start()
            accepted = [s for s in stream.metadata["calibration_segments"] if s["accepted"]]
            transforms, phases, segments = [], [], []
            from mh6_guided_calibration import stages_for_protocol
            for phase in (*stages_for_protocol("staged_v2"),"verify_neutral"):
                source_phase = "thumb_rotate" if phase == "thumb_abduct" else phase
                for segment in accepted:
                    if segment["phase"] != source_phase: continue
                    start = len(transforms)
                    window = stream.transforms[segment["start"]:segment["end"]]
                    transforms.extend(window); phases.extend([phase]*len(window))
                    segments.append(dict(phase=phase,start=start,end=len(transforms),accepted=True))
            stream.transforms = np.asarray(transforms)
            stream.phases = np.asarray(phases)
            stream.metadata.update(calibration_protocol="staged_v2",calibration_segments=segments)
            self.assertEqual(calibration_from_staged_stream(stream).to_dict(),expected.to_dict())

    def test_m4_stage_failure_never_completes_or_substitutes_an_endpoint(self):
        clock = fixtures.FakeClock(); operator = fixtures.Operator(clock)
        original_play = operator.play
        def play(cue):
            original_play(cue)
            if cue == "thumb_rotate": operator.cue = "thumb_rotate_flat"
        recorder = HandSessionRecorder("unused.npz")
        with patch.object(operator,"play",side_effect=play), \
                patch("mh6_guided_calibration.time.monotonic",side_effect=clock.monotonic), \
                patch("mh6_guided_calibration.time.sleep",side_effect=clock.sleep), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError,"范围不足"):
                fixtures.run_guided_calibration(operator,operator,recorder=recorder,
                                                rate=20,neutral_seconds=.3,hold_seconds=.3,move_seconds=.2)
        self.assertFalse(recorder.metadata["calibration_complete"])
        self.assertNotIn("complete",operator.calls)
        failures = recorder.metadata["calibration_report"]["attempt_errors"]
        self.assertEqual([x["phase"] for x in failures],["thumb_rotate","thumb_rotate"])

    def test_saved_mapping_drives_m4_through_real_replay_without_human_hardware_parameters(self):
        apply_stage(self.mapper,"thumb_abduct",[[abduct(30)]*10]*2,self.zero)
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            mapping, hardware, session, log = (folder/name for name in ("mapping.json","hardware.json","hand.npz","preview.jsonl"))
            self.mapper.calibration.save_json(mapping)
            hardware.write_text(json.dumps(output_fixtures.configured()))
            self.assertNotIn("m4_mapping",json.loads(hardware.read_text()))
            recorder = HandSessionRecorder(session)
            for i, angle in enumerate((0,15,30)):
                points = abduct(angle)
                transforms = np.tile(np.eye(4),(27,1,1)); transforms[:,:3,3] = points
                recorder.record(VisionProHandFrame(points,transforms,i*.05,"right"),"teleop")
            recorder.save()
            with redirect_stdout(io.StringIO()),patch("mh6_teleop_run.HardwareSender",side_effect=AssertionError("hardware")):
                self.assertEqual(teleop_main(["--replay-session",str(session),"--mapping-calibration",str(mapping),
                    "--hardware-config",str(hardware),"--replay-no-wait","--filter-tau","0","--debug-log",str(log)]),0)
            rows = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual([r["control"]["positions"]["M4"] for r in rows],[500,600,700])
            self.assertTrue(all(r["control"]["m4_mapping"]["calibrated"] for r in rows))
            self.assertTrue(all(not r["control"]["hardware_enabled"] for r in rows))

    def test_m4_neutral_failure_retries_only_verification_and_keeps_endpoints(self):
        apply_stage(self.mapper,"thumb_abduct",[[abduct(30)]*10]*2,self.zero)
        before = self.mapper.calibration.to_dict()
        clock = fixtures.FakeClock(); operator = fixtures.Operator(clock)
        original_play = operator.play
        def play(cue):
            original_play(cue)
            operator.cue = "thumb_abduct" if len(operator.calls)==1 else "neutral"
        recorder = HandSessionRecorder("unused.npz",metadata={"calibration_protocol":PROTOCOL,
            "calibration_segments":[],"calibration_report":{"passed":False}})
        with patch.object(operator,"play",side_effect=play), \
                patch("mh6_guided_calibration.time.monotonic",side_effect=clock.monotonic), \
                patch("mh6_guided_calibration.time.sleep",side_effect=clock.sleep), redirect_stdout(io.StringIO()):
            finish_neutral_verification(operator,operator,self.mapper,recorder,rate=20,move_seconds=.2,neutral_seconds=.3)
        self.assertEqual(operator.calls,["neutral","neutral"])
        self.assertEqual(self.mapper.calibration.to_dict(),before)
        self.assertIn("m4",recorder.metadata["calibration_report"]["verification_attempts"][0]["failed_components"])

    def test_missing_m4_recording_uses_text_without_disabling_other_voice(self):
        original_exists = Path.exists
        with patch("mh6_calibration_voice.Path.exists",lambda path: path.name != "thumb_abduct.wav" and original_exists(path)), \
                patch("mh6_calibration_voice.shutil.which",return_value="paplay"), \
                patch("mh6_calibration_voice.subprocess.run") as player, redirect_stdout(io.StringIO()) as printed:
            voice = CalibrationVoice()
            voice.play("thumb_abduct"); voice.play("neutral")
            self.assertIn("M4",printed.getvalue())
            self.assertEqual(player.call_count,1)


if __name__ == "__main__":
    unittest.main()
