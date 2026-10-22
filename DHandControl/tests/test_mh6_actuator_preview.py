"""Test configurations below are synthetic, never shipped as hardware defaults."""
import copy
from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mh6_hardware_config import HardwareConfig, FINGERS, PALM, CHANNELS
from mh6_actuator_preview import NineChannelPreview, prepare_packet
from mh6_palm_solver_adapter import PalmSolverAdapter
from mh6_teleop_run import main, parse_args, HardwareSender
from mh6_hand_session import HandSessionRecorder
from visionpro_stream import VisionProHandFrame


def configured():
    """Deliberately synthetic full-domain test settings, not machine calibration."""
    data = HardwareConfig.load().to_dict()
    for name, spec in data["actuators"].items():
        spec.update(min=0, max=2000 if name in FINGERS else 1000,
                    initial_position=20 if name in FINGERS else 500,
                    max_position_step=2000 if name in FINGERS else 1000)
        if name in FINGERS:
            spec.update(open_position=20, closed_position=1950)
        if name == "M4":
            spec.update(neutral_position=500, endpoint_position=700)
    return data


def result(curl=0., abduction=25., zero=10., endpoint=40.):
    normalized = min(max((abduction-zero)/(endpoint-zero),0),1)
    return {"low_dim": {**{key: curl for key in ("u_thumb", "u_index", "u_middle", "u_ring", "u_little")},
                        "u_thumb_abduction": normalized},
            "intent": {"thumb_abduction_raw": math.radians(abduction)},
            "m4_mapping": {"calibrated": True, "human_neutral_deg":zero,
                           "human_endpoint_deg":endpoint, "normalized":normalized}}


def roots(plus=(501., 502., 503.)):
    return {"candidates": [[490., 480., 470.], list(plus)],
            "candidate_branch_ids": ["minus_acos", "plus_acos"]}


class NineChannelSenderTest(unittest.TestCase):
    def test_prepared_positions_reach_driver_unchanged_including_m4(self):
        preview = NineChannelPreview(HardwareConfig(configured()))
        packet = prepare_packet(preview.propose(result(curl=.5), roots()))
        sender = HardwareSender("unused",115200)
        sender.hand = Mock()
        sender.hand.move_hand.return_value = True
        self.assertTrue(sender.send(packet))
        sender.hand.move_hand.assert_called_once_with(**packet,wait_status=False,verify_issued=True)
        sender.hand.map_finger_positions.assert_not_called()
        sender.hand.validate_palm_motor_positions.assert_not_called()
        self.assertEqual(packet["finger_positions"],[985]*5)
        self.assertEqual(packet["palm_positions"],[501,502,503,600])
        self.assertEqual(preview.generation,0)

    def test_incomplete_invalid_and_disconnected_packets_never_write(self):
        packet = prepare_packet(NineChannelPreview(HardwareConfig(configured())).propose(result(),roots()))
        sender = HardwareSender("unused",115200)
        self.assertFalse(sender.send(packet))
        sender.hand = Mock()
        self.assertFalse(sender.send(None))
        changes = [lambda p: p["palm_positions"].pop(),
                   lambda p: p["palm_positions"].__setitem__(3,500.5),
                   lambda p: p["finger_positions"].__setitem__(0,True),
                   lambda p: p["palm_positions"].__setitem__(0,1001),
                   lambda p: p["palm_ids"].__setitem__(3,3),
                   lambda p: p["palm_times"].__setitem__(0,0)]
        for change in changes:
            invalid = copy.deepcopy(packet); change(invalid)
            with self.assertRaises(ValueError): sender.send(invalid)
        sender.hand.move_hand.assert_not_called()

    def test_driver_failure_is_returned_without_committing_history(self):
        preview = NineChannelPreview(HardwareConfig(configured()))
        proposal = preview.propose(result(),roots())
        sender = HardwareSender("unused",115200)
        sender.hand = Mock()
        sender.hand.move_hand.return_value = False
        self.assertFalse(sender.send(prepare_packet(proposal)))
        self.assertEqual(preview.previous["M4"],500)
        self.assertEqual(preview.generation,0)


class HardwareConfigTest(unittest.TestCase):
    def test_default_is_explicit_plus_position_with_no_invented_positions(self):
        config = HardwareConfig.load()
        self.assertEqual(config.fixed_branch, "plus_acos")
        self.assertEqual(config.data["jump_guard"], "position")
        for spec in config.actuators.values():
            self.assertTrue(all(x is None for x in spec.values()))
        preview = NineChannelPreview(config).propose(result(), roots())
        self.assertTrue(all(x is None for x in preview["positions"].values()))
        self.assertTrue(all(x["status"] == "UNCONFIGURED" for x in preview["channels"].values()))
        self.assertIsNone(prepare_packet(preview))

    def test_invalid_or_misspelled_parameters_are_rejected(self):
        changes = [lambda d: d.update(extra=True),
                   lambda d: d.update(jump_guard="speed"),
                   lambda d: d["actuators"]["M1"].update(min=700, max=500),
                   lambda d: d["actuators"]["M1"].update(initial_position=1001),
                   lambda d: d["actuators"]["M1"].update(initial_position=True),
                   lambda d: d["actuators"]["F1"].update(open_position=-1),
                   lambda d: d["actuators"]["F1"].update(max_position_step=0),
                   lambda d: d["actuators"]["M4"].update(endpoint_position=500),
                   lambda d: d.update(m4_mapping={"human_neutral_deg":10,"human_endpoint_deg":40})]
        for change in changes:
            d = configured(); change(d)
            with self.subTest(change=change), self.assertRaises(ValueError):
                HardwareConfig(d)

    def test_configuration_is_copied_and_diagnostics_use_same_bounds(self):
        d = configured(); d["actuators"]["M3"].update(min=300, max=600)
        config = HardwareConfig(d); d["actuators"]["M3"]["max"] = 700
        adapter = PalmSolverAdapter(motor_limits=config.palm_diagnostic_limits())
        self.assertEqual(adapter.motor_limits[3], (300, 600))
        self.assertFalse(config.missing())


class NineChannelPreviewTest(unittest.TestCase):
    def test_plus_identity_is_used_independently_of_candidate_order(self):
        controller = NineChannelPreview(HardwareConfig(configured()))
        a = controller.propose(result(), roots())
        swapped = roots(); swapped["candidates"].reverse(); swapped["candidate_branch_ids"].reverse()
        b = controller.propose(result(), swapped)
        self.assertEqual(a["positions"], b["positions"])
        self.assertEqual(a["palm"]["selected_motor"], [501, 502, 503])

    def test_unavailable_plus_never_selects_minus(self):
        controller = NineChannelPreview(HardwareConfig(configured()))
        proposal = controller.propose(result(), {"candidates": [[490, 480, 470]], "candidate_branch_ids": ["minus_acos"]})
        self.assertEqual(proposal["palm"]["reason"], "FIXED_BRANCH_UNAVAILABLE")
        self.assertEqual(proposal["palm"]["selected_motor"], [500, 500, 500])

    def test_each_single_axis_limit_violation_holds_complete_palm(self):
        for axis in range(3):
            controller = NineChannelPreview(HardwareConfig(configured()))
            good = controller.propose(result(), roots()); controller.commit_preview(good)
            candidate = [510, 511, 512]; candidate[axis] = 1001
            held = controller.propose(result(), roots(candidate))
            self.assertEqual(held["palm"]["selected_motor"], [501, 502, 503])
            self.assertEqual(held["palm"]["reason"], "OUT_OF_LIMITS")
            self.assertTrue(all(not held["channels"][k]["accepted"] for k in PALM))

    def test_new_config_bounds_override_legacy_m3_range(self):
        d = configured(); d["actuators"]["M3"].update(min=300, max=600)
        p = NineChannelPreview(HardwareConfig(d)).propose(result(), roots((501,502,350)))
        self.assertTrue(p["palm"]["accepted"])
        self.assertEqual(p["positions"]["M3"],350)

    def test_hold_does_not_modify_mapping_or_solver_request(self):
        controller = NineChannelPreview(HardwareConfig(configured()))
        mapping, solution = result(), roots((501,502,1001))
        original = copy.deepcopy((mapping, solution))
        p = controller.propose(mapping, solution); controller.commit_preview(p)
        self.assertEqual((mapping, solution), original)

    def test_integer_position_cap_applies_to_complete_triplet(self):
        d = configured()
        for name in PALM: d["actuators"][name]["max_position_step"] = 3
        controller = NineChannelPreview(HardwareConfig(d))
        good = controller.propose(result(), roots((503.4, 502, 501)))
        self.assertTrue(good["palm"]["accepted"])
        controller.commit_preview(good)
        held = controller.propose(result(), roots((506.6, 503, 502)))
        self.assertEqual(held["palm"]["reason"], "POSITION_JUMP")
        self.assertEqual(held["palm"]["selected_motor"], [503, 502, 501])
        # No timestamp/velocity parameter influences this rule.
        self.assertNotIn("timestamp", held)

    def test_proposing_does_not_advance_reference_and_commit_is_explicit(self):
        controller = NineChannelPreview(HardwareConfig(configured()))
        proposal = controller.propose(result(), roots())
        self.assertEqual(controller.previous["M1"], 500)
        self.assertEqual(controller.propose(result(), roots())["generation"], 0)
        controller.commit_preview(proposal)
        self.assertEqual(controller.previous["M1"], 501)
        with self.assertRaises(ValueError): controller.commit_preview(proposal)

    def test_finger_and_m4_guards_are_independent_of_palm(self):
        d = configured(); d["actuators"]["F1"]["max_position_step"] = 5
        d["actuators"]["M4"]["max_position_step"] = 5
        p = NineChannelPreview(HardwareConfig(d)).propose(result(curl=1), roots())
        self.assertEqual(p["positions"]["F1"], 20)
        self.assertEqual(p["positions"]["F2"], 1950)
        self.assertEqual(p["positions"]["M4"], 500)
        self.assertEqual(p["palm"]["selected_motor"], [501, 502, 503])

    def test_m4_has_separate_angle_direction_and_motor_direction(self):
        d = configured()
        d["actuators"]["M4"]["endpoint_position"] = 300
        p = NineChannelPreview(HardwareConfig(d)).propose(result(abduction=-5,endpoint=-20), roots())
        self.assertAlmostEqual(p["m4_mapping"]["normalized"], .5)
        self.assertEqual(p["positions"]["M4"], 400)
        self.assertEqual(prepare_packet(p)["palm_positions"], [501, 502, 503, 400])

    def test_m4_does_not_depend_on_solver_thumb_input_and_clips_at_endpoints(self):
        controller = NineChannelPreview(HardwareConfig(configured()))
        x = result(abduction=100); x["low_dim"]["u_thumb_rotation"] = 0
        a = controller.propose(x, roots())
        x["low_dim"]["u_thumb_rotation"] = 1
        b = controller.propose(x, roots((520, 530, 540)))
        self.assertEqual(a["positions"]["M4"], b["positions"]["M4"])
        self.assertEqual(a["positions"]["M4"], 700)

    def test_missing_or_non_finite_m4_measurement_holds_only_m4(self):
        for value in (None, float("nan")):
            r = result(); r["intent"]["thumb_abduction_raw"] = value
            p = NineChannelPreview(HardwareConfig(configured())).propose(r, roots())
            self.assertEqual(p["positions"]["M4"], 500)
            self.assertEqual(p["channels"]["M4"]["reason"], "INVALID_INPUT")
            self.assertTrue(p["palm"]["accepted"])
            json.dumps(p, allow_nan=False)

    def test_missing_initial_reference_never_auto_accepts_first_solution(self):
        d = configured(); d["actuators"]["M2"]["initial_position"] = None
        p = NineChannelPreview(HardwareConfig(d)).propose(result(), roots())
        self.assertEqual(p["palm"]["reason"], "CONFIG_REQUIRED")
        self.assertEqual(p["positions"]["M2"], None)
        self.assertIsNone(prepare_packet(p))

    def test_packet_contains_nine_integer_actuators_on_their_respective_buses(self):
        p = NineChannelPreview(HardwareConfig(configured())).propose(result(curl=.5), roots())
        packet = prepare_packet(p)
        self.assertEqual(packet["finger_ids"], [1,2,3,4,5])
        self.assertEqual(packet["palm_ids"], [1,2,3,4])
        self.assertEqual(packet["palm_times"], [80]*4)
        self.assertTrue(all(type(x) is int for x in packet["finger_positions"]+packet["palm_positions"]))


class EntryPointTest(unittest.TestCase):
    def test_configured_runner_logs_m4_packet_and_uses_no_speed_or_fallback(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            config, session, log = (folder / n for n in ("hardware.json", "hand.npz", "preview.jsonl"))
            config.write_text(json.dumps(configured()))
            recorder = HandSessionRecorder(str(session))
            points = np.zeros((27,3)); points[9] = (0,.15,0)
            transforms = np.repeat(np.eye(4)[None],27,axis=0); transforms[:,:3,3] = points
            for t in (0,.05,.10):
                recorder.record(VisionProHandFrame(points,transforms,t,"right"),"teleop")
            recorder.save()
            mapping = result(curl=.5)
            mapping["palm_command"] = {"vertical":0.,"lateral":0.,"thumb_rotation_command":0.}
            with redirect_stdout(io.StringIO()), \
                    patch("mh6_teleop_run.MH6HandMapper.step",return_value=mapping), \
                    patch("mh6_teleop_run.print_mapping_line"), \
                    patch("mh6_teleop_run.PalmInputSlewLimiter",side_effect=AssertionError("speed limiter")), \
                    patch("mh6_teleop_run.PalmFallbackController",side_effect=AssertionError("fallback")), \
                    patch("mh6_teleop_run.HardwareSender",side_effect=AssertionError("hardware")):
                self.assertEqual(main(["--replay-session",str(session),"--replay-no-wait", "--use-default-calibration",
                                       "--hardware-config",str(config),"--filter-tau","0","--debug-log",str(log)]),0)
            rows = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual(len(rows),3)
            for row in rows:
                self.assertEqual(row["packet_preview"]["palm_ids"],[1,2,3,4])
                self.assertEqual(row["packet_preview"]["palm_positions"][-1],600)
                self.assertEqual(row["control"]["fixed_branch"],"plus_acos")
                self.assertEqual(row["control"]["palm"]["reason"],None)
                self.assertEqual(len(row["control"]["positions"]),9)
                self.assertFalse(row["control"]["hardware_enabled"])
                self.assertEqual(row["raw_mapping"],mapping)

    def test_hardware_stays_locked_before_any_connection(self):
        with patch("mh6_teleop_run.HardwareSender", side_effect=AssertionError("hardware")), \
                patch("mh6_teleop_run.VisionProHandStream", side_effect=AssertionError("stream")), redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--enable-hardware"]), 2)

    def test_speed_fallback_and_unguarded_options_cannot_bypass_position_policy(self):
        for options in (["--palm-input-speed","2"], ["--palm-fallback-delay","1"], ["--palm-no-jump-guard"]):
            with redirect_stdout(io.StringIO()), patch("mh6_teleop_run.VisionProHandStream", side_effect=AssertionError("stream")):
                self.assertEqual(main(options), 2)

    def test_default_cli_uses_unified_configuration(self):
        self.assertTrue(Path(parse_args([]).hardware_config).is_file())


if __name__ == "__main__":
    unittest.main()
