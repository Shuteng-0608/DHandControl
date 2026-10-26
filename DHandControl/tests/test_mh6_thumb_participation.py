"""Independent thumb intent: no tripod contributions, unchanged direct axes."""
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
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from mh6_mapping import MappingCalibration, MH6HandMapper
from mh6_intent_fusion import ThumbParticipationFusion, DEFAULT_INTENT_FUSION_PATH
from mh6_guided_calibration import apply_stage
from mh6_hand_session import HandSessionRecorder
from mh6_teleop_run import main, parse_args
from visionpro_stream import VisionProHandFrame
from test_mh6_guided_calibration import pose


def human_calibration():
    mapper=MH6HandMapper()
    neutral=apply_stage(mapper,"neutral",[[pose("neutral")]*10])
    for phase in ("grasp","thumb_rotate","thumb_abduct","opp_index","opp_middle","opp_ring","opp_little"):
        rounds=2 if phase in ("grasp","thumb_rotate","thumb_abduct") else 1
        apply_stage(mapper,phase,[[pose(phase)]*10 for _ in range(rounds)],neutral)
    return mapper.calibration


class ThumbParticipationTest(unittest.TestCase):
    def test_configuration_and_weight_normalization(self):
        profile=ThumbParticipationFusion(thumb_rotation_weight=2,thumb_abduction_weight=1)
        self.assertAlmostEqual(profile.participation(1,0),2/3)
        self.assertAlmostEqual(profile.participation(0,1),1/3)
        self.assertEqual(profile.participation(1,1),1.)
        self.assertEqual(profile.participation(-1,-1),0.)
        self.assertEqual(ThumbParticipationFusion.load().to_dict()["vertical_thumb_gain"],.1)
        for values in ({"thumb_rotation_weight":-1},{"vertical_thumb_gain":2},
                       {"lateral_thumb_gain":float("nan")},{"thumb_abduction_weight":True},
                       {"thumb_rotation_weight":0,"thumb_abduction_weight":0},
                       {"vertical_opposition_gain":0.}, {"lateral_power_gain":.1}):
            with self.assertRaises(ValueError): ThumbParticipationFusion(**values)
        with self.assertRaisesRegex(ValueError,"M4 calibration"):
            profile.participation(.5,None)

    def test_fusion_schema_rejects_misspelled_fields(self):
        data=ThumbParticipationFusion().to_dict();data["extra_gain"]=1
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"fusion.json";path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):ThumbParticipationFusion.load(path)
            data.pop("extra_gain");data["schema_version"]=1
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,"schema_version 2"):ThumbParticipationFusion.load(path)

    def test_tripod_is_absent_and_old_tripod_gains_cannot_influence_commands(self):
        config=MappingCalibration(thumb_abduction_neutral_deg=0,thumb_abduction_endpoint_deg=30,
                                  vertical_tripod_gain=100,thumb_tripod_compensation_gain=100)
        mapper=MH6HandMapper(config,intent_fusion=ThumbParticipationFusion())
        curls=dict(thumb=1.,index=1.,middle=1.,ring=0.,little=0.)
        opposition=dict(p_I=1.,p_M=1.,p_R=0.,p_L=0.)
        intent=mapper.compute_grasp_intents(curls,opposition,thumb_rotation_measured=0.,thumb_abduction_measured=0.)
        self.assertFalse(set(intent)&{"tripod_flexion","tripod_proximity","tripod_precision","t"})
        command=mapper.compute_palm_commands(intent,0.)
        self.assertAlmostEqual(command["vertical"],.8*intent["power_grasp"]+.1*intent["opposition_cross"])
        self.assertAlmostEqual(command["lateral"],.05*intent["power_grasp"]+.5*intent["opposition_cross"])
        self.assertEqual(command["thumb_rotation_compensation"],0.)
        self.assertEqual(command["thumb_rotation_command"],0.)

    def test_both_thumb_angles_contribute_monotonically_to_both_folds(self):
        config=MappingCalibration()
        profile=ThumbParticipationFusion()
        for fixed in (0.,.5,1.):
            for axis in (0,1):
                previous=(-1.,-1.)
                for variable in np.linspace(0,1,11):
                    r,a=(variable,fixed) if axis==0 else (fixed,variable)
                    thumb=profile.participation(r,a)
                    out=profile.commands(dict(power_grasp=0.,opposition_cross=0.,thumb_participation=thumb),r,config)
                    self.assertGreaterEqual(out["vertical"],previous[0])
                    self.assertGreaterEqual(out["lateral"],previous[1])
                    self.assertAlmostEqual(out["vertical"],.1*thumb)
                    self.assertAlmostEqual(out["lateral"],.45*thumb)
                    self.assertEqual(out["thumb_rotation_command"],r)
                    previous=(out["vertical"],out["lateral"])

    def test_clip_and_contribution_logging_remain_explicit(self):
        out=ThumbParticipationFusion().commands(dict(power_grasp=1.,opposition_cross=1.,thumb_participation=1.),.7,MappingCalibration())
        self.assertEqual((out["vertical"],out["lateral"]),(1.,1.))
        self.assertAlmostEqual(out["vertical_before_clip"],1.)
        self.assertAlmostEqual(out["lateral_before_clip"],1.)
        self.assertAlmostEqual(out["vertical_from_opposition"],.1)
        self.assertEqual(out["thumb_rotation_command"],.7)

    def test_old_mapping_gains_do_not_override_fusion_and_inputs_are_bounded(self):
        calibration=MappingCalibration(vertical_power_gain=.75,lateral_power_gain=.2,lateral_opposition_gain=.4)
        out=ThumbParticipationFusion().commands(dict(power_grasp=.4,opposition_cross=1.5,thumb_participation=.5),.8,calibration)
        self.assertAlmostEqual(out["vertical"],.47)
        self.assertAlmostEqual(out["lateral"],.745)
        self.assertEqual(out["thumb_rotation_command"],.8)

    def test_each_intent_has_requested_gain_and_mixed_inputs_do_not_saturate_early(self):
        profile=ThumbParticipationFusion.load()
        for active, expected in (("power_grasp",(.8,.05)),
                                 ("opposition_cross",(.1,.5)),
                                 ("thumb_participation",(.1,.45))):
            intent=dict(power_grasp=0.,opposition_cross=0.,thumb_participation=0.)
            intent[active]=1.
            out=profile.commands(intent,0.,MappingCalibration())
            self.assertAlmostEqual(out["vertical"],expected[0])
            self.assertAlmostEqual(out["lateral"],expected[1])
        for g in np.linspace(0,1,11):
            for o in np.linspace(0,1,11):
                for p in np.linspace(0,1,11):
                    out=profile.commands(dict(power_grasp=g,opposition_cross=o,thumb_participation=p),0.,MappingCalibration())
                    for axis in ("vertical","lateral"):
                        self.assertGreaterEqual(out[axis+"_before_clip"],0.)
                        self.assertLessEqual(out[axis+"_before_clip"],1.+1e-12)
                        if min(g,o,p)<1.:
                            self.assertLess(out[axis],1.)

    def test_real_mapping_preserves_calibration_fingers_m4_and_base_intents(self):
        calibration=human_calibration();before=copy.deepcopy(calibration.to_dict())
        old=MH6HandMapper(calibration)
        new=MH6HandMapper(calibration,intent_fusion=ThumbParticipationFusion())
        for phase in ("neutral","grasp","thumb_rotate","thumb_abduct","opp_index","opp_middle","opp_ring","opp_little"):
            baseline,changed=old.step(pose(phase)),new.step(pose(phase))
            self.assertEqual(baseline["finger_bending"],changed["finger_bending"])
            self.assertEqual(baseline["m4_mapping"],changed["m4_mapping"])
            self.assertEqual(baseline["low_dim"]["u_thumb_abduction"],changed["low_dim"]["u_thumb_abduction"])
            for key in ("power_grasp","opposition_cross"):
                self.assertEqual(baseline["grasp_intent"][key],changed["grasp_intent"][key])
            self.assertNotIn("tripod_precision",changed["intent"])
            self.assertEqual(changed["palm_command"]["thumb_rotation_command"],changed["low_dim"]["u_thumb_rotation"])
        self.assertEqual(calibration.to_dict(),before)
        natural=new.step(pose("neutral"))["palm_command"]
        self.assertEqual((natural["vertical"],natural["lateral"],natural["thumb_rotation_command"]),(0.,0.,0.))

    def test_new_policy_requires_inward_range_and_cli_default_is_new(self):
        with self.assertRaises(ValueError):MH6HandMapper(palm_motion_range="signed",intent_fusion=ThumbParticipationFusion())
        self.assertEqual(parse_args([]).intent_fusion,"thumb_participation")
        with patch("mh6_teleop_run.VisionProHandStream",side_effect=AssertionError("AVP")),redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--use-default-calibration"]),2)

    def test_default_cli_logs_new_intent_and_original_finger_m4_channels_offline(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);mapping=folder/"mapping.json";session=folder/"hand.npz";log=folder/"preview.jsonl"
            calibration=human_calibration();calibration.save_json(mapping)
            recorder=HandSessionRecorder(session)
            for i,phase in enumerate(("neutral","thumb_abduct","neutral")):
                points=pose(phase);tf=np.tile(np.eye(4),(27,1,1));tf[:,:3,3]=points
                recorder.record(VisionProHandFrame(points,tf,i*.05,"right"),"teleop")
            recorder.save()
            with patch("mh6_teleop_run.HardwareSender",side_effect=AssertionError("hardware")),redirect_stdout(io.StringIO()):
                status=main(["--replay-session",str(session),"--mapping-calibration",str(mapping),"--replay-no-wait",
                             "--filter-tau","0","--debug-log",str(log)])
            self.assertEqual(status,0)
            rows=[json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual(rows[0]["runtime_config"]["intent_fusion"],"thumb_participation")
            self.assertAlmostEqual(rows[0]["runtime_config"]["intent_fusion_parameters"]["vertical_opposition_gain"],.1)
            self.assertAlmostEqual(rows[1]["raw_mapping"]["grasp_intent"]["thumb_participation"],.5)
            self.assertNotIn("tripod_precision",rows[1]["raw_mapping"]["intent"])
            self.assertFalse(rows[1]["execution"]["hardware_enabled"])
            self.assertAlmostEqual(rows[1]["raw_mapping"]["palm_command"]["vertical_from_thumb"],.05)


if __name__=="__main__":unittest.main()
