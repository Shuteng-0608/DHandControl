"""Send-only integration uses mocked AVP input and motor I/O exclusively."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mh6_actuator_io import ActuatorIO
from mh6_actuator_preview import NineChannelPreview
from mh6_hardware_config import HardwareConfig
from mh6_mapping import MappingCalibration
from mh6_teleop_run import HardwareSender, main
from test_mh6_actuator_preview import configured, result, roots


class SendOnlyTest(unittest.TestCase):
    def driver(self):
        return SimpleNamespace(
            read_finger_status=Mock(side_effect=AssertionError("feedback forbidden")),
            read_palm_status=Mock(side_effect=AssertionError("feedback forbidden")),
            last_move_result=None)

    def test_success_and_failed_send_never_poll_or_commit_failed_targets(self):
        planner = NineChannelPreview(HardwareConfig(configured()))
        sender = SimpleNamespace(hand=self.driver(), send=Mock(return_value=True))
        control = ActuatorIO(planner, sender, feedback_enabled=False)
        _, execution, snapshot = control.cycle(result(curl=.5), roots())
        self.assertTrue(execution["write_acknowledged"])
        self.assertFalse(snapshot["enabled"])
        previous = planner.previous.copy()
        sender.send.return_value = False
        control.cycle(result(curl=.8), roots())
        control.cycle(result(curl=.9), roots())
        self.assertEqual(sender.send.call_count, 2)
        self.assertEqual(planner.previous, previous)
        sender.hand.read_finger_status.assert_not_called()
        sender.hand.read_palm_status.assert_not_called()

    def test_startup_clears_five_fingers_without_motion_or_status_polling(self):
        sender = HardwareSender("memory-only", 115200, enable_motion=True)
        with patch("modbus_dev.DexHandControl") as factory:
            driver = factory.return_value
            sender.start()
            sender.stop()
        self.assertTrue(factory.call_args.kwargs["enable_motion"])
        self.assertEqual([c.args[0] for c in driver.clear_error.call_args_list], list(range(1, 6)))
        driver.move_hand.assert_not_called()
        driver.read_finger_status.assert_not_called()
        driver.read_palm_status.assert_not_called()
        driver.stop_persistent_connection.assert_called_once()

    def test_failed_clear_closes_and_prevents_motion(self):
        sender = HardwareSender("memory-only", 115200, enable_motion=True)
        with patch("modbus_dev.DexHandControl") as factory:
            driver = factory.return_value
            driver.clear_error.side_effect = [True, False]
            with self.assertRaisesRegex(RuntimeError, "F2"):
                sender.start()
        self.assertIsNone(sender.hand)
        driver.stop_persistent_connection.assert_called_once()
        driver.move_hand.assert_not_called()

    def test_missing_thresholds_fail_before_avp_or_serial(self):
        # Do not depend on a user's untracked calibration in the test suite.
        with tempfile.TemporaryDirectory() as folder:
            mapping = Path(folder) / "mapping.json"
            config = Path(folder) / "hardware.json"
            incomplete = configured()
            incomplete["actuators"]["F1"]["max_position_step"] = None
            config.write_text(json.dumps(incomplete))
            MappingCalibration(thumb_abduction_neutral_deg=10,
                               thumb_abduction_endpoint_deg=40).save_json(str(mapping))
            with patch("mh6_teleop_run.VisionProHandStream", side_effect=AssertionError("AVP")), \
                    patch("mh6_teleop_run.HardwareSender", side_effect=AssertionError("serial")), \
                    redirect_stdout(io.StringIO()) as output:
                status = main(["--enable-hardware", "--no-feedback", "--confirm-initial-position",
                               "--mapping-calibration", str(mapping), "--hardware-config", str(config)])
            self.assertEqual(status, 2)
            self.assertIn("max_position_step", output.getvalue())

    def test_live_loop_sends_and_logs_without_reading_feedback(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            mapping, config, log = [folder / name for name in ("mapping.json", "hardware.json", "send.jsonl")]
            MappingCalibration(thumb_abduction_neutral_deg=10,
                               thumb_abduction_endpoint_deg=40).save_json(str(mapping))
            config.write_text(json.dumps(configured()))
            frame = SimpleNamespace(points=np.zeros((27, 3)), timestamp=1.)
            stream = SimpleNamespace(start=Mock(), stop=Mock(),
                                     get_latest_frame=Mock(side_effect=[frame, KeyboardInterrupt]))
            sender = SimpleNamespace(hand=self.driver(), start=Mock(), stop=Mock(), send=Mock(return_value=True))
            mapping_result = result(curl=.5)
            mapping_result["palm_command"] = {"vertical":0., "lateral":0., "thumb_rotation_command":0.}
            with patch("mh6_teleop_run.VisionProHandStream", return_value=stream), \
                    patch("mh6_teleop_run.HardwareSender", return_value=sender), \
                    patch("mh6_teleop_run.MH6HandMapper.step", return_value=mapping_result), \
                    patch("mh6_teleop_run.PalmSolverAdapter.solve_motor_from_teleop", return_value=roots()), \
                    patch("mh6_teleop_run.print_mapping_line"), \
                    patch("mh6_teleop_run.time.sleep"), redirect_stdout(io.StringIO()):
                status = main(["--enable-hardware", "--no-feedback", "--confirm-initial-position",
                               "--mapping-calibration", str(mapping), "--hardware-config", str(config),
                               "--filter-tau", "0", "--debug-log", str(log)])
            self.assertEqual(status, 0)
            sender.send.assert_called_once()
            sender.stop.assert_called_once()
            sender.hand.read_finger_status.assert_not_called()
            sender.hand.read_palm_status.assert_not_called()
            row = json.loads(log.read_text())
            self.assertTrue(row["execution"]["write_acknowledged"])
            self.assertTrue(row["control"]["hardware_enabled"])
            self.assertFalse(row["feedback"]["enabled"])


if __name__ == "__main__":
    unittest.main()
