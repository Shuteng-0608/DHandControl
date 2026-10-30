from pathlib import Path
import sys
import unittest
from unittest.mock import Mock,patch
from contextlib import redirect_stdout
import io
import json
import tempfile
from threading import RLock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from mh6_finger_guard_status import FeedbackSummary,decode,main


def valid_registers(count=1,force=-20):
    data=[0]*86;data[:6]=[0x4647,1,0x1F00,0x0204,0,0]
    for i in range(5):
        data[6+16*i:22+16*i]=[0,1,20,20,20,force&65535,40,25,3,0,0,count,255,20,65535,1]
    return data


class FingerGuardStatusTest(unittest.TestCase):
    def test_signed_forces_positions_and_distinct_desired_planned_actual(self):
        data=[0]*86;data[:6]=[0x4647,1,3,0x0204,1,0]
        data[6:22]=[2,0,1500,900,895,65536-125,100,25,3,0,1,2,3,910,5,1]
        result=decode(data);finger=result["fingers"]["F1"]
        self.assertTrue(result["configured"])
        self.assertEqual(finger["state"],"HOLDING")
        self.assertEqual(finger["force_g"],-125)
        self.assertEqual(finger["actual_position"],895)
        self.assertEqual(finger["latest_desired_position"],1500)
        self.assertEqual(finger["planned_position"],900)
        self.assertEqual(finger["valid_reply_count"],65538)
        for invalid in ([],[0]*86):
            with self.assertRaises(ValueError):decode(invalid)

    def test_readonly_tool_never_clears_or_sends_targets(self):
        device=Mock();registers=[0]*86;registers[:4]=[0x4647,1,0,0x0204]
        device.client.read_holding_registers.return_value=Mock(registers=registers)
        device.transaction_lock=RLock()
        with patch("modbus_dev.DexHandControl",return_value=device) as factory,redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--port","memory-only"]),0)
        self.assertFalse(factory.call_args.kwargs["enable_motion"])
        device.client.read_holding_registers.assert_called_once_with(address=80,count=86,device_id=1)
        device.move_hand.assert_not_called();device.clear_error.assert_not_called()
        device.client.write_register.assert_not_called();device.client.write_registers.assert_not_called()
        device.stop_persistent_connection.assert_called_once()

    def test_summary_excludes_duplicate_stale_and_fault_feedback(self):
        summary=FeedbackSummary()
        first=decode(valid_registers())
        summary.observe(first);summary.observe(first)
        summary.observe(decode(valid_registers(2,-10)))
        stale=decode(valid_registers(3,30000))
        for finger in stale["fingers"].values():finger["feedback_fresh"]=False
        summary.observe(stale)
        faulty=decode(valid_registers(4,30000))
        for finger in faulty["fingers"].values():finger["error_flags"]=1
        summary.observe(faulty)
        result=summary.result()
        row=result["fingers"]["F1"]
        self.assertEqual(row["observations"],5)
        self.assertEqual(row["unique_valid_samples"],2)
        self.assertEqual(row["stale_observations"],1)
        self.assertEqual(row["fault_observations"],1)
        self.assertEqual((row["force_g_min"],row["force_g_max"],row["force_g_mean"]),(-20,-10,-15))
        self.assertEqual(row["current_ma_max"],40)
        self.assertFalse(result["load_thresholds_inferred"])
        self.assertFalse(result["force_sign_inferred"])

    def test_summary_without_actual_replies_does_not_treat_zeros_as_measurements(self):
        summary=FeedbackSummary()
        data=decode(valid_registers(0,0))
        summary.observe(data)
        row=summary.result()["fingers"]["F1"]
        self.assertEqual(row["unique_valid_samples"],0)
        self.assertIsNone(row["force_g_mean"])
        self.assertNotIn("force_g_min",row)

    def test_timed_capture_saves_raw_and_summary_without_motion(self):
        device=Mock();device.transaction_lock=RLock()
        device.client.read_holding_registers.side_effect=[
            Mock(registers=valid_registers()),Mock(registers=valid_registers(2,-10))]
        with tempfile.TemporaryDirectory() as folder:
            raw=Path(folder)/"feedback.jsonl";report=Path(folder)/"summary.json"
            with patch("modbus_dev.DexHandControl",return_value=device) as factory, \
                    patch("mh6_finger_guard_status.time.monotonic",side_effect=[0,.01,.21,.22,.3]), \
                    patch("mh6_finger_guard_status.time.sleep"),redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--port","memory-only","--duration","0.3",
                    "--output",str(raw),"--summary-output",str(report)]),0)
            frames=[json.loads(line) for line in raw.read_text().splitlines()]
            summary=json.loads(report.read_text())
        self.assertEqual(len(frames),2)
        self.assertEqual(summary["fingers"]["F1"]["force_g_mean"],-15)
        self.assertFalse(factory.call_args.kwargs["enable_motion"])
        self.assertEqual(device.client.read_holding_registers.call_count,2)
        device.move_hand.assert_not_called();device.clear_error.assert_not_called()
        device.client.write_register.assert_not_called();device.client.write_registers.assert_not_called()

    def test_invalid_capture_duration_never_connects(self):
        for duration in ("0","-1","nan","inf"):
            with self.subTest(duration=duration),patch("modbus_dev.DexHandControl") as factory, \
                    patch("sys.stderr",new_callable=io.StringIO):
                with self.assertRaises(SystemExit):main(["--duration",duration])
                factory.assert_not_called()

    def test_existing_summary_is_preserved_without_connecting(self):
        device=Mock()
        with tempfile.TemporaryDirectory() as folder:
            report=Path(folder)/"summary.json";report.write_text("existing data")
            with patch("modbus_dev.DexHandControl",return_value=device),redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--summary-output",str(report)]),1)
            self.assertEqual(report.read_text(),"existing data")
        device.start_persistent_connection.assert_not_called()
