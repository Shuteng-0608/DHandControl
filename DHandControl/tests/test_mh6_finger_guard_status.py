from pathlib import Path
import sys
import unittest
from unittest.mock import Mock,patch
from contextlib import redirect_stdout
import io

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from mh6_finger_guard_status import decode,main


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
        from threading import RLock
        device.transaction_lock=RLock()
        with patch("modbus_dev.DexHandControl",return_value=device) as factory,redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--port","memory-only"]),0)
        self.assertFalse(factory.call_args.kwargs["enable_motion"])
        device.client.read_holding_registers.assert_called_once_with(address=80,count=86,device_id=1)
        device.move_hand.assert_not_called();device.clear_error.assert_not_called()
        device.client.write_register.assert_not_called();device.client.write_registers.assert_not_called()
        device.stop_persistent_connection.assert_called_once()
