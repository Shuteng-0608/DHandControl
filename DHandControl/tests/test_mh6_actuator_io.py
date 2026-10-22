"""Hardware I/O tests use injected memory drivers only; never construct serial."""
from contextlib import redirect_stdout
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from mh6_actuator_io import ActuatorIO, FeedbackPoller
from mh6_actuator_preview import NineChannelPreview
from mh6_hardware_config import HardwareConfig, CHANNELS, FINGERS
from mh6_feedback_read import main as feedback_main
from mh6_teleop_run import HardwareSender
from modbus_dev import DexHandControl
from mh6_trajectory_audit import memory_sender
from test_mh6_actuator_preview import configured, result, roots


class Clock:
    def __init__(self): self.now = 100.
    def __call__(self): return self.now
    def advance(self,seconds=.101): self.now += seconds


def feedback(device_id, finger=True, **changes):
    return {"id":device_id,"query_ok":True,"checksum_ok":True,
            "firmware_status":0x93 if finger else 0x94,"response_error_code":0,
            "current_position":100 if finger else 500,"error_flags":0 if finger else None,
            **changes}


class FeedbackTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.driver = SimpleNamespace(read_finger_status=Mock(side_effect=lambda i:feedback(i)),
                                      read_palm_status=Mock(side_effect=lambda i:feedback(i,False)))

    def test_round_robin_rate_and_no_catchup_bursts(self):
        poller = FeedbackPoller(self.driver,10,self.clock)
        names = []
        for _ in range(9):
            sample = poller.poll(); names.append(sample["channel"])
            self.assertIsNone(poller.poll())
            self.clock.advance()
        self.assertEqual(names,list(CHANNELS))
        self.assertEqual(self.driver.read_finger_status.call_count,5)
        self.assertEqual(self.driver.read_palm_status.call_count,4)
        self.clock.advance(5)
        self.assertIsNotNone(poller.poll())
        self.assertIsNone(poller.poll())

    def test_failed_read_does_not_refresh_previous_valid_sample(self):
        poller = FeedbackPoller(self.driver,10,self.clock)
        first = poller.poll()
        poller.index = 0; self.clock.advance(1)
        self.driver.read_finger_status.return_value = None
        self.driver.read_finger_status.side_effect = None
        self.assertFalse(poller.poll()["valid"])
        state = poller.snapshot()["channels"]["F1"]
        self.assertEqual(state["last_valid"],first)
        self.assertAlmostEqual(state["age_seconds"],1)
        self.assertIsNone(state["last_attempt"]["data"])

    def test_bad_id_checksum_firmware_and_exception_are_not_feedback(self):
        for invalid in (feedback(2),feedback(1,checksum_ok=False),feedback(1,firmware_status=0xE0),
                        feedback(1,current_position=True),None):
            self.driver.read_finger_status.side_effect = None
            self.driver.read_finger_status.return_value = invalid
            self.assertFalse(FeedbackPoller(self.driver,10,self.clock).poll()["valid"])
        self.driver.read_finger_status.side_effect = TimeoutError("timed out")
        sample = FeedbackPoller(self.driver,10,self.clock).poll()
        self.assertFalse(sample["valid"])
        self.assertIn("TimeoutError",sample["error"])

    def test_read_only_entry_never_calls_a_motion_method(self):
        self.driver.start_persistent_connection = Mock(return_value=True)
        self.driver.stop_persistent_connection = Mock()
        self.driver.move_hand = Mock(side_effect=AssertionError("motion"))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"feedback.jsonl"
            with patch("mh6_feedback_read.DexHandControl",return_value=self.driver), \
                    patch("mh6_feedback_read.time.monotonic",self.clock), \
                    patch("mh6_feedback_read.time.sleep",self.clock.advance), \
                    patch("mh6_actuator_io.time.monotonic",self.clock), redirect_stdout(io.StringIO()):
                # Clock must be injected because default arguments are bound at import.
                with patch("mh6_feedback_read.FeedbackPoller",side_effect=lambda d,r:FeedbackPoller(d,r,self.clock)):
                    self.assertEqual(feedback_main(["--port","unused","--output",str(path),"--duration","1"]),0)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual({r["sample"]["channel"] for r in rows},set(CHANNELS))
            self.driver.move_hand.assert_not_called()
            self.driver.stop_persistent_connection.assert_called_once()


class ExecutionTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.planner = NineChannelPreview(HardwareConfig(configured()))
        self.driver = SimpleNamespace(last_move_result=None,
            read_finger_status=Mock(side_effect=lambda i:feedback(i)),
            read_palm_status=Mock(side_effect=lambda i:feedback(i,False)))
        self.sender = SimpleNamespace(hand=self.driver,send=Mock(return_value=True))
        self.control = ActuatorIO(self.planner,self.sender,clock=self.clock)

    def test_acknowledged_only_and_failed_send_latches_without_retry(self):
        _,execution,_ = self.control.cycle(result(),roots())
        self.assertTrue(execution["write_acknowledged"])
        self.assertEqual(self.planner.previous["M4"],600)
        before = copy.deepcopy(self.planner.previous)
        generation = self.planner.generation
        self.sender.send.return_value = False
        _,failed,_ = self.control.cycle(result(abduction=40),roots((520,530,540)))
        self.assertEqual(failed["status"],"SEND_FAILED_EXECUTION_UNCERTAIN")
        self.assertFalse(failed["history_committed"])
        self.assertEqual(self.planner.previous,before)
        self.assertEqual(self.planner.generation,generation)
        _,blocked,_ = self.control.cycle(result(),roots())
        self.assertEqual(blocked["status"],"BLOCKED_AFTER_FAILURE")
        self.assertEqual(self.sender.send.call_count,2)

    def test_exception_latches_and_feedback_continues(self):
        self.sender.send.side_effect = TimeoutError("ack lost")
        _,failed,_ = self.control.cycle(result(),roots())
        self.assertIn("ack lost",failed["fault"]["error"])
        self.clock.advance()
        self.control.cycle(result(),roots())
        self.assertEqual(self.driver.read_finger_status.call_count,2)
        self.assertEqual(self.sender.send.call_count,1)

    def test_configuration_incomplete_and_simulation_never_send(self):
        planner = NineChannelPreview(HardwareConfig.load())
        control = ActuatorIO(planner,self.sender,clock=self.clock)
        _,execution,_ = control.cycle(result(),roots())
        self.assertEqual(execution["status"],"CONFIGURATION_INCOMPLETE")
        self.sender.send.assert_not_called()
        control = ActuatorIO(planner,clock=self.clock)
        _,execution,snapshot = control.cycle(result(),roots())
        self.assertEqual(execution["status"],"SIMULATED_NOT_SENT")
        self.assertFalse(snapshot["enabled"])
        self.assertIsNone(execution["started_monotonic"])

    def test_recovery_requires_all_post_fault_readbacks_and_is_explicit(self):
        self.sender.send.return_value = False
        self.control.cycle(result(),roots())
        with self.assertRaisesRegex(ValueError,"post-fault"):
            self.control.reconcile_from_feedback()
        for _ in range(9):
            self.clock.advance(); self.control.feedback.poll()
        # Polls do not clear the fault or change command references.
        self.assertIsNotNone(self.control.fault)
        self.assertEqual(self.planner.previous["M1"],500)
        self.control.reconcile_from_feedback()
        self.assertIsNone(self.control.fault)
        self.assertEqual(self.planner.previous["F1"],100)
        self.assertEqual(self.sender.send.call_count,1)

    def test_valid_device_fault_blocks_later_writes(self):
        self.driver.read_finger_status.side_effect = lambda i:feedback(i,error_flags=4)
        self.control.cycle(result(),roots())
        self.assertEqual(self.control.fault["reason"],"ACTUATOR_FAULT")
        self.control.cycle(result(),roots())
        self.assertEqual(self.sender.send.call_count,1)


class ModbusFeedbackTest(unittest.TestCase):
    def test_client_disables_implicit_retransmission(self):
        with patch("modbus_dev.ModbusClient") as client:
            DexHandControl(port="unused",timeout=.15)
        self.assertEqual(client.call_args.kwargs["retries"],0)
        self.assertEqual(client.call_args.kwargs["timeout"],.15)

    def driver(self, registers, status=0x94):
        hand = DexHandControl.__new__(DexHandControl)
        hand.persistent_connection = True; hand.transaction_lock = threading.Lock()
        hand._client_connected = Mock(return_value=True)
        hand.client = Mock()
        hand.client.write_register.return_value = SimpleNamespace(isError=lambda:False)
        hand.client.read_holding_registers.side_effect = [
            SimpleNamespace(isError=lambda:False,registers=[status]),
            SimpleNamespace(isError=lambda:False,registers=registers)]
        return hand

    def test_palm_feedback_decodes_actual_position_and_unsupported_fields(self):
        hand = self.driver([4,1,614,32,7400,0,1,99])
        sample = hand.read_palm_status(4)
        self.assertTrue(sample["query_ok"])
        self.assertEqual(sample["current_position"],614)
        self.assertEqual(sample["voltage_mv"],7400)
        self.assertIsNone(sample["current_ma"])
        self.assertEqual([call.kwargs["value"] for call in hand.client.write_register.call_args_list],[1,4,8])

    def test_old_firmware_wrong_id_and_bad_checksum_never_claim_position(self):
        for regs,status in (([4,1,600,30,7400,0,1,1],0xE0),
                            ([3,1,600,30,7400,0,1,1],0x94),
                            ([4,1,600,30,7400,0,0,1],0x94),([4],0x94)):
            sample = self.driver(regs,status).read_palm_status(4)
            self.assertFalse(sample["query_ok"])
            self.assertIsNone(sample["current_position"])

    def test_real_driver_partial_write_does_not_advance_history(self):
        sender = memory_sender()
        original = sender.hand.client.write_register
        sender.hand.client.write_register = Mock(return_value=SimpleNamespace(isError=lambda:True))
        planner = NineChannelPreview(HardwareConfig(configured()))
        control = ActuatorIO(planner,sender)
        before = copy.deepcopy(planner.previous)
        with redirect_stdout(io.StringIO()):
            execution = control.dispatch(planner.propose(result(),roots()))
        self.assertEqual(execution["status"],"SEND_FAILED_EXECUTION_UNCERTAIN")
        self.assertTrue(execution["driver_result"]["execution_uncertain"])
        self.assertEqual(execution["driver_result"]["stage"],"command_write")
        self.assertEqual(planner.previous,before)
        sender.hand.client.write_register = original

    def test_firmware_rejection_after_write_latches_and_keeps_history(self):
        sender = memory_sender()
        sender.hand.client.read_holding_registers = Mock(return_value=SimpleNamespace(
            isError=lambda:False,registers=[0xE9]))
        planner = NineChannelPreview(HardwareConfig(configured()))
        control = ActuatorIO(planner,sender)
        with redirect_stdout(io.StringIO()):
            execution = control.dispatch(planner.propose(result(),roots()))
        self.assertEqual(execution["status"],"SEND_FAILED_EXECUTION_UNCERTAIN")
        self.assertEqual(execution["driver_result"]["firmware_status"],0xE9)
        self.assertEqual(planner.generation,0)


if __name__ == "__main__": unittest.main()
