"""Atomic transport tests use memory UARTs and compiled host firmware only."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import copy
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from mh6_atomic_protocol import (
    AtomicHandTransport, CAPABILITIES, crc16, decode_response, encode_request, with_crc,
)
from mh6_actuator_io import ActuatorIO
from mh6_actuator_preview import NineChannelPreview
from mh6_hardware_config import HardwareConfig
from mh6_teleop_run import parse_args, HardwareSender
from mh6_transport_defaults import DEFAULT_CONTROL_TRANSPORT
from mh6_compare_transport_logs import summarize
from test_modbus_hardware_config import hand
from test_mh6_actuator_preview import configured, result, roots


def packet():
    return {"finger_ids":[1,2,3,4,5],"finger_positions":[100,200,300,400,500],
            "palm_ids":[1,2,3,4],"palm_positions":[300,450,450,300],"palm_times":[80]*4}


def response(sequence, status=0x95):
    return with_crc(struct.pack(">BBBBIH",1,0x41,1,1,sequence,status))


class MemorySerial:
    def __init__(self):
        self.timeout, self.write_timeout = .15, None
        self.writes, self.input = [], bytearray()
        self.answer = lambda sequence: response(sequence)
        self.reads, self.flushes = 0, 0
        self.partial = False

    def write(self, data):
        self.writes.append(data)
        if data[3] == 1:
            self.input.extend(self.answer(int.from_bytes(data[4:8],"big")))
        return len(data)-1 if self.partial else len(data)

    def flush(self):
        self.flushes += 1

    def read(self, size):
        self.reads += 1
        n = min(size,3,len(self.input))  # Fragment responses deliberately.
        data = bytes(self.input[:n]); del self.input[:n]
        return data


class AtomicTransportTest(unittest.TestCase):
    def transport(self, mode="atomic_ack", config=None):
        device = hand(config)
        device.client.socket = MemorySerial()
        for i,value in enumerate(CAPABILITIES): device.client.registers[50+i] = value
        for i,value in enumerate((0x4647,1,0x1F03,0x0204)): device.client.registers[80+i] = value
        device.client.read_holding_registers = Mock(wraps=device.client.read_holding_registers)
        transport = AtomicHandTransport(device,mode)
        transport.sequence = 0x10203040
        transport.verify_firmware()
        return device, transport

    def test_fixed_packet_crc_layout_and_correlated_response(self):
        data = encode_request(packet(),0x10203040)
        self.assertEqual(len(data),64)
        self.assertEqual(data[:8],bytes.fromhex("0141010110203040"))
        self.assertEqual(int.from_bytes(data[-2:],"little"),crc16(data[:-2]))
        self.assertEqual(struct.unpack(">27H",data[8:-2])[:3],(5,1,100))
        self.assertEqual(decode_response(response(7),7),0x95)

    def test_ack_stream_uses_one_write_and_no_modbus_status_queries(self):
        device, transport = self.transport()
        device.client.read_holding_registers.reset_mock()
        self.assertTrue(transport.send(packet()))
        self.assertEqual(len(device.client.socket.writes),1)
        self.assertEqual(device.client.socket.flushes,1)
        self.assertEqual(device.client.writes,[])
        device.client.read_holding_registers.assert_not_called()
        result = device.last_move_result
        self.assertEqual(result["status"],"GUARDED_TARGET_ACCEPTED")
        self.assertTrue(result["device_confirmed"])
        self.assertFalse(result["execution_uncertain"])
        self.assertEqual(set(result["timings_ms"]),{"wire_write","ack_wait"})
        self.assertEqual(device.client.socket.timeout,.15)
        self.assertIsNone(device.client.socket.write_timeout)

    def test_oneway_never_reads_and_never_claims_device_confirmation(self):
        device, transport = self.transport("atomic_oneway")
        device.client.read_holding_registers.reset_mock()
        device.client.socket.read = Mock(side_effect=AssertionError("one-way read"))
        self.assertTrue(transport.send(packet()))
        device.client.read_holding_registers.assert_not_called()
        device.client.socket.read.assert_not_called()
        self.assertEqual(device.client.socket.writes[0][3],0)
        result = device.last_move_result
        self.assertEqual(result["status"],"HOST_WRITTEN_UNCONFIRMED")
        self.assertFalse(result["device_confirmed"])
        self.assertTrue(result["execution_uncertain"])
        self.assertEqual(set(result["timings_ms"]),{"wire_write"})

    def test_wrong_firmware_blocks_motion(self):
        device, transport = self.transport()
        device.client.registers[50] = 0
        with self.assertRaisesRegex(RuntimeError,"firmware mismatch"):
            transport.verify_firmware()
        self.assertFalse(transport.send(packet()))
        self.assertEqual(device.last_move_result["status"],"NOT_VERIFIED")
        self.assertEqual(device.client.socket.writes,[])

    def test_unarmed_or_missing_guard_blocks_startup_before_clear_in_all_modes(self):
        for mode in ("atomic_ack","atomic_oneway","legacy"):
            device, _ = self.transport()
            device.client.registers[82]=0
            device.clear_error=Mock(return_value=True)
            sender=HardwareSender("memory-only",115200,enable_motion=True,control_transport=mode)
            with patch("modbus_dev.DexHandControl",return_value=device),self.assertRaisesRegex(RuntimeError,"guard is locked"):
                sender.start()
            device.clear_error.assert_not_called()
            self.assertEqual(device.client.socket.writes,[])

    def test_sender_startup_checks_capability_clears_once_and_sends_atomic_packet(self):
        device, _ = self.transport()
        device.client.read_holding_registers.reset_mock()
        device.clear_error = Mock(return_value=True)
        sender = HardwareSender("memory-only",115200,enable_motion=True,control_transport="atomic_ack")
        with patch("modbus_dev.DexHandControl",return_value=device):
            sender.start()
            self.assertTrue(sender.send(packet()))
            sender.stop()
        self.assertEqual(device.client.read_holding_registers.call_count,2)
        self.assertEqual([c.args[0] for c in device.clear_error.call_args_list],list(range(1,6)))
        self.assertEqual(len(device.client.socket.writes),1)
        self.assertFalse(device.persistent_connection)

    def test_sender_capability_mismatch_closes_before_any_clear_or_target(self):
        device, _ = self.transport()
        device.client.registers[50] = 0
        device.clear_error = Mock(return_value=True)
        sender = HardwareSender("memory-only",115200,enable_motion=True,control_transport="atomic_oneway")
        with patch("modbus_dev.DexHandControl",return_value=device), self.assertRaisesRegex(RuntimeError,"firmware mismatch"):
            sender.start()
        self.assertIsNone(sender.hand)
        device.clear_error.assert_not_called()
        self.assertEqual(device.client.socket.writes,[])
        self.assertFalse(device.persistent_connection)

    def test_bad_response_timeout_and_rejection_never_retry(self):
        responses = [lambda seq:b"", lambda seq:response(seq+1),
                     lambda seq:response(seq)[:-1]+bytes([response(seq)[-1]^1]),
                     lambda seq:response(seq,0x0103)]
        for answer in responses:
            device, transport = self.transport()
            device.client.socket.answer = answer
            self.assertFalse(transport.send(packet()))
            self.assertEqual(len(device.client.socket.writes),1)
            self.assertEqual(device.last_move_result["status"],"FAILED")
            self.assertIn("ack_wait",device.last_move_result["timings_ms"])
            self.assertEqual(device.client.socket.timeout,.15)
            self.assertIsNone(device.client.socket.write_timeout)

    def test_partial_write_fails_without_read_or_retry(self):
        for mode in ("atomic_ack","atomic_oneway"):
            device, transport = self.transport(mode)
            device.client.socket.partial = True
            self.assertFalse(transport.send(packet()))
            self.assertEqual(device.client.socket.reads,0)
            self.assertEqual(len(device.client.socket.writes),1)
            self.assertTrue(device.last_move_result["execution_uncertain"])

    def test_hardware_lock_limits_and_incomplete_packet_never_write(self):
        device, transport = self.transport()
        invalid = packet(); invalid["palm_positions"][-1] = 401
        self.assertFalse(transport.send(invalid))
        invalid = packet(); invalid["finger_ids"][-1] = 1
        self.assertFalse(transport.send(invalid))
        self.assertFalse(transport.send({}))
        device.motion_enabled = False
        self.assertFalse(transport.send(packet()))
        self.assertEqual(device.client.socket.writes,[])

    def test_oneway_history_is_labeled_and_failed_write_blocks_later_commands(self):
        device, transport = self.transport("atomic_oneway",configured())
        planner = NineChannelPreview(HardwareConfig(configured()))
        sender = SimpleNamespace(hand=device,send=transport.send,control_transport="atomic_oneway")
        control = ActuatorIO(planner,sender,feedback_enabled=False)
        _, execution, feedback = control.cycle(result(curl=.5),roots())
        self.assertEqual(execution["status"],"HOST_WRITTEN_UNCONFIRMED")
        self.assertFalse(execution["write_acknowledged"])
        self.assertTrue(execution["host_write_completed"])
        self.assertEqual(execution["history_reference"],"last_host_written_target")
        self.assertEqual(planner.previous["F1"],985)
        self.assertFalse(feedback["enabled"])
        before = copy.deepcopy(planner.previous)
        device.client.socket.partial = True
        _, failed, _ = control.cycle(result(curl=1),roots())
        self.assertFalse(failed["history_committed"])
        self.assertEqual(planner.previous,before)
        control.cycle(result(),roots())
        self.assertEqual(len(device.client.socket.writes),2)

    def test_cli_uses_this_branch_default_and_allows_explicit_comparison(self):
        self.assertEqual(parse_args([]).control_transport,DEFAULT_CONTROL_TRANSPORT)
        self.assertEqual(parse_args(["--control-transport","legacy"]).control_transport,"legacy")

    def test_comparison_separates_oneway_host_writes_from_device_acks(self):
        import json
        rows = []
        for i in range(3):
            rows.append({"runtime_config":{"control_transport":"atomic_oneway"},
                         "execution":{"started_monotonic":100.+i*.05,"duration_ms":2.,
                                      "status":"HOST_WRITTEN_UNCONFIRMED","write_acknowledged":False,
                                      "driver_result":{"timings_ms":{"wire_write":2.}}},
                         "control":{"channels":{},"palm":{"reason":None}}})
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"log.jsonl"
            path.write_text("\n".join(json.dumps(row) for row in rows)+'\n{"partial":')
            summary=summarize(path)
        self.assertEqual(summary["device_acknowledged"],0)
        self.assertEqual(summary["host_only_sends"],3)
        self.assertAlmostEqual(summary["host_send_hz"],20.)
        self.assertEqual(set(summary["stage_ms"]),{"wire_write"})


class AtomicFirmwareTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"),"host C++ compiler unavailable")
    def test_actual_firmware_with_python_encoded_packets(self):
        root=Path(__file__).resolve().parents[1]
        firmware=root/"DH6Modbus"; stubs=Path(__file__).parent/"firmware_stubs"
        source=(firmware/"DH6Modbus.ino").read_text()
        prototypes=re.findall(r"^(?:void|bool|uint\d+_t|int)\s+\w+\([^;{}]*\)\s*(?=\{)",source,re.M)
        anchor=source.index("bool isRegisterAddressValid")
        source=source[:anchor]+"\n"+";\n".join(prototypes)+";\n"+source[anchor:]
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder)
            arrays=[]
            for name,ack in (("request_ack",True),("request_oneway",False)):
                data=encode_request(packet(),0x10203040,acknowledge=ack)
                arrays.append("const uint8_t "+name+"[] = {"+",".join(map(str,data))+"};")
            (folder/"atomic_packets.h").write_text("\n".join(arrays))
            unit=folder/"atomic.cpp"; binary=folder/"atomic_test"
            unit.write_text(source+"\n"+(stubs/"atomic_control_test.cpp").read_text())
            build=subprocess.run(["g++","-std=gnu++17","-I",str(folder),"-I",str(stubs),"-I",str(firmware),
                                  str(unit),str(firmware/"LobotSerialServoControl.cpp"),
                                  str(firmware/"MicroServoControl.cpp"),
                                  *[str(firmware/name) for name in ("FingerGuard.cpp","FingerGuardBus.cpp","FingerGuardRuntime.cpp")],"-o",str(binary)],
                                 capture_output=True,text=True,timeout=30)
            self.assertEqual(build.returncode,0,build.stderr)
            run=subprocess.run([str(binary)],capture_output=True,text=True,timeout=5)
            self.assertEqual(run.returncode,0,run.stderr)
            ackline=next(line for line in run.stdout.splitlines() if line.startswith("ACK_HEX "))
            self.assertEqual(decode_response(bytes.fromhex(ackline[8:]),0x10203040),0x95)
            self.assertIn("atomic bridge: OK",run.stdout)


if __name__ == "__main__": unittest.main()
