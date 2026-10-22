"""Compile/run actual ESP32 feedback code against host UART stubs, no device."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


class FirmwareFeedbackTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"),"host C++ compiler unavailable")
    def test_palm_query_parser_and_actual_sketch_bridge(self):
        root = Path(__file__).resolve().parents[1]
        firmware = root/"DH6Modbus"
        stubs = Path(__file__).parent/"firmware_stubs"
        source = (firmware/"DH6Modbus.ino").read_text(encoding="utf-8")
        # Arduino normally inserts global function prototypes. Reproduce that
        # preprocessing step for a host C++ build after the sketch's structs.
        prototypes = re.findall(r"^(?:void|bool|uint\d+_t|int)\s+\w+\([^;{}]*\)\s*(?=\{)",source,re.M)
        anchor = source.index("bool isRegisterAddressValid")
        source = source[:anchor]+"\n"+";\n".join(prototypes)+";\n"+source[anchor:]
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            unit, binary = folder/"bridge.cpp",folder/"feedback_test"
            unit.write_text(source+"\n"+(stubs/"palm_feedback_test.cpp").read_text(),encoding="utf-8")
            compilation = subprocess.run(["g++","-std=gnu++17","-I",str(stubs),"-I",str(firmware),
                str(unit),str(firmware/"LobotSerialServoControl.cpp"),str(firmware/"MicroServoControl.cpp"),
                "-o",str(binary)],capture_output=True,text=True,timeout=30)
            self.assertEqual(compilation.returncode,0,compilation.stderr)
            run = subprocess.run([str(binary)],capture_output=True,text=True,timeout=5)
            self.assertEqual(run.returncode,0,run.stderr)
            self.assertIn("bridge registers: OK",run.stdout)
