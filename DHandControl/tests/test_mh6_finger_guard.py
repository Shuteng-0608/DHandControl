"""Run the actual protection controller/UART scheduler without hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class FingerGuardFirmwareTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "C++ compiler unavailable")
    def test_real_controller_protocol_and_uart_scenarios(self):
        root=Path(__file__).resolve().parents[1]
        firmware=root/"DH6Modbus"
        stubs=Path(__file__).parent/"firmware_stubs"
        with tempfile.TemporaryDirectory() as folder:
            binary=Path(folder)/"guard_test"
            build=subprocess.run(["g++","-std=gnu++17","-Wall","-Wextra",
                "-I",str(stubs),"-I",str(firmware),str(stubs/"finger_guard_test.cpp"),
                *[str(firmware/name) for name in ("FingerGuard.cpp","FingerGuardBus.cpp","FingerGuardRuntime.cpp")],
                "-o",str(binary)],capture_output=True,text=True,timeout=30)
            self.assertEqual(build.returncode,0,build.stderr)
            run=subprocess.run([str(binary)],capture_output=True,text=True,timeout=5)
            self.assertEqual(run.returncode,0,run.stderr)
            self.assertIn("finger guard scenarios: OK",run.stdout)
