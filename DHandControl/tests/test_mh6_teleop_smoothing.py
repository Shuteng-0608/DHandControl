from contextlib import redirect_stdout
import io
import math
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_palm_solution_selector import PalmInputSlewLimiter, PalmSolutionSelector
from mh6_palm_solver import MH6PalmSolver
from mh6_teleop_run import (
    CommandLowPassFilter,
    HardwareSender,
    PalmDebugLogger,
    print_palm_control_preview,
    select_palm_motor_preview,
)


def command_result(value: float):
    return {
        "low_dim": {"u_h": value, "u_v": -value},
        "palm_command": {
            "vertical": value,
            "lateral": -value,
            "thumb_rotation_command": value,
        },
    }


class NoSolutionSolver:
    def solve_motor_from_normalized(self, _u1, _u2, _u3):
        return []


class CommandLowPassFilterTest(unittest.TestCase):
    def test_first_frame_caps_nominal_dt(self) -> None:
        command_filter = CommandLowPassFilter(
            0.24,
            nominal_dt=1.0,
            max_dt=0.10,
        )

        result = command_filter.apply(command_result(1.0), now=1.0)

        expected = 1.0 - math.exp(-0.10 / 0.24)
        self.assertAlmostEqual(result["low_dim"]["u_h"], expected)

    def test_first_frame_ramps_from_neutral(self) -> None:
        command_filter = CommandLowPassFilter(
            0.24,
            nominal_dt=0.05,
            max_dt=0.10,
        )

        result = command_filter.apply(command_result(1.0), now=1.0)

        expected = 1.0 - math.exp(-0.05 / 0.24)
        self.assertAlmostEqual(result["low_dim"]["u_h"], expected)
        self.assertAlmostEqual(result["palm_command"]["vertical"], expected)

    def test_frame_gap_uses_bounded_dt(self) -> None:
        command_filter = CommandLowPassFilter(
            0.24,
            nominal_dt=0.05,
            max_dt=0.10,
        )
        command_filter.apply(command_result(0.0), now=1.0)

        result = command_filter.apply(command_result(1.0), now=2.0)

        expected = 1.0 - math.exp(-0.10 / 0.24)
        self.assertAlmostEqual(result["low_dim"]["u_h"], expected)
        self.assertLess(result["low_dim"]["u_h"], 0.5)

    def test_palm_aliases_follow_filtered_command(self) -> None:
        command_filter = CommandLowPassFilter(0.24)

        result = command_filter.apply(command_result(1.0), now=1.0)

        self.assertEqual(result["palm"], result["palm_command"])
        self.assertEqual(result["palm_fold"], result["palm_command"])


class SolverInputRecoveryTest(unittest.TestCase):
    def test_real_solver_thumb_path_advances_without_branch_jumps(self) -> None:
        limiter = PalmInputSlewLimiter()
        selector = PalmSolutionSelector()
        solver = MH6PalmSolver()
        previous_motor_3 = 500.0

        for index in range(10):
            applied = limiter.apply((0.0, 0.0, 1.0), timestamp=index * 0.05)
            selection = selector.select(
                applied,
                solver.solve_motor_from_normalized(*applied),
                timestamp=index * 0.05,
            )
            self.assertEqual(selection.status, "SELECTED")
            self.assertLess(selection.selected_motor[2], previous_motor_3)
            previous_motor_3 = selection.selected_motor[2]

    def test_no_solution_rolls_slew_state_back_to_last_feasible_input(self) -> None:
        limiter = PalmInputSlewLimiter()
        selector = PalmSolutionSelector()

        preview = select_palm_motor_preview(
            command_result(1.0),
            NoSolutionSolver(),
            selector,
            input_limiter=limiter,
            timestamp=1.0,
        )

        self.assertEqual(tuple(preview[1].values()), (0.1, -0.1, 0.1))
        self.assertEqual(preview[3].status, "HELD_NEUTRAL_NO_SOLUTION")
        self.assertEqual(limiter.applied, [0.0, 0.0, 0.0])


class PreparedHardwareSenderTest(unittest.TestCase):
    def test_uses_positive_finger_commands_and_selected_actual_palm_motors(self) -> None:
        sender = HardwareSender("unused", 115200)
        sender.hand = Mock()
        sender.hand.map_finger_positions.return_value = {
            1: 20,
            2: 200,
            3: 400,
            4: 600,
            5: 800,
        }
        sender.hand.validate_palm_motor_positions.return_value = {
            1: 247,
            2: 500,
            3: 500,
        }
        sender.hand.move_hand.return_value = True
        result = {
            "low_dim": {
                "u_thumb": -0.5,
                "u_index": 0.1,
                "u_middle": 0.2,
                "u_ring": 0.3,
                "u_little": 0.4,
            }
        }
        selection = SimpleNamespace(selected_motor=[247.0, 500.0, 500.0])

        self.assertTrue(sender.send(result, selection))

        sender.hand.map_finger_positions.assert_called_once_with(
            [0.0, 0.1, 0.2, 0.3, 0.4]
        )
        sender.hand.validate_palm_motor_positions.assert_called_once_with(
            [247.0, 500.0, 500.0]
        )
        sender.hand.move_hand.assert_called_once_with(
            finger_ids=[1, 2, 3, 4, 5],
            finger_positions=[20, 200, 400, 600, 800],
            palm_ids=[1, 2, 3],
            palm_positions=[247, 500, 500],
            palm_times=[80, 80, 80],
            wait_status=False,
        )


class PalmDebugOutputTest(unittest.TestCase):
    @staticmethod
    def make_preview():
        solver_selection = SimpleNamespace(
            selected_motor=[247.0, 500.0, 500.0],
            status="HELD_NEUTRAL_NO_SOLUTION",
            held_previous=True,
            normalized_jump=None,
        )
        fallback = SimpleNamespace(
            mode="FALLBACK",
            status="FALLBACK_ACTIVE",
            signed_closure=0.8,
            requested_closure=0.85,
            applied_closure=0.4,
            selected_motor=[397.6, 424.0, 480.2],
            no_solution_duration=1.0,
            entry_distance=0.0,
        )
        return (
            (
                {"palm_flexion": 0.8, "palm_cross": 0.8, "thumb_inward": 0.8},
                {"palm_flexion": 0.1, "palm_cross": 0.1, "thumb_inward": 0.1},
                [],
                solver_selection,
            ),
            fallback,
        )

    def test_console_always_prints_solver_before_fallback_control(self) -> None:
        output = io.StringIO()
        with redirect_stdout(output):
            print_palm_control_preview(self.make_preview())

        lines = output.getvalue().splitlines()
        self.assertIn("palm solver preview", lines[0])
        self.assertIn("HELD_NEUTRAL_NO_SOLUTION", lines[0])
        self.assertIn("palm control preview", lines[1])
        self.assertIn("mode=FALLBACK", lines[1])

    def test_every_frame_log_contains_solver_and_final_control_sections(self) -> None:
        preview = self.make_preview()

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "debug.jsonl"
            logger = PalmDebugLogger(str(path))
            logger.write(10.0, preview)
            logger.close()
            record = json.loads(path.read_text(encoding="utf-8"))

        self.assertEqual(record["solver"]["status"], "HELD_NEUTRAL_NO_SOLUTION")
        self.assertEqual(record["control"]["mode"], "FALLBACK")


if __name__ == "__main__":
    unittest.main()
