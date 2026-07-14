import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_palm_fallback import PalmFallbackController
from mh6_palm_solution_selector import PalmInputSlewLimiter, PalmSolutionSelector
from mh6_teleop_run import select_palm_control_preview


def selection(status, motors=(247.0, 500.0, 500.0)):
    return SimpleNamespace(status=status, selected_motor=list(motors))


def command_result(vertical, lateral, thumb=0.0):
    return {
        "palm_command": {
            "vertical": vertical,
            "lateral": lateral,
            "thumb_rotation_command": thumb,
        }
    }


class NoSolutionSolver:
    def solve_motor_from_normalized(self, _u1, _u2, _u3):
        return []


class PalmFallbackTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.controller = PalmFallbackController()

    def test_weighted_intent_uses_vertical_and_lateral(self):
        self.assertAlmostEqual(self.controller.combine_intent(1.0, 0.0), 0.8)
        self.assertAlmostEqual(self.controller.combine_intent(0.0, 1.0), 0.2)
        self.assertAlmostEqual(self.controller.combine_intent(1.0, -1.0), 0.6)

    def test_signed_closure_preserves_outward_neutral_and_inward_states(self):
        self.assertEqual(self.controller.signed_to_closure(-1.0), 0.0)
        self.assertEqual(self.controller.signed_to_closure(0.0), 0.25)
        self.assertEqual(self.controller.signed_to_closure(1.0), 1.0)

    def test_piecewise_path_hits_all_three_calibration_points(self):
        self.assertEqual(self.controller.closure_to_motors(0.0), [0.0, 630.0, 536.0])
        self.assertEqual(
            self.controller.closure_to_motors(0.25),
            [247.0, 500.0, 500.0],
        )
        self.assertEqual(
            self.controller.closure_to_motors(1.0),
            [1000.0, 120.0, 401.0],
        )

    def test_nearest_path_point_recognizes_neutral(self):
        closure, distance = self.controller.nearest_closure([247, 500, 500])
        self.assertEqual(closure, 0.25)
        self.assertEqual(distance, 0.0)


class PalmFallbackStateMachineTest(unittest.TestCase):
    def test_continuous_no_solution_activates_after_delay_and_rate_limits_closure(self):
        controller = PalmFallbackController(
            activation_delay=0.5,
            max_closure_speed_per_sec=1.0,
            nominal_dt=0.05,
            max_dt=0.10,
        )
        held = selection("HELD_NEUTRAL_NO_SOLUTION")

        pending = controller.update(1.0, 1.0, held, timestamp=1.0)
        self.assertEqual(pending.mode, "SOLVER")
        self.assertEqual(pending.status, "FALLBACK_PENDING")

        still_pending = controller.update(1.0, 1.0, held, timestamp=1.40)
        self.assertEqual(still_pending.mode, "SOLVER")

        entered = controller.update(1.0, 1.0, held, timestamp=1.50)
        self.assertEqual(entered.mode, "FALLBACK")
        self.assertEqual(entered.status, "FALLBACK_ENTERED")
        self.assertAlmostEqual(entered.applied_closure, 0.25)

        active = controller.update(1.0, 1.0, held, timestamp=1.60)
        self.assertEqual(active.status, "FALLBACK_ACTIVE")
        self.assertAlmostEqual(active.applied_closure, 0.35)
        self.assertEqual(
            active.selected_motor,
            controller.closure_to_motors(active.applied_closure),
        )

    def test_non_ik_failure_does_not_activate_fallback(self):
        controller = PalmFallbackController(activation_delay=0.0)
        invalid = selection("HELD_NEUTRAL_NO_VALID_SOLUTION")

        result = controller.update(1.0, 1.0, invalid, timestamp=1.0)

        self.assertEqual(result.mode, "SOLVER")
        self.assertEqual(result.status, "SOLVER_ACTIVE")

    def test_solver_recovery_resets_continuous_failure_timer(self):
        controller = PalmFallbackController(activation_delay=0.5)
        held = selection("HELD_NEUTRAL_NO_SOLUTION")
        recovered = selection("SELECTED")

        controller.update(1.0, 1.0, held, timestamp=1.0)
        controller.update(1.0, 1.0, recovered, timestamp=1.4)
        restarted = controller.update(1.0, 1.0, held, timestamp=1.6)

        self.assertEqual(restarted.mode, "SOLVER")
        self.assertEqual(restarted.status, "FALLBACK_PENDING")
        self.assertEqual(restarted.no_solution_duration, 0.0)

    def test_tracking_pause_clears_failure_time_without_changing_closure(self):
        controller = PalmFallbackController(activation_delay=0.5)
        held = selection("HELD_NEUTRAL_NO_SOLUTION")
        controller.update(1.0, 1.0, held, timestamp=1.0)

        controller.pause()
        recovered = controller.update(1.0, 1.0, held, timestamp=10.0)

        self.assertEqual(recovered.mode, "SOLVER")
        self.assertEqual(recovered.status, "FALLBACK_PENDING")
        self.assertEqual(recovered.no_solution_duration, 0.0)
        self.assertEqual(recovered.applied_closure, 0.25)

    def test_entry_is_refused_when_last_motor_is_far_from_fallback_path(self):
        controller = PalmFallbackController(
            activation_delay=0.0,
            entry_distance_limit=0.01,
        )
        off_path = selection("HELD_NO_SOLUTION", (0.0, 120.0, 401.0))

        result = controller.update(1.0, 1.0, off_path, timestamp=1.0)

        self.assertEqual(result.mode, "SOLVER")
        self.assertEqual(result.status, "FALLBACK_ENTRY_UNSAFE")

    def test_fallback_exits_only_after_stable_neutral_solver_recovery(self):
        controller = PalmFallbackController(
            activation_delay=0.0,
            recovery_frames=3,
            max_closure_speed_per_sec=10.0,
        )
        held = selection("HELD_NEUTRAL_NO_SOLUTION")
        controller.update(1.0, 1.0, held, timestamp=1.0)
        self.assertTrue(controller.active)

        neutral = selection("SELECTED")
        first = controller.update(0.0, 0.0, neutral, timestamp=1.1)
        second = controller.update(0.0, 0.0, neutral, timestamp=1.2)
        third = controller.update(0.0, 0.0, neutral, timestamp=1.3)

        self.assertEqual(first.mode, "FALLBACK")
        self.assertEqual(second.mode, "FALLBACK")
        self.assertEqual(third.mode, "SOLVER")
        self.assertEqual(third.status, "FALLBACK_EXITED")
        self.assertFalse(controller.active)


class PalmFallbackIntegrationTest(unittest.TestCase):
    def test_preview_switches_to_print_only_fallback_after_prolonged_no_solution(self):
        solver = NoSolutionSolver()
        selector = PalmSolutionSelector()
        limiter = PalmInputSlewLimiter()
        fallback = PalmFallbackController(activation_delay=0.5)
        result = command_result(1.0, 1.0, 1.0)

        first = select_palm_control_preview(
            result, solver, selector, fallback, limiter, timestamp=1.0
        )
        activated = select_palm_control_preview(
            result, solver, selector, fallback, limiter, timestamp=1.5
        )
        moving = select_palm_control_preview(
            result, solver, selector, fallback, limiter, timestamp=1.6
        )

        self.assertEqual(first[1].mode, "SOLVER")
        self.assertEqual(activated[1].mode, "FALLBACK")
        self.assertEqual(activated[1].selected_motor, [247.0, 500.0, 500.0])
        self.assertNotEqual(moving[1].selected_motor, [247.0, 500.0, 500.0])


if __name__ == "__main__":
    unittest.main()
