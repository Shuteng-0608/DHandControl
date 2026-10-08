import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_palm_solution_selector import PalmInputSlewLimiter, PalmSolutionSelector


class PalmSolutionSelectorTest(unittest.TestCase):
    def test_fixed_branch_never_uses_a_closer_other_root(self):
        selector = self.integer_selector(fixed_branch_id="plus_acos")
        result = selector.select((0, 0, 0), [[247, 500, 501], [247, 500, 520]],
                                 branch_ids=["minus_acos", "plus_acos"])
        self.assertEqual(result.selected_motor, [247, 500, 520])
        self.assertEqual(result.branch_selection["selected_branch_id"], "plus_acos")
        self.assertEqual(result.branch_selection["reason"], "fixed_branch")
        self.assertEqual(result.branch_selection["switch_penalty"], 0)

    def test_fixed_branch_out_of_limits_holds_despite_legal_other_root(self):
        selector = self.integer_selector(fixed_branch_id="plus_acos")
        selector.select((0, 0, 0), [[247, 500, 500]], branch_ids=["plus_acos"])
        result = selector.select((.1, .1, .1), [[247, 500, 600], [247, 500, 501]],
                                 branch_ids=["plus_acos", "minus_acos"])
        self.assertEqual(result.status, "HELD_FIXED_BRANCH_OUT_OF_LIMITS")
        self.assertEqual(result.selected_motor, [247, 500, 500])
        self.assertEqual(result.valid_candidate_count, 1)
        self.assertEqual(selector.previous_valid_input, (0, 0, 0))
        self.assertEqual(result.branch_selection["fixed_branch_valid_candidate_count"], 0)
        self.assertFalse(result.branch_selection["branch_changed"])

    def test_fixed_branch_missing_or_unidentified_holds_and_recovers_same_root(self):
        selector = self.integer_selector(fixed_branch_id="minus_acos")
        for candidates, identities in (([[247, 500, 500]], ["plus_acos"]),
                                        ([[247, 500, 500]], [None]), ([], [])):
            result = selector.select((0, 0, 0), candidates, branch_ids=identities)
            self.assertEqual(result.status, "HELD_NEUTRAL_FIXED_BRANCH_UNAVAILABLE")
            self.assertEqual(result.selected_motor, [247, 500, 500])
            self.assertIsNone(selector.previous_motor)
        result = selector.select((0, 0, 0), [[247, 500, 510]], branch_ids=["minus_acos"])
        self.assertEqual(result.status, "SELECTED")
        self.assertEqual(selector.previous_branch_id, "minus_acos")

    def test_fixed_root_is_not_switched_when_position_guard_rejects_it(self):
        selector = self.integer_selector(fixed_branch_id="plus_acos", max_motor_step=(1, 1, 1))
        selector.select((0, 0, 0), [[247, 500, 500]], branch_ids=["plus_acos"])
        result = selector.select((0, 0, 0), [[247, 500, 510], [247, 500, 500]],
                                 branch_ids=["plus_acos", "minus_acos"])
        self.assertTrue(result.held_previous)
        self.assertEqual(result.branch_selection["candidate_branch_id"], "plus_acos")
        self.assertEqual(result.selected_motor, [247, 500, 500])

    def test_invalid_fixed_branch_is_rejected(self):
        with self.assertRaises(ValueError):
            self.integer_selector(fixed_branch_id="first")

    def integer_selector(self, **kwargs):
        return PalmSolutionSelector(selection_policy="integer_continuous",
                                    max_normalized_speed_per_sec=None,
                                    allow_unguarded=True, **kwargs)

    def test_integer_policy_keeps_integer_reference_and_scores_actual_commands(self):
        selector = self.integer_selector(branch_switch_penalty=0)
        first = selector.select((0, 0, 0), [[247.49, 500.49, 500.49]], branch_ids=["A"])
        self.assertEqual(first.selected_motor, [247, 500, 500])
        result = selector.select((0, 0, 0), [[247.49, 500.49, 501.49]], branch_ids=["A"])
        score = result.branch_selection["candidate_scores"][0]
        self.assertEqual(score["position_delta"], [0, 0, 1])
        self.assertAlmostEqual(score["motion_cost"], (1/135)**2)
        self.assertEqual(selector.previous_motor, [247, 500, 501])

    def test_switch_penalty_holds_branch_for_small_gain_and_switches_for_large_gain(self):
        selector = self.integer_selector(branch_switch_penalty=.001)
        selector.select((0, 0, 0), [[247, 500, 500]], branch_ids=["A"])
        result = selector.select((0, 0, 0), [[247, 500, 506], [247, 500, 505]], branch_ids=["A", "B"])
        self.assertEqual(result.selected_motor, [247, 500, 506])
        self.assertEqual(result.branch_selection["reason"], "stay_branch_hysteresis")
        self.assertFalse(result.branch_selection["branch_changed"])
        result = selector.select((0, 0, 0), [[247, 500, 520], [247, 500, 510]], branch_ids=["A", "B"])
        self.assertEqual(result.selected_motor, [247, 500, 510])
        self.assertEqual(result.branch_selection["selected_branch_id"], "B")
        self.assertTrue(result.branch_selection["branch_changed"])

    def test_branch_order_does_not_change_selection_including_integer_ties(self):
        for reverse in (False, True):
            selector = self.integer_selector(branch_switch_penalty=0)
            selector.select((0, 0, 0), [[247, 500, 500]], branch_ids=["A"])
            candidates = [[247, 500, 501.4], [247, 500, 501.1]]
            branches = ["A", "B"]
            if reverse:
                candidates.reverse(); branches.reverse()
            result = selector.select((0, 0, 0), candidates, branch_ids=branches)
            self.assertEqual(result.selected_motor, [247, 500, 501])
            self.assertEqual(result.branch_selection["selected_branch_id"], "A")
            self.assertEqual(result.branch_selection["reason"], "integer_tie_keep_branch")

    def test_branch_memory_survives_no_solution_and_changes_when_old_root_invalid(self):
        selector = self.integer_selector()
        selector.select((0, 0, 0), [[247, 500, 500]], branch_ids=["A"])
        held = selector.select((.1, .1, .1), [], branch_ids=[])
        self.assertEqual(held.branch_selection["selected_branch_id"], "A")
        self.assertEqual(selector.previous_valid_input, (0, 0, 0))
        result = selector.select((.1, .1, .1), [[247, 500, 600], [247, 500, 510]], branch_ids=["A", "B"])
        self.assertEqual(result.branch_selection["selected_branch_id"], "B")
        self.assertEqual(result.branch_selection["reason"], "only_valid_candidate")
        self.assertFalse(result.branch_selection["candidate_scores"][0]["within_motor_limits"])

    def test_position_rejection_does_not_commit_proposed_branch(self):
        selector = self.integer_selector(max_motor_step=(1, 1, 1))
        selector.select((0, 0, 0), [[247, 500, 500]], branch_ids=["A"])
        result = selector.select((.1, .1, .1), [[247, 500, 510]], branch_ids=["B"])
        self.assertEqual(result.branch_selection["candidate_branch_id"], "B")
        self.assertEqual(result.branch_selection["selected_branch_id"], "A")
        self.assertFalse(result.branch_selection["branch_changed"])
        self.assertEqual(selector.previous_motor, [247, 500, 500])

    def test_unknown_identity_and_reset_clear_branch_memory(self):
        selector = self.integer_selector()
        selector.select((0, 0, 0), [[247, 500, 500]], branch_ids=["A"])
        selector.select((0, 0, 0), [[247, 500, 501]])
        self.assertIsNone(selector.previous_branch_id)
        selector.select((0, 0, 0), [[247, 500, 502]], branch_ids=["B"])
        selector.reset()
        self.assertIsNone(selector.previous_motor)
        self.assertIsNone(selector.previous_branch_id)

    def test_branch_options_and_alignment_are_validated(self):
        for penalty in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                self.integer_selector(branch_switch_penalty=penalty)
        with self.assertRaises(ValueError):
            PalmSolutionSelector(selection_policy="invalid")
        selector = self.integer_selector()
        for branches in ([], [""], [123]):
            with self.assertRaises(ValueError):
                selector.select((0, 0, 0), [[247, 500, 500]], branch_ids=branches)

    def test_position_guard_is_independent_of_timestamp_gap(self) -> None:
        for dt in (.001, 10.0):
            selector = PalmSolutionSelector(
                max_motor_step=(10, 10, 10), max_normalized_speed_per_sec=None,
            )
            selector.select((0, 0, 0), [[247, 500, 500]], timestamp=0.0)
            accepted = selector.select((.1, .1, .1), [[252, 505, 505]], timestamp=dt)
            self.assertEqual(accepted.status, "SELECTED")
            rejected = selector.select((.2, .2, .2), [[270, 505, 505]], timestamp=dt+1)
            self.assertTrue(rejected.held_previous)
            self.assertEqual(rejected.jump_rejection_reason, "position_jump")
            self.assertEqual(rejected.selected_motor, [252, 505, 505])

    def test_position_guard_uses_integer_commands_and_inclusive_boundary(self) -> None:
        selector = PalmSolutionSelector(
            max_motor_step=(10, 10, 10), max_normalized_speed_per_sec=None,
        )
        selector.select((0, 0, 0), [[247, 500, 500]], timestamp=0)
        result = selector.select((0, 0, 0), [[257.49, 500, 500]], timestamp=1)
        self.assertEqual(result.status, "SELECTED")
        self.assertEqual(result.candidate_motor_position_delta, [10, 0, 0])
        result = selector.select((0, 0, 0), [[268, 500, 500]], timestamp=2)
        self.assertEqual(result.jump_rejection_reason, "position_jump")
        self.assertEqual(selector.previous_motor, [257.49, 500, 500])

    def test_position_guard_tries_other_branch_when_nearest_exceeds_axis_limit(self) -> None:
        selector = PalmSolutionSelector(
            max_motor_step=(5, 10, 10), max_normalized_speed_per_sec=None,
        )
        selector.select((0, 0, 0), [[247, 500, 500]], timestamp=0)
        # First branch is closer in calibrated distance but exceeds the M1 cap.
        result = selector.select((0, 0, 0), [[253, 500, 500], [247, 500, 509]], timestamp=1)
        self.assertEqual(result.status, "SELECTED")
        self.assertEqual(result.selected_candidate_index, 1)
        self.assertEqual(result.selected_motor, [247, 500, 509])

    def test_position_guard_rejects_invalid_configuration(self) -> None:
        for caps in [(10, 10), (10, 0, 10), (10, float("nan"), 10)]:
            with self.assertRaises(ValueError):
                PalmSolutionSelector(max_motor_step=caps, max_normalized_speed_per_sec=None)
        with self.assertRaises(ValueError):
            PalmSolutionSelector(max_normalized_speed_per_sec=None)

    def test_first_frame_selects_solution_closest_to_neutral(self) -> None:
        selector = PalmSolutionSelector()

        result = selector.select(
            (0.0, 0.0, 0.0),
            [[1000, 120, 401], [250, 500, 500]],
            timestamp=1.0,
        )

        self.assertEqual(result.selected_motor, [250.0, 500.0, 500.0])
        self.assertEqual(result.status, "SELECTED")

    def test_initial_solution_still_obeys_jump_limit(self) -> None:
        selector = PalmSolutionSelector(
            max_normalized_speed_per_sec=2.0,
            nominal_dt=1.0,
            max_dt=0.10,
        )

        result = selector.select(
            (1.0, 1.0, 1.0),
            [[1000, 120, 401]],
            timestamp=1.0,
        )

        self.assertEqual(result.status, "HELD_NEUTRAL_INITIAL_JUMP_REJECTED")
        self.assertEqual(result.selected_motor, [247, 500, 500])

    def test_asymmetric_calibration_is_used_for_branch_distance(self) -> None:
        selector = PalmSolutionSelector(max_normalized_speed_per_sec=100.0)
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select(
            (0.1, 0.1, 0.1),
            [[347, 500, 536], [247, 550, 500]],
            timestamp=1.1,
        )

        self.assertEqual(result.selected_motor, [247.0, 550.0, 500.0])

    def test_no_solution_holds_previous_valid_motor(self) -> None:
        selector = PalmSolutionSelector()
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select((0.8, 0.8, 0.8), [], timestamp=1.05)

        self.assertEqual(result.selected_motor, [247.0, 500.0, 500.0])
        self.assertEqual(result.status, "HELD_NO_SOLUTION")
        self.assertTrue(result.held_previous)
        self.assertEqual(selector.previous_valid_input, (0.0, 0.0, 0.0))

    def test_invalid_solution_holds_previous(self) -> None:
        selector = PalmSolutionSelector()
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select((0.2, 0.2, 0.2), [[247, 700, 500]], timestamp=1.05)

        self.assertEqual(result.status, "HELD_NO_VALID_SOLUTION")
        self.assertEqual(result.selected_motor, [247.0, 500.0, 500.0])

    def test_excessive_jump_is_rejected(self) -> None:
        selector = PalmSolutionSelector(max_normalized_speed_per_sec=2.0)
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select(
            (1.0, 1.0, 1.0),
            [[1000, 120, 401]],
            timestamp=1.05,
        )

        self.assertEqual(result.status, "HELD_JUMP_REJECTED")
        self.assertEqual(result.selected_motor, [247.0, 500.0, 500.0])
        self.assertEqual(selector.previous_valid_input, (0.0, 0.0, 0.0))

    def test_jump_allowance_caps_dt_after_long_gap(self) -> None:
        selector = PalmSolutionSelector(
            max_normalized_speed_per_sec=2.0,
            max_dt=0.10,
        )
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select(
            (1.0, 1.0, 1.0),
            [[1000, 120, 401]],
            timestamp=10.0,
        )

        self.assertEqual(result.status, "HELD_JUMP_REJECTED")


class PalmInputSlewLimiterTest(unittest.TestCase):
    def test_first_step_caps_nominal_dt(self) -> None:
        limiter = PalmInputSlewLimiter(
            max_speed_per_sec=(2.0, 2.0, 2.0),
            nominal_dt=1.0,
            max_dt=0.10,
        )

        first = limiter.apply((1.0, 1.0, 1.0), timestamp=1.0)

        self.assertEqual(first, (0.2, 0.2, 0.2))

    def test_limits_each_axis_and_caps_dt_after_gap(self) -> None:
        limiter = PalmInputSlewLimiter(
            max_speed_per_sec=(2.0, 1.0, 0.5),
            nominal_dt=0.05,
            max_dt=0.10,
        )

        first = limiter.apply((1.0, -1.0, 1.0), timestamp=1.0)
        after_gap = limiter.apply((1.0, -1.0, 1.0), timestamp=2.0)

        self.assertEqual(first, (0.1, -0.05, 0.025))
        self.assertAlmostEqual(after_gap[0], 0.3)
        self.assertAlmostEqual(after_gap[1], -0.15)
        self.assertAlmostEqual(after_gap[2], 0.075)

    def test_hold_restores_last_feasible_input(self) -> None:
        limiter = PalmInputSlewLimiter()
        limiter.apply((1.0, 1.0, 1.0), timestamp=1.0)

        limiter.hold((0.0, 0.0, 0.0))

        self.assertEqual(limiter.applied, [0.0, 0.0, 0.0])

if __name__ == "__main__":
    unittest.main()
