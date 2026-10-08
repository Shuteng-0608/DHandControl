import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from mh6_palm_solver_v2 import MH6PalmSolver
from mh6_trajectory_audit import (
    fit_passive, memory_sender, motor_to_angles, passive_candidates, pose_residual,
)


class TrajectoryAuditTest(unittest.TestCase):
    def test_motor_inverse_uses_physical_id_order_and_calibrated_endpoints(self):
        np.testing.assert_allclose(motor_to_angles([1000, 120, 401]),
                                   [90.8, -180, -23.6])

    def test_independent_forward_kinematics_agrees_with_inverse_solver(self):
        solver = MH6PalmSolver()
        for u in [(0, 0, 0), (.2, .5, .6), (.9, 1, 1)]:
            angles = solver.map_workspace_conditional(*u)
            for _, _, a1, _, _ in solver.solve_remaining(*angles):
                active = [angles[0], angles[1], a1]
                roots, _ = passive_candidates(solver, active)
                self.assertTrue(roots)
                self.assertLess(min(abs(p[0]-angles[2]) for p in roots), 1e-7)
                for p in roots:
                    self.assertLess(pose_residual(solver, active, p)["rotation_max_abs"], 1e-12)
                    self.assertLess(pose_residual(solver, active, p)["translation_mm"], 1e-4)

    def test_integer_command_inside_motor_limits_can_be_outside_rigid_workspace(self):
        # Rounded trajectory example. cos(theta2)>1 independently proves that
        # individual motor bounds do not establish rigid closed-chain feasibility.
        result = fit_passive(MH6PalmSolver(), [304, 328, 423], [-14, 0, -34])
        self.assertGreater(result["analytic_fk_cos_theta2"], 1.001)
        self.assertFalse(result["numerically_closed"])
        self.assertEqual(result["reason"], "no_real_passive_solution_in_rigid_model")

    def test_memory_sender_packs_real_driver_commands_and_rejects_before_write(self):
        sender = memory_sender()
        mapping = {"low_dim": {"u_"+f: .5 for f in
                              ("thumb", "index", "middle", "ring", "little")}}
        self.assertTrue(sender.send(mapping, SimpleNamespace(selected_motor=[247.2, 500.4, 499.6])))
        writes = sender.hand.client.writes
        self.assertEqual(len(writes), 2)
        self.assertEqual(writes[0]["address"], 20)
        block = writes[0]["values"]
        self.assertEqual([block[13], block[16], block[19]], [247, 500, 500])
        self.assertEqual([block[14], block[17], block[20]], [80, 80, 80])
        with self.assertRaises(ValueError):
            sender.send(mapping, SimpleNamespace(selected_motor=[247, 500, 537]))
        self.assertEqual(len(writes), 2)


if __name__ == "__main__":
    unittest.main()
