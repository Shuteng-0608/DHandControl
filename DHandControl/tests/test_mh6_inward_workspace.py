"""Physical inward workspace and assembly continuity, with no device I/O."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from mh6_palm_solver_adapter import PalmSolverAdapter, PalmAdapterCalibration
from mh6_actuator_preview import NineChannelPreview
from mh6_hardware_config import HardwareConfig
from test_mh6_actuator_preview import configured, result


class InwardWorkspaceTest(unittest.TestCase):
    def test_calibrated_motor_output_matches_unchanged_rigid_solver(self):
        adapter = PalmSolverAdapter()
        for h,v,r in np.random.default_rng(123).random((100,3)):
            solved = adapter.solve_motor_from_teleop(h,v,r)
            raw = adapter.solver.solve_motor_safe(*solved["used_angles"],motor_order="timeseries")
            self.assertTrue(solved["success"])
            for motor in solved["solutions"]:
                self.assertIn(motor,raw["solutions"])
            self.assertFalse(solved["projected"])
            self.assertLess(max(root[3] for root in adapter.solver.solve_remaining(*solved["used_angles"])),1e-12)

    def test_grasp_axis_opens_and_closes_in_correct_direction(self):
        adapter = PalmSolverAdapter()
        targets=[adapter.solve_motor_from_teleop(float(h),0.,0.)["solutions"][0]
                 for h in np.linspace(0.,1.,21)]
        self.assertEqual(targets[0],[247.,500.,500.])
        self.assertEqual(targets[-1],[247.,120.,500.])
        self.assertTrue(all(b[1] <= a[1] for a,b in zip(targets,targets[1:])))
        self.assertEqual(adapter.solve_motor_from_teleop(0.,0.,0.)["solutions"],[[247.,500.,500.]])

    def test_tightened_physical_bounds_are_used_during_generation(self):
        limits={1:(247,300),2:(400,500),3:(480,500)}
        adapter=PalmSolverAdapter(motor_limits=limits)
        for h,v,r in np.random.default_rng(456).random((60,3)):
            solved=adapter.solve_motor_from_teleop(h,v,r)
            self.assertTrue(solved["solutions"])
            for motor in solved["solutions"]:
                self.assertTrue(all(limits[i][0] <= value <= limits[i][1] for i,value in enumerate(motor,1)))

    def test_configuration_excluding_the_open_pose_is_rejected(self):
        with self.assertRaisesRegex(ValueError,"open pose"):
            PalmSolverAdapter(motor_limits={1:(300,1000),2:(120,500),3:(401,500)})

    def test_native_profile_remains_available_without_changes(self):
        adapter=PalmSolverAdapter(PalmAdapterCalibration(mapping_mode="workspace_conditional",
            input_domain="neutral_to_grasp", workspace_profile="native", neutral=(0,0,0)))
        solved=adapter.solve_motor_from_teleop(0,0,0)
        self.assertEqual(solved["requested_angles"], [5.,7.796455861146719,-2.])
        self.assertTrue(any(row[0]<247 for row in solved["candidates"]))

    def test_control_cannot_select_a_different_workspace_assembly(self):
        data=configured(); data["fixed_branch"]="minus_acos"
        planner=NineChannelPreview(HardwareConfig(data))
        solved=PalmSolverAdapter().solve_motor_from_teleop(.2,.02,.2)
        proposal=planner.propose(result(),solved)
        self.assertEqual(proposal["palm"]["reason"],"FIXED_BRANCH_UNAVAILABLE")
        self.assertEqual(proposal["palm"]["selected_motor"],[500,500,500])

    def test_small_input_path_has_no_motor_jump_from_the_flat_boundary(self):
        adapter=PalmSolverAdapter()
        path=[adapter.solve_motor_from_teleop(float(h),.1*float(h),.5)["solutions"][0]
              for h in np.linspace(0.,.05,101)]
        self.assertLess(np.max(np.abs(np.diff(path,axis=0))),1.)

    def test_no_capacity_jump_when_the_closed_limit_first_binds(self):
        adapter=PalmSolverAdapter()
        path=[adapter.solve_motor_from_teleop(float(h),1.,1.)["solutions"][0]
              for h in np.linspace(.84,.88,201)]
        self.assertLess(np.max(np.abs(np.diff(path,axis=0))),10.)

    def test_lateral_input_stays_on_the_first_m3_closing_arc(self):
        adapter=PalmSolverAdapter()
        for h in (.1,.5,1.):
            for r in (.1,.5,1.):
                motor=[adapter.solve_motor_from_teleop(h,float(v),r)["solutions"][0][2]
                       for v in np.linspace(0,1,41)]
                self.assertTrue(all(b <= a+.0001 for a,b in zip(motor,motor[1:])))


if __name__ == "__main__": unittest.main()
