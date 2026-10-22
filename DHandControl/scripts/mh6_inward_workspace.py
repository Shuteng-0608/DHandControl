"""Generate inward-only poses using the unchanged rigid-link Solver.

Motor calibration is authoritative: ID order is alpha3/alpha2/alpha1.
Only the plus-acos assembly is parameterized. The admissible lateral span
depends on grasp and thumb input; a box of independent joint angles is not
a closed-chain workspace. No output motor is clipped or rescaled.
"""
from functools import lru_cache
import math

import numpy as np


CLOSING_MOTOR_LIMITS = {1: (247., 1000.), 2: (120., 500.), 3: (401., 500.)}
OPEN_MOTOR = (247., 500., 500.)


class InwardPalmWorkspace:
    branch = "plus_acos"

    def __init__(self, solver, motor_limits):
        self.solver = solver
        for axis, zero in enumerate(OPEN_MOTOR, 1):
            if not motor_limits[axis][0] <= zero <= motor_limits[axis][1]:
                raise ValueError("inward workspace requires the open pose [247,500,500] within motor limits")
        self.a2_max = min(90.8, (500.-motor_limits[2][0])*90.8/380.)
        self.a3_min = max(-180., (247.-motor_limits[1][1])*180./753.)
        self.a1_min = max(-23.6, (motor_limits[3][0]-500.)*23.6/99.)

    def _a3(self, a2, lateral_magnitude, thumb):
        if lateral_magnitude == 0. or a2 == 0.:
            return 0.
        interval = self.solver.feasible_arpha3_interval(a2, -lateral_magnitude,
                                                     target_abs_value=1.)
        if interval is None:
            # The flat pose is an acos double root; tolerate only roundoff
            # confirmed by the original Solver, not an arbitrary fallback pose.
            if self.solver.diagnose_input(a2, 0., -lateral_magnitude)["valid"]:
                return 0.
            return None
        lower, upper = max(self.a3_min, interval[0]), min(0., interval[1])
        if lower > upper:
            return None
        # Stay on the interior side of the geometric branch-merging endpoint.
        # The full open pose itself intentionally remains the exact double root.
        coordinate = math.sin(math.pi*thumb/2.)
        return upper + coordinate*(1.-1e-6)*(lower-upper)

    def _plus_alpha1(self, a2, a3, t1):
        roots = self.solver.solve_remaining(a2, a3, t1)
        if len(roots) == 1:
            return roots[0][2]
        vector = (self.solver.RyRz(20, a3) @ self.solver.RyRz(225, 0)
                  @ np.array([0., 0., 1.]))
        phase = math.atan2(vector[1], vector[0])
        for _, t3, a1, _, _ in roots:
            if math.sin(math.radians(t3)+phase) > 0.:
                return a1
        return None

    def _valid(self, a2, magnitude, thumb):
        a3 = self._a3(a2, magnitude, thumb)
        if a3 is None:
            return False
        a1 = self._plus_alpha1(a2, a3, -magnitude)
        return a1 is not None and self.a1_min-1e-9 <= a1 <= 1e-9

    @lru_cache(maxsize=2048)
    def _lateral_capacity(self, a2, thumb):
        if a2 <= 1e-8:
            return 0.
        # On this assembly alpha1 first bends inward, then can reverse. Use
        # only its first inward arc. Taking an entire connected interval can
        # jump when a narrow forbidden pocket appears around its minimum.
        def alpha1(magnitude):
            a3 = self._a3(a2, magnitude, thumb)
            value = self._plus_alpha1(a2, a3, -magnitude) if a3 is not None else None
            return math.inf if value is None else value

        low, high = 0., 23.7
        ratio = (math.sqrt(5.)-1.)/2.
        left, right = high-ratio*(high-low), low+ratio*(high-low)
        fleft, fright = alpha1(left), alpha1(right)
        for _ in range(40):
            if fleft <= fright:
                high, right, fright = right, left, fleft
                left = high-ratio*(high-low)
                fleft = alpha1(left)
            else:
                low, left, fleft = left, right, fright
                right = low+ratio*(high-low)
                fright = alpha1(right)
        minimum_at = (low+high)/2.
        if alpha1(minimum_at) >= self.a1_min:
            return minimum_at*(1.-1e-6)
        low, high = 0., minimum_at
        for _ in range(28):
            middle = (low+high)/2.
            if self._valid(a2, middle, thumb):
                low = middle
            else:
                high = middle
        return low*(1.-1e-6)

    def map(self, grasp, thumb, lateral):
        values = (grasp, thumb, lateral)
        if any(isinstance(v, bool) or not math.isfinite(v) or not 0. <= v <= 1. for v in values):
            raise ValueError("inward workspace coordinates must lie in [0,1]")
        a2 = self.a2_max*grasp
        capacity = self._lateral_capacity(a2, thumb)
        magnitude = capacity*lateral
        a3 = self._a3(a2, magnitude, thumb)
        if a3 is None or not self._valid(a2, magnitude, thumb):
            raise ValueError("inward workspace has no continuous plus-branch pose for this input")
        return (a2, a3, -magnitude), {
            "profile": "closing_only", "required_branch": self.branch,
            "open_motor": list(OPEN_MOTOR),
            "angle_bounds_deg": {"alpha2": [0., self.a2_max],
                                 "alpha3": [self.a3_min, 0.], "alpha1": [self.a1_min, 0.]},
            "lateral_capacity_deg": capacity,
            "lateral_capacity_policy": "first_inward_arc_or_motor_limit",
            "thumb_coordinate": math.sin(math.pi*thumb/2.),
            "collapsed_thumb_axis": a3 == 0. and thumb > 0.,
            "collapsed_lateral_axis": capacity == 0. and lateral > 0.,
        }
