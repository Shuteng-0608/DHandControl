"""
给定 arpha2, arpha3, theta1 (deg)，显式求解 theta2, theta3, arpha1 (deg)。

变量命名：
  arpha2 (已知) - Wb->W1 的 Rz 转角 (R1 = Ry(80)*Rz(arpha2))
  arpha3 (已知) - W4->W5 的 Rz 转角 (R5 = Ry(20)*Rz(arpha3))
  theta1 (已知) - W1->W2 的 Rz 转角 (R2 = Ry(-80)*Rz(theta1))
  theta2 (待求) - W2->W3 的 Rz 转角 (R3 = Ry(80)*Rz(theta2))
  theta3 (待求) - W3->W4 的 Rz 转角 (R4 = Ry(35)*Rz(theta3))
  arpha1 (待求) - W5->Wb 的 Rz 转角 (R6 = Ry(225)*Rz(arpha1))

核心推导:
  约束 R1*R2*R3*R4*R5*R6 = I
  -> R3*R4*R5*R6 = (R1*R2)^T = P

  取 C = Ry(-80) * P
  -> C = Rz(theta2)*Ry(35)*Rz(theta3)*Ry(20)*Rz(arpha3)*Ry(225)*Rz(arpha1)

  Rz(arpha1) 不影响第三列:
  C[:,2] = Rz(t2)*Ry(35)*Rz(t3)*Ry(20)*Rz(a3)*Ry(225)*[0,0,1]^T

  令 v = Ry(20)*Rz(a3)*Ry(225)*[0,0,1]^T  (已知)
  w = Ry(35)*Rz(t3)*v
  C[:,2] = Rz(t2)*w

  C[2,2] = w_z = -sin35*(v_x*cos(t3) - v_y*sin(t3)) + cos35*v_z
  -> cos(t3 + φ) = (cos35*v_z - C[2,2]) / (sin35 * sqrt(v_x^2+v_y^2))
  其中 φ = atan2(v_y, v_x)

  已知 t3 -> theta2 = atan2(C[1,2]*w_x - C[0,2]*w_y,
                            C[0,2]*w_x + C[1,2]*w_y)

  已知 t2,t3 -> arpha1 = atan2(J[1,0], J[1,1])
  其中 J = Ry(-225)*(R1*R2*R3*R4*R5)^T

归一化输入映射:
  默认 workspace_conditional：映射到主工作空间，并根据 arpha2/theta1
  动态计算可闭合的 arpha3 区间，避免把归一化输入送入机构无解区。

  兼容模式 motor_range（旧版电机全行程映射）:
    u1,u2,u3 ∈ [0,1] -> arpha2 = 121.9*u1 - 31.1
                         arpha3 = -239*u2 + 59
                         theta1 = -32.3*u3 + 8.6

  保守独立模式 workspace_independent:
    arpha2 ∈ [5°, 85°], arpha3 ∈ [-5°, -35°], theta1 ∈ [-2°, -23.7°]

物理限位 (基于电机标定):
  arpha2  ∈ [-31.1, 90.8]   (输入硬限位)
  arpha3  ∈ [-180, 59]      (输入硬限位)
  theta1  ∈ [-23.7, 8.6]    (输入硬限位)
  theta2  ∈ 无限制
  arpha1  ∈ 无限制

电机输入值换算 (三组独立标定):
  arpha1: 500 + (99.0/23.6) * arpha1      0°→500, -23.6°→401
  arpha2: 500 - (380.0/90.8) * arpha2      0°→500, 90.8°→120   (原始 arpha2)
  arpha3: 247 - (753.0/180.0) * arpha3     0°→247, -180°→1000

输出约定:
  arpha2* = -arpha2 （即输出时 arpha2 取负）

用法:
  from mh6_palm_solver.solve_from_arpha2_arpha3_theta1 import MH6PalmSolver

  solver = MH6PalmSolver()

  # 电机值输出（推荐）
  solutions = solver.solve_motor(arpha2, arpha3, theta1)        # 角度输入
  solutions = solver.solve_motor_from_normalized(u1, u2, u3)    # 默认工作空间条件映射
  legacy = solver.solve_motor_from_normalized(u1, u2, u3, mode="motor_range")
  # 返回 [[motor1, motor2, motor3], ...]

  # 角度输出
  solutions = solver.solve_arpha(arpha2, arpha3, theta1)        # 角度输入
  solutions = solver.solve_arpha_from_normalized(u1, u2, u3)    # [0,1]归一化输入
  # 返回 [[arpha1, arpha2*, arpha3], ...]
"""
import math
import numpy as np


class MH6PalmSolver:

    MAPPING_WORKSPACE_CONDITIONAL = "workspace_conditional"
    MAPPING_WORKSPACE_INDEPENDENT = "workspace_independent"
    MAPPING_MOTOR_RANGE = "motor_range"

    MOTOR_ORDER_API = "api"                  # [arpha1 motor, arpha2 motor, arpha3 motor]
    MOTOR_ORDER_TIMESERIES = "timeseries"    # 历史 CSV: [arpha3 motor, arpha2 motor, arpha1 motor]
    DEFAULT_MOTOR_BOUNDS = (0.0, 1000.0)

    def __init__(self):
        # 常量
        self.C35 = math.cos(math.radians(35))
        self.S35 = math.sin(math.radians(35))
        self.C20 = math.cos(math.radians(20))
        self.S20 = math.sin(math.radians(20))
        self.C225 = math.cos(math.radians(225))
        self.S225 = math.sin(math.radians(225))
        self.C80 = math.cos(math.radians(80))
        self.S80 = math.sin(math.radians(80))

        self.P1 = self.p_from_RyTz(-61.83241, 56.9724)
        self.P2 = self.p_from_RyTz(-39.03855, 76.61523)
        self.P3 = self.p_from_RyTz(125.90102, 67.19535)
        self.P4 = self.p_from_RyTz(114.76245, 32.21445)
        self.P5 = self.p_from_RyTz(135.16303, 19.2719)
        self.P6 = self.p_from_RyTz(119.75471, 29.11805)

    def RyRz(self, phi_deg, psi_deg):
        """Ry(phi)*Rz(psi) rotation matrix"""
        cp = math.cos(math.radians(phi_deg))
        sp = math.sin(math.radians(phi_deg))
        cq = math.cos(math.radians(psi_deg))
        sq = math.sin(math.radians(psi_deg))
        return np.array([
            [cp*cq, -cp*sq,  sp],
            [   sq,     cq,   0],
            [-sp*cq,  sp*sq,  cp]
        ])

    def p_from_RyTz(self, phi_deg, d):
        sa = math.sin(math.radians(phi_deg))
        ca = math.cos(math.radians(phi_deg))
        return np.array([d * sa, 0, d * ca])

    def compute_translation_error(self, arpha2_deg, arpha3_deg, arpha1_deg, theta1_deg, theta2_deg, theta3_deg):
        """计算平移约束误差 |p_total|"""
        R1 = self.RyRz(80, arpha2_deg)
        R2 = self.RyRz(-80, theta1_deg)
        R3 = self.RyRz(80, theta2_deg)
        R4 = self.RyRz(35, theta3_deg)
        R5 = self.RyRz(20, arpha3_deg)
        R6 = self.RyRz(225, arpha1_deg)
        R12 = R1 @ R2; R123 = R12 @ R3; R1234 = R123 @ R4; R12345 = R1234 @ R5
        p_total = self.P1 + R1 @ self.P2 + R12 @ self.P3 + R123 @ self.P4 + R1234 @ self.P5 + R12345 @ self.P6
        return np.linalg.norm(p_total)

    def check_theta1_range(self, t1_deg, verbose=False):
        """Check theta1 input limit [-23.7, 8.6]"""
        ok = -23.7 <= t1_deg <= 8.6
        if verbose:
            print(f"  theta1 = {t1_deg:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [-23.7, 8.6])")
        return ok

    def _norm_to_180(self, deg):
        value = float(deg)
        # 保留已在规范区间内的浮点值，避免无意义的模运算舍入改变双分支排序。
        if -180.0 <= value < 180.0:
            return value
        return (value + 180.0) % 360.0 - 180.0

    def canonicalize_input_angles(self, arpha2_deg, arpha3_deg, theta1_deg):
        """返回用于求解和标定的规范输入角。

        ``arpha2`` 和 ``arpha3`` 是周期角，统一归一化到 ``[-180, 180)``。
        ``theta1`` 有机械限位但不按周期角处理。这样可避免例如 ``arpha3=280``
        在闭环求解中等价于 ``-80``，却在电机标定中仍按 ``280`` 计算的问题。
        """
        values = tuple(float(v) for v in (arpha2_deg, arpha3_deg, theta1_deg))
        if not all(math.isfinite(v) for v in values):
            raise ValueError("all angles must be finite")
        return self._norm_to_180(values[0]), self._norm_to_180(values[1]), values[2]

    def check_arpha2_range(self, a2_deg, verbose=False):
        """Check arpha2 input limit [-31.1, 90.8]"""
        v = self._norm_to_180(a2_deg)
        ok = -31.1 <= v <= 90.8
        if verbose:
            print(f"  arpha2 = {v:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [-31.1, 90.8])")
        return ok

    def check_arpha3_range(self, a3_deg, verbose=False):
        """Check arpha3 input limit [-180, 59]"""
        v = self._norm_to_180(a3_deg)
        ok = -180 <= v <= 59
        if verbose:
            print(f"  arpha3 = {v:.2f} deg -> {'OK' if ok else 'OUT OF RANGE'} (limit [-180, 59])")
        return ok

    def _validate_normalized(self, u1, u2, u3):
        values = tuple(float(v) for v in (u1, u2, u3))
        if not all(math.isfinite(v) and 0.0 <= v <= 1.0 for v in values):
            raise ValueError("u1, u2 and u3 must be finite values in [0,1]")
        return values

    def map_motor_range_normalized(self, u1, u2, u3):
        """旧版映射：[0,1]^3 独立覆盖三个电机标定行程。"""
        u1, u2, u3 = self._validate_normalized(u1, u2, u3)
        return 121.9 * u1 - 31.1, -239.0 * u2 + 59.0, -32.3 * u3 + 8.6

    def map_workspace_independent(self, u1, u2, u3):
        """保守独立映射；实测连续空间覆盖率约 94.9%，但不保证必有解。"""
        u1, u2, u3 = self._validate_normalized(u1, u2, u3)
        return 5.0 + 80.0 * u1, -5.0 - 30.0 * u2, -2.0 - 21.7 * u3

    def closure_value(self, arpha2_deg, arpha3_deg, theta1_deg):
        """返回解析求解中的余弦右端项；刚性旋转闭合要求 abs(value) <= 1。"""
        a2 = np.deg2rad(np.asarray(arpha2_deg, dtype=float))
        a3 = np.deg2rad(np.asarray(arpha3_deg, dtype=float))
        t1 = np.deg2rad(np.asarray(theta1_deg, dtype=float))

        ca2, sa2 = np.cos(a2), np.sin(a2)
        ct1, st1 = np.cos(t1), np.sin(t1)
        c_zz = self.S80**2 * (
            self.C80 * (ct1 * (1.0 - ca2) + ca2) + sa2 * st1
        ) + self.C80**3

        ca3, sa3 = np.cos(a3), np.sin(a3)
        vx = self.C20 * self.S225 * ca3 + self.S20 * self.C225
        vy = self.S225 * sa3
        vz = -self.S20 * self.S225 * ca3 + self.C20 * self.C225
        rv = np.hypot(vx, vy)
        value = (self.C35 * vz - c_zz) / (self.S35 * rv)
        return float(value) if np.ndim(value) == 0 else value

    def feasible_arpha3_interval(
            self, arpha2_deg, theta1_deg, target_abs_value=1.0):
        """解析计算最宽的 arpha3 区间，可要求远离 abs(value)=1 的边界。

        closure_value 仅通过 v_z 依赖 arpha3，而
        v_z = -S225 * (S20*cos(arpha3)-C20)。将
        abs(closure_value)<=target_abs_value 平方后得到关于 v_z 的二次不等式，
        因此无需逐点扫描或数值求根。
        """
        if not 0.0 < target_abs_value <= 1.0:
            raise ValueError("target_abs_value must lie in (0,1]")

        a2 = math.radians(float(arpha2_deg))
        t1 = math.radians(float(theta1_deg))
        ca2, sa2 = math.cos(a2), math.sin(a2)
        ct1, st1 = math.cos(t1), math.sin(t1)
        c_zz = self.S80**2 * (
            self.C80 * (ct1 * (1.0 - ca2) + ca2) + sa2 * st1
        ) + self.C80**3

        k2 = target_abs_value**2
        qa = self.C35**2 + k2 * self.S35**2
        qb = -2.0 * self.C35 * c_zz
        qc = c_zz**2 - k2 * self.S35**2
        discriminant = qb**2 - 4.0 * qa * qc
        if discriminant < -1e-14:
            return None
        root = math.sqrt(max(0.0, discriminant))
        z_lower = (-qb - root) / (2.0 * qa)
        z_upper = (-qb + root) / (2.0 * qa)

        radial_scale = -self.S225
        cos_lower = (z_lower / radial_scale + self.C20) / self.S20
        cos_upper = (z_upper / radial_scale + self.C20) / self.S20
        cos_lower = max(-1.0, cos_lower)
        cos_upper = min(1.0, cos_upper)
        if cos_lower > cos_upper:
            return None

        acos_lower = math.degrees(math.acos(cos_lower))
        acos_upper = math.degrees(math.acos(cos_upper))
        intervals = []

        negative = (max(-180.0, -acos_lower), min(0.0, -acos_upper))
        if negative[0] <= negative[1]:
            intervals.append(negative)
        positive = (max(0.0, acos_upper), min(59.0, acos_lower))
        if positive[0] <= positive[1]:
            intervals.append(positive)
        if len(intervals) == 2 and abs(intervals[0][1] - intervals[1][0]) < 1e-12:
            intervals = [(intervals[0][0], intervals[1][1])]
        if not intervals:
            return None
        return max(intervals, key=lambda pair: pair[1] - pair[0])

    def map_workspace_conditional(
            self, u1, u2, u3, inward_fraction=1e-6,
            target_abs_value=0.999):
        """映射到主工作空间，并将 u2 映射到条件可行的 arpha3 区间。"""
        u1, u2, u3 = self._validate_normalized(u1, u2, u3)
        if not 0.0 <= inward_fraction < 0.5:
            raise ValueError("inward_fraction must lie in [0,0.5)")
        arpha2 = 5.0 + 80.0 * u1
        theta1 = -2.0 - 21.7 * u3
        interval = self.feasible_arpha3_interval(
            arpha2, theta1, target_abs_value=target_abs_value)
        if interval is None:
            raise RuntimeError("workspace base map has no feasible arpha3 interval")
        lower, upper = interval
        q = inward_fraction + (1.0 - 2.0 * inward_fraction) * u2
        arpha3 = upper - q * (upper - lower)
        return arpha2, arpha3, theta1

    def map_normalized(self, u1, u2, u3, mode=MAPPING_WORKSPACE_CONDITIONAL):
        """按指定模式将 [0,1]^3 映射到 (arpha2, arpha3, theta1)。"""
        if mode == self.MAPPING_WORKSPACE_CONDITIONAL:
            return self.map_workspace_conditional(u1, u2, u3)
        if mode == self.MAPPING_WORKSPACE_INDEPENDENT:
            return self.map_workspace_independent(u1, u2, u3)
        if mode == self.MAPPING_MOTOR_RANGE:
            return self.map_motor_range_normalized(u1, u2, u3)
        raise ValueError(
            f"unknown mapping mode {mode!r}; expected one of "
            f"{self.MAPPING_WORKSPACE_CONDITIONAL!r}, "
            f"{self.MAPPING_WORKSPACE_INDEPENDENT!r}, {self.MAPPING_MOTOR_RANGE!r}")

    def solve_remaining(self, arpha2_deg, arpha3_deg, theta1_deg):
        """
        给定 arpha2, arpha3, theta1, 求解 theta2, theta3, arpha1.

        返回 [(theta2, theta3, arpha1, rotation_error, translation_error), ...]
        按旋转误差排序。若无解返回 [].
        """
        try:
            arpha2_deg, arpha3_deg, theta1_deg = self.canonicalize_input_angles(
                arpha2_deg, arpha3_deg, theta1_deg)
        except (TypeError, ValueError):
            return []

        # ---- theta1 physical limit check ----
        if not self.check_theta1_range(theta1_deg):
            return []

        # ---- arpha2/arpha3 input limit check ----
        if not self.check_arpha2_range(arpha2_deg) or not self.check_arpha3_range(arpha3_deg):
            return []

        # ---- known rotations ----
        R1 = self.RyRz(80, arpha2_deg)
        R2 = self.RyRz(-80, theta1_deg)
        R5 = self.RyRz(20, arpha3_deg)

        # ---- P = (R1*R2)^T ----
        R12 = R1 @ R2
        P_mat = R12.T

        # ---- C = Ry(-80) * P ----
        Ry_neg80 = self.RyRz(-80, 0)
        C = Ry_neg80 @ P_mat

        # ---- v = Ry(20)*Rz(arpha3)*Ry(225)*[0,0,1]^T ----
        e3 = np.array([0.0, 0.0, 1.0])
        s3 = self.RyRz(225, 0) @ e3  # Ry(225)*[0,0,1]^T
        u = self.RyRz(0, arpha3_deg) @ s3  # Rz(arpha3)*s3
        v = self.RyRz(20, 0) @ u  # Ry(20)*u

        vx, vy, vz = v[0], v[1], v[2]

        # ---- solve for theta3 ----
        Rv = math.hypot(vx, vy)
        if Rv < 1e-15:
            return []
        phi = math.atan2(vy, vx)

        val = (self.C35 * vz - C[2, 2]) / (self.S35 * Rv)
        if abs(val) > 1.0 + 1e-12:
            return []
        val = max(-1.0, min(1.0, val))

        phi_t3 = math.acos(val)
        t3_candidates = [-phi + phi_t3, -phi - phi_t3]

        results = []
        for t3_val in t3_candidates:
            t3_deg = math.degrees(t3_val) % 360

            # ---- w = Ry(35)*Rz(t3)*v ----
            Rz_t3 = self.RyRz(0, t3_deg)
            w = self.RyRz(35, 0) @ (Rz_t3 @ v)
            wx, wy = w[0], w[1]

            # ---- theta2 = atan2(...) ----
            cx, cy = C[0, 2], C[1, 2]
            denom = cx * wx + cy * wy
            numer = cy * wx - cx * wy
            t2_val = math.atan2(numer, denom)
            t2_deg = math.degrees(t2_val)
            t2_deg_norm = (t2_deg + 180) % 360 - 180

            # ---- arpha1 = atan2(J[1,0], J[1,1]) ----
            R3 = self.RyRz(80, t2_deg)
            R4 = self.RyRz(35, t3_deg)
            R12345 = R1 @ R2 @ R3 @ R4 @ R5
            J = self.RyRz(-225, 0) @ R12345.T
            a1_val = math.atan2(J[1, 0], J[1, 1])
            a1_deg = math.degrees(a1_val)
            a1_deg_norm = (a1_deg + 180) % 360 - 180

            # ---- verify rotation ----
            R6 = self.RyRz(225, a1_deg)
            Rloop = R1 @ R2 @ R3 @ R4 @ R5 @ R6
            rot_err = np.max(np.abs(Rloop - np.eye(3)))

            # ---- verify translation ----
            trans_err = self.compute_translation_error(
                arpha2_deg, arpha3_deg, a1_deg, theta1_deg, t2_deg, t3_deg)

            results.append((t2_deg_norm, t3_deg, a1_deg_norm, rot_err, trans_err))

        # dedup + sort by error
        unique = []
        for r in results:
            if not any(abs(r[0]-u[0])<1e-6 and abs(r[1]-u[1])<1e-6 and abs(r[2]-u[2])<1e-6 for u in unique):
                unique.append(r)
        unique.sort(key=lambda x: x[3])
        return unique

    def solve_from_normalized(self, u1, u2, u3, mode=MAPPING_WORKSPACE_CONDITIONAL):
        """
        从归一化输入 [0,1]³ 直接求解。

        返回 [(theta2, theta3, arpha1, rot_err, trans_err), ...]
        """
        a2, a3, t1 = self.map_normalized(u1, u2, u3, mode=mode)
        return self.solve_remaining(a2, a3, t1)

    def solve_arpha(self, arpha2_deg, arpha3_deg, theta1_deg):
        """
        角度输入，仅输出 arpha 三元组，不含误差信息。

        返回: [[arpha1, arpha2, arpha3], ...]  每个解一个三元组
        """
        try:
            arpha2_deg, arpha3_deg, theta1_deg = self.canonicalize_input_angles(
                arpha2_deg, arpha3_deg, theta1_deg)
        except (TypeError, ValueError):
            return []
        raw = self.solve_remaining(arpha2_deg, arpha3_deg, theta1_deg)
        return [[self._norm_to_180(r[2]), -arpha2_deg, arpha3_deg] for r in raw]

    def solve_arpha_from_normalized(self, u1, u2, u3, mode=MAPPING_WORKSPACE_CONDITIONAL):
        """
        归一化输入，仅输出 arpha 三元组，不含误差信息。

        返回: [[arpha1, arpha2, arpha3], ...]  每个解一个三元组
        """
        a2, a3, t1 = self.map_normalized(u1, u2, u3, mode=mode)
        return self.solve_arpha(a2, a3, t1)

    # ---- 电机值转换 ----

    def solve_motor(self, arpha2_deg, arpha3_deg, theta1_deg):
        """
        角度输入，输出三个电机的输入值。

        注意: arpha2 用原始角度（未取负）计算电机值。

        校准:
          arpha1: 0°→500, -23.6°→401
          arpha2: 0°→500, 90.8°→120   (原始 arpha2，非 arpha2*)
          arpha3: 0°→247, -180°→1000

        返回: [[motor1, motor2, motor3], ...]  每个解一个三元组
        """
        try:
            arpha2_deg, arpha3_deg, theta1_deg = self.canonicalize_input_angles(
                arpha2_deg, arpha3_deg, theta1_deg)
        except (TypeError, ValueError):
            return []
        raw = self.solve_arpha(arpha2_deg, arpha3_deg, theta1_deg)
        results = []
        for a1, a2_star, a3 in raw:
            # a1 = arpha1 (已归一化到 -180~180)
            # a2_star = -arpha2 (已取负)
            # a3 = arpha3
            m1 = 500 + (99.0 / 23.6) * a1
            m2 = 500 - (380.0 / 90.8) * arpha2_deg  # 用原始 arpha2
            m3 = 247 - (753.0 / 180.0) * arpha3_deg
            results.append([round(m1, 4), round(m2, 4), round(m3, 4)])
        return results

    def solve_motor_from_normalized(self, u1, u2, u3, mode=MAPPING_WORKSPACE_CONDITIONAL):
        """
        归一化输入，输出三个电机的输入值。

        返回: [[motor1, motor2, motor3], ...]  每个解一个三元组
        """
        a2, a3, t1 = self.map_normalized(u1, u2, u3, mode=mode)
        return self.solve_motor(a2, a3, t1)

    # ---- 诊断、投影和连续轨迹接口 ----

    def diagnose_input(self, arpha2_deg, arpha3_deg, theta1_deg):
        """解释输入为何有解或无解，不再只返回静默的空列表。"""
        try:
            a2, a3, t1 = self.canonicalize_input_angles(
                arpha2_deg, arpha3_deg, theta1_deg)
        except (TypeError, ValueError):
            return {
                "valid": False,
                "reason": "non_finite_input",
                "message": "all angles must be finite",
                "limit_violations": ["non_finite"],
                "closure_value": None,
                "closure_margin": None,
            }

        violations = []
        if not -31.1 <= a2 <= 90.8:
            violations.append("arpha2")
        if not -180.0 <= a3 <= 59.0:
            violations.append("arpha3")
        if not -23.7 <= t1 <= 8.6:
            violations.append("theta1")
        if violations:
            return {
                "valid": False,
                "reason": "input_limit",
                "message": "one or more input angles are outside calibrated limits",
                "limit_violations": violations,
                "closure_value": None,
                "closure_margin": None,
            }

        value = self.closure_value(a2, a3, t1)
        margin = 1.0 - abs(value)
        valid = abs(value) <= 1.0 + 1e-12
        return {
            "valid": valid,
            "reason": "ok" if valid else "rotational_workspace",
            "message": (
                "input lies inside the rigid-link rotational workspace"
                if valid else
                "no rigid rotational closure exists because the acos argument lies outside [-1,1]"
            ),
            "limit_violations": [],
            "closure_value": value,
            "closure_margin": margin,
        }

    def motor_range_normalized_from_angles(self, arpha2_deg, arpha3_deg, theta1_deg):
        """旧电机全行程映射的逆变换，用作投影距离的无量纲坐标。"""
        arpha2_deg, arpha3_deg, theta1_deg = self.canonicalize_input_angles(
            arpha2_deg, arpha3_deg, theta1_deg)
        return (
            (arpha2_deg + 31.1) / 121.9,
            (59.0 - arpha3_deg) / 239.0,
            (8.6 - theta1_deg) / 32.3,
        )

    def _closure_value_from_motor_u(self, u):
        a2, a3, t1 = self.map_motor_range_normalized(*u)
        return self.closure_value(a2, a3, t1)

    def project_angles_to_workspace(
            self, arpha2_deg, arpha3_deg, theta1_deg,
            target_abs_value=0.999, max_iterations=50):
        """将限位内输入局部投影到闭合工作空间内部，并显式返回角度偏差。

        投影距离在旧电机全行程归一化坐标中计算。target_abs_value 小于 1
        可避免把轨迹放在两个解析分支合并的 acos 奇异边界上。算法沿闭合
        判据梯度迭代，不保证得到全局欧氏最近的可行点。
        """
        if not 0.0 < target_abs_value < 1.0:
            raise ValueError("target_abs_value must lie in (0,1)")
        requested_angles = np.array(
            [float(arpha2_deg), float(arpha3_deg), float(theta1_deg)], dtype=float)
        diagnostic = self.diagnose_input(*requested_angles)
        if diagnostic["reason"] in ("input_limit", "non_finite_input"):
            raise ValueError(diagnostic["message"])

        original_angles = np.array(
            self.canonicalize_input_angles(*requested_angles), dtype=float)
        u = np.array(self.motor_range_normalized_from_angles(*original_angles), dtype=float)
        if np.any(u < -1e-12) or np.any(u > 1.0 + 1e-12):
            raise ValueError("input angles must lie inside calibrated motor limits")
        u = np.clip(u, 0.0, 1.0)
        original_u = u.copy()
        before = self._closure_value_from_motor_u(u)
        if abs(before) <= target_abs_value:
            return {
                "projected": False,
                "requested_angles": requested_angles.tolist(),
                "canonical_requested_angles": original_angles.tolist(),
                "angles": original_angles.tolist(),
                "closure_value_before": before,
                "closure_value_after": before,
                "normalized_distance": 0.0,
                "angle_delta_deg": [0.0, 0.0, 0.0],
            }

        target = math.copysign(target_abs_value, before)
        for _ in range(int(max_iterations)):
            residual = self._closure_value_from_motor_u(u) - target
            if abs(residual) < 1e-11:
                break
            h = 1e-5
            gradient = np.empty(3)
            for axis in range(3):
                upper, lower = u.copy(), u.copy()
                upper[axis] = min(1.0, upper[axis] + h)
                lower[axis] = max(0.0, lower[axis] - h)
                gradient[axis] = (
                    self._closure_value_from_motor_u(upper)
                    - self._closure_value_from_motor_u(lower)
                ) / (upper[axis] - lower[axis])
            norm_squared = float(gradient @ gradient)
            if norm_squared < 1e-16:
                raise RuntimeError("workspace projection gradient vanished")
            step = -residual * gradient / norm_squared
            old_error = abs(residual)
            scale = 1.0
            for _ in range(20):
                candidate = np.clip(u + scale * step, 0.0, 1.0)
                if abs(self._closure_value_from_motor_u(candidate) - target) < old_error:
                    u = candidate
                    break
                scale *= 0.5
            else:
                raise RuntimeError("workspace projection line search stalled")
        else:
            raise RuntimeError("workspace projection did not converge")

        after = self._closure_value_from_motor_u(u)
        if abs(after) > 1.0 + 1e-10:
            raise RuntimeError("workspace projection ended outside the feasible set")
        projected_angles = np.array(self.map_motor_range_normalized(*u), dtype=float)
        delta = projected_angles - original_angles
        return {
            "projected": True,
            "requested_angles": requested_angles.tolist(),
            "canonical_requested_angles": original_angles.tolist(),
            "angles": projected_angles.tolist(),
            "closure_value_before": before,
            "closure_value_after": after,
            "normalized_distance": float(np.linalg.norm(u - original_u)),
            "angle_delta_deg": delta.tolist(),
        }

    def reorder_motor_solution(self, solution, motor_order=MOTOR_ORDER_API):
        values = [float(v) for v in solution]
        if len(values) != 3:
            raise ValueError("a motor solution must contain exactly three values")
        if not all(math.isfinite(v) for v in values):
            raise ValueError("a motor solution must contain three finite values")
        if motor_order == self.MOTOR_ORDER_API:
            return values
        if motor_order == self.MOTOR_ORDER_TIMESERIES:
            return [values[2], values[1], values[0]]
        raise ValueError("motor_order must be 'api' or 'timeseries'")

    def select_continuous_motor_solution(
            self, solutions, previous_motor=None, motor_order=MOTOR_ORDER_API):
        """选择与上一帧电机向量欧氏距离最小的解析分支。"""
        ordered = [self.reorder_motor_solution(s, motor_order) for s in solutions]
        if not ordered:
            return None, None
        if previous_motor is None:
            reference = np.array(
                [500.0, 500.0, 247.0]
                if motor_order == self.MOTOR_ORDER_API else
                [247.0, 500.0, 500.0], dtype=float)
        else:
            reference = np.asarray(previous_motor, dtype=float)
            if reference.shape != (3,) or not np.all(np.isfinite(reference)):
                raise ValueError("previous_motor must contain three finite values")
        matrix = np.asarray(ordered, dtype=float)
        index = int(np.argmin(np.linalg.norm(matrix - reference[None, :], axis=1)))
        return ordered[index], index

    def solve_motor_safe(
            self, arpha2_deg, arpha3_deg, theta1_deg, previous_motor=None,
            project_invalid=False, enforce_margin=False, target_abs_value=0.999,
            motor_order=MOTOR_ORDER_API, max_projection_delta_deg=None,
            motor_bounds=DEFAULT_MOTOR_BOUNDS):
        """带诊断、显式投影许可、输出限幅和连续分支选择的安全接口。

        默认不修改无解输入。若调用方显式启用 ``project_invalid``，建议同时传入
        ``max_projection_delta_deg=[d_arpha2, d_arpha3, d_theta1]``；超过任一阈值时
        返回失败而不是生成电机命令。``motor_bounds`` 默认要求每路控制值均在
        ``[0,1000]``，传入 ``None`` 可关闭该软件层检查。
        """
        requested = [float(arpha2_deg), float(arpha3_deg), float(theta1_deg)]
        diagnostic = self.diagnose_input(*requested)
        canonical_requested = None
        if diagnostic["reason"] != "non_finite_input":
            canonical_requested = list(self.canonicalize_input_angles(*requested))

        projection_limits = None
        if max_projection_delta_deg is not None:
            projection_limits = np.asarray(max_projection_delta_deg, dtype=float)
            if (projection_limits.shape != (3,)
                    or not np.all(np.isfinite(projection_limits))
                    or np.any(projection_limits <= 0.0)):
                raise ValueError(
                    "max_projection_delta_deg must contain three finite positive values")

        validated_motor_bounds = None
        if motor_bounds is not None:
            validated_motor_bounds = tuple(float(v) for v in motor_bounds)
            if (len(validated_motor_bounds) != 2
                    or not all(math.isfinite(v) for v in validated_motor_bounds)
                    or validated_motor_bounds[0] >= validated_motor_bounds[1]):
                raise ValueError("motor_bounds must be a finite increasing pair")

        if diagnostic["reason"] in ("input_limit", "non_finite_input"):
            return {
                "success": False,
                "projected": False,
                "requested_angles": requested,
                "canonical_requested_angles": canonical_requested,
                "used_angles": None,
                "diagnostic": diagnostic,
                "used_diagnostic": None,
                "solutions": [],
                "rejected_motor_solutions": [],
                "selected": None,
                "selected_index": None,
                "projection": None,
                "projection_within_limits": None,
                "error": diagnostic["reason"],
            }

        needs_projection = (
            (not diagnostic["valid"] and project_invalid)
            or (enforce_margin and abs(diagnostic["closure_value"]) > target_abs_value)
        )
        projection = None
        used = canonical_requested
        if needs_projection:
            try:
                projection = self.project_angles_to_workspace(
                    *requested, target_abs_value=target_abs_value)
            except RuntimeError as exc:
                return {
                    "success": False,
                    "projected": False,
                    "requested_angles": requested,
                    "canonical_requested_angles": canonical_requested,
                    "used_angles": None,
                    "diagnostic": diagnostic,
                    "used_diagnostic": None,
                    "solutions": [],
                    "rejected_motor_solutions": [],
                    "selected": None,
                    "selected_index": None,
                    "projection": None,
                    "projection_within_limits": None,
                    "error": f"workspace_projection_failed: {exc}",
                }
            used = projection["angles"]
            if (projection_limits is not None
                    and np.any(np.abs(projection["angle_delta_deg"]) > projection_limits)):
                return {
                    "success": False,
                    "projected": True,
                    "requested_angles": requested,
                    "canonical_requested_angles": canonical_requested,
                    "used_angles": None,
                    "diagnostic": diagnostic,
                    "used_diagnostic": self.diagnose_input(*used),
                    "solutions": [],
                    "rejected_motor_solutions": [],
                    "selected": None,
                    "selected_index": None,
                    "projection": projection,
                    "projection_within_limits": False,
                    "error": "projection_delta_limit_exceeded",
                }
        elif not diagnostic["valid"]:
            return {
                "success": False,
                "projected": False,
                "requested_angles": requested,
                "canonical_requested_angles": canonical_requested,
                "used_angles": None,
                "diagnostic": diagnostic,
                "used_diagnostic": None,
                "solutions": [],
                "rejected_motor_solutions": [],
                "selected": None,
                "selected_index": None,
                "projection": None,
                "projection_within_limits": None,
                "error": diagnostic["reason"],
            }

        all_api_solutions = self.solve_motor(*used)
        rejected_motor_solutions = []
        api_solutions = []
        for solution in all_api_solutions:
            if (validated_motor_bounds is not None
                    and not all(validated_motor_bounds[0] <= value <= validated_motor_bounds[1]
                                for value in solution)):
                rejected_motor_solutions.append(solution)
            else:
                api_solutions.append(solution)
        solutions = [self.reorder_motor_solution(s, motor_order) for s in api_solutions]
        selected, index = self.select_continuous_motor_solution(
            api_solutions, previous_motor=previous_motor, motor_order=motor_order)
        success = bool(solutions)
        return {
            "success": success,
            "projected": bool(projection and projection["projected"]),
            "requested_angles": requested,
            "canonical_requested_angles": canonical_requested,
            "used_angles": used,
            "diagnostic": diagnostic,
            "used_diagnostic": self.diagnose_input(*used),
            "solutions": solutions,
            "rejected_motor_solutions": rejected_motor_solutions,
            "selected": selected,
            "selected_index": index,
            "projection": projection,
            "projection_within_limits": (
                True if projection and projection_limits is not None else None),
            "error": (
                None if success else
                "motor_bounds_rejected_all_solutions"
                if rejected_motor_solutions else "no_closed_chain_solution"),
        }

    def solve_motor_safe_from_normalized(
            self, u1, u2, u3, mode=MAPPING_WORKSPACE_CONDITIONAL, **kwargs):
        angles = self.map_normalized(u1, u2, u3, mode=mode)
        return self.solve_motor_safe(*angles, **kwargs)


# ============================================================
if __name__ == "__main__":
    print("=" * 60)
    print("arpha2,arpha3,theta1 -> theta2,theta3,arpha1 solver test")
    print("=" * 60)

    mh6_solver = MH6PalmSolver()

    # Test 1: near-zero
    print("\nTest 1: near-zero (arpha2=0, arpha3=0, theta1=0)")
    sols = mh6_solver.solve_remaining(0, 0, 0)
    for t2, t3, a1, rot_err, trans_err in sols:
        print(f"  theta2={t2:.4f} deg, theta3={t3:.4f} deg, arpha1={a1:.4f} deg  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
    if not sols:
        print("  (no solution)")

    # Test 2: normalized input (0,0,0)
    print("\nTest 2: normalized (u1=0, u2=0, u3=0)")
    sols = mh6_solver.solve_from_normalized(0.0, 0.0, 0.0)
    for t2, t3, a1, rot_err, trans_err in sols:
        print(f"  theta2={t2:.4f} deg, theta3={t3:.4f} deg, arpha1={a1:.4f} deg  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
    if not sols:
        print("  (no solution)")

    # Test 3: SolidWorks verification
    print("\nTest 3: SolidWorks (arpha2=47, arpha3=-80, theta1=-20)")
    sols = mh6_solver.solve_remaining(47, -80, -20)
    for t2, t3, a1, rot_err, trans_err in sols:
        t2n = t2 if t2 <= 180 else t2 - 360
        a1n = a1 if a1 <= 180 else a1 - 360
        print(f"  theta2={t2n:.4f} deg, theta3={t3:.4f} deg, arpha1={a1n:.4f} deg  rot_err={rot_err:.2e}  |p|={trans_err:.2e}")
    if not sols:
        print("  (no solution)")

    # Test 4: simplified output solve_arpha
    print("\nTest 4: solve_arpha (arpha2=47, arpha3=-80, theta1=-20)")
    arpha_sols = mh6_solver.solve_arpha(47, -80, -20)
    for i, a_trip in enumerate(arpha_sols):
        print(f"  [{a_trip[0]:.4f}, {a_trip[1]:.0f}, {a_trip[2]:.0f}]")

    print(f"\n{'=' * 60}")
