# MH6 手掌闭环求解器

输入三个已知角度 `arpha2、arpha3、theta1`，解析求解闭环机构的 `theta2、theta3、arpha1`，并可输出项目标定的三路电机控制数值。

完整的模型、接口、安全约束、验证结果与复现实验见 `MH6手掌闭环求解器_程序说明.pdf`；其 LaTeX 源文件位于 `docs/MH6手掌闭环求解器_程序说明.tex`。

## 先看结论

- 直接角度求解接口没有改变。
- 归一化输入的默认含义变了：现在优先映射到机构能闭合的工作空间。
- 如果必须恢复旧版“覆盖全部电机行程”的含义，请显式使用 `mode="motor_range"`。
- 无解角度可以尝试投影到可行域，但安全接口默认不投影；必须显式开启并检查偏差。
- 周期输入角会先规范到 `[-180°,180°)`，避免闭环求解与电机标定使用不同角度表示。
- 安全接口默认拒绝不在 `[0,1000]` 内的电机控制值。

## 安装与快速使用

环境：Python 3.12，NumPy 2.3.5。

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

### 直接输入角度

所有角度单位都是度。

```python
from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver

solver = MH6PalmSolver()

raw = solver.solve_remaining(47, -80, -20)
# 每组：(theta2, theta3, arpha1, 旋转误差, 平移误差)

angles = solver.solve_arpha(47, -80, -20)
# 每组：[arpha1, arpha2*, arpha3]，其中 arpha2* = -arpha2
# [[-4.3990, -47, -80],
#  [-19.0601, -47, -80]]

motors = solver.solve_motor(47, -80, -20)
# 返回两组 [motor1, motor2, motor3]
```

输入硬限位：

| 输入 | 范围 |
|---|---:|
| `arpha2` | `[-31.1°, 90.8°]` |
| `arpha3` | `[-180°, 59°]` |
| `theta1` | `[-23.7°, 8.6°]` |

超限或机构无法闭合时，旧接口返回空列表 `[]`。

### 输入归一化量 `[0,1]³`

```python
motors = solver.solve_motor_from_normalized(0.641, 0.582, 0.885)
# 默认 mode="workspace_conditional"
# [[378.8643, 264.4670, 366.0361],
#  [634.0242, 264.4670, 366.0361]]
```

默认模式会让 `arpha3` 依赖 `arpha2、theta1`，因此三个归一化输入不再分别代表三个电机的固定行程百分比。

## 三种归一化模式

下表覆盖率来自步长 0.05 的 `21³=9261` 点离线网格。

| `mode` | 含义 | 有解点 |
|---|---|---:|
| `workspace_conditional` | 默认；动态选择当前可闭合的 `arpha3` 区间 | `9261/9261`（100%） |
| `workspace_independent` | 三个量独立映射到收缩后的固定范围 | `8604/9261`（92.9%） |
| `motor_range` | 旧版；独立覆盖三个电机完整标定行程 | `1932/9261`（20.9%） |

默认映射：

- `arpha2 = 5 + 80·u1`，范围 `[5°,85°]`；
- `theta1 = -2 - 21.7·u3`，范围 `[-23.7°,-2°]`；
- `arpha3` 映射到当前 `(arpha2,theta1)` 对应的可闭合区间。

固定独立映射：

- `arpha2∈[5°,85°]`；
- `arpha3∈[-35°,-5°]`；
- `theta1∈[-23.7°,-2°]`。

恢复旧行为：

```python
legacy = solver.solve_motor_from_normalized(
    0.641, 0.582, 0.885,
    mode="motor_range",
)
# [[420.6188, 303.1454, 582.0766],
#  [480.6452, 303.1454, 582.0766]]
```

旧模式失败率高不是数值精度问题。很多角度分别没有超限，但组合后无法满足刚性闭环条件：

```text
abs(closure_value) <= 1
```

默认条件映射是在改变输入语义后避开无解组合，并不是让机构的完整电机行程全部变得可达。

## 外部角度轨迹：推荐安全入口

先诊断，不想修改轨迹时可关闭投影。下面示例可以直接运行：

```python
arpha2, arpha3, theta1 = 47.0, -80.0, -20.0
last_motor = None  # 第一帧；后续帧传入上一帧实际采用的电机值

print(solver.diagnose_input(arpha2, arpha3, theta1))
result = solver.solve_motor_safe(
    arpha2, arpha3, theta1,
    previous_motor=last_motor,
    project_invalid=False,
)
if not result["success"] or result["selected"] is None:
    raise RuntimeError(result["diagnostic"]["message"])
```

`diagnose_input()` 的 `reason` 包括：

- `ok`：可闭合；
- `rotational_workspace`：角度未超限，但机构无法闭合；
- `input_limit`：超过标定限位；
- `non_finite_input`：出现 NaN 或无穷值。

如果业务允许修改无解轨迹，可启用投影，但必须检查偏差。建议把单帧处理封装成函数，由调用方按 `[arpha2, arpha3, theta1]` 顺序传入已经审批的三轴阈值：

```python
import math

def solve_frame(arpha2, arpha3, theta1, allowed_delta_deg, last_motor=None):
    if not isinstance(allowed_delta_deg, (list, tuple)) or len(allowed_delta_deg) != 3:
        raise ValueError("allowed_delta_deg 必须是三个正数")
    limits = [float(x) for x in allowed_delta_deg]
    if any(not math.isfinite(x) or x <= 0 for x in limits):
        raise ValueError("allowed_delta_deg 必须是三个有限正数")

    result = solver.solve_motor_safe(
        arpha2, arpha3, theta1,
        previous_motor=last_motor,
        project_invalid=True,
        enforce_margin=True,
        target_abs_value=0.999,
        max_projection_delta_deg=limits,
    )
    if not result["success"] or result["selected"] is None:
        raise RuntimeError(result["error"] or result["diagnostic"]["message"])

    return result  # 保留 projected、projection 和 diagnostic，供日志与追溯

result = solve_frame(47, -80, -20, allowed_delta_deg=[2.0, 5.0, 1.0])
command = result["selected"]
```

上面 `[2.0,5.0,1.0]` 只演示参数顺序，不是推荐的实机阈值；正式数值必须由项目方审批。

`target_abs_value=0.999` 是无量纲的解析边界余量：它让 `abs(closure_value)` 不超过 0.999，而不是贴着数学极限 1。它不是经过实机认证的安全距离；项目可以在 `(0,1)` 内改值，但必须重新扫描、回归并做实机验证。

关键返回字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否最终得到电机解 |
| `selected` | 推荐使用的解析分支 |
| `projected` | 是否修改了原角度 |
| `projection.angle_delta_deg` | 按 `[arpha2,arpha3,theta1]` 排列的修改量 |
| `projection_within_limits` | 启用投影偏差阈值时是否通过 |
| `canonical_requested_angles` | 周期角规范化后的请求值 |
| `rejected_motor_solutions` | 因超出软件层电机范围而拒绝的分支 |
| `diagnostic` | 原输入失败原因和闭合值 |
| `error` | 投影迭代失败等错误说明 |

投影是局部梯度修正，不保证是全局最近点；连续选解直接比较三路电机控制数值的未加权欧氏距离，没有做归一化或速度规划，也不包含碰撞、加速度、力矩或急停保护。

在现有 302 帧抓握轨迹实验中，原来有 133 帧无解。为同时处理无解点和过于靠近闭合边界的点，共投影 135 帧，之后全部有解；但其中 `arpha3` 最大修改量达到 `25.54°`。因此“投影后有解”不等于“动作仍符合抓握意图”。

## 电机列顺序

这里的“电机值”是 `电机角度关系.txt` 中标定公式得到的无量纲控制数值，不是角度，也不应自行解释为编码器计数。默认 API 顺序：

```text
[arpha1电机, arpha2电机, arpha3电机]
```

历史时序 CSV 的顺序是：

```text
[arpha3电机, arpha2电机, arpha1电机]
```

需要让输出和上一帧向量都采用历史 CSV 列顺序时可使用：

```python
result = solver.solve_motor_safe(
    arpha2, arpha3, theta1,
    previous_motor=last_motor,
    motor_order="timeseries",
)
```

`motor_order="timeseries"` 会同时重排返回的 `solutions`、`selected`，并按相同顺序解释 `previous_motor`；输入角度顺序始终是 `(arpha2, arpha3, theta1)`。

三路换算具有不同零位和标定斜率。文件 `电机角度关系.txt` 中的 `0.239` 表示近似的
“角度变化/控制值变化”，不是把绝对角度直接乘以 `0.239` 得到控制值。正式换算以
`solve_motor()` 中的三组标定式为准。

## 复验

```powershell
# 14 项单元测试
python -B -m unittest -v test_mh6_palm_solver.py

# 默认条件映射的 21³ 全网格验收
python -B audit_conditional_grid.py --step 0.05

# 三种模式覆盖率
python -B _scan_full.py --mode motor_range --step 0.05
python -B _scan_full.py --mode workspace_independent --step 0.05
python -B _scan_full.py --mode workspace_conditional --step 0.05

# 解析区间和随机样本实验
python -B workspace_mapping_experiment.py

# 302 帧轨迹投影实验
python -B project_trajectory_experiment.py
```

已验证结果：

- 14 项单元测试全部通过；
- 条件映射 9261 个网格点全部有两组解；
- 18522 组解的最大旋转误差 `9.15e-16`；
- 最大平移闭环误差 `6.84e-6 mm`；
- 三路电机值均在当前实验使用的 `[0,1000]` 数值范围内。

## 包内主要文件

| 文件 | 用途 |
|---|---|
| `solve_from_arpha2_arpha3_theta1.py` | 正式求解器 |
| `test_mh6_palm_solver.py` | 回归测试 |
| `audit_conditional_grid.py` | 21³ 全网格验收 |
| `_scan_full.py` | 三种映射覆盖率扫描 |
| `workspace_mapping_experiment.py` | 解析映射实验 |
| `project_trajectory_experiment.py` | 302 帧轨迹实验 |
| `requirements.txt` | 固定 Python 依赖 |
| `MH6交付包_SHA256SUMS.txt` | 包内文件完整性清单 |

## 交付状态

当前是经过离线测试的候选包，可以用于代码评审和集成准备，但不是已经放行的实机控制器。

改进版安全约定：

- `solve_motor_safe()` 默认 `project_invalid=False`，无解输入不会被静默修改；
- 显式开启投影时，可使用 `max_projection_delta_deg` 对三轴偏差逐项限幅；
- 默认 `motor_bounds=(0,1000)`，超出范围的解析分支不会进入候选命令；
- 周期等价输入在求解、诊断与电机标定中统一使用规范角度。

集成发布前仍需：

1. 清点所有旧归一化接口调用，并明确选择哪种 `mode`；
2. 建立修改前代码基线和可执行回滚；
3. 固化投影偏差、速度、加速度、碰撞和急停策略；
4. 完成低速、无负载、有人监护的实机验证。

完整性以 `MH6交付包_SHA256SUMS.txt` 为准。交付 ZIP 的 SHA-256 在交付时另行提供。
