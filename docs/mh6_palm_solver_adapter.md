# 三轴遥操作到新 Palm Solver 的适配

> 本文下方保留父分支 `codex/palm-fallback-control` 的有符号适配说明与旧实验。
> 当前 `codex/palm-neutral-to-grasp` 使用 `[0,1]` 输入和原始条件工作空间零输入，
> 具体接口、自然位和运行命令以 [当前分支说明](mh6_palm_neutral_to_grasp.md) 为准。
> 本分支入口不接受本文中的旧配置或 legacy 模式。

当前 `mh6_teleop_run.py` 默认使用 `PalmSolverAdapter`，接收映射端的三个
`[-1,1]` 语义量，转换为新 solver 的 `[0,1]` 坐标，再输出硬件 ID 顺序的电机候选。
硬件发送仍由入口原有保护锁定，适配器本身不打开串口、不发送命令。

## 函数调用和参数

```text
MH6HandMapper.step(points)
    palm_command = {vertical, lateral, thumb_rotation_command}
        ↓ CommandLowPassFilter.apply(result, timestamp)
        ↓ extract_palm_normalized_inputs(result)
    requested = (palm_flexion, palm_cross, thumb_inward)       [-1,1]
        ↓ palm_solver_input_values(requested) → (h, r, v)
        ↓ PalmInputSlewLimiter.apply(requested, timestamp)
    applied_values = (h, r, v)                              [-1,1]，兼容旧入口顺序
        ↓ named_palm_inputs(applied_values) 恢复语义名称
        ↓ PalmSolverAdapter.solve_motor_from_teleop(
              vertical=h, lateral=v, thumb_rotation_command=r, previous_motor=...)
        ↓ map_teleop_to_workspace(h, v, r)
    (u1, u2, u3)                                           [0,1]，按 h/r/v 标定
        ↓ MH6PalmSolver.map_normalized(u1, u2, u3, mode=...)
    (arpha2, arpha3, theta1)                                度
        ↓ 仅修正限位端点不超过 1e-10 度的浮点越界
        ↓ MH6PalmSolver.solve_motor_safe(a2, a3, t1, motor_order="timeseries", ...)
        ↓ solve_motor(a2_used, a3_used, t1_used)
        ↓ solve_arpha(a2_used, a3_used, t1_used)
        ↓ solve_remaining(a2_used, a3_used, t1_used)
    每条分支 (theta2, theta3, arpha1, rotation_error, translation_error)
        ↑ solve_arpha 返回 [arpha1, -arpha2, arpha3]
        ↑ solve_motor 换算，原始 API 顺序 [a1电机, a2电机, a3电机]
        ↑ solve_motor_safe 重排成 [a3电机, a2电机, a1电机]
    电机候选                                               硬件 ID [1,2,3]
        ↓ 当前标定限位检查
        ↓ PalmSolutionSelector.select(applied, candidates, timestamp)
    selected_motor                                         三个实际控制值
```

适配器只生成候选及诊断，不保存执行状态。最终采用哪个候选、是否满足跳变门限，
仍由 `PalmSolutionSelector` 决定；只有被接受的输出才更新其历史状态。

运行层保留远端旧入口的内部位置顺序 `(palm_flexion, thumb_inward, palm_cross)`，
用于限速和 `--palm-solver legacy`。新适配入口使用具名参数，在归一化标定前显式
重排为 `(h,r,v)`：`vertical → arpha2`、`thumb_rotation_command → arpha3`、
`lateral → theta1`。具名参数本身不能替代这个内部重排；JSONL 字段按语义名称记录。

## 默认标定保留平面自然位

### 条件工作空间对照调用

指定 `--palm-adapter-config DHandControl/config/mh6_palm_adapter_workspace_conditional.json`
时，适配器将上述 `(u1,u2,u3)` 直接交给归一化安全接口：

```python
result = solver.solve_motor_safe_from_normalized(
    u1, u2, u3,
    mode="workspace_conditional",
    previous_motor=last_motor,
    project_invalid=False,
    motor_order="timeseries",  # 与项目硬件 ID [1,2,3] 顺序一致
)
```

此配置保持人手 Mapping、手指权重和归一化坐标不变。`workspace_conditional`
把 `alpha2` 映射到 `[5,85]°`，把 `theta1` 映射到 `[-2,-23.7]°`，再根据两者计算
可行的 `alpha3` 区间并映射 `u2`。`project_invalid=False` 关闭的是这一步之后的角度投影，
并不意味着仍请求旧映射的角度。

实验中 `h=v=r=0` 的请求角为 `[25.4102,19.1020,-7.7777]°`。
配置中的 `neutral` 为保持前后 `u` 一致而沿用原坐标中点，不是新的机器人自然位标定。
固定 599 帧输入得到 599 帧几何有解、549 帧至少一个实际电机限位内的分支。

带 `last_motor` 的独立对照确认，全部候选和无历史逐帧测试完全相同，但 `selected`
仅按上游默认 `[0,1000]` 限制选解：599 帧上游 success 中，实际限位内 selected 为 507 帧；
另有 42 帧存在实际可用分支但未被上游 selected 选中。适配器保留原返回选择作诊断，
外层仍应从满足项目限位的 `solutions` 进行选择。

### 原平面零位基线配置

配置文件：`DHandControl/config/mh6_palm_adapter_calibration.json`。

默认显式选择新 solver 的 `motor_range` 模式，以保留
`h=v=r=0 → 三个机构角为0 → 电机[247,500,500]`。新 solver 的默认
`workspace_conditional` 模式不包含这个平面零位，不能直接使用 `(s+1)/2` 替代。

三轴的 `[0,1]` 自然位坐标是原始角度映射的逆变换：

```text
u1_neutral = 31.1 / 121.9
u2_neutral = 59.0 / 239.0
u3_neutral = 8.6 / 32.3
```

对每个轴，`s` 是人手语义值，`q_out/q_0/q_in` 是对应配置中的三个坐标。
配置数组始终按 `[arpha2,arpha3,theta1]` 排列，对应 `[h,r,v]`：

```text
s < 0: q = q_0 + (-s)*(q_out - q_0)
s ≥ 0: q = q_0 + s*(q_in - q_0)
```

默认 `q_out=0、q_in=1`，最终得到的角度方向为：

| 语义输入 | -1 | 0 | +1 |
|---|---:|---:|---:|
| 上下翻折 h → arpha2 | -31.1° | 0° | 90.8° |
| 拇指旋转 r → arpha3 | 59° | 0° | -180° |
| 左右翻折 v → theta1 | 8.6° | 0° | -23.7° |

正值表示向内。上下翻折通过 `arpha2*=-arpha2` 表达内折；另外两轴向内对应负角度。
这些是单轴标定端点，不保证三轴端点能同时闭合。
`thumb_rotation_command` 已含映射端的三指补偿，适配器不再次叠加补偿。

## 返回值与电机限位

```python
from mh6_palm_solver_adapter import PalmSolverAdapter

adapter = PalmSolverAdapter()
result = adapter.solve_motor_from_teleop(
    vertical=0.5,
    lateral=0.5,
    thumb_rotation_command=0.1,
    previous_motor=[247.0, 500.0, 500.0],
)
```

此例请求角度约为 `[45.4,-18.0,-11.85]`，符合当前限位的候选为
`[[322.3,310.0,412.7855]]`；另一条分支的第三台电机越界，单独记录为被拒绝分支。
最终预览输出还需通过外部选择器的跳变检查。原预览采用速度门限；
`--palm-max-motor-step M1 M2 M3` 可单独测试位置门限，比较取整后的候选指令与
上一次接受输出的位置差，不根据帧间隔换算为速度。所有限位内分支都会参与检查，
全部超限则保持上一输出。取整后严格机构模型复算只用于离线诊断，不作为控制门限。

固定根实验可用 `--palm-fixed-branch plus_acos` 或 `minus_acos`。
外部选择器只接受指定根；该根越界或无法唯一识别时保持整组上一有效位置，
不会使用另一根或 fallback 替代。首次有效输出前参考中立位置 `[247,500,500]`。
日志保留所有候选，同时记录 `fixed_branch_id`、`fixed_branch_valid_candidate_count`
及 `fixed_branch_out_of_limits` / `fixed_branch_unavailable` 原因。
此模式不改变 Solver 请求角度，且禁止与跳过外部选择器的 `--solver-only` 组合。

| 返回字段 | 含义 |
|---|---|
| `semantic_input` | 实际传入适配器的 h/v/r；取决于上游滤波与输入限速配置 |
| `workspace_input` | 归一化后的 u1/u2/u3；所选模式决定其含义 |
| `mapping_mode` | 当前使用的归一化映射模式 |
| `solver_entrypoint` | 本次使用角度接口还是归一化安全接口 |
| `requested_angles` | 请求角度，顺序 `[arpha2,arpha3,theta1]` |
| `angle_input_order` | 这些角度对应的输入 `[vertical,thumb_rotation_command,lateral]` |
| `used_angles` | 实际求解角度；失败或投影偏差被拒绝时可为空 |
| `angle_delta_deg` | 实际采用角度相对请求角度的偏差 |
| `candidates` | 全部电机候选，含越界分支，供预览和外部选择器过滤 |
| `candidate_branch_ids` | 与 candidates 对齐的解析根标识 plus_acos/minus_acos；根合并或标识不唯一时为 null |
| `solutions` | 满足当前各电机标定限位的候选 |
| `core_selected_motor/core_selected_index` | 上游按自身限制和 previous_motor 选中的分支；不是外层已接受的硬件目标 |
| `rejected_motor_solutions` | 被拒绝的电机值及越界的硬件 ID |
| `diagnostic/used_diagnostic` | 请求角度及实际采用角度的闭链诊断 |
| `projected/projection` | 是否投影及具体偏差 |
| `status/error` | 求解、限位或投影失败原因 |

所有电机列表均按硬件 ID `[1,2,3]` 排列：

```text
ID1 ← arpha3，限位 [0,1000]
ID2 ← arpha2，限位 [120,630]
ID3 ← 闭链求出的 arpha1，限位 [401,536]
```

越界候选被拒绝，不能独立裁剪三台电机值。全部候选越界时，运行层返回
`HELD_NO_VALID_SOLUTION` 或其启动中性保持变体；真正无几何解时返回
`HELD_NO_SOLUTION` 或其启动变体，并保留详细诊断。

## 配置与运行

默认运行方式不变：

```bash
python3 DHandControl/scripts/mh6_teleop_run.py --avp-ip 192.168.8.145
```

回放可查看逐帧的参数转换：

```bash
python3 DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/right_power_grasp_01.npz \
  --replay-no-wait --debug-log /tmp/mh6-adapter-debug.jsonl
```

通过 `--palm-adapter-config path.json` 选择自己的机器人标定配置。
通过 `--palm-solver legacy` 恢复仓库原有 solver 及原有角度方向，便于比较。

`workspace_conditional` 和 `workspace_independent` 必须显式提供 `neutral` 坐标。
其中条件模式根据上下翻折、拇指旋转动态选择左右翻折的几何可行区间；适配器仍
检查实际电机限位，因此不承诺所有输入都有合格电机分支。
这两个模式改变机器人自然位，若用于运行入口，还需标定参考姿态及连续入场路径；
现有从平面中性值启动的选择器可能保持输出，而不会直接跳到新参考位。

默认关闭投影。启用 `project_invalid` 必须同时配置按
`[arpha2,arpha3,theta1]` 顺序排列的三个正数 `max_projection_delta_deg`。
`enforce_margin=true` 也必须显式开启投影，避免上游接口的该开关绕过投影许可。
平面零位处的分支合并仍属于边界，不能靠开启裕度投影自动完成入场。

控制台新增 `palm adapter preview`。JSONL 在 `solver.adapter` 中保存以上阶段数据，
原有 solver/control 字段继续保留。

## 新核心来源和验证

`DHandControl/scripts/mh6_palm_solver_v2.py` 是用户提供目录
`/Users/wangshuteng/Downloads/a1t1a3_7_20/solve_from_arpha2_arpha3_theta1.py`
的原样副本，运行不依赖 Downloads 目录。完整原始交付包已保存到仓库
`third_party/mh6_palm_solver/a1t1a3_7_20/`，包含原始源码、方向图、程序说明、
测试及实验数据；详见 [原始包说明](../third_party/mh6_palm_solver/README.md)。
运行副本与存档源码逐字节一致。SHA-256：

```text
7d95710ada4e22978519a0b2e7fbf6f0215ede2b07155600271f6bcf3ddecdc9
```

适配层修正了全行程插值在端点可能产生的微小浮点越界；没有修改解析几何核心。
测试涵盖中性位、正负方向、输入契约、电机顺序及标定限位、投影许可与偏差门限、
保持策略、日志和原始关键点回放。交付包中按候选排列顺序比较的回归，在这里按
候选集合比较，避免机器精度以下旋转误差影响分支排序。

```bash
python3 -m unittest discover -s DHandControl/tests
```
