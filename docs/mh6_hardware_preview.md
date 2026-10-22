# 九路执行器离线预览配置

当前入口仍拒绝 `--enable-hardware`，不会打开机器人串口。配置填写完成也不会解锁。
目前包含配置、固定分支、位置保护、M4 映射、九路报文预览、反馈轮询和发送失败处理。
正常遥操作入口仍只运行离线模拟；只读反馈有独立入口。启动运动和主动停止尚未实现。

## 唯一配置入口

编辑 `DHandControl/config/mh6_hardware.json`，或用 `--hardware-config PATH` 选择副本。
该文件已按用户提供的数据填写初始位置、限位及五指/M4 电机映射端点；位置跳变阈值仍为 `null`。
不要把 null 改成 0 来表示未知。
字段缺失或拼写错误会报错；合法的 null 保留为待配置，不伪造启动位、限位或阈值。

编号：F1/F2/F3/F4/F5 分别是拇指、食指、中指、无名指、小指电缸 ID 1..5；
M1/M2/M3 是 Solver 的实机输出顺序，M4 是新增拇指偏转舵机 ID 4。
F 和 M 是两种设备总线，F4 与 M4 不冲突。实际 M4 型号、总线兼容性和 ID 仍需台架确认。

每个执行器均需填写：

| 字段 | 含义 |
| --- | --- |
| `min`, `max` | 实际允许位置的下、上限，包含端点 |
| `initial_position` | 离线模拟的起始参考位置，不是实际反馈，也不会触发启动运动 |
| `max_position_step` | 相邻接受帧的最大整数位置差，必须为正整数，与时间间隔无关 |

当前用户参数（单位均为电机协议位置单位）：

| 通道 | min | max | initial_position | 归一化 0 → 1 的电机端点 |
| --- | ---: | ---: | ---: | --- |
| F1–F5 各自 | 20 | 1950 | 20 | 20 → 1950 |
| M1 | 247 | 1000 | 247 | Solver 直接输出，不缩放 |
| M2 | 120 | 500 | 500 | Solver 直接输出，不缩放 |
| M3 | 401 | 500 | 500 | Solver 直接输出，不缩放 |
| M4 | 200 | 600 | 200 | 200 → 600 |

用户确认 `[247,500,500,200,20,20,20,20,20]` 是 M1..M4、F1..F5 的**初始位置**，
不将它们解释为人手自然位或 Solver 零输入对应的机构位置。
M4 的 `neutral_position=200`、`endpoint_position=600` 来自其归一化电机映射范围，
不改变 Mapping 文件内每次重新采集的人手自然角度。
所有 `max_position_step` 尚未填写，因此仍保持初始参考、不能接受新目标，也不能生成完整发送报文。

原生 Solver 零输入的一组电机解约为 `[214.38,479.07,514.42]`，
不满足上述 M1 下限和 M3 上限；当前按用户限位拒绝，不修改 Solver、不裁剪电机解。
配置初始位置不会自动产生到 Solver 起点的运动，启动衔接仍待验证。

## 驱动控制入口统一配置

`modbus_dev.DexHandControl` 默认读取同一个 `mh6_hardware.json`，可用构造参数
`hardware_config=路径/字典/HardwareConfig` 指定独立副本。驱动保存配置快照，
不再读取旧 `mh6_palm_calibration.py` 的运行时范围。`finger_limit` 和 `palm_safe_limits`
仅是统一配置的派生视图，不能通过修改它们替代配置文件。

所有运动入口默认锁定，构造参数 `enable_motion` 默认为 False：运动请求返回 False，
`last_move_result.status=MOTION_LOCKED`；读取反馈仍然可用。本次没有在任何实际硬件调用处开启它。
旧演示脚本也不会因普通构造驱动而发送运动。遥操作入口另外保留原来的 CLI 输出锁。

| 入口 | 当前行为 |
| --- | --- |
| `move_fingers()` | 按 F1..F5 的配置限位检查实际整数位置 |
| `move_palms()` | 支持 M1..M4，逐路检查实际整数位置及运动时间 |
| `move_hand()` | 完整校验两组参数后才写入；某一路无效时整包拒绝 |
| `single_control()` | 按设备类型和 ID 检查对应限位，不再使用通用 [0,2000] |
| `finger_free()` | 使用明确的 release_position；未指定时使用该指的 open_position |
| `palm_free()` / `free_all()` | 包含 M4；必须先配置所需手掌的 release_position，否则拒绝，且不先移动手指 |
| `move_hand_normalized()` | 五指归一化 + Solver 实际三元组 + M4 归一化，生成完整九路报文 |

越限、空组控、重复或未知 ID、布尔值、非整数位置和错误时间均在串口写入前拒绝。
实际位置不做裁剪。`initial_position` 仍只是参考，既不会自动发送，也不会替代释放位置。
驱动层负责绝对限位和报文校验；固定分支、三电机整组保持及相邻帧跳变保护仍由九路规划器负责。

M1..M4 新增可选 `release_position`，当前全部为 null。只影响释放命令，不加入遥操作的
必填字段；它们不会阻塞普通报文检查。填写时必须在对应 min/max 范围内。
`free_all()` 是位置运动，不是急停、卸载或断电；即使端点在限位内，实际路径也仍需台架验证。

归一化接口的参数格式现在为：

```python
# 仅展示数据准备格式；默认运动锁会阻止实际下发。
hand.move_hand_normalized(
    finger_values=[u_thumb, u_index, u_middle, u_ring, u_little],
    palm_positions=[m1, m2, m3],  # Solver 实际位置，检查后取整，不缩放
    m4_value=u_m4,               # [0,1]，当前映射为 [200,600]
)
```

旧 `map_palm_positions()` 和旧 `palm_values=` 归一化三电机插值明确报错，
防止将人手三分量误当成电机位置；M4 可单独用 `map_m4_position()` 映射。
正常位置接口拒绝浮点数，Solver 浮点位置应先调用 `validate_palm_motor_positions()`
或上述九路便捷接口；不对取整位置重新做机构模型验证。

五指另外填写 `open_position`, `closed_position`，对应人手弯曲归一化 0 和 1。
允许闭合端点小于张开端点，由端点顺序决定方向。
F 位置只能处于协议域 [0,2000]，M 位置只能处于协议域 [0,1000]；
这些协议范围不是本程序替操作者确定的实际机构行程。

M1..M3 不再使用旧 `mh6_palm_calibration.py` 的位置、范围或归一化标尺来改变运行时输出。
那个文件及旧控制辅助类仍供历史离线分析使用，不是此入口的配置来源。
原始 `mh6_palm_solver_v2.py` 的角度限制、工作空间映射与电机换算完全不改动。

## 独立 M4 自动标定

硬件配置为 `schema_version: 2`，不再包含人手角度或 `m4_mapping`。
每次新标定在自然位采集 M4 零参考，在原有两次 `thumb_rotate` 动作中同时采集旋转和偏转端点。
两项分别检查，不增加动作或语音；任一项失败时重试同一拇指动作阶段。
结果自动写入新 Mapping JSON 的 `mapping` 内，单位为度：

- `thumb_abduction_neutral_deg`：自然位出掌面角的中位数。
- `thumb_abduction_endpoint_deg`：两次稳定端点中位数的中位数。

支持端点角小于自然位角，M4 仍归一化为自然 0、偏转端点 1，超过范围截到 [0,1]。
旧的 version-1 Mapping 文件可以继续加载，但缺少 M4 标定时 M4 标记 `UNCONFIGURED`，
不借用 Solver 的拇指旋转参考、不自动猜测端点。

`actuators.M4` 内只需另外填写两个**电机位置**：

- `neutral_position`：人手自然位对应的 M4 目标。
- `endpoint_position`：人手偏转端点对应的 M4 目标。

归一化公式为 `u = clip((angle - thumb_abduction_neutral_deg) / (thumb_abduction_endpoint_deg - thumb_abduction_neutral_deg), 0, 1)`。
目标为 `neutral_position + u * (endpoint_position - neutral_position)`。
支持人手角度方向和电机位置方向分别反向。这里只使用自然到偏转的一侧范围。
来源为 `intent.thumb_abduction_raw`（程序内部弧度，映射时转换为度），
不复用参与 Solver 的 `thumb_rotation_command`。原始值、归一化值和是否已标定也写入
`raw_mapping.m4_mapping`，`low_dim.u_thumb_abduction` 是给 M4 控制的归一化输入。
自定义的旧硬件配置副本需要删除 `m4_mapping` 并将 schema_version 改成 2；电机侧数值保持。

## 默认控制规则

- 配置显式写 `fixed_branch: "plus_acos"`, `jump_guard: "position"`。
- 按解析根身份匹配分支，不按候选数组下标。固定根不可用时不切换另一根。
- M1..M3 是整体。无合适根、任一电机超限、任一整数位置差超阈值时，三路全部保持上一完整三元组。
- 五指和 M4 独立判断，某一路拒绝不会冻结其他独立通道。
- 不按电机位置差做独立插值，不使用速度阈值、Solver 输入限速或闭合坐标 fallback。
- 不对取整后的命令重新做模型验证。原有 `--filter-tau` 低通滤波独立保留，可用 0 关闭。
- `servo_move_time_ms` 默认保留旧执行时间 80 ms，报文中用于 M1..M4；它不是速度保护阈值。
- `--palm-fixed-branch`、`--palm-max-motor-step` 可覆盖配置中的三电机分支/阈值；有效配置会写入日志。
- 速度/fallback/不设保护等旧 CLI 模式已拒绝。历史分析脚本的直接辅助函数保持原行为。

`initial_position` 仅作为明确的离线参考；未填写时输出位置为 null，不把第一帧候选自动认作
实机位置。如果某通道配置不全，其状态为 `UNCONFIGURED`；完整九路报文预览为 null。
配置齐全的其他独立通道仍可进行模拟；硬件参数齐全但 Mapping 缺少 M4 标定时，同样不生成完整报文。
预览使用 `propose` 后显式 `commit_preview` 推进**模拟**历史，不代表实际发送或到位确认。
实机发送适配层现通过 `ActuatorIO` 单独处理写入确认、失败锁定和反馈，不能把此模拟方法当成设备确认。

## 发送接口与频率

九路链路为：五指与 M4 的人手特征 → 各自归一化 [0,1] → 配置中的电机端点范围；
手掌 h/v/r → Solver → M1/M2/M3 实际位置值。三路 Solver 输出不再按端点缩放。
位置检查、整组保持和取整完成后，`prepare_packet(proposal)` 生成九路参数；
`HardwareSender.send(packet)` 直接交给 `DexHandControl.move_hand`，不会再次映射。
配置不完整、报文为 None 或未连接时不发送；报文结构、整数与协议域也会检查。
历史八路发送仅保留为 `send_legacy` 供旧分析使用。

当前主循环仍未调用真实发送，`--enable-hardware` 仍在初始化前拒绝。
新接口已用模拟驱动验证，尚未进行实机通信或板上固件执行测试。发送接口立即检查固件下发状态 0x90，
返回 True 表示写入及桥接下发状态得到确认，
不代表电机到位，也不能据此断定九个执行器在物理上同步运动。

实时主循环频率由 `--rate 20` 设置，单位 Hz，默认每周期约 50 ms。
每周期取最新人手帧并完成映射、求解和九路预览；计算超时后不补发积压周期。
这是目标频率，尚无实际串口及反馈耗时验证。离线回放按录制时间与回放速度运行，
`--replay-no-wait` 只用于尽快离线计算，不能作为实机频率设置。
`servo_move_time_ms` 为 M1..M4 每条位置指令的运动时间（默认 80 ms），与控制周期独立。
频率变化还会改变每秒允许更新次数，即便 `max_position_step` 数值不变，也需重新观察轨迹。

## 离线回放

```bash
conda activate mh6
python /home/stw/DHandControl/DHandControl/scripts/mh6_teleop_run.py \
  --replay-session /home/stw/DHandControl/recordings/grasp_01.npz \
  --mapping-calibration /home/stw/DHandControl/recordings/mapping_guided_01.json \
  --hardware-config /home/stw/DHandControl/DHandControl/config/mh6_hardware.json \
  --replay-no-wait \
  --debug-log /home/stw/DHandControl/results/actuator_preview/grasp_01.jsonl
```

普通预览日志现为 `format_version: 2`，每帧包含：

- `raw_mapping`：原始人手映射，不受输出保持影响。
- `solver`：本帧 Solver 请求角度、所有分支及根身份，不受输出保持影响。
- `control.channels`：九路的请求位置、整数目标、接受/保持位置、位置差和拒绝原因。
- `control.palm`：整组三电机候选及接受/保持决定。
- `control.m4_mapping`：M4 原始角度、归一化值和电机请求。
- `packet_preview`：配置完整时为五路电缸＋四路舵机的整数报文参数，始终不发送。
- `execution`：模拟/写入确认/失败锁定、是否尝试发送、耗时及驱动失败阶段；普通入口始终是 `SIMULATED_NOT_SENT`。
- `feedback`：逐路最后一次查询、最后有效反馈及其年龄；普通入口 `enabled=false`，不伪造实机数据。
- 第一帧 `hardware_config`：该次运行使用的完整参数快照。

`--solver-only` 仍仅记录逐帧 Solver 测试，不执行分支提交和输出保护。
有填写的三电机限位使用新配置；空的物理限位只保留 Solver 自身 [0,1000] 诊断域。
汇总里的 `metadata.physical_limits_complete: false` 表示这种有解率**不是实机限位内通过率**。
与此前使用旧 [401,536] 等范围的历史报告比较时，必须区分限位配置。

原始录制、旧 JSON 标定、既有结果和图片均保留。没有添加 AVP 追踪新鲜度监测或主动停止，
也没有刷写或驱动实机。反馈读取、固件依赖及故障恢复规则见 `docs/mh6_feedback_io.md`。
