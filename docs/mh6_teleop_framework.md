# MH6 Vision Pro Teleoperation Framework

## 1. Purpose

本文档描述 Apple Vision Pro 到 MH6 灵巧手的遥操作框架设计。

本项目计划使用 `Improbable-AI/VisionProTeleop` 作为 Vision Pro 侧的人手追踪数据来源。这里应将 VisionProTeleop 视为“人手关键点输入源”，而不是机器人手的底层控制器。

机器人手的底层输出链路保持为：

```text
Python -> Modbus RTU over RS485 -> ESP32
ESP32 -> MicroServoControl -> 5 个手指直线电缸
ESP32 -> LobotSerialServoControl -> 分段手掌舵机
```

高频遥操作输出路径应使用：

```python
DexHandControl.move_hand(..., wait_status=False)
```

## 2. High-Level Pipeline

目标数据流如下：

```text
Vision Pro hand keypoints
-> grasp intention extraction
-> 8D low-dimensional control vector
-> MH6 palm closed-chain solver preview
-> actuator command conversion
-> DexHandControl.move_hand(..., wait_status=False)
```

也就是说，Vision Pro 只提供人手状态。中间层负责从人手骨架中提取抓握意图，再转换成 MH6 手可以执行的电缸和手掌舵机目标。

## 3. Why Not Direct Joint Retargeting

不采用直接关节重定向，原因如下：

- 人手和 MH6 手的结构、自由度和运动范围不同。
- MH6 的每根手指只有一个直线电缸，属于欠驱动/低维执行结构。
- 手掌由多个舵机驱动，存在分段和协同运动，不等价于人手掌骨关节。
- Vision Pro 的人手关键点适合表达“意图”，但不应被逐点映射成机器人关节角。

因此，本框架采用意图驱动的低维控制映射，而不是点对点的人手关节重定向。

## 4. Input From VisionProTeleop

第一版集成不应硬编码 VisionProTeleop 的具体 API。应先定义适配层，将 VisionProTeleop 的输出转换成项目内部的中立表示。

预期输入可以抽象为：

- 27 个手部关键点，或等价的人手骨架。
- 包含 wrist、各手指关节、各 fingertip。
- 关键点坐标为 3D 位置，坐标系由适配层统一。

建议第一版实现：

```text
VisionProTeleop output -> VisionProAdapter -> HandSkeleton
```

`HandSkeleton` 之后的所有映射逻辑不依赖 VisionProTeleop 的原始数据格式。

## 5. Neutral Data Structures

建议使用以下中立数据结构。

### HandSkeleton

保存人手关键点的 3D 位置。

应至少包含：

- wrist
- thumb joints and fingertip
- index joints and fingertip
- middle joints and fingertip
- ring joints and fingertip
- little joints and fingertip

### TeleopCalibration

保存用户和硬件的标定数据。

应至少包含：

- 每根手指的 outward/natural/inward bending reference。
- 右手拇指旋转的 outward/natural/inward angle reference。
- thumb 与 index/middle/ring/little 的 outward/natural/inward distance reference。
- 手指电缸目标范围。
- 手掌舵机目标范围。
- 速率限制参数。

### LowDimHandCommand

8 维归一化控制向量（当前按右手定义）：

```text
[u_thumb, u_thumb_rotation, u_index, u_middle, u_ring, u_little, u_h, u_v]
```

含义：

- `u_thumb`: 纯拇指弯曲量，过伸/自然/弯曲对应 `-1/0/+1`
- `u_thumb_rotation`: 右手拇指向外/自然/向掌心旋转对应 `-1/0/+1`
- `u_index`: 食指过伸/自然/弯曲对应 `-1/0/+1`
- `u_middle`: 中指过伸/自然/弯曲对应 `-1/0/+1`
- `u_ring`: 无名指过伸/自然/弯曲对应 `-1/0/+1`
- `u_little`: 小指过伸/自然/弯曲对应 `-1/0/+1`
- `u_h`: 上下翻折量，正方向为手指完全伸直到完全弯曲，范围 `-1..1`
- `u_v`: 左右翻折量，正方向为拇指侧向小指侧跨掌靠近，范围 `-1..1`

### ActuatorCommand

保存最终要发给 `DexHandControl.move_hand()` 的执行器命令：

- `finger_ids`
- `finger_positions`
- `palm_ids`
- `palm_positions`
- `palm_times`

## 6. Finger Intention Mapping

每根手指的弯曲程度由相邻骨段夹角计算。

对每根手指 `i`，先计算原始弯曲角度和：

```text
c_i = sum(adjacent_bone_angles_i)
```

先保持自然姿态采集零点，再快速执行两次“过度伸直→完整抓合”动作。运动范围使用样本的 5%/95% 分位数作为 outward/inward 边界，避免追踪毛刺成为极值。当前版本采用 outward→natural 与 natural→inward 两段独立斜率：

- 向外边界映射到 `-1`
- 自然姿态映射到 `0`
- 向内边界映射到 `+1`
- 指尖距离的方向相反：距离增大为负，距离减小为正

有方向的基础运动量保持 `-1..1`；`power_grasp`、`tripod_precision`、阈值后对指等抓取意图只取正半轴，继续保持 `0..1`，因此过伸或远离不会被误判成抓取。

```text
c_hat_i = (c_i - c_natural_i) / (c_natural_i - c_outward_i),  c_i < c_natural_i
c_hat_i = (c_i - c_natural_i) / (c_inward_i - c_natural_i),   c_i >= c_natural_i
c_hat_i = clip(c_hat_i, -1, 1)
```

拇指对掌意图通过拇指指尖到其他手指指尖的距离计算。分别得到：

```text
p_I = opposition_strength(thumb_tip, index_tip)
p_M = opposition_strength(thumb_tip, middle_tip)
p_R = opposition_strength(thumb_tip, ring_tip)
p_L = opposition_strength(thumb_tip, little_tip)
```

总对掌强度：

```text
P_opp = max(p_I, p_M, p_R, p_L)
```

五根手指的低维量只保存纯弯曲，不再混入指尖距离：

```text
u_thumb  = c_hat_thumb
u_index  = c_hat_index
u_middle = c_hat_middle
u_ring   = c_hat_ring
u_little = c_hat_little
```

`opposition = [p_I, p_M, p_R, p_L]` 作为独立特征保留，只在后续抓取意图层使用。
这样指尖靠近不会改变手指弯曲量；如果机器人执行器需要对捏合进行补偿，应在执行器转换层完成，而不是覆盖原始低维特征。

所有有方向的 `u_*` 输出都必须再次 `clip(..., -1, 1)`。抓取意图先取基础运动量的正半轴，再限制到 `0..1`。

右手拇指旋转量使用掌部局部坐标系计算：小指根部到食指根部为拇指侧方向，
手腕到中指根部为掌部前向。将拇指近端骨方向投影到这个掌平面后计算角度
`r_thumb`，再使用张手和旋入标定值归一化：

```text
u_thumb_rotation = signed_piecewise(
    r_thumb_outward,
    r_thumb_natural,
    r_thumb_inward
)
```

因此 `u_thumb_rotation=-1/0/+1` 分别表示向外偏转、自然姿态、向掌心旋转至对掌方向。该量与原有 `u_thumb` 分开保存，当前不直接绑定硬件执行器。

## 7. Grasp Intention and Palm Mapping

手掌控制采用四层结构：纯弯曲、独立对掌、抓取意图、手掌命令。所有权重和增益保存在 `MappingCalibration` 中。

### Power grasp

力量型抓持由五指整体弯曲的加权平均得到：

```text
power_grasp = sum(power_weight_i * max(curl_i, 0)) / sum(power_weight_i)
```

默认权重为 thumb/index/middle/ring/little = `0.15/0.20/0.25/0.22/0.18`。单指弯曲只产生有限贡献，五指整体弯曲才接近 1。

### Tripod precision grasp

三指精确抓持不使用时间速度。它同时要求拇指、食指、中指弯曲，并要求拇指同时接近食指和中指：

```text
tripod_flexion  = min(c_thumb, c_index, c_middle)
tripod_proximity = min(p_I, p_M)
tripod_precision = min(tripod_flexion, tripod_proximity)
```

任意一根手指未弯曲，或任意一组指尖未靠近，三指精确意图都不会成立。
`tripod_proximity` 使用从自然姿态开始连续变化的距离归一化值，不经过单指对指的 dead-zone；因此能够表达“两个距离同时减小”的早期趋势。独立的 `pinch_*` 和 `opposition_cross` 仍使用阈值后的对指强度，以降低轻微抖动造成的左右翻折。

### Individual opposition and cross-palm intent

四种对指都会推动拇指侧向小指侧的左右翻折，但无名指、小指贡献更大：

```text
opposition_cross = max(
    0.15*p_I,
    0.30*p_M,
    0.75*p_R,
    1.00*p_L
)
```

采用加权最大值是为了避免拇指靠近某根手指时，与相邻指尖的距离同时减小而被重复累计。

### Vertical and lateral folds

上下翻折主要服务 power grasp，并受到 tripod precision 的额外辅助：

```text
palm_flexion_positive = clip(
    1.00*power_grasp + 0.35*tripod_precision,
    0,
    1
)
palm_flexion_negative = weighted_mean(min(curl_i, 0))
palm_flexion = dominant_direction(
    palm_flexion_negative,
    palm_flexion_positive
)
```

左右翻折主要服务跨掌对指，power grasp 只提供少量协同：

```text
palm_cross_positive = clip(
    0.10*power_grasp + 1.00*opposition_cross,
    0,
    1
)
palm_cross_negative = min(
    min(thumb_rotation_measured, 0),
    0.35*weighted_outward_tip_distance
)
palm_cross = dominant_direction(palm_cross_negative, palm_cross_positive)
```

`dominant_direction()` 选择绝对值更大的正向或负向候选，避免主动向外运动与抓取辅助简单相加后相互抵消。左右翻折的负方向以拇指主动向外旋转为主，指尖距离增大只作较弱辅助。

### Thumb inward compensation

保留 Vision Pro 实测的拇指旋转量，并单独生成给 Palm Solver 的补偿命令：

```text
thumb_compensation = 0.35 * tripod_precision * (1 - max(thumb_rotation_measured, 0))
thumb_rotation_command = clip(
    thumb_rotation_measured + thumb_compensation,
    -1,
    1
)
```

补偿只填补尚未完成的旋转量；操作员已经主动旋入时不会继续等量叠加。原始 `u_thumb_rotation` 不被覆盖。

## 8. Actuator Conversion

当前打印验证路径会把滤波后的 `palm_flexion`、`thumb_rotation_command`、`palm_cross` 按此顺序传给
`MH6PalmSolver.solve_motor_from_normalized()`。求解器可能返回多个闭链运动学分支，
也可能返回空列表表示当前组合无解。

Solver 返回值已经转换为实际手掌硬件 ID `[1,2,3]` 顺序：

```text
Motor 1 <- arpha3
Motor 2 <- arpha2
Motor 3 <- closed-loop solved arpha1
```

三点电机标定为：

```text
             outward  neutral  inward
Motor 1          0       247     1000
Motor 2        630       500      120
Motor 3        536       500      401
```

因此 Solver 自然位输出为 `[247,500,500]`。这些三点当前分别作为每个电机的标定值和安全范围使用；三台电机的 outward/inward 端点是否构成可同时到达的完整闭链姿态仍待确认，代码不作此假设。

当前解选择器会：

- 按各电机安全范围过滤候选解。
- 将三个电机按 outward/neutral/inward 三点反归一化到 `-1/0/+1`，再计算与上一帧的加权距离。
- 第一次选择距离自然位 `[247,500,500]` 最近的有效解。
- 后续选择距离上一次有效电机值最近的解，不依赖 Solver 返回的分支编号。
- 无解或全部解越界时保持上一次有效电机值；首次无解则保持自然位。
- 按归一化速度限制拒绝过大的单帧跳变，默认上限为每秒 `2.0` 个归一化行程。

选择器只在真正采用有效解时更新上一次有效输入和电机位置。当前仍只打印选中值与状态，不向硬件发送。状态包括 `SELECTED`、`HELD_NO_SOLUTION`、`HELD_NO_VALID_SOLUTION`、`HELD_JUMP_REJECTED` 和首次候选离自然位过远时的 `HELD_NEUTRAL_INITIAL_JUMP_REJECTED`。

### Runtime smoothing layers

运行时平滑分为两层：

1. Mapping 之后的一阶低通：对 `low_dim` 和 `palm_command` 做时间感知低通，默认 `tau=0.24s`。滤波从自然零值渐入，不再让首帧直接跳到当前手势。
2. Solver 之前的输入限速：对 `u_h/u_v/thumb_rotation_command` 分轴限速，默认每秒最多变化 `2.0`，每一个中间点都会重新经过闭链 Solver。

低通、Solver 输入限速和 Solver 输出跳变门限使用的 `dt` 都限制为最多 `0.10s`。因此较长掉帧后恢复时不会因为累计了很大的 `dt` 而直接跳到新目标。默认连续 `0.25s` 没有有效帧会进入 tracking-lost 状态并保持上一次命令；恢复后从保持状态继续渐进。

当前 tracking-lost 依据 `VisionProHandStream` 是否返回有效帧判断；上游接口没有提供可依赖的采集序号或设备时间戳，因此无法严格识别“上游反复返回同一份但仍格式有效的数据”。在解除硬件锁之前仍需结合实际 Vision Pro 流确认其 `get_latest()` 新鲜度语义。

如果 Solver 对限速后的中间输入无解，电机值和输入限速器都会回退到上一次可行状态，不会让内部 applied input 在电机保持时继续向不可行目标前进。

日志明确区分：

- `raw features/commands`：当前帧未经时间滤波的 Mapping 输出。
- `filtered commands`：一阶低通后的命令，使用 `--print-filtered` 显示。
- `requested`：低通后希望交给手掌的目标。
- `applied`：输入空间限速后真正交给 Solver 的目标。
- `selected`：分支选择后保持或采用的实际 M1/M2/M3。

`free_all()`、`palm_free()` 和 `finger_free()` 已改为读取各设备的标定限位：手指移动到各自 `finger_limit` 的张开端，手掌移动到各自三点标定的 outward 端，不再统一硬编码位置 `0`。

### Solver fallback preview

Solver 无论当前是否处于 fallback 都会继续逐帧运行。控制台的 `palm solver preview`
始终先打印 requested/applied、全部候选、选中电机和 solver status；随后独立打印
`palm control preview`，说明最终处于 `SOLVER` 还是 `FALLBACK` 模式。默认控制台约
5Hz 打印，使用 `--print-every-frame` 可显示每一个处理帧。

连续纯 `NO_SOLUTION` 达到默认 `0.5s` 后，fallback 将上下/左右翻折按默认
`0.8/0.2` 合成为一个整手闭合量，并沿
`[0,630,536] -> [247,500,500] -> [1000,120,401]` 双段同步轨迹生成预览值。
当前硬件输出仍被锁定，fallback 只用于打印和离线调试。

### Raw session recording and replay

入口支持从数据源层录制完整操作过程，文件保存三个阶段的原始 `27x4x4` transforms：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --record-session recordings/grasp_01.npz \
  --debug-log recordings/grasp_01.jsonl
```

录制阶段包括 `neutral`、`range` 和 `teleop`。按 Ctrl+C 退出时使用临时文件和原子
替换保存压缩 NPZ，避免留下半写入的目标文件。JSONL 调试日志每帧包含独立的
`solver` 与 `control` 字段，但只作为结果比较，不作为回放输入。

使用录制文件替代 Apple Vision Pro：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/grasp_01.npz
```

可选模式：

```bash
--replay-speed 0.5   # 半速，滤波和限速时间也按半速时间线运行
--replay-loop        # 只循环正式 teleop 阶段，不重复标定阶段
--replay-no-wait     # 不等待墙上时间，逐帧确定性离线处理
--print-every-frame  # 每帧打印 solver 和最终控制结果
```

回放仍然完整经过标定、Mapping、滤波、Solver、解选择和 fallback，因此同一份原始
录制可以用于比较不同版本算法的输出。回放帧时间戳来自录制时间线，不使用离线处理
速度，保证滤波和速度限制可重复。

归一化命令必须通过标定范围转换成实际执行器目标。

HardwareSender 已准备为：手指只取有符号弯曲量的正半轴并映射到实际电缸位置，手掌直接使用选择后的实际 M1/M2/M3。当前 signed mapping 阶段仍禁止硬件输出，只进行 Palm Solver 候选值打印测试。

后续执行器转换应采用双段物理映射，例如：

```text
finger_position = piecewise_map(u_finger, -1, 0, 1,
                                finger_outward, finger_natural, finger_inward)
```

手掌：

```text
palm_position = piecewise_map(u_palm, -1, 0, 1,
                              palm_outward, palm_natural, palm_inward)
```

要求：

- 电缸位置范围必须可配置。
- 舵机位置范围必须可配置。
- 执行器范围允许反向，即 `open_position > closed_position` 或 `open_position < closed_position` 都必须支持。
- `map_range()` 不能假设目标范围单调递增。
- 映射数学只处理归一化值，不硬编码具体硬件上下限。
- 映射之后必须再次按执行器物理范围 clamp，防止标定、滤波或速率限制误差导致越界。

## Coordinate and Keypoint Convention

VisionProTeleop adapter 负责将原始输出转换为 `HandSkeleton`。映射层只消费 `HandSkeleton`，不直接依赖 VisionProTeleop 的原始 API。

约定：

- 映射主要使用相对 bone vectors 和 fingertip distances。
- finger joint order 必须一致，例如从 wrist/metacarpal 侧到 fingertip 侧。
- 每根手指的关键点命名和顺序必须在 adapter 中统一。
- 坐标系变化、左右手镜像、单位缩放等问题应在 adapter 或 calibration 中处理。
- 如果 VisionProTeleop API 发生变化，只应修改 adapter，不应修改低维意图映射和机器人控制输出层。

## 9. Teleop Runtime Design

遥操作运行时应采用 latest-frame policy：

- 永远只处理最新的人手追踪帧。
- 不排队旧目标。
- 如果新帧到来时上一帧还未发送完成，应丢弃旧帧或合并到最新目标。

Modbus 侧运行建议：

```python
hand.start_persistent_connection()
try:
    hand.move_hand(..., wait_status=False)
finally:
    hand.stop_persistent_connection()
```

高频循环中：

- 使用持久 Modbus 连接。
- 使用 `move_hand(..., wait_status=False)` 进行流式控制。
- 不在每帧读取状态寄存器。
- 状态应由独立低频任务周期性轮询。
- ID 管理、clear-error 等配置/维护命令不进入高频循环。

推荐初始控制频率：

- 第一阶段：`20 Hz`
- 只有在硬件验证稳定后再尝试 `30 Hz`
- 不应在未验证通信和执行器温升前追求更高频率

## 10. Safety Design

安全策略必须在软件和硬件命令输出前同时生效。

基本要求：

- Clamp 所有归一化命令。
- Clamp 所有执行器目标范围。
- 对命令变化做 rate limit。
- 处理 tracking loss。
- 提供 emergency stop。
- Demo 模式默认不驱动真实硬件。
- ID management 和 clear-error 不是高频控制循环的一部分。

Tracking loss 策略建议：

- 短暂丢帧：保持上一帧或缓慢回到安全姿态。
- 长时间丢帧：停止发送新运动目标，进入安全姿态或等待人工恢复。
- 恢复追踪后：通过 rate limit 平滑恢复，不允许跳变。

## 11. Current Implementation Status

当前实现状态：

- `modbus_dev.py` 已提供 `DexHandControl.move_hand(..., wait_status=False)` 快速输出路径。
- `move_hand()` 已使用 Modbus `write_registers()` 上传组合控制 payload。
- `move_fingers()` 和 `move_palms()` 保留为调试/兼容 API。
- `mh6_teleop_run.py` 已作为当前右手遥操作、标定、录制和回放入口。
- `VisionProHandStream` 已通过 `avp_stream` 接入 Vision Pro，并可由 `ReplayHandStream` 替换。
- 原始 hand transforms 支持 NPZ 分阶段录制与确定性回放。
- Palm Solver 与 fallback 结果支持控制台同步显示和逐帧 JSONL 记录。
- VisionProTeleop 不应直接控制机器人硬件。

## 12. Planned Implementation Stages

建议后续实现顺序：

```text
Stage A: skeleton README and data structures
Stage B: mh6_teleop.py runtime loop
Stage C: mapping functions from HandSkeleton to LowDimHandCommand
Stage D: calibration file format
Stage E: VisionProTeleop adapter
Stage F: hardware benchmark and safety tests
```

每个阶段都应保持底层驱动和通信协议稳定，优先验证安全性、延迟和可重复性。
