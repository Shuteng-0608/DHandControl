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
-> 7D low-dimensional control vector
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

- 每根手指的 open bending reference。
- 每根手指的 closed bending reference。
- thumb 与 index/middle/ring/little 的 open/closed distance reference。
- 手指电缸目标范围。
- 手掌舵机目标范围。
- 速率限制参数。

### LowDimHandCommand

7 维归一化控制向量：

```text
[u_thumb, u_index, u_middle, u_ring, u_little, u_h, u_v]
```

含义：

- `u_thumb`: 拇指弯曲/对掌意图，范围 `0..1`
- `u_index`: 食指抓握意图，范围 `0..1`
- `u_middle`: 中指抓握意图，范围 `0..1`
- `u_ring`: 无名指抓握意图，范围 `0..1`
- `u_little`: 小指抓握意图，范围 `0..1`
- `u_h`: 水平手掌包络意图，范围 `0..1`
- `u_v`: 横向手掌弯曲意图，建议范围 `-1..1`

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

再使用 open/closed 标定归一化：

```text
c_hat_i = clip((c_i - c_open_i) / (c_closed_i - c_open_i), 0, 1)
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

手指控制值组合弯曲和对掌意图：

```text
u_thumb  = max(0.7 * c_hat_thumb, P_opp)
u_index  = max(c_hat_index,  p_I)
u_middle = max(c_hat_middle, p_M)
u_ring   = max(c_hat_ring,   p_R)
u_little = max(c_hat_little, p_L)
```

注意：`u_index` 等控制量使用的是归一化弯曲 `c_hat_i`，不是原始角度和 `c_i`。

所有 `u_*` 输出都必须再次 `clip(..., 0, 1)`。

## 7. Palm Intention Mapping

手掌控制不直接复制人手关节，而是从抓握意图中提取低维手掌协同量。

全手包络强度：

```text
g = 0.2*u_index + 0.3*u_middle + 0.3*u_ring + 0.2*u_little
```

三指抓握强度：

```text
t = min(u_thumb, u_index, u_middle)
```

对掌对水平手掌包络的贡献：

```text
o_h = clip(0.20*p_I + 0.35*p_M + 0.70*p_R + 1.00*p_L, 0, 1)
```

水平手掌包络命令：

```text
u_h = clip(0.55*g + 0.20*t + 0.35*o_h, 0, 1)
```

由手指抓握分布得到的横向偏置：

```text
b_f = 0.5*(u_ring + u_little) - 0.5*(u_index + u_middle)
```

对掌对横向手掌弯曲的贡献：

```text
o_v = clip(-0.25*p_I - 0.45*p_M + 0.75*p_R + 1.00*p_L, -1, 1)
```

横向手掌弯曲命令：

```text
u_v = clip(0.30*b_f + 0.70*o_v, -1, 1)
```

将 `u_h` 和 `u_v` 展开成左右侧手掌块命令：

```text
thumbSide  = clip(u_h - u_v, 0, 1)
littleSide = clip(u_h + u_v, 0, 1)
```

理想四块手掌模型：

```text
UL = LL = thumbSide
UR = LR = littleSide
```

其中：

- `UL`: upper-left palm block
- `LL`: lower-left palm block
- `UR`: upper-right palm block
- `LR`: lower-right palm block

真实硬件可能有 3 个或 4 个 palm servo。`TeleopCalibration` 负责将这些抽象 palm block 命令映射到实际的 `palm_ids` 和 palm servo target positions。具体 palm servo 数量、ID、方向和每个舵机的贡献权重都应由硬件标定配置决定，不应写死在意图映射公式中。

## 8. Actuator Conversion

归一化命令必须通过标定范围转换成实际执行器目标。

手指：

```text
finger_position = map_range(u_finger, 0, 1, finger_open_position, finger_closed_position)
```

手掌：

```text
palm_position = map_range(u_palm, 0, 1, palm_open_position, palm_closed_position)
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
- `mh6_teleop.py` 应作为遥操作应用层。
- VisionProTeleop 集成尚未实现，应通过 adapter 接入。
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
