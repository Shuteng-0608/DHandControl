# 拇指参与意图实验分支

分支 `codex/thumb-participation-intent` 从 ACK 分支 `5f94bcd` 建立。遥操作入口默认采用
`thumb_participation` 策略，原通信实验分支不修改。

删除的是 `tripod_precision` 三指捏取意图：它由拇指/食指/中指弯曲与两组指尖距离共同构造，
与握拳、逐指对指使用的观测量重叠。四组逐指对指意图 `opposition_cross` 和对应标定仍保留。
新策略的三类主要意图为握拳 G、逐指对指 O、拇指参与 P。

## 融合规则

先用每次人手标定，把掌面内旋转归一化为 r，抬离掌面偏转归一化为 a，均为 [0,1]。
再融合，不能把角度度数、弧度或电机位置直接加权：

```text
P = (w_rotation × r + w_abduction × a) / (w_rotation + w_abduction)
h = clip(0.8 × G + 0.1 × O + 0.1 × P, 0, 1)
v = clip(0.05 × G + 0.5 × O + 0.45 × P, 0, 1)
Solver 第三路 r_command = r
M4 归一化输入 = a
```

h 为上下翻折，v 为左右翻折。传入 Solver 的位置顺序仍为 `(h,r,v)`。
三指意图对 h 的旧贡献与对 r 的旧补偿一并移除；r 不会改成 P，M4 也不会改成 P。
五指弯曲、M4 映射、握拳/逐指对指意图的内部权重、滤波、Solver/闭合分支、限位、跳变保护、30 Hz 目标频率和 ACK 通信均保持。
每个翻折通道的三个融合权重均非负且总和为 1，不再沿用 Mapping 文件中的旧翻折增益。
进入融合的 G/O/P 限制在 [0,1]，因此合成不会因权重相加而提前截断；最终 clip 保留为边界保护。
两轴所有权重都为正，只有 G/O/P 同时为 1 才达到 1。单独完整握拳对应 h=0.8，单独完整对指意图对应 v=0.5。
这不会让所有动作在统计上独立，也不能恢复输入归一化时已截断的变化。

## 权重填写位置

只调整 `DHandControl/config/mh6_thumb_participation.json`：

| 参数 | 起始值 | 含义 |
|---|---:|---|
| `thumb_rotation_weight` | 0.5 | 掌面内旋转在 P 中的相对权重 |
| `thumb_abduction_weight` | 0.5 | 抬离掌面偏转在 P 中的相对权重 |
| `vertical_power_gain` | 0.8 | G 对上下翻折 h 的贡献 |
| `vertical_opposition_gain` | 0.1 | O 对上下翻折 h 的贡献 |
| `vertical_thumb_gain` | 0.1 | P 对上下翻折 h 的贡献 |
| `lateral_power_gain` | 0.05 | G 对左右翻折 v 的贡献 |
| `lateral_opposition_gain` | 0.5 | O 对左右翻折 v 的贡献 |
| `lateral_thumb_gain` | 0.45 | P 对左右翻折 v 的贡献 |

前两项自动按总和归一化，不能同时为 0；每轴三个增益须非负且总和为 1，否则启动时报错，不会静默重配比例。均无量纲。
例如仅 r=1、a=0 时 P=0.5，对 h/v 分别贡献 0.05/0.225；两项都到 1 时分别贡献 0.1/0.45。
融合配置升级为 schema_version=2，包含全部六个翻折增益。旧版 schema_version=1 的自定义配置须更新为完整新版格式，避免读取旧参数后静默改变语义。
人手自然位/端点继续保存在 Mapping 文件，电机参数继续保存在硬件配置，均不挪入这个权重文件。

## 使用与记录

原遥操作启动命令无需额外参数，默认加载这个配置。
需要完整 M4 人手标定；推荐按现有 `staged_v4` 分开标定两个拇指分量。
缺少 M4 端点会报错，不用默认角度或另一分量代替。
生成的 Mapping JSON 仍是原 format_version=2；旧三指权重字段仅为读取兼容而保留，在新策略中不生效。

日志新增：

- `raw_mapping.grasp_intent.thumb_participation`：未滤波的 P。
- `raw_mapping.intent_fusion` 和首帧 `runtime_config.intent_fusion_parameters`：本次实际权重。
- `palm_command.vertical_from_power` / `vertical_from_opposition` / `vertical_from_thumb`：h 的三项贡献。
- `palm_command.lateral_from_power` / `lateral_from_opposition` / `lateral_from_thumb`：v 的三项贡献。
- `vertical_before_clip` / `lateral_before_clip`：截断前的合成值，检查饱和。
- `thumb_rotation_compensation=0`：保留日志列兼容，不再施加三指补偿。

新路径不计算或输出 `tripod_flexion`、`tripod_proximity`、`tripod_precision`、`t`。
仅为历史对照保留 `--intent-fusion legacy`，该选项恢复原融合，不修改保存的人手端点。
基础库 `MH6HandMapper` 默认仍供标定与旧数据重建使用；程序式使用新策略需传
`intent_fusion=ThumbParticipationFusion.load()`。标定质量检查与语音不受新融合权重影响。

## 离线验证

```bash
python artifacts/thumb_participation/compare_recordings.py
```

复用三类已录制动作与 `mapping_guided_02.json`，对同一骨架分别计算旧/新融合，使用相同 0.24 秒滤波。
逐帧断言五指与 M4 的直接归一化命令、握拳 G、逐指对指 O 完全一致；记录 h/v/r 差异、饱和与 Solver 返回。
结果输出到 `results/thumb_participation/`，包含每帧 JSONL、汇总与曲线，无串口或 AVP 连接。
仓库中的 [汇总](../artifacts/thumb_participation/summary.json) 与 [曲线](../artifacts/thumb_participation/comparison.png) 是该脚本的一次快照。
这些跨次录制使用同一份旧标定作成对比较，不能代替新独立标定后的实际动作评估。
Solver 有解率统计在位置跳变保护之前，不是实际更新率或实机可用率。
原有近零工作空间异常仍未改动，离线脚本遇到时记录为 `WORKSPACE_GENERATION_ERROR`；实机入口的原退出行为保留。

本次 2395 帧对照：旧三指意图大于 0.05 的 579 帧中，579 帧握拳 G 同时大于 0.05，575 帧逐指对指 O 也同时大于 0.05。
三组数据的新旧限位内 plus 解率均为 100%，此次调整没有证明求解率提升。
当前六项权重下，2395 帧的 h/v 叠加值均未超过 1，滤波后均保持 [0,1]，每轴三项贡献之和也逐帧核对一致。

| 录制 | 帧数 | 滤波 h 峰值 | 滤波 v 峰值 | 限位内 plus 解率 |
|---|---:|---:|---:|---:|
| 握拳 | 599 | 0.923489 | 0.754007 | 100% |
| 四组对指 | 1197 | 0.343835 | 0.957153 | 100% |
| 三指捏取动作 | 599 | 0.397850 | 0.760249 | 100% |

“三指捏取”在此只是原始录制动作的名称，不代表重新启用已移除的三指意图。
旧标定下 M4 归一化输入达到 1 的帧较多，应在新的独立标定后重新比较，再决定是否进一步调整权重。
