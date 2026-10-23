# 分阶段标定与简短语音提示

当前实时标定默认使用旋转和 M4 偏转分开采集的 `staged_v4` 流程。8 条语音来自用户
提供的 `/home/stw/Downloads/clib_voice/`，已复制到仓库
`DHandControl/assets/calibration_voice/`；运行不依赖 Downloads 目录。
“拇指向掌心内侧旋转”阶段只标定送给 Solver 的掌面内旋转；新增独立“拇指抬离掌面”阶段只标定 M4 出掌面偏转。
两阶段各做两次，只更新各自端点，失败时只重试对应阶段。自然位仍同时采集两个零参考。
动作允许伴随少量另一分量变化，不要求人手完全解耦；程序不会拿伴随变化替代另一阶段的端点。
新增 M4 阶段暂无录音时使用文字提示，其他阶段继续播放现有录音。
原始 M4A 与可直接播放的单声道 24 kHz、16 位 PCM WAV 都保留，manifest 记录来源、
时长和校验值。运行时不需要 FFmpeg、GStreamer 或在线语音服务。

## 直接使用

在仓库根目录、mh6 环境执行，替换 AVP 地址：

```bash
python DHandControl/scripts/visionpro_session.py record \
  --avp-ip 192.168.8.109 --mode calibration \
  --output recordings/calibration_guided_03.npz \
  --mapping-output recordings/mapping_guided_03.json
```

这条命令只采集人手标定，不启动机器人。成功后保存：

- `calibration_guided_03.npz`：稳定片段、动作过渡、失败重试片段和质量报告元数据。
- `mapping_guided_03.json`：可直接传给遥操作入口的人手标定参数。
- `mapping_guided_03.report.json`：每阶段的端点、噪声、跨度、重试原因和自然位验证结果。

已有文件受保护；换新文件名或明确使用 `--overwrite`。不指定 `--mapping-output`
时只保存 NPZ，之后仍可用 `visionpro_session.py calibrate --input ... --output ...` 导出。

也可以在标定后立即开始 Solver 测试与录制：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --solver-only --avp-ip 192.168.8.145 \
  --save-mapping-calibration recordings/mapping_guided_02.json \
  --record-session recordings/full_guided_02.npz \
  --debug-log results/solver_only/guided_02.jsonl
```

按 Ctrl+C 结束正式动作测试。标定成功后先保存原始标定检查点，再播放“标定完成”；
最终退出时原子更新 NPZ，加入正式动作。`--mode full` 的独立录制也采用相同检查点机制。

## 操作员流程

每个动作做到位后保持，听到“自然张开”再放松；“自然张开”指自然放松，不要过度伸直。
程序只复用现有 8 条短录音，不增加长说明或倒计时语音。

| 阶段 | 次数 | 使用的数据 |
|---|---:|---|
| 自然放松 | 1 | 五指弯曲、拇指旋转、M4 出掌面角、四组指尖距离的中位数零参考 |
| 缓慢握拳 | 2 | 只更新五指弯曲的向内端点 |
| 拇指沿掌面向内旋转，尽量少抬起 | 2 | 只更新 Solver 所用的拇指旋转端点 `thumb_rotation_closed` |
| 拇指抬离掌面，尽量少沿掌面旋转 | 2 | 只更新 M4 出掌面角端点 `thumb_abduction_endpoint_deg`，允许相对自然位正反两个角度方向 |
| 拇指与食指、中指、无名指、小指依次对指 | 每组 1 | 各自只更新对应指尖距离端点 |
| 回到自然位 | 1 | 验证 h/v/r 及 M4 返回误差接近零，不更新标定 |

每条语音同步播放完成后，默认留 3 秒动作时间，再采稳定片段 1.5 秒；自然位采样
默认 3 秒。动作之间播放自然位提示，再留 2 秒放松。全流程约 2 分钟，含播放时长；
失败重试会延长时间。可用 `--calibration-move-seconds`、`--calibration-hold-seconds`、
`--calibrate-seconds` 调整，稳定窗口必须允许至少 5 帧。

端点取稳定窗口的中位数；两次握拳、旋转、偏转的端点取两次中位数的中位数。
只有该阶段所有相关特征通过检查才更新参数。四组对指不会使用握拳阶段的距离，
各阶段也不会互相覆盖端点。

## 质量检查及记录

最低向内跨度为五指弯曲 0.08 rad、拇指旋转 0.06 rad、M4 偏转绝对跨度 0.06 rad、对指距离 5 mm，同时必须
超过自然位及端点稳定窗口的 3 倍稳健噪声估计（1.4826 × MAD）。还检查窗口的
95–5 百分位跨度和重复端点一致性。最终自然位 h/v/r 中位数均须不超过 0.15；
M4 则检查 `median(abs(当前角-自然角)/abs(端点角-自然角)) <= 0.15`，避免向相反侧偏离时
被 [0,1] 截断掩盖。M4 标定也拒绝退化掌坐标系和零长度拇指根部骨段。
这些是人手采样质量的初始测试阈值，可按真实录制分析调整，与机器人限位无关。

范围不足、采样不足或保持不稳定会打印具体原因，并只自动重试当前阶段一次。
再次失败则停止标定，不沿用默认端点、不播放完成。中断或失败时已采数据仍保存到
NPZ；不完整的分阶段录制不能导出为有效标定文件。

返回自然位也会单独重试一次；失败提示显示 h/v/r、M4 返回误差、阈值及超限分量，报告保留每次
验证的数值和窗口下标。不会因为最后一步失败而重新采集通过的握拳、旋转和对指。

如果所有动作阶段已通过、仅最终自然位未通过，可以从原始录制补做这一步：

```bash
python DHandControl/scripts/visionpro_session.py record \
  --avp-ip 192.168.8.109 --mode calibration \
  --resume-calibration recordings/calibration_guided_01.npz \
  --output recordings/calibration_guided_01_retry.npz \
  --mapping-output recordings/mapping_guided_01.json \
  --calibration-move-seconds 4
```

只需恢复与开始标定时一致的自然姿态，尤其让拇指放松，不必再握拳或对指。
原 NPZ 不会覆盖，新录制保留原始帧和端点，并追加自然位验证；记录恢复来源及其
SHA-256。动作阶段不完整的文件不能用此方式恢复。只有新验证通过才生成正式
Mapping JSON，不放宽 0.15 门限，也不自动修改自然位零点。

首次真实录制 `calibration_guided_01.npz` 共 1800 帧，握拳和旋转重试后均通过，
四组对指也通过。失败的最终中位数为 h=0.02586、v=0.00259、r=0.16134；
仅 r 超过 0.15。末尾拇指旋转代理角比起始高约 7.65°，各窗口本身波动较小。
骨架记录不能单独区分操作姿态变化与追踪估计差异。原数据仍标记为未完成，尚未
强制生成标定文件，诊断见 `artifacts/calibration_guided_01/diagnosis.json`。

本次验证只检查返回自然位；还没有自动完成一次独立的握拳、旋转及四组对指复测。
自然位噪声会保存，目前没有自动新增控制死区。

新 Mapping JSON 为 format_version=2，包含 `thumb_abduction_neutral_deg` 与
`thumb_abduction_endpoint_deg` 两个人手角度。每次新标定重新采集，不写入硬件配置。
硬件配置只保存电机启动位、限位、位置阈值和电机映射端点。

NPZ 保留原容器格式，当前阶段标签包括：`grasp`、`thumb_rotate`、`thumb_abduct`、`opp_index`、
`opp_middle`、`opp_ring`、`opp_little`、`transition`、`verify_neutral`。
`metadata.calibration_protocol=staged_v4`；`calibration_segments` 以帧下标明确标记
各次稳定窗口及是否被接受。离线导出只采用被接受的窗口，并重新检查质量。

## 播放与兼容性

Linux 默认使用 `paplay`，没有该命令时使用 `aplay`；macOS 可使用 `afplay`。
缺少播放器或播放失败会提示并转为文字引导。可以用
`--no-calibration-voice` 显式静音，或用 `--calibration-voice-dir PATH` 指定包含同名
8 个 WAV 文件的目录。Ctrl+C 会终止正在播放的提示并进入保存退出流程。

新增 M4 录音只需一句：“请将拇指抬离掌面，到位后保持。”
准备好单声道 PCM WAV 后命名为 `thumb_abduct.wav`，放入同一语音目录即可自动播放。
没有该文件不阻止标定，不会占用错误动作的旧录音。

所有离线回放和固定 JSON 标定运行都不初始化播放器，不播放语音、不等待语音计时。
旧 `staged_v2` 按其独立 `thumb_abduct` 阶段重建；旧 `staged_v3` 继续在 `thumb_rotate` 的相同窗口中重建两项端点。
新 `staged_v4` 则严格要求两个独立阶段均完成，不能用旧旋转窗口补齐缺失的 M4 阶段。
旧 Mapping JSON 仍可使用，但不会自动获得独立动作的端点，采用新流程需要重新录制标定。
旧 `staged_v1` 原样重建和恢复，不凭旧动作猜测 M4 端点；旧 format_version=1 Mapping
仍可加载，仅 M4 保持未标定状态。旧 `neutral/range/teleop` 录制自动沿用历史标定算法；需要新录制也使用旧流程时，
显式指定 `--calibration-flow legacy`。`--range-calibrate-seconds` 仅作用于旧流程。

现有 `mapping_01.json`、录制文件、抓持权重、Solver 源码、原生工作空间零输入和
硬件输出锁均未修改。分阶段标定产生的人手三分量仍按 `(u1,u2,u3)=(h,r,v)` 输入
`closing_only` 条件工作空间，再调用原闭环角度求解器；见 [向内工作空间](mh6_inward_workspace.md)。
