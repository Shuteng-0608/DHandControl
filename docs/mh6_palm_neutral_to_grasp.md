# 手掌自然状态到向内抓

本分支为 `codex/palm-neutral-to-grasp`，由 `codex/palm-fallback-control` 分出。
手掌 Mapping 的三分量改为 `[0,1]`，采用用户确认的 Solver 原生
`workspace_conditional` 零输入作为起点。原解析求解器、机构参数、角度范围、电机换算
均未修改，也没有新增自定义 Solver 映射。

## 输入含义与参数顺序

| Mapping 分量 | 含义 | Solver 输入 | 请求角 |
|---|---|---|---|
| h / vertical | 上下翻折 | u1 | alpha2 |
| r / thumb_rotation_command | 大拇指旋转 | u2 | alpha3 |
| v / lateral | 左右翻折 | u3 | theta1 |

0 表示人手自然状态，1 表示该控制分量的最大向内强度。向外的特征不再产生负的
手掌命令，也不再压过正的抓持贡献；原来的正向权重、对指贡献和三指补偿保留。
这一步不是 `(旧值+1)/2`，不会把自然状态移到 0.5。五指弯曲和原始特征仍单独记录，
没有因为手掌范围调整而改变五指控制逻辑。

默认配置为 `DHandControl/config/mh6_palm_adapter_neutral_to_grasp.json`：
`neutral=[0,0,0]`、`inward=[1,1,1]`。因此适配层只检查数值范围并重排参数，
不再用旧平面自然位坐标插值：

```python
result = solver.solve_motor_safe_from_normalized(
    h, r, v,
    mode="workspace_conditional",
    previous_motor=last_motor,
    project_invalid=False,
    motor_order="timeseries",  # 硬件 ID [1,2,3]
)
```

条件映射中的 alpha2 和 theta1 分别由 u1 和 u3 决定；alpha3 的可行范围还依赖
这两个角度，再由 u2 在该范围内取值。因此固定一个人手分量，并不保证对应机器人
角度在其他分量变化时保持不变，也不保证每台电机随抓持强度单调变化。

## 起点由 Solver 定义

原生 `(u1,u2,u3)=(0,0,0)` 的返回值为：

- 请求角 `[alpha2,alpha3,theta1]=[5,7.796455861,-2]°`。
- 电机候选（硬件 ID 顺序）：`[214.3848,479.0749,514.4233]` 和
  `[214.3848,479.0749,526.9359]`。
- 上游默认参考下的 selected 为第一组；外层仍从满足实际限位的所有候选中选解。

这些数值由原始 Solver 计算，没有强制变成平面角度 `[0,0,0]` 或电机
`[247,500,500]`。历史平面零位配置保留用于父分支复现，本分支入口拒绝该配置。

实际电机限位仍为 M1 `[0,1000]`、M2 `[120,630]`、M3 `[401,536]`。
这些是既有实机保护边界，外层只过滤候选；不会修改 Solver 目标角、单独裁剪电机值，
也不会按电机相对旧平面零位的正负方向人为缩小 Solver 工作空间。

## 固定录制验证

复用 `recordings/teleop_01.npz` 和 `recordings/mapping_01.json`，没有重新调权重：

| 检查 | 结果 |
|---|---:|
| 总帧数 / 时长 | 599 / 29.99 秒 |
| 几何有解 | 599 / 599（100%） |
| 至少一个实际电机限位内候选 | 556 / 599（92.82%） |
| 全部候选越界 | 43 帧 |
| 默认保护预览更新 | 0 帧，启动跳变拒绝保持 599 帧 |
| 无低通且关闭跳变保护预览 | 556 帧选解，43 帧保持 |
| 上述无保护预览最大单步变化 M1/M2/M3（含启动） | 308 / 124 / 112 |
| 自动测试 | 170 项通过 |

逐帧确认三分量均在 `[0,1]`、三个回放的原始 Mapping 一致，并且全部请求角度与
直接调用原始 Solver 的条件映射完全一致。另测试了 11×11×11 个组合的目标角一致性。
取整后的预览保持原有连续分支选择；没有增加取整后严格机构模型复算门限。

默认保护预览与无保护预览同时存在低通、输入限速设置差异，不能用两条曲线的差异
单独归因于跳变门限。无保护预览只用于观察候选轨迹，不能作为实机发送轨迹。
几何有解率、限位内有解率和保护后的更新率是三个不同指标。

默认预览仍从控制器现有参考 `[247,500,500]` 开始，第一帧附近的 Solver 目标就与
它有明显差异，因此默认保护一直保持。不能假定实机已经位于 Solver 的零输入位置。
实机启动需要读取实际位置并验证到 Solver 起点的连续入场路径；硬件输出继续锁定，
之前审计出的发送确认、反馈、停机与数据新鲜度问题也尚未解决。

## 重现与图表

以下命令在仓库根目录、mh6 环境执行。只测试到 Solver 返回值并静默保存：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --solver-only --replay-session recordings/teleop_01.npz \
  --mapping-calibration recordings/mapping_01.json --replay-no-wait \
  --debug-log results/solver_only/neutral_to_grasp/teleop_01_solver.jsonl
```

保存默认保护预览和用于观察的无保护预览：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/teleop_01.npz \
  --mapping-calibration recordings/mapping_01.json --replay-no-wait \
  --debug-log results/solver_only/neutral_to_grasp/teleop_01_guarded.jsonl

python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/teleop_01.npz \
  --mapping-calibration recordings/mapping_01.json --replay-no-wait \
  --filter-tau 0 --palm-no-jump-guard \
  --debug-log results/solver_only/neutral_to_grasp/teleop_01_unguarded.jsonl

python DHandControl/scripts/mh6_neutral_grasp_report.py
```

报告与图表位于 `results/solver_only/neutral_to_grasp/summary.json` 和
`mapping_and_motor_trajectories.png` / `.svg`，JSONL 保留每帧的三个 Mapping 值、
Solver 请求角、全部分支、限位内分支和最终预览状态。
在另一台电脑恢复本次所有结果：

```bash
python artifacts/teleop_01/restore_analysis.py --snapshot neutral_to_grasp
```

新录制的范围标定提示已改为自然放松到完整抓合。旧标定文件仍可使用；原始数据和
父分支历史结果保留，重新标定时无需执行过度张开动作。
