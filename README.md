# DHandControl

## 当前分支：自然状态到向内抓

`codex/palm-neutral-to-grasp` 的手掌三分量统一为 `[0,1]`：自然状态输出 0，
向内抓输出正值，过度张开不再产生负的手掌目标。原有抓持权重与录制标定未修改。
按 `(u1,u2,u3)=(h,r,v)` 直接调用原始 `workspace_conditional` Solver；机构角度、
电机换算和原生零输入位置均以 Solver 为准。详见
[本分支接口与回放结果](docs/mh6_palm_neutral_to_grasp.md)。

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --solver-only \
  --replay-session recordings/teleop_01.npz \
  --mapping-calibration recordings/mapping_01.json \
  --replay-no-wait \
  --debug-log results/solver_only/neutral_to_grasp/teleop_01_solver.jsonl

python artifacts/teleop_01/restore_analysis.py --snapshot neutral_to_grasp
```

599 帧离线验证：几何有解 599 帧，实际电机限位内有解 556 帧（92.82%）。
Solver 零输入请求角为 `[5,7.796455861,-2]°`，不是平面零位；现有控制器初始参考
仍为 `[247,500,500]`，默认跳变保护会保持全部帧。硬件输出继续锁定。
本分支拒绝旧的 signed 适配配置与 `--palm-solver legacy`。

**下方保留的有符号映射、旧配置命令及统计属于父分支 `codex/palm-fallback-control`。
复现旧实验请切换父分支；本分支使用上方命令和新文档。**

## 在另一台电脑继续当前工作

当前录制、标定和分析进展位于 `codex/palm-fallback-control` 分支。
请先阅读 [跨电脑恢复与当前结论](docs/mh6_continue_on_another_computer.md)。
原始输入在 `recordings/`；全部分析日志、权重搜索、图表和脚本已无损打包，运行：

```bash
python artifacts/teleop_01/restore_analysis.py
python artifacts/teleop_01/restore_analysis.py --snapshot postprocessing
```

即可恢复原分析和后续条件工作空间、位置保护、连续/固定分支实验，包含
`results/solver_only/` 和 `results/trajectory_audit/` 中的逐帧日志、统计与图表。
若原分析包中已有文件被后续实验更新，第二条命令会报告冲突；确认需要补充包
中的新版本后，为第二条命令加上 `--overwrite`。
高有解率权重存在动作意图失真，尚未替换原标定；
继续工作时优先查看恢复后的 `weight_search/intent_audit/`。

## 仅测试到 Palm Solver 返回值，记录运动有解率

增加 `--solver-only` 后，运行链路为：

```text
AVP / NPZ → 人手标定 → Mapping → Palm Solver → 逐帧记录与有解率汇总
```

默认把原始 Mapping 输出直接送入 solver，不经过低通、输入限速、上一可行输入
回退、连续解选择或 fallback。每帧独立求解，保留全部返回分支；不连接机器人。
这样无解后的下一帧仍测试当前手部姿态，不会被上一帧的保持状态影响。
此模式只保存数据到文件，不在控制台打印 Mapping、solver 结果或有解率。
回放正常完成时静默退出；实时采集只保留必要的标定、开始动作提示和错误信息。

用仓库现有完整录制测试（自动复用文件内的标定阶段）：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --solver-only \
  --replay-session recordings/right_power_grasp_02.npz \
  --replay-no-wait \
  --debug-log results/solver_only/right_power_grasp_02.jsonl
```

仅录了动作的 NPZ 可以复用固定标定参数：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --solver-only \
  --replay-session recordings/avp_grasp_01.npz \
  --mapping-calibration recordings/right_mapping_01.json \
  --replay-no-wait \
  --debug-log results/solver_only/avp_grasp_01.jsonl
```

实时测试并同时保存原始手部运动：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --solver-only --avp-ip 192.168.8.145 \
  --record-session recordings/right_solver_test_01.npz \
  --debug-log results/solver_only/right_solver_test_01.jsonl
```

结束时按 Ctrl+C，会保存原始录制和当前统计。JSONL 记录每个有效 teleop 采样帧，
包括相对时间、原始 Mapping 特征、实际 solver 输入、请求机构角、全部电机候选、
限位内候选、状态、求解耗时，以及适配器的完整诊断。所有电机列表按硬件 ID
`[1,2,3]` 排列；不生成保持值或最终控制命令。

三个请求角按 `[arpha2,arpha3,theta1]` 排列，分别来自上下翻折 `vertical`、
拇指旋转 `thumb_rotation_command`、左右翻折 `lateral`。适配器日志中的
`angle_input_order` 显式记录这个对应关系。

同目录自动生成 `*.summary.json`，也可用 `--solver-summary PATH` 指定位置：

- **几何有解率**：返回至少一个闭链电机候选的帧数 / 有效 teleop 采样帧总数。
- **电机限位内有解率**：至少一个候选满足三台电机当前标定限位的帧数 / 同一总数。
- 全部候选越界计为“几何有解、限位内无解”，不会混同为几何无解。
- 标定帧与没有有效追踪数据的时段不进入分母；没有测试帧时，两项比例为 `null`。

汇总中的 rate 是 `0..1` 比例。汇总也保存本次实际使用的
人手标定、机器人适配标定、输入来源和滤波参数，便于比较实验。
未指定 `--debug-log` 时，自动保存到仓库的
`results/solver_only/<录制名称或live>_<运行时间>.jsonl`，并生成对应的汇总文件。
只指定 `--solver-summary` 时，逐帧日志使用同名 `.jsonl` 文件。
`--print-every-frame`、`--print-filtered` 不会让 solver-only 模式打印数据。

要研究低通后的运动有解率，显式增加 `--solver-input filtered`（默认 `tau=0.24s`，
可用 `--filter-tau` 调整）。即使选择 filtered，仍不经过限速、连续选解和 fallback。
此模式要求适配配置的 `project_invalid`、`enforce_margin` 为 `false`，保证测试的是
请求姿态本身；默认配置已经满足。`--palm-solver legacy` 可用于旧求解器对比。

### 按归一化安全接口测试条件工作空间

使用条件工作空间配置时，适配器直接调用
`solve_motor_safe_from_normalized(u1, u2, u3, mode="workspace_conditional", ...)`：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --solver-only \
  --replay-session recordings/teleop_01.npz \
  --mapping-calibration recordings/mapping_01.json \
  --palm-adapter-config DHandControl/config/mh6_palm_adapter_workspace_conditional.json \
  --replay-no-wait \
  --debug-log results/solver_only/teleop_01_workspace_conditional.jsonl
```

此对照沿用原手势权重及原三分量到 `u1/u2/u3` 的转换，只替换归一化坐标到机构角的映射。
599 帧录制中，几何有解为 599 帧，至少一个分支满足实际电机限位为 549 帧。
按此前动作审计的判据，充分握拳可用 57/90 帧，完成对指可用 51/51 帧。
配置未开启投影；条件映射本身会先根据两个翻折量确定拇指的可行角度区间。

因此，输入分量相同也不代表机器人角度相同：此实验的 `h=v=r=0` 对应
`[alpha2,alpha3,theta1] ≈ [25.4102,19.1020,-7.7777]°`，不再是平面零位。
这里的中点是为了保持对照实验的归一化输入一致，尚未重新标定机器人自然位。
默认平面零位配置仍保留；要复现本次调用，请带上上述条件工作空间配置。

`--solver-only` 仍逐帧独立求解。常规预览调用会传递已有电机历史。
适配器日志新增 `solver_entrypoint`、`core_selected_motor` 和 `core_selected_index`，
其中 `core_selected_motor` 是上游按默认 `[0,1000]` 限制选择的分支，未必满足项目更窄的
电机限位；判断可用解仍应查看经过实际限位检查的 `solutions`。

### 连续解分支选择与固定轨迹对照

常规预览默认采用 `--palm-branch-policy integer_continuous`：将限位内候选取整，
相对上一有效整数目标计算三个电机的加权平方位置差，以总行程 `[1000,510,135]`
归一化。切换解析根时加上 `--palm-branch-switch-penalty`，默认实验值为 `0.005`。
等价整数目标优先保留上一分支；原分支越界时仍从其他合法分支选择。
Solver 分支通过 `theta3=-phi±acos(...)` 的根标识区分，独立于返回列表顺序。
根合并或无法唯一识别时不给它任意标号，也不施加切换惩罚。

该策略保持 Mapping 输入和机构请求角度不变。`--palm-branch-policy calibrated`
保留旧标定距离选解，用于对照；`--solver-only` 仍跳过外部连续选解。

单独测试连续选解时，用 `--palm-no-jump-guard` 关闭位置/速度门限及输入限速，
并用 `--filter-tau 0` 关闭低通。程序仅做离线预览，硬件入口仍锁定：

```bash
mkdir -p results/solver_only/branch_selection
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/teleop_01.npz \
  --mapping-calibration recordings/mapping_01.json \
  --palm-adapter-config DHandControl/config/mh6_palm_adapter_workspace_conditional.json \
  --replay-no-wait --filter-tau 0 --palm-no-jump-guard \
  --palm-branch-policy integer_continuous --palm-branch-switch-penalty 0.005 \
  --debug-log results/solver_only/branch_selection/teleop_01_integer_hysteresis.jsonl \
  > results/solver_only/branch_selection/teleop_01_integer_hysteresis.console.log 2>&1
```

同一命令改为 `--palm-branch-policy calibrated`，日志名改为 `teleop_01_calibrated.jsonl`，
生成旧策略对照；改为 `integer_continuous --palm-branch-switch-penalty 0`，日志名改为
`teleop_01_integer_nearest.jsonl`，生成整数最近解对照。控制台重定向文件也对应改名。
准备好三个日志后运行：

```bash
python DHandControl/scripts/mh6_branch_selection_report.py
```

报告保存至 `results/solver_only/branch_selection/`，包括全程电机轨迹、M3 局部候选及
分支切换对照、切换惩罚敏感性统计。它检查同一轨迹的 Mapping/请求角度一致性，
以及反转候选顺序、改变时间戳间隔后选解结果是否一致。
若原始 solver-only 日志及此前取整轨迹日志存在，还会额外核对这些历史参考；
仅凭上述三个新回放日志也能生成报告和图。
固定 599 帧中，默认实验策略把分支切换从 20 次减少至 18 次；549 帧采用有效候选、
50 帧无限位内解保持。三个电机的最大整数位置变化仍为 `[332,94,121]`，
说明该策略减少部分分支往返，尚未解决录制中的大跳变。

逐帧日志中的 `solver.branch_selection` 包含上一整数参考、上一/当前根标识、
是否切换、选择理由，以及每条候选的整数目标、位置差、运动代价和切换惩罚。
切换惩罚属于实验选解参数，不是实机允许的位置跳变阈值。

### 固定解析分支回放

`--palm-fixed-branch plus_acos` 或 `minus_acos` 将整个回放限定为同一个解析根。
该根缺失、无法唯一识别或超过实际电机限位时，保持上一有效的三电机位置；
首次接受前使用中立位置 `[247,500,500]`。即使另一根合法也不会切换，
并在日志中记录固定根的候选、限位状态和保持原因。此选项不能与 `--solver-only`
或旧版求解器组合使用。固定模式不使用切换惩罚。

分别录制两条固定分支，再生成原始取整解及越界保持后的输出图：

```bash
mkdir -p results/solver_only/fixed_branch
for branch in plus_acos minus_acos; do
  python DHandControl/scripts/mh6_teleop_run.py \
    --replay-session recordings/teleop_01.npz \
    --mapping-calibration recordings/mapping_01.json \
    --palm-adapter-config DHandControl/config/mh6_palm_adapter_workspace_conditional.json \
    --replay-no-wait --filter-tau 0 --palm-no-jump-guard \
    --palm-fixed-branch "$branch" \
    --debug-log "results/solver_only/fixed_branch/teleop_01_${branch}.jsonl" \
    > "results/solver_only/fixed_branch/teleop_01_${branch}.console.log" 2>&1
done
python DHandControl/scripts/mh6_fixed_branch_report.py
```

本次 599 帧中，两根均每帧几何有解；`plus_acos` 有 481 帧满足实际限位，
`minus_acos` 有 263 帧满足实际限位，分支切换均为零。
`fixed_branch_targets.jsonl` 保留包含越界值的逐帧整数解，供分析固定分支自身轨迹。

### 独立测试位置跳变保护

常规预览可加入 `--palm-max-motor-step M1 M2 M3`。三个值分别限制硬件电机
ID `[1,2,3]` 相对上一次接受输出的整数位置变化，单位为电机位置单位，与帧间隔无关。
程序先过滤所有限位内候选的位置跳变，再从通过的分支中选最近解；全部超限时，
保持三个电机的上一次输出，不分别截断三个电机的位置。

此选项关闭选择器的速度门限与 Mapping 输入限速，保留可配置的低通滤波。
下面用 `--filter-tau 0` 关闭滤波，单独测试原始轨迹的位置跳变：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/teleop_01.npz \
  --mapping-calibration recordings/mapping_01.json \
  --palm-adapter-config DHandControl/config/mh6_palm_adapter_workspace_conditional.json \
  --replay-no-wait --filter-tau 0 \
  --palm-max-motor-step 100 60 20 \
  --debug-log results/solver_only/teleop_01_position_guard_100_60_20.jsonl \
  > results/solver_only/teleop_01_position_guard_100_60_20.console.log 2>&1
```

这些阈值只是离线实验值，不代表实机允许值。首次输出也相对 `[247,500,500]`
检查；初始姿态差异或分支跳变可能导致长时间保持。不要加 `--solver-only`，
该模式只记录求解结果，跳过控制保护。`--palm-max-motor-step` 和
`--palm-no-jump-guard` 均未使用时，原有速度门限与输入限速仍启用。
日志包含候选位置差、三个阈值、通过位置检查的分支数和保持原因。
接受新位置的比例应与几何有解率、限位内候选比例分开统计。

取整后严格闭环复算属于离线敏感性诊断，不参与运行时控制门限。
数学模型精确闭合失败不能直接解释为实机动作失败或实机可用率下降。

## 独立录制 AVP，再回放开发与测试

使用 `DHandControl/scripts/visionpro_session.py` 可以单独采集手部数据。录制过程只依赖
NumPy 和 `avp_stream`，无需运行 Mapping、Palm Solver 或连接机器人。
默认只录正式遥操作动作，保存右手 27 个关节的完整 `4×4` 变换矩阵（位置和朝向）、
时间信息及阶段标签。标定数据和动作数据可以独立管理，也可直接复用固定标定参数。
文件沿用现有 `.npz` 格式；仓库已有的完整录制也能直接播放和导出标定参数。

在仓库根目录、已经安装项目依赖的 Python 环境中执行：

```bash
# 只录遥操作动作，30 秒后自动保存
python DHandControl/scripts/visionpro_session.py record \
  --avp-ip 192.168.8.145 \
  --output recordings/avp_grasp_01.npz \
  --duration 30
```

将 IP 替换为 AVP 的实际地址。程序先等待有效手部数据，提示并预留默认 3 秒准备，
然后采集正式动作；可以用 `--prepare-seconds` 调整准备时间。
省略 `--duration` 时持续采集，按 Ctrl+C 保存退出。已有文件默认受到保护，
确实需要替换时使用 `--overwrite`。

测试时使用固定的内置 Mapping 参数，直接播放动作，跳过自然姿态和范围标定：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/avp_grasp_01.npz \
  --use-default-calibration \
  --replay-no-wait \
  --debug-log /tmp/avp_grasp_01_test.jsonl
```

内置参数适合测试代码流程。要比较真实手部映射效果，建议固定一份从实际标定得到的
JSON 参数：用同一份参数和动作文件重复实验，避免每次标定结果不同影响比较。

### 单独录制标定，生成固定参数

```bash
# 只录自然姿态和运动范围，不录正式动作
python DHandControl/scripts/visionpro_session.py record \
  --avp-ip 192.168.8.145 \
  --mode calibration \
  --output recordings/right_calibration_01.npz

# 离线计算标定参数并保存；仅需做一次
python DHandControl/scripts/visionpro_session.py calibrate \
  --input recordings/right_calibration_01.npz \
  --output recordings/right_mapping_01.json

# 以后只播放遥操作动作，复用固定参数
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/avp_grasp_01.npz \
  --mapping-calibration recordings/right_mapping_01.json \
  --replay-no-wait \
  --debug-log /tmp/avp_grasp_01_test.jsonl
```

标定录制包含 `neutral`（默认 2 秒自然放松姿态）和 `range`（默认 8 秒，完成两次
“过度伸直 → 完整抓合”），每阶段预留默认 3 秒准备。可用 `--calibrate-seconds`、
`--range-calibrate-seconds` 和 `--prepare-seconds` 调整。

也可以直接使用两个 NPZ 文件，每次实验从独立标定文件离线计算参数：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --calibration-session recordings/right_calibration_01.npz \
  --replay-session recordings/avp_grasp_01.npz \
  --replay-no-wait
```

现有完整录制 `recordings/right_power_grasp_01.npz` 同样可作为 `calibrate --input`
或 `--calibration-session` 的输入，无需重新找操作员录制标定。
`--use-default-calibration`、`--mapping-calibration`、`--calibration-session` 三选一。
使用 `--save-mapping-calibration PATH` 可保存当前 MH6 运行所使用的全部人手映射参数。
这些参数与 `--palm-adapter-config` 控制的机器人手掌求解器标定是两套独立配置；
旧 `mh6_teleop_default_calibration.json` 属于旧入口，不适用于 `--mapping-calibration`。

### 检查、播放和组件 API

检查文件和单独播放手部点位（无需连接 AVP）：

```bash
python DHandControl/scripts/visionpro_session.py info \
  --input recordings/avp_grasp_01.npz

python DHandControl/scripts/visionpro_session.py replay \
  --input recordings/avp_grasp_01.npz \
  --speed 0.5 --loop
```

单独播放默认选择 `teleop`，在终端打印手腕和食指尖点位；可用 `--phase neutral` 或
`--phase range` 检查标定数据。支持 `--no-wait` 逐帧快速播放、`--print-every-frame`
逐帧打印；循环只支持 `teleop`，Ctrl+C 结束。

实时速度实验可去掉 `--replay-no-wait`；加入 `--replay-speed 0.5` 可以半速运行，
加入 `--replay-loop` 可以循环正式动作。当前 MH6 映射入口只接受右手录制，仍保持
打印测试模式。纯数据播放支持左手。

组件 API 复用 `HandSessionRecorder` 和 `ReplayHandStream`：

```python
from mh6_hand_session import ReplayHandStream

stream = ReplayHandStream("recordings/avp_grasp_01.npz", no_wait=True, hand="right")
try:
    stream.start()
    stream.set_phase("teleop")
    while not stream.phase_finished:
        frame = stream.get_latest_frame()
        if frame is not None:
            process_hand_points(frame.points, frame.timestamp)  # 替换为自己的处理函数
finally:
    stream.stop()
```

`get_latest_points()`、`get_latest_transforms()` 也可作为读取入口；每次调用消费一帧，
同时需要点位、朝向和时间戳时使用 `get_latest_frame()`。
实时播放尚未到下一帧时返回 `None`，调用方应短暂等待后再读取。

录制按 `--rate` 采样接收端的当前有效帧，时间戳来自接收端单调时钟；追踪缺失期间的
时间间隔会保留在相邻帧时间戳中。上游当前接口没有可依赖的采集序号，因此相邻采样
可能读取同一个 AVP 帧。短片段先保存在内存里，正常结束或 Ctrl+C 时原子写入压缩 NPZ。
若在标定期间退出，将保存部分数据并报告当前录制模式缺失的阶段。

录制模式为 `--mode teleop`（默认）、`--mode calibration` 或 `--mode full`（三个
阶段都录）。其他通用参数包括 `--rate 20`、`--connect-timeout 15`、`--hand left` 和
`--origin sim`。完整录制文件仍可沿用下面的旧流程，从文件内的标定阶段重新计算参数。

## 录制一次完整的右手遥操作

录制文件用于替代 Apple Vision Pro 重放同一段手部动作。一次完整录制必须包含：

1. 自然姿态标定；
2. 两次“过度伸直 → 完整抓合”的范围标定；
3. 正式遥操作过程。

录制保存的是每帧原始 `27×4×4` 手部 transforms。后续回放仍会完整经过标定、
Mapping、滤波、Palm Solver、解选择和 fallback，适合重复比较不同版本的算法。

### 1. 开始前准备

- 确认 Apple Vision Pro 和运行电脑处于可通信网络中。
- 确认 Python 环境已经安装项目依赖。
- 在仓库根目录执行命令。
- 当前程序仍为打印测试模式，不会向真实手指或手掌电机发送命令。

建议每次使用独立、含义明确的文件名，例如：

```text
recordings/right_power_grasp_01.npz
recordings/right_power_grasp_01.jsonl
```

其中：

- `.npz`：原始手部帧，是以后回放的数据源；
- `.jsonl`：逐帧调试结果，包含 solver 和最终 control/fallback 状态。

### 2. 启动录制

将 `--avp-ip` 替换为当前 Apple Vision Pro 的实际地址：

```bash
python DHandControl/scripts/mh6_teleop_run.py --avp-ip 192.168.8.145 --record-session recordings/right_power_grasp_01.npz --debug-log recordings/right_power_grasp_01.jsonl
```

程序会自动创建 `recordings` 目录。控制台默认约 5Hz 显示一次完整的 solver 和
control preview；JSONL 会保存每一个处理帧。如果还需要控制台逐帧打印，增加：

```bash
--print-every-frame
```

### 3. 自然姿态标定

看到下面的提示后：

```text
Keep the right hand in a relaxed natural pose for neutral calibration...
```

操作者应：

- 右手保持在 Vision Pro 的稳定追踪范围内；
- 手腕和手掌自然放松；
- 五指自然伸展，不刻意张到最开；
- 不抓握，也不主动进行拇指对掌；
- 默认保持约 2 秒，直到程序进入下一阶段。

### 4. 运动范围标定

看到下面的提示后：

```text
Perform TWO quick cycles now: over-extend all fingers, then close/grasp the hand through its comfortable full range...
```

在默认约 8 秒的标定时间内，连续完成两次完整动作：

```text
过度伸直五指 → 完整抓合 → 过度伸直五指 → 完整抓合
```

要求：

- 过度伸直应明显超过自然状态，但不要造成疼痛；
- 完整抓合应覆盖舒适范围内的最大弯曲和拇指向内对掌；
- 动作应连续、清晰，不要长时间停在中间位置；
- 两次动作都应被 Vision Pro 完整追踪；
- 标定阶段不要离开追踪区域。

### 5. 正式操作录制

看到下面的提示后开始正式操作：

```text
Entering mapping loop. Press Ctrl-C to stop.
```

此时执行需要重复调试的右手操作，例如张手、力量型抓持、三指捏取或不同手指对指。

运行过程中，每次控制台输出都包含：

```text
palm solver preview: ... status=...
palm control preview: mode=... status=...
```

即使已经进入 fallback，Palm Solver 仍然会继续逐帧解算；solver 候选、选择结果和
状态不会被 fallback 日志替代。

### 6. 正常结束并保存

完成操作后，在终端按一次：

```text
Ctrl+C
```

程序会退出循环，并打印类似：

```text
Saved hand session: recordings/right_power_grasp_01.npz
```

必须使用 `Ctrl+C` 正常结束。不要使用 `kill -9` 或直接关闭进程，否则内存中的录制
帧可能来不及写入。NPZ 使用临时文件加原子替换保存，不会把半写入文件当成有效录制。

### 7. 核验录制文件

确认两个文件均已生成且大小不为零：

```bash
ls -lh \
  recordings/right_power_grasp_01.npz \
  recordings/right_power_grasp_01.jsonl
```

随后进行一次不等待墙上时间的完整离线回放：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/right_power_grasp_01.npz \
  --replay-no-wait \
  --debug-log recordings/right_power_grasp_01_replay.jsonl
```

正常完成时会看到：

```text
Replay teleoperation phase completed.
```

这表示录制中的自然标定、范围标定和正式操作三个阶段均能被读取和处理。

### 8. 常用回放方式

按原始速度回放：

```bash
python DHandControl/scripts/mh6_teleop_run.py --replay-session recordings/right_power_grasp_01.npz
```

半速观察：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/right_power_grasp_01.npz \
  --replay-speed 0.5 \
  --print-every-frame
```

循环正式 teleop 阶段：

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --replay-session recordings/right_power_grasp_01.npz \
  --replay-loop
```

`--replay-loop` 只循环正式操作，不会反复执行自然姿态和运动范围标定；按 `Ctrl+C`
结束循环回放。

## 相关文档

- [新 Palm Solver 原始交付包与源码位置](third_party/mh6_palm_solver/README.md)
- [三轴遥操作到新 Palm Solver 的适配](docs/mh6_palm_solver_adapter.md)
- [MH6 右手遥操作映射与控制框架 Handoff](docs/mh6_teleop_mapping_handoff.md)
- [MH6 遥操作框架](docs/mh6_teleop_framework.md)
