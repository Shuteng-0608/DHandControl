# DHandControl

## 在另一台电脑继续当前工作

当前录制、标定和分析进展位于 `codex/palm-fallback-control` 分支。
请先阅读 [跨电脑恢复与当前结论](docs/mh6_continue_on_another_computer.md)。
原始输入在 `recordings/`；全部分析日志、权重搜索、图表和脚本已无损打包，运行：

```bash
python artifacts/teleop_01/restore_analysis.py
```

即可恢复到 `results/solver_only/`。高有解率权重存在动作意图失真，尚未替换原标定；
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
