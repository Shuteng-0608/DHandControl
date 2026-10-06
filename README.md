# DHandControl

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
