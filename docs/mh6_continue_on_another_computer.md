# 在另一台电脑继续 MH6 Mapping / Solver 工作

本次工作保存在 `codex/palm-fallback-control` 分支。使用该分支，不要只拉取 `master`。
以下命令均在仓库根目录执行；不依赖原电脑的 `/home/stw/DHandControl` 路径。

## 克隆、安装离线环境、恢复结果

```bash
git clone --branch codex/palm-fallback-control https://github.com/Shuteng-0608/DHandControl.git
cd DHandControl

conda create -n mh6 python=3.12 -y
conda activate mh6
python -m pip install -r requirements-offline.txt

python artifacts/teleop_01/restore_analysis.py
```

恢复脚本仅使用 Python 标准库，也可以在安装科学计算依赖之前运行。
离线测试无需 AVP、ROS、串口或机器人。要重新连接 AVP 采集数据，再按 README 安装
`requirements-avp.txt` 并指定当前 AVP 地址。

`artifacts/teleop_01/analysis_results.zip` 保存完整 `results/solver_only/` 快照，
包括逐帧日志、全部权重搜索候选、统计、标定候选、PNG/SVG 图和分析脚本。
较大的 JSONL 超过 GitHub 普通 Git 的单文件限制，因此采用无损 ZIP；不需要 Git LFS。
仅排除可重新生成的 Python 字节码缓存。

恢复前会核对 ZIP 及每个文件的 SHA-256。已存在且相同的文件跳过；文件内容不同时默认
停止，不会覆盖你在新电脑上的新结果。可用 `--verify-only` 只校验，或
`--destination PATH` 解压到另一个目录。历史报告中的绝对路径是原电脑的溯源记录，
阅读时将仓库前缀换成当前克隆路径；恢复后的分析脚本按自身位置定位仓库。

## 当前输入和正确的测试基线

| 文件 | 用途 |
|---|---|
| `recordings/calibration_01.npz` | 原始自然姿态、运动范围标定录制 |
| `recordings/mapping_01.json` | 原始人手 Mapping 参数；仍是当前基线 |
| `recordings/teleop_01.npz` | 599 帧、约 29.99 秒的固定动作录制 |
| `DHandControl/config/mh6_palm_adapter_calibration.json` | 人手三分量到机器人角度的适配参数 |
| `third_party/mh6_palm_solver/a1t1a3_7_20/` | 上游 Solver 源码、说明 PDF 和测试 |

旧的 `right_power_grasp_01/02` 录制、日志及图也保留在 `recordings/`。

```bash
python DHandControl/scripts/mh6_teleop_run.py \
  --solver-only \
  --replay-session recordings/teleop_01.npz \
  --mapping-calibration recordings/mapping_01.json \
  --replay-no-wait \
  --debug-log results/solver_only/teleop_01_new_computer.jsonl
```

该命令逐帧独立测试 Mapping → Solver，保存日志后静默退出，不连接机器人。
对应 `teleop_01_new_computer.summary.json` 应为：599 帧，几何有解 128 帧，
电机范围内可用 127 帧。耗时字段会随电脑变化。

角度语义已修正：`alpha2 = 上下翻折 h`、`alpha3 = 拇指旋转 r`、
`theta1 = 左右翻折 v`。Solver 角度顺序是 `(h, r, v)`，适配器公开接口是 `(h, v, r)`。
人手 `[-1,1]` 通过标定中点分段转换成 Solver 的 `[0,1]`，保留自然姿态零角度。

正确历史基线是 `results/solver_only/teleop_01_axis_corrected.jsonl`。
同目录早期 `teleop_01.jsonl` 及未带 `axis_corrected` 的旧图保存了轴顺序修正前的结果，
仅供追溯，不能作为当前基线。

## 最新结论：优先查看动作意图审计

用户确认动作顺序为：三次握拳 → 食指、中指、无名指、小指依次与拇指对指 → 三次握拳。
三路搜索累计评估 203,736 次候选。几何有解最高为 251/599（41.90%），
电机可用最高为 230/599（38.40%），但这些参数有提前饱和、改变翻折方向及手指作用的现象。

**这些高分参数未被采用，也不能视为保持了动作意图。**此前名为 `balanced` 的候选
只在全局相关性、标准差上较接近基线，按动作分段复核后仍有对指减弱和局部方向变化。

- 按四长指平均归一化弯曲 ≥0.8 筛出充分握拳 90 帧，四组参数都只有 6 帧电机可用。
- 按实际目标指尖距离 <20 mm 且为最近手指筛出完成对指 51 帧，四组参数均无解。
- 原始 Mapping 也存在相邻手指串扰；`max(weight × proximity)` 并不识别实际对指目标。
- 末尾帧 525–598（26.3256–29.9866 秒）的全部原始手部矩阵相同。可确认数据重复，
  无法从缺少源帧 ID / 有效性标记的录制中确定原因；不要将这段解释成第七次握拳。
- 下一步应先约束握拳渐进程度、对指目标和非目标手指串扰，再评估各类动作完成时的有解率。
  原始 Mapping 仅是对照，仍需机器人端目标或任务指标验证。

恢复结果后，优先阅读：

| 路径（相对 `results/solver_only/`） | 内容 |
|---|---|
| `weight_search/intent_audit/summary.json` | 最新动作意图审计与候选判定 |
| `weight_search/intent_audit/finger_review.json` | 从原始 NPZ 复算的手指、对指目标及重复数据审计 |
| `weight_search/intent_audit/robust_review.json` | 分动作统计、提前饱和和方向变化 |
| `weight_search/intent_audit/gesture_intent_audit.png` | 动作意图对照图 |
| `weight_search/intent_audit/frames.jsonl` | 每帧原始特征、四组映射及求解状态 |
| `weight_search/summary.json` | 搜索规模、参数、全帧统计；需结合最新审计解读 |
| `weight_search/selected/` | 基线、三组实验参数、真实 NPZ 回放日志与汇总 |
| `teleop_01_mapping_feasibility_diagnosis.json` | Mapping 可行域与 Solver 归因分析 |
| `teleop_01_solver_return_audit.json` | Solver 分支、电机过滤与误差审计 |

## 绘图、重新审计和验证

```bash
python results/solver_only/plot_solver_results.py \
  --log results/solver_only/teleop_01_new_computer.jsonl

python results/solver_only/weight_search/intent_audit/audit_gesture_intent.py

python -m unittest discover -s DHandControl/tests
```

原始 PNG 已包含中文。重新绘图时如缺少中文字形，请安装 Noto CJK 字体；图表脚本在
Linux 上会优先读取 `/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc`。
离线回放和数值结果不依赖字体。

搜索脚本和全候选记录也在快照中，可继续研究；重新搜索会更新对应实验目录的输出，
需要保留当前快照时请先复制实验目录。恢复脚本不会改动原始录制或标定文件。
