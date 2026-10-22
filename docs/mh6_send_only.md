# 实时遥操作：只发送测试

此模式由操作员执行。代理仅进行文件核验、离线分析和模拟测试，不连接设备。
默认入口仍仅预览；实际发送必须同时指定三个显式选项：
`--enable-hardware --no-feedback --confirm-initial-position`。

## 启动前

确认九路电机已经处于 `DHandControl/config/mh6_hardware.json` 的 `initial_position`：
M1/M2/M3/M4 为 `[247,500,500,200]`，F1～F5 均为 20。
`--confirm-initial-position` 是操作者的人工确认，不会读取实际位置，也不会自动回初始位。
限位及全部 `max_position_step` 必须填写；任意缺失会在 AVP、串口初始化前报错。
使用包含 M4 人手端点的正式标定文件，不支持默认人手端点或实机回放。

关闭占用同一串口的测试脚本和串口监视器。在 `mh6` 环境由操作者运行：

```bash
python /home/stw/DHandControl/DHandControl/scripts/mh6_teleop_run.py \
  --avp-ip 192.168.8.109 \
  --mapping-calibration /home/stw/DHandControl/recordings/mapping_guided_02.json \
  --port /dev/ttyUSB0 --rate 30 \
  --enable-hardware --no-feedback --confirm-initial-position
```

30 Hz 是目标循环频率；Modbus 往返和计算耗时可能降低实际频率，不积压补发旧帧。
默认低通时间常数为 0.24 秒。控制仍采用固定 plus 分支和绝对限位；
M1～M3 接受完整三元组或保持上一组三元组，超过位置跳变阈值仍整组保持。
F1～F5 和 M4 对有效目标执行单帧位置增量限制：与上一已确认指令比较，超过阈值就向目标最多走一个阈值，逐帧逼近且不越过目标；无效输入或超出绝对限位仍保持。
日志状态 `POSITION_STEP_LIMITED` 表示目标有效但本帧位置增量已受限；原目标保留在 `integer_target`，拟发送位置在 `position`，实际增量在 `applied_position_delta`。
阈值测试初值不代表已核实的机械安全额定值，后续应按独立动作轨迹调整。

本次测试初值（均为单帧执行器位置单位）：F1～F5 各 150；M1=40、M2=50、M3=20、M4=30。
填写阈值时，曾使用旧原生工作空间，将标定录制的映射结果离线插值至 30 Hz，
使用 0.24 秒低通，共分析 3848 个时刻（当前 closing_only 结果见下一段链接）：
五指和 M4 在该诊断中均通过位置检查；手掌三元组 1015 次接受、2471 次限位保持、362 次跳变保持。
这只是标定动作上的离线诊断，不是独立遥操作轨迹、实机频率测量或硬件安全验证。
M1～M3 的逐帧跳变拒绝可能保持较长时间，不通过裁剪耦合电机来绕过；五指和 M4 已改为独立的单帧位置增量限制。

开机固定清除五指错误；任一路失败就结束，不开始遥操作。
不在运动过程中定时盲目清错。清错应先确认故障原因已排除；固件每指清错还会等待 20 ms 并查询错误状态，会占用发送链路。
`--no-feedback` 禁用 `read_finger_status`、`read_palm_status` 周期查询。
仍保留正常 Modbus 应答、固件下发状态 0x90 检查及开机清错确认，它们不证明电机到位。
固件清错内部自身包含一次错误状态验证，这不是主机周期轮询。

发送成功后才更新上一已确认位置；发送失败则记录日志并退出，不自动重试、回零或恢复。

每帧 `execution.driver_result.timings_ms` 记录三个主机侧通信步骤的耗时（毫秒）：
`parameter_write` 写九路目标参数，`command_write` 写执行触发，`firmware_status_read` 读固件下发确认。
异常/超时的步骤也保留耗时，未执行的步骤不出现。这些计时随现有日志保存，不增加请求、不打印逐步计时，也不代表电机到位时间。
Ctrl+C 只结束主机发送并关闭连接，已经下发的运动可能继续完成。

默认每次自动创建 `results/hardware_send_only/live_时间戳.jsonl`，包含每帧映射、Solver 候选、
九路整数报文、保持原因、发送确认和耗时；`feedback.enabled=false`。
也可用 `--debug-log PATH` 指定输出文件（现有同名文件会被覆盖）。

当前默认 `workspace_profile=closing_only`，人手零意图生成机构角 `[0,0,0]`，
由原 Solver 返回伸开位置 `[247,500,500]`。生成工作空间时就约束闭合侧与 plus 分支，
不裁剪或重新缩放三路电机。细节和刚才实机日志的离线重算见 [向内工作空间](mh6_inward_workspace.md)。

## 本次人手标定核验

`calibration_guided_02.npz` 为完整 `staged_v3`，1620 帧，时间跨度约 128.25 秒。
从 NPZ 重新生成的标定与 `mapping_guided_02.json` 一致。M4 人手角度为
自然参考 18.8171°、偏转端点 32.0751°，跨度约 13.2580°。
最终返回验证 h/v/r 中位数分别约 0.0529/0.00529/0.00599，M4 返回误差约 0.0895，
均低于 0.15；首次 M4 返回验证失败后，第二次通过。

## 2026-10-22 调试检查点

- M4 电机范围和映射端点已调整为 200～400，初始位置仍为 200。
- 本机操作员将 FTDI `/dev/ttyUSB0/latency_timer` 从 16 改为 1 后，下发频率从约 15.64 Hz 提高到 23.67 Hz；这是系统运行时设置，不会随 Git 配置自动应用到其他电脑。
- 加入通信分段计时后的独立记录为 1463 帧、约 61.87 秒，实际约 23.63 Hz；平均写参数 15.85 ms、写触发 10.57 ms、读下发确认 10.39 ms，通信外平均 5.45 ms。
- 汇总保存到 [USB 延迟对比](../artifacts/teleop_checkpoint_20261022/usb_latency_16_to_1_comparison.json) 和 [通信分段计时](../artifacts/teleop_checkpoint_20261022/live_20261022_213123_392720_stage_timing.json)。原始 `results/` 运行日志继续按 `.gitignore` 保留在本地；这些指标是主机下发时序，不是电机到位时序。
- 已知未修复问题：向内工作空间在滤波输入衰减到接近零时，可能因数值边界判断抛出 `inward workspace has no continuous plus-branch pose for this input`，主循环随后退出。此前已离线复现，当前提交仅记录该问题。
- 三次 Modbus 往返仍保留；尚未合并触发、取消确认或修改固件判帧。测试脚本 `mh6_fast_control_test.py` 保留操作者当前编辑内容（启用运动且调用 `free_all()`），不属于默认锁定的预览入口。
