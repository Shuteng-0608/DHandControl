# 原子报文：两种遥操作通信实验

从检查点 `91ce145` 开始，两个分支共用协议与固件：

| 分支 | 默认 `--control-transport` | 每帧通信 |
|---|---|---|
| `codex/teleop-atomic-ack` | `atomic_ack` | 一包目标并执行，一包结果应答 |
| `codex/teleop-atomic-oneway` | `atomic_oneway` | 一包目标并执行，不读取/不产生应答 |

两版均可用 `--control-transport legacy` 对比原先的三次 Modbus 往返。
库接口 `DexHandControl.move_hand()` 和直接构造 `HardwareSender` 的默认行为仍为 legacy；
分支默认只作用于 `mh6_teleop_run.py` 的遥操作入口。手动测试脚本不会自动变成新协议。

映射、滤波、Solver、固定 plus 分支、硬件限位、位置增量阈值、默认目标 30 Hz 和舵机运动时间 80 ms 均保持。
已知近零工作空间异常也保留，发生时应单独记为算法退出，不能计作通信失败。

## 固件准备（操作者执行）

1. 拉取任一实验分支，用 Arduino IDE 打开 `DHandControl/DH6Modbus/DH6Modbus.ino`。
2. 选择实际开发板与烧录端口，编译并烧录。代理不会连接/烧录设备。
3. 这份固件同时支持 ACK、单向与旧 Modbus；比较两个分支无需再次烧录。
4. 保持单一控制进程独占串口。目标发送前，程序通过原有 Modbus 只读检查寄存器 50～52 是否为 `[0x4D36,1,3]`。旧固件不匹配会退出，不发送运动目标。
5. 两版开机都只清错一次（F1～F5），然后开始实验。单向模式的“不读取”指运动循环，不包括开机能力检查与清错确认。

本机 FTDI 的 1 ms USB 延迟是系统设置，Git 切换不会自动设置其他电脑；比较时应保持一致。
新帧是固定 64 字节，收到完整帧并通过 CRC 后立即处理；旧 Modbus 的 5 ms 判帧等待保持原样。
这一接收优化在两个实验分支中相同。

## 操作者测试

九路电机先处于配置初始位，使用同一份正式人手标定。每轮执行相同动作顺序和近似速度，保持 `--rate 30`，关闭反馈。
先测试 ACK 版：

```bash
git fetch origin
git switch codex/teleop-atomic-ack
python DHandControl/scripts/mh6_teleop_run.py \
  --avp-ip 192.168.8.109 \
  --mapping-calibration recordings/mapping_guided_02.json \
  --port /dev/ttyUSB0 --rate 30 \
  --enable-hardware --no-feedback --confirm-initial-position \
  --debug-log results/transport_compare/atomic_ack_01.jsonl
```

结束当前程序、确认九路回到相同初始位后，再切换单向版：

```bash
git switch codex/teleop-atomic-oneway
python DHandControl/scripts/mh6_teleop_run.py \
  --avp-ip 192.168.8.109 \
  --mapping-calibration recordings/mapping_guided_02.json \
  --port /dev/ttyUSB0 --rate 30 \
  --enable-hardware --no-feedback --confirm-initial-position \
  --debug-log results/transport_compare/atomic_oneway_01.jsonl
```

Windows 将端口改为实际 COM 口；脚本路径相对仓库根目录，无需修改源码中的绝对路径。
日志目录自动创建。同名日志会被覆盖；反复测试请更换编号。
可显式指定 `--control-transport`，在同一分支、同一固件上切换三种模式，避免其他差异影响对比。

离线比较，不连接设备：

```bash
python DHandControl/scripts/mh6_compare_transport_logs.py \
  results/transport_compare/atomic_ack_01.jsonl \
  results/transport_compare/atomic_oneway_01.jsonl \
  --output results/transport_compare/comparison.json
```

目标 30 Hz 达到后，两版显示的发送频率都可能接近 30；此时主要比较发送占用、间隔抖动、限幅次数和主观跟随体验，不应据此声称两版端到端延迟相等。
单向发送耗时只包含本机写入和本地发送队列排空，不包括任何设备接收/运动确认；不与 ACK 版的完整往返耗时当作同一种测量。
位置增量阈值仍按帧计算；实际频率提高后，每秒允许累计位移也会增加，解读运动体验时需考虑这一点。

## 应答、历史位置与失败

- ACK 成功：`execution.status=WRITE_ACKNOWLEDGED`，固件回显相同序号并返回 `0x90`，才更新上一确认目标；不是电机到位确认。
- 单向成功：`execution.status=HOST_WRITTEN_UNCONFIRMED`、`write_acknowledged=false`、`history_reference=last_host_written_target`，只更新上一主机写出的目标。
- 单向每帧不读任何数据，固件即使拒绝该帧也不应答；若线路丢帧或设备掉线，主机可能仍写出成功。丢帧后不能保证电机实际相邻收到的目标差仍小于单帧位置阈值。
- ACK 超时、错误序号、CRC 错、固件拒绝，或任一模式本地短写/写失败，均记录并结束；不自动重试过时目标，也不更新失败目标的历史。
- Ctrl+C 仍仅关闭连接，不自动复位或停止已经下发的动作。

首帧 `runtime_config` 保存模式、目标频率、滤波和串口超时。
`execution.driver_result.timings_ms.wire_write` 是写入及本地队列排空时间；ACK 版另有 `ack_wait`，单向版没有。
旧三步计时仅用于 legacy 模式。

## 报文定义 v1

全部整数按大端编码，CRC16/Modbus 按低字节在先。固定从站地址 1、自定义功能码 `0x41`，不伪装成标准 Modbus 写寄存器响应。

请求共 64 字节：地址、功能码、版本、标志（0=单向、1=应答），32 位序号，27 个目标寄存器，CRC。
27 个寄存器沿用旧 `move_hand` 布局，必须包含五指 ID1～5 与手掌 ID1～4 各一次；末三个保留字为 0。
固件先检查整包版本、数量、ID、协议位置域与运动时间，再发布目标并下发，任何一轴不合法都不部分执行。
机械限位仍以 Python `mh6_hardware.json` 为准；固件域检查只是五指 0～2000、手掌 0～1000 的协议范围。

应答共 12 字节：地址、功能码、版本、标志 1，相同序号，16 位结果码，CRC。
`0x90` 表示固件已调用电机下发；`0x0101～0x0105` 分别表示版本/标志、数量/保留字、ID、位置域、运动时间错误。
CRC 错、错误从站地址、缺帧静默丢弃，ACK 主机随后超时。
53～55 为最近处理的序号高/低字和结果，56/57 为成功/拒绝计数（16 位回绕）；运动循环不查询这些寄存器。
