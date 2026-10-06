# MH6 Palm Solver 原始交付包

[`a1t1a3_7_20/`](a1t1a3_7_20/) 保存用户提供的新 solver 完整交付包。
原始文件保持不变，包括求解器、测试、依赖说明、方向图、程序说明及 LaTeX 源文件、
工作空间分析脚本、轨迹数据和实验结果；上游校验清单也一并保留。

## 文件位置

| 内容 | 路径 |
|---|---|
| 原始求解器源码 | [solve_from_arpha2_arpha3_theta1.py](a1t1a3_7_20/solve_from_arpha2_arpha3_theta1.py) |
| 原始使用说明 | [README.md](a1t1a3_7_20/README.md) |
| 初始位姿和折叠方向图 | [手掌关节角正方向设定说明.pdf](a1t1a3_7_20/手掌关节角正方向设定说明.pdf) |
| 程序说明 | [MH6手掌闭环求解器_程序说明.pdf](a1t1a3_7_20/MH6手掌闭环求解器_程序说明.pdf) |
| 上游单元测试 | [test_mh6_palm_solver.py](a1t1a3_7_20/test_mh6_palm_solver.py) |
| 项目运行核心 | [mh6_palm_solver_v2.py](../../DHandControl/scripts/mh6_palm_solver_v2.py) |
| 遥操作适配器 | [mh6_palm_solver_adapter.py](../../DHandControl/scripts/mh6_palm_solver_adapter.py) |
| 集成说明 | [三轴遥操作适配](../../docs/mh6_palm_solver_adapter.md) |

原始包作为版本存档，项目运行使用 `DHandControl/scripts/mh6_palm_solver_v2.py`。
该运行文件与原始源码逐字节一致，适配器直接导入它，运行无需本机 Downloads 目录。
今后更新上游时，应同时核对存档版本和运行副本。

注意两层默认值不同：原始求解器归一化入口默认 `workspace_conditional`；
遥操作适配层默认显式选择 `motor_range`，以保留平面自然位。
具体参数语义、电机顺序和项目限位以集成说明为准。

## 校验

在仓库根目录下执行（仅使用 Python 标准库）：

```bash
python third_party/mh6_palm_solver/verify_delivery.py
```

该脚本兼容原始校验清单的 Windows 换行，验证全部 25 项文件校验和，
并检查项目运行核心与原始源码一致。Git 属性保留原始文件的字节和换行。

求解器源码的 SHA-256：

```text
7d95710ada4e22978519a0b2e7fbf6f0215ede2b07155600271f6bcf3ddecdc9
```

存档中的 CSV 是随包交付的实验数据，局部 `.gitignore` 只为这些文件开放提交。
