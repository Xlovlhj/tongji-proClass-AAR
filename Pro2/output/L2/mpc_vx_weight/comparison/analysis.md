# MPC 前进速度误差权重对照实验分析

> 本文由 `Pro2/tools/analyze_l2_results.py` 从各运行目录的 `run.json` 和 `states.csv` 生成。数值结果保存在同目录的 `metrics.csv`。

## 1. 改了什么

本组唯一自变量是 `state_weights[9]`，单位为 `1`。实验配置路径为：

- `Pro2/experiments/L2/mpc_vx_weight/low.json`
- `Pro2/experiments/L2/mpc_vx_weight/baseline.json`
- `Pro2/experiments/L2/mpc_vx_weight/high.json`

各条件的实际取值读取自 `run.json`，具体来源记录在 `metrics.csv` 的 `variable_value_source` 列。

**单变量配置检查通过：**在运动、仿真和控制相关字段中，三次运行只改变了本组指定的自变量。

## 2. 观察到什么

统一分析窗口为 `5.0 s` 至日志末尾。`最大 Roll/Pitch 分量` 定义为该窗口内 `max(max|Roll|, max|Pitch|)`。

| 条件 | 自变量 | 运行/数据/验收校验 | 分析样本 | 内部参考 vx RMSE (m/s) | 最大 Roll/Pitch 分量 (rad) | 高度标准差 (m) | 力矩平方积分 (N²·m²·s) | MPC qpOASES QP CPU time P95 (ms) | MPC+WBC 非零状态样本数 | 恢复时间原始值 (s) |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 低权重 0.2 | 0.2 | completed / ok / ok | 700 | 0.0443 | 0.03886 | 0.004685 | 1.205e+05 | 0.506 | 0 | — |
| 基线 1.0 | 1 | completed / ok / ok | 700 | 0.02354 | 0.03739 | 0.004629 | 1.192e+05 | 0.4672 | 0 | — |
| 高权重 5.0 | 5 | completed / ok / ok | 700 | 0.01814 | 0.03685 | 0.004604 | 1.21e+05 | 0.4421 | 0 | — |

- 相对基线，**低权重 0.2** 的内部参考速度 RMSE 增加 88.2%，最大 Roll/Pitch 分量 增加 3.9%，力矩平方积分 增加 1.1%。
- 相对基线，**高权重 5.0** 的内部参考速度 RMSE 降低 22.9%，最大 Roll/Pitch 分量 降低 1.4%，力矩平方积分 增加 1.5%。

对应对比图：
- [`speed_tracking.png`](speed_tracking.png)
- [`posture.png`](posture.png)
- [`control_effort.png`](control_effort.png)

## 3. 为什么可能出现这些变化

`state_weights[9]` 增大后，MPC 目标函数会提高前进速度误差的代价，求解器可能分配更积极的接触 wrench。是否因此改善闭环表现，仍要同时检查姿态、足底估计力、关节力矩和约束求解状态；权重更大本身不能证明更优。

这段解释给出与控制结构一致的机制假设。因果判断以本组单变量配置有效、运行完成且重复实验趋势一致为前提。

## 4. 结论边界

- 所有常规对比指标统一使用 `5.0 s` 至日志末尾的数据；扰动恢复姿态基准使用施力前 1.0 s。
- `controller:left_force_z_est_N` 与 `controller:right_force_z_est_N` 是动力学反算值，不是 PhysX 接触传感器实测值。
- 力矩平方积分是控制努力代理量；绝对机械功也是机械侧代理量，二者都不是电机电能。
- `controller:mpc_solve_s` 与 `controller:wbc_solve_s` 是 qpOASES 报告的 QP CPU time，由本实验 CSV 以 100 Hz 抽样记录；它们不包含预测矩阵构造、QP 组装，也不等于完整控制循环耗时。
- `MPC+WBC 非零状态样本数` 是统一分析窗口内两个求解器状态字段非零样本数之和，不等于互不重复的时间点数。
- 当前结果来自 Isaac Sim/PhysX 中的一次确定配置运行。没有重复试验时，只能说明本次仿真的差异。
- 图中只放曲线、标题、坐标轴和图例；全部数值、样本数与条件保存在 `metrics.csv`。
