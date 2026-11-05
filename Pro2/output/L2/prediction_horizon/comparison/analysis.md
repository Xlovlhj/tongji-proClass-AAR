# MPC 预测时域对照实验分析

> 本文由 `Pro2/tools/analyze_l2_results.py` 从各运行目录的 `run.json` 和 `states.csv` 生成。数值结果保存在同目录的 `metrics.csv`。

## 1. 改了什么

本组唯一自变量是 `mpc_horizon_steps`，单位为 `step`。实验配置路径为：

- `Pro2/experiments/L2/prediction_horizon/short_n5.json`
- `Pro2/experiments/L2/prediction_horizon/baseline_n10.json`
- `Pro2/experiments/L2/prediction_horizon/long_n15.json`

各条件的实际预测步数优先读取 `run.json`；若运行清单未记录该字段，则按条件目录的设计值暂填，并在 `metrics.csv` 的 `variable_value_source` 中明确标记，验收前必须人工复核。

**单变量配置检查通过：**在运动、仿真和控制相关字段中，三次运行只改变了本组指定的自变量。

## 2. 观察到什么

统一分析窗口为 `5.0 s` 至日志末尾。`最大 Roll/Pitch 分量` 定义为该窗口内 `max(max|Roll|, max|Pitch|)`。

| 条件 | 自变量 | 运行/数据/验收校验 | 分析样本 | 内部参考 vx RMSE (m/s) | 最大 Roll/Pitch 分量 (rad) | 高度标准差 (m) | 力矩平方积分 (N²·m²·s) | MPC qpOASES QP CPU time P95 (ms) | MPC+WBC 非零状态样本数 | 恢复时间原始值 (s) |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 短时域 N=5 | 5 | completed / ok / ok | 700 | 0.02482 | 0.03883 | 0.004624 | 1.189e+05 | 0.4799 | 0 | — |
| 基线时域 N=10 | 10 | completed / ok / ok | 700 | 0.02354 | 0.03739 | 0.004629 | 1.192e+05 | 0.4721 | 0 | — |
| 长时域 N=15 | 15 | completed / ok / ok | 700 | 0.02613 | 0.03491 | 0.004823 | 1.195e+05 | 0.536 | 0 | — |

- 相对基线，**短时域 N=5** 的内部参考速度 RMSE 增加 5.4%，最大 Roll/Pitch 分量 增加 3.9%，力矩平方积分 降低 0.2%。
- 相对基线，**长时域 N=15** 的内部参考速度 RMSE 增加 11.0%，最大 Roll/Pitch 分量 降低 6.6%，力矩平方积分 增加 0.3%。

对应对比图：
- [`speed_tracking.png`](speed_tracking.png)
- [`posture.png`](posture.png)
- [`solver_time.png`](solver_time.png)

## 3. 为什么可能出现这些变化

预测步数 `N` 改变了状态预测矩阵及其覆盖的未来时间窗；`N=5/10/15` 在 `mpc_dt=0.005 s` 下分别对应 25/50/75 ms。当前实现的控制时域固定为 `ch=3`，单步输入维数 `nu=13`，因此 qpOASES 的决策变量维数始终是 `nu*ch=39`，不会随 `N` 增大。`N` 仍会改变压缩前的预测矩阵、组装结果和 QP 系数，但日志字段 `controller:mpc_solve_s` 只记录 qpOASES QP CPU time，不包含预测矩阵与 QP 组装时间。

这段解释给出与控制结构一致的机制假设。因果判断以本组单变量配置有效、运行完成且重复实验趋势一致为前提。

## 4. 结论边界

- 所有常规对比指标统一使用 `5.0 s` 至日志末尾的数据；扰动恢复姿态基准使用施力前 1.0 s。
- `controller:left_force_z_est_N` 与 `controller:right_force_z_est_N` 是动力学反算值，不是 PhysX 接触传感器实测值。
- 力矩平方积分是控制努力代理量；绝对机械功也是机械侧代理量，二者都不是电机电能。
- `controller:mpc_solve_s` 与 `controller:wbc_solve_s` 是 qpOASES 报告的 QP CPU time，由本实验 CSV 以 100 Hz 抽样记录；它们不包含预测矩阵构造、QP 组装，也不等于完整控制循环耗时。
- `MPC+WBC 非零状态样本数` 是统一分析窗口内两个求解器状态字段非零样本数之和，不等于互不重复的时间点数。
- 当前结果来自 Isaac Sim/PhysX 中的一次确定配置运行。没有重复试验时，只能说明本次仿真的差异。
- 图中只放曲线、标题、坐标轴和图例；全部数值、样本数与条件保存在 `metrics.csv`。
