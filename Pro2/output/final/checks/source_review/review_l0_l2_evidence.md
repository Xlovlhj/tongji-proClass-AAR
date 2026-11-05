# 汇报 PPT 的 L0 / L2 证据核对

核对日期：2026-11-04。这里只整理正式结果，不修改原报告或重新运行仿真。数值原表同时整理至 `Pro2/output/final/evidence_l0_l2.json`。

## 1. 必须区分的口径

- **L0** 改 `u_weight`：MPC 接触 wrench 偏离名义重力支撑输入的总惩罚系数。三档为 `1e-7 / 1e-6 / 1e-5`，源配置 `Pro2/experiments/L0/mpc_input_weight/`。
- **L2-1** 改 `state_weights[9]`：前进速度状态误差权重。三档为 `0.2 / 1.0 / 5.0`，源配置 `Pro2/experiments/L2/mpc_vx_weight/`。
- 两组都是调参，但自变量不同、正式运行不同，不能把它们说成一个实验，也不能用 L2 的数据代替 L0。
- L0 的“最大倾角”是 `max(hypot(roll,pitch))`；L2 的 `max_tilt_rad` 是 `max(max|roll|, max|pitch|)`。直接横向比较会混淆指标定义。
- L2 总报告第 2 节有泛化的“原项目书建议 ROS + Gazebo/MuJoCo”旧表述。汇报应根据项目书明确：本项目采用人形路线 OpenLoong，原演示为 MuJoCo；实现保留 C++ 控制算法并接入 Isaac Sim，无 ROS 运行依赖。

## 2. 共同设计

每次运行 12 s、12,000 物理步、1,200 条日志；分析窗口 5.00–11.99 s，共 700 个样本。物理周期 1 ms，MPC 周期 5 ms，3 s 起步，目标前向速度 0.2 m/s；每类三档只改一个变量。L0 三次及 L2 十二次正式运行均完成，所有记录中 MPC/WBC 状态为 0。每个条件只有一次确定配置运行，不能声称统计显著性或普适最优。

速度 RMSE 使用机身系速度和控制器内部平滑参考；力矩平方积分是控制努力代理；绝对机械功代理不等于电机电能。L2 常规足底竖直力是控制器反算估计，不是接触传感器实测。

## 3. L0：补充调参已经完成

推荐一页标题：**输入代价三点调参：跟踪与控制努力之间的折中**。

| u_weight | vx RMSE (m/s) | 组合倾角峰值 (rad) | 高度标准差 (m) | 峰值关节力矩 (N·m) | 力矩平方积分 (N²·m²·s) |
|---:|---:|---:|---:|---:|---:|
| 1e-7 | 0.021894 | 0.050387 | 0.005872 | 173.01 | 128057.66 |
| 1e-6 | 0.023611 | 0.038328 | 0.004643 | 150.42 | 119002.22 |
| 1e-5 | 0.020213 | 0.033809 | 0.003989 | 159.41 | 122374.41 |

当前平地无扰动直行中推荐 `1e-5`：较基线速度 RMSE −14.39%、组合倾角峰值 −11.79%、控制努力 +2.83%、峰值力矩 +5.98%。只在三个候选值与当前场景中推荐，不是全局最优。现有默认值仍为 `1e-6`；推荐值保存在独立实验配置中。

- 首选图：`Pro2/output/L0/mpc_input_weight/comparison/speed_tracking.png`。
- 备选图：同目录 `posture.png` 与 `control_effort.png`。后者显示瞬时最大绝对关节力矩，不能称作力矩范数。
- 数值：`Pro2/output/L0/mpc_input_weight/comparison/metrics.json`、`metrics.csv`。
- 原始运行：`Pro2/output/L0/mpc_input_weight/low_1e-7/`、`baseline_1e-6/`、`high_1e-5/`。
- 已补做：不能再写“L0 尚未调参”。综合分数为人为加权的三档内部评价，建议 PPT 展示真实指标权衡，不突出综合分数。

## 4. L2 四组须分四页

### 4.1 速度误差权重：更准的跟踪需要更大峰值力矩

| state_weights[9] | vx RMSE (m/s) | 峰值力矩 (N·m) | 力矩平方积分 (N²·m²·s) |
|---:|---:|---:|---:|
| 0.2 | 0.04430 | 157.63 | 120548.5 |
| 1.0 | 0.02354 | 149.17 | 119201.3 |
| 5.0 | 0.01814 | 165.87 | 121014.3 |

高权重相对基线 RMSE −22.9%、峰值力矩 +11.2%、控制努力 +1.5%。提高 Q 中速度误差的代价，使求解器更积极分配接触 wrench；权重 5.0 是三档中速度误差最低，不是全指标最优。

- 首选图：`Pro2/output/L2/mpc_vx_weight/comparison/speed_tracking.png`，图形单面板，适于横向 PPT。低权重跟踪起伏较明显。
- 证据：`Pro2/output/L2/mpc_vx_weight/comparison/metrics.csv` 和 `analysis.md`。
- 三份配置：`Pro2/experiments/L2/mpc_vx_weight/low.json`、`baseline.json`、`high.json`。

### 4.2 预测时域：预测更远不一定更好

| N / 预测时窗 | vx RMSE (m/s) | Pitch峰值 (rad) | 抽样 native 控制器均值 (ms) | 抽样 native 控制器 P95 (ms) | QP CPU time P95 (ms) |
|---|---:|---:|---:|---:|---:|
| 5 / 25 ms | 0.02482 | 0.01603 | 0.934 | 1.070 | 0.480 |
| 10 / 50 ms | 0.02354 | 0.01711 | 1.148 | 1.272 | 0.472 |
| 15 / 75 ms | 0.02613 | 0.02620 | 1.562 | 1.746 | 0.536 |

50 ms 的速度误差最低。25 ms 抽样控制器均值较基线 −18.7%；75 ms +36.1%，Pitch 峰值 +53.1%。步数不是万能增益：未来窗变长会改变模型近似和 QP 组装。

- 固定 `mpc_dt=5 ms`，控制时域 `ch=3`，单步输入 `nu=13`；优化变量始终为 39，不能讲成随 N 线性增加决策变量。
- 三个控制库分别构建并在 run.json 留有 SHA256，确实改变了编译时预测长度。
- 首选原图：`Pro2/output/L2/prediction_horizon/comparison/solver_time.png`。**这张图仅是 qpOASES QP CPU time；不是上表完整 native 调用耗时。** 若 PPT 核心论点为“计算代价 +36.1%”，应在图外给完整调用均值，或据 metrics.csv 另画小柱状图，两种口径明确分开。
- 原图备选：同目录 `speed_tracking.png` 与 `posture.png`。
- 证据：`Pro2/output/L2/prediction_horizon/comparison/metrics.csv` 和 `analysis.md`。
- 配置：`Pro2/experiments/L2/prediction_horizon/short_n5.json`、`baseline_n10.json`、`long_n15.json`。
- 耗时为 100 Hz 日志抽样，且与 MPC 更新对齐，不是全部 1 kHz 步的时间统计，不能以此声称硬实时。

### 4.3 期望步高：平地低抬腿减少波动与机械功代理

| step_height (m) | vx RMSE (m/s) | 高度标准差 (m) | 峰值力矩 (N·m) | 绝对机械功代理 (J) |
|---:|---:|---:|---:|---:|
| 0.08 | 0.01788 | 0.00340 | 138.16 | 546.19 |
| 0.12 | 0.02354 | 0.00463 | 149.17 | 766.78 |
| 0.16 | 0.02708 | 0.00570 | 162.70 | 1051.31 |

0.08 m 较 0.12 m：速度 RMSE −24.0%，高度标准差 −26.6%，绝对机械功代理 −28.8%。高足端轨迹要求更大的腿部运动，也可能增加机身周期扰动。0.08 m 是本次无障碍平地三档中的较好候选。

- 首选图：`Pro2/output/L2/step_height/comparison/base_height.png`。图中低抬腿的基座波动幅值较小，单面板适于汇报。
- 证据：`Pro2/output/L2/step_height/comparison/metrics.csv`、`analysis.md`。
- 配置：`Pro2/experiments/L2/step_height/low_008.json`、`baseline_012.json`、`high_016.json`。
- 边界：这是期望步高，日志没有实测足端离地间隙；不能把平地结果直接外推到越障。

### 4.4 侧向外力：加倍冲量带来近似加倍的扰动响应

在 7.0 s 开始，沿世界 +Y 施力 0.1 s。三档 0 / 50 / 100 N，对应冲量 0 / 5 / 10 N·s；`local_frame=False`。7 s 以前三组轨迹一致。

| 外力 (N) | 同时间 ΔRoll 峰值 (rad) | 同时间 Δvy 峰值 (m/s) | 额外高度下降 (m) |
|---:|---:|---:|---:|
| 0 | 0 | 0 | 0 |
| 50 | 0.01809 | 0.08821 | 0.00490 |
| 100 | 0.03578 | 0.18292 | 0.01071 |

在 7–9 s 先逐时间点做 `push(t)−no_push(t)`，再取绝对差峰值，避免自然步态起伏误导。100 N 组仍完整运行 12 s，MPC/WBC 状态均为 0，本次 10 N·s 冲量未触发失稳判据。

- 首选原图：`Pro2/output/L2/lateral_push/comparison/disturbance_response.png`，上部前向速度、下部横向速度。下部横向响应更有表达力。原图较高，整图缩小到半页会太小；可单独展示下半面板，或从三份 states.csv 以同时间差分重画一张宽图。
- 不报告有效恢复时间：原固定阈值连 no_push 周期步态也不能连续满足，空结果不表示未恢复。应改为相对自然步态包络或步态相位对齐判据再研究。
- 证据：`Pro2/output/L2/lateral_push/comparison/metrics.csv`、`analysis.md`。
- 配置：`Pro2/experiments/L2/lateral_push/no_push.json`、`push_50N.json`、`push_100N.json`。

## 5. 最终视频状态

L2 十二次正式定量运行没有 MP4（`record_video=false`），不能在 PPT 写“L2 演示视频已完成”。L0 有历史默认示例视频 `Pro2/output/L0/walk_10s_video/demo.mp4`。这次任务是汇总 PPT，没有重新运行或录制视频。
