# L4 汇报证据核查

已完整阅读 `Pro2/output/L4/L4验收报告.md`，并直接核对正式 JSON、CSV 和运行记录。原报告与原始实验文件未改动。所有下列路径从项目根目录起算；精确数字及 SHA-256 保存在 `Pro2/output/final/evidence_l4.json`。

## 汇报的总口径

五个开放方向均有实施成果，但验收能力不同：H1 为模型接入与独立站立；路径为闭环跟踪与到点事件；地形为单个障碍 seed 区域穿越和其他地形部分进入；跳跃完成八阶段与两次重复；NMPC 为带保护回退的闭环；力矩分析是正式 Convex 实验的派生分析。不能把所有目录的 `completed` 直接翻译为全部具体性能目标达成。

原 L4 报告第二节称“项目书按 ROS 与 MuJoCo 组织”，这句话过于笼统。最终汇报应根据项目书人形路线写为 **OpenLoong 原生 MuJoCo → Isaac Sim / PhysX 适配，人形路线无需 ROS**；四足路线与人形路线的环境要求不能混用。

## 1. 模型更换：H1-2 独立站立

正式目录：`Pro2/output/L4/01_robot_swap/h1_fixstand_feedback_8s_final_v2/`。

直接核对 `run.json` 与 `metrics.json`：运行 completed、acceptance_passed=true、**openloong_mpc_wbc_used=false**。27 关节 H1-2，与 AzureLoong 31 关节不同；采用 Unitree FixStand 目标、PD drive 与骨盆反馈，非 OpenLoong MPC/WBC 移植。

| 指标 | 正式值 |
|---|---:|
| 运行 / 过渡 / 保持 | 8 / 3 / 5 s |
| 最低骨盆高度 | 0.989278 m |
| 最大单轴 Roll/Pitch | 0.077643 rad |
| 最大水平漂移 | 0.060733 m |
| 最终关节目标 RMSE | 0.036503 rad |
| PD 力矩估计峰值 | 48.383448 N·m |

推荐标题：**H1-2 模型接入与独立站立通过**。
推荐展示高度和关节误差；图与数字分别放置：

- `Pro2/output/L4/01_robot_swap/h1_fixstand_feedback_8s_final_v2/analysis/base_height.png`
- `Pro2/output/L4/01_robot_swap/h1_fixstand_feedback_8s_final_v2/analysis/joint_tracking.png`

备注必须解释：显式驱动力矩字段为 0 是因为使用 PhysX 隐式 PD drive，不代表无力矩。

## 2. 路径跟踪：五组输出，四组触发到点

指标源：`Pro2/output/L4/02_path_tracking/path_tracking_summary.json`。五组 completed，四组 reached_goal=true。

| 正式目录后缀 | 时长 s | 横向 RMSE m | 航向 RMSE rad | 最终目标距离 m | 到点 |
|---|---:|---:|---:|---:|---|
| straight_run02 | 18 | 0.079088 | 0.030874 | 0.031838 | 是 |
| turn_90 | 28 | 0.201024 | 0.715777 | 0.018601 | 是 |
| circle_open_loop_run02 | 50 | 0.144800 | 0.368396 | 0.270813 | 否 |
| circle_closed_loop_run02 | 40 | 0.133950 | 0.233396 | 0.156979 | 是 |
| waypoints_run06 | 30 | 0.217320 | 0.682859 | 0.287403 | 是，锁存 |

这五组位于 `Pro2/output/L4/02_path_tracking/`。

**公平开闭环对照要另用历史 circle_closed_loop，而不是 circle_closed_loop_run02。**
核对二者 run.json，`circle_open_loop_run02` 与 `circle_closed_loop` 同为 50 s、walk_start=5 s、到点容差 0.12 m、同一控制库 hash `2030b85c...676ea74f`。

| 指标 | 开环 | 公平闭环 | 解释 |
|---|---:|---:|---|
| 横向 RMSE m | 0.144800 | 0.144047 | 变化很小 |
| 航向 RMSE rad | 0.368396 | 0.192389 | 下降约 47.78% |
| 最大路径距离 m | 0.292181 | 0.354232 | 变差 |
| 到点 | 否 | 否 | 不能声称到点改善 |

推荐标题：**闭环主要改善圆形航向跟踪，折线和多航点也形成到点证据**。
建议两张主图（开闭环并列）明确写同为 50 s；如果另需展示转弯，可在下一页用 turn_90 和 waypoints：

- `Pro2/output/L4/02_path_tracking/circle_open_loop_run02/analysis/planar_trajectory.png`
- `Pro2/output/L4/02_path_tracking/circle_closed_loop/analysis/planar_trajectory.png`
- 备用：`Pro2/output/L4/02_path_tracking/turn_90/analysis/planar_trajectory.png`
- 备用：`Pro2/output/L4/02_path_tracking/waypoints_run06/analysis/planar_trajectory.png`

备注边界：40 s run02 用 0.25 m 容差，适合证明到点，不适合直接与 50 s 开环作误差改进归因。waypoints 在 24.47 s 首次进入 0.25 m 后锁存，零速踏步产生末态 0.287403 m 漂移；锁存不等于最后一帧在容差内。地面路径可视化是后来加入，历史五组数据不证明它们当时已显示参考线。

## 3a. 地形：一个区域穿越，三个部分进入

指标源：`Pro2/output/L4/03_terrain_and_jump/terrain_summary.json`。
判据是 completed + 基座 y 位于地形宽度内 + 基座 x 越过末端。这是 **基座包络穿越代理**，未记录足端 contact prim，不能证明具体脚逐个踩到障碍。

| 正式目录后缀 | 最大 x m | 进度 | 判定 |
|---|---:|---:|---|
| slope_05deg | 1.720137 | 36.946% | 进入未穿越 |
| rough_medium_run02 | 2.142166 | 47.935% | 进入未穿越 |
| random_obstacles_seed01_full | 3.309629 | 100% | 基座穿过区域 |
| blind_step_up_run02 | 1.546552 | 16.164% | 进入未穿越 |

随机障碍正式运行 45 s、seed=1、12 个障碍；实际障碍包络 x=0.804211–2.801533 m，配置区域终点 3.0 m。最大 x=3.309629 m 同时越过两者。最低高度 1.04786 m、最大单轴倾角 0.070783 rad。仅单 seed 单次结果。

盲踩是基座越过前缘 0.646552 m，非足端接触序列证明。
推荐标题：**随机障碍区域完成穿越，其余地形保留部分进入结果**。
推荐图：

- `Pro2/output/L4/03_terrain_and_jump/random_obstacles_seed01_full/analysis/planar_trajectory.png`
- `Pro2/output/L4/03_terrain_and_jump/random_obstacles_seed01_full/analysis/base_attitude.png`

`rough_low` / `rough_medium` 的旧 Approach 有几何间隙，已作废；不能用作算法失败证据。

## 3b. 跳跃：八阶段，两次独立正式验收

正式目录分别为：

- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run01/`
- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run02/`

直接核对每个目录 `analysis_final/acceptance.json`：均 pass=true，均 **21/21** 检查通过；同一控制库 hash `8e8d5f8c...b6e1bc48`；两个 states.csv 的 SHA-256 不同。关键轨迹指标相同，属于确定性设置下两次重复，不是跨条件鲁棒性统计。

八阶段：prepare → crouch → thrust → flight_up → flight_down → landing → recovery → complete。PPT 可以仅列 **准备、下蹲、蹬伸、腾空、落地、恢复** 六个大模块，再在备注交代上升/下降和完成保持的细分，不在框图写参数和长解释。

| 指标 | 两次相同值 | 口径 |
|---|---:|---|
| 运行时长 | 13 s | 1300 个 100 Hz 日志点 |
| 参考目标高度 | 0.120 m | 不是实测高度 |
| 保守实际高度增量 | 0.060816 m | 100 Hz CSV 自起跳样本到最高点 |
| 高频事件实际高度增量 | 0.070812 m | 1 kHz 事件摘要 |
| 观测腾空 | 0.250 s / 26 样本 | 100 Hz CSV |
| 高频事件腾空 | 0.269 s | 8.632–8.901 s |
| 腾空 MPC 开启样本 | 0 | 腾空切 IK/PV |
| 外力辅助峰值 | 0 N | 自身力矩蹬伸 |
| 末 0.5 s 双足接触占比 | 100% | 终态稳定 |
| 末段最大水平速度 | 0.002306 m/s | 终态稳定 |
| 末段最大倾角 | 0.001027 rad | 最大单轴 |
| 末段最大高度误差 | 0.004190 m | 相对 1.08 m |
| 全程水平净漂移 | 0.022466 m | 首末位置差 |
| MPC / WBC 失败日志样本 | 0 / 0 | 有效控制阶段 |
| 高频右脚接触峰值 | 8354.200 N | 1 kHz 摘要 |
| 高频最大关节命令 | 396 N·m | 1 kHz 摘要 |

推荐标题：**跳跃功能与恢复通过两次重复，实际起跳增量仍低于目标**。
推荐图：

- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run01/analysis_final/height.png`
- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run01/analysis_final/contact_forces.png`

可在附录再展示 run02 对应高度图，或把两次数据在 final 下统一重绘，不必主展示相同五图两套。

必须保留的结论边界：验收高度阈值是 0.06 m（目标的一半），实际未达到配置目标 0.12 m。8.35 kN 冲击与 396 N·m 命令需要继续分析裕量；当前只证明仿真功能，非实机结构安全或无饱和。contact_forces.png 是100 Hz曲线，不能从图直接读出1 kHz峰值。

上游对应 `Pro2/OpenLoong-Dyn-Control/demo/jump_mpc.cpp`；新增 `Pro2/isaac/jump_controller.py`、桥接低层模式切换和 PhysX 双足反馈。腾空关闭接触 MPC，落地及恢复使用 WBC。恢复低优先级加速度使用受零空间约束逆，但只在该模式的加速度层使用，不能说整个WBC全部换算法。

## 4. Convex MPC / NMPC：带回退的同二进制对照

正式目录：

- `Pro2/output/L4/04_mpc_comparison/cpp_convex_walk_run03/`
- `Pro2/output/L4/04_mpc_comparison/nmpc_rti_walk_run04/`
- 分析：`Pro2/output/L4/04_mpc_comparison/analysis_same_binary/`

`metrics.json.comparison_checks` 全为真：时间向量、参考命令、关键配置、控制库 SHA-256 一致，均 completed。两者 10 s，walk_start=3 s，vx=0.08 m/s，平地；每种只运行一次。

| 指标 | C++ Convex | SQP-RTI NMPC |
|---|---:|---:|
| 前向速度 RMSE m/s | 0.035321 | 0.034077 |
| 横向速度 RMSE m/s | 0.112147 | 0.112465 |
| 行走期求解均值 ms | 0.354704 | 17.128871 |
| 行走期求解 P95 ms | 0.479031 | 18.440481 |
| 最大求解时间 ms | 0.591040 | 21.523801 |
| 更新成功 / 失败 | 日志未见失败 | 314 / 36 |
| 更新失败率 | — | 36/350 = 10.286% |
| 20 ms 截止期未满足 | — | 4/350 = 1.143% |

**回退与采样口径核查**：原报告61.5%是全部1000个CSV样本的外部wrench比例，含前3s站立；38.5%不能写作NMPC回退率。
直接重算 `nmpc_rti_walk_run04/states.csv`：

- 全时段：1000样本，外部615，内部385。
- 行走 t≥3s：700样本，外部615，内部85，内部来源12.143%；包含3.000s尚未首次更新的1个初始化样本。
- 已开始外部更新（update_count>0）：699样本，外部615，内部84，内部来源12.017%。

后两者是100Hz日志中的控制来源占比，不是50Hz更新失败率；内部来源还可能来自接触不匹配。PPT建议只展示 **36/350 更新失败** 与 **带内部Convex保护回退**，省去难讲清的回退率。若需要回退指标，精确标为“外部更新启动后内部wrench占100Hz日志样本12.02%”。

求解时间统计总体也不同：C++是100Hz日志对200Hz内部更新的抽样；NMPC按计数器去重后350个50Hz更新全集。可比较量级，不能宣称逐次一一公平计时或严格48.3倍性能优劣。

推荐标题：**跟踪结果接近，NMPC 的计算成本与保护回退更突出**。
推荐图：

- `Pro2/output/L4/04_mpc_comparison/analysis_same_binary/vx_world_m_s_comparison.png`
- `Pro2/output/L4/04_mpc_comparison/analysis_same_binary/vy_world_m_s_comparison.png`

不应只展示前向略降而声称全面优于Convex；单次结果不支持统计显著性。完整 SLSQP 仅离线自检，在线是 SQP-RTI，不是 iLQR。

## 5. 力矩分解：跟踪从 MPC 到执行器的信号

原始数据：`Pro2/output/L4/04_mpc_comparison/cpp_convex_walk_run03/states.csv`。
派生目录：`Pro2/output/L4/05_control_decomposition/azureloong_walk_same_binary/`。
这是同一次Convex运行的分析，不另计独立仿真实验。

信号关系：MPC足底wrench进入WBC；关节限幅前命令=WBC前馈+滤波PD；限幅后写入Isaac applied。Isaac projected可能包含约束反力，不能与电机命令混称。

| 关节 | WBC RMS N·m | 滤波PD RMS N·m | 命令峰值 N·m | 限幅采样数 |
|---|---:|---:|---:|---:|
| 左髋 | 8.680944 | 9.578359 | 49.291127 | 0 |
| 左膝 | 59.644677 | 32.740084 | 184.912921 | 0 |
| 左踝 | 16.622128 | 7.396451 | 58.500000 | 5 |
| 右髋 | 8.940653 | 9.404351 | 43.294966 | 0 |
| 右膝 | 62.102652 | 33.264878 | 167.100634 | 0 |
| 右踝 | 15.203814 | 7.596064 | 58.500000 | 10 |

command/applied RMSE 为3.72e-7至1.65e-6 N·m量级，验证记录/施加链路一致；不代表所有控制误差为零。
推荐标题：**前馈和 PD 共同形成关节命令，踝关节出现限幅**。
推荐主图（一个典型大力矩关节，一个限幅关节）：

- `Pro2/output/L4/05_control_decomposition/azureloong_walk_same_binary/figures/J_knee_l_pitch.png`
- `Pro2/output/L4/05_control_decomposition/azureloong_walk_same_binary/figures/J_ankle_r_pitch.png`

足底输入图可放备份页：`Pro2/output/L4/05_control_decomposition/azureloong_walk_same_binary/figures/mpc_vertical_contact_force.png`。
