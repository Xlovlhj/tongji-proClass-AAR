# L4 成果索引

本目录只保存 L4 成果。L0、L1、L2、L3 分别保存在各自层级目录；L4 的每次仿真实验继续使用独立子目录。

详细报告见 [L4验收报告.md](L4验收报告.md)。主报告已经把 H1 正式站立的五张曲线、路径五组轨迹/误差图、四组地形的实测步态与稳定性分析、两次跳跃正式重复各五张验收图、Convex/NMPC 速度对照和七张控制量分解图嵌入正文，并把每组图片对应到正式运行目录及 JSON/CSV 指标。H1 曲线只由正式目录 `states.csv` 生成，没有借用失败调参画面。

## 证据口径

`run.json.status == "completed"` 只表示仿真执行结束，不自动等于任务验收通过：

- 路径还要检查到点事件；
- 地形结合实测步态、基座姿态、速度跟踪和控制异常评价观察期间的行走质量；运行时长和覆盖范围另行记录；
- MPC 对照还要核对时间轴、参考命令和控制器库哈希；
- 模型更换还要区分独立站立和 OpenLoong 控制器移植；
- 失败与作废目录保留为调试证据，不进入主结果。

## 五项任务状态

| 项目 | 当前状态 | 主要证据 |
|---|---|---|
| 更换机器人模型 | H1-2 FixStand 8 s 独立站立通过；OpenLoong MPC/WBC 未移植到 H1 | `01_robot_swap/h1_fixstand_feedback_8s_final_v2/` |
| 路径跟踪 | 五组正式运行均完成；四组触发到点，圆形开环是未到点对照；当前到点后策略为零速踏步 | `02_path_tracking/path_tracking_summary.json` |
| 地形/障碍/跳跃 | 四种地形均形成行走观察与量化分析，重点检查步态连续性、姿态摆动和速度跟踪；Isaac 竖直跳跃完成八阶段并通过两次独立重复验收 | `Pro2/output/L4/03_terrain_and_jump/gait_stability_analysis/summary.json`、`Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run01/analysis_final/acceptance.json`、`Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run02/analysis_final/acceptance.json` |
| Convex MPC 与 NMPC | 同一控制器库、同一时间轴和参考命令完成 10 s 对照 | `04_mpc_comparison/analysis_same_binary/metrics.json` |
| MPC/WBC/PD 力矩分解 | 基于同二进制 Convex 正式运行生成 10 s、1000 点分析和 7 张图 | `05_control_decomposition/azureloong_walk_same_binary/` |

项目书第三项允许在地形、障碍或跳跃中选择。本项目完成四种地形中的真实行走实验和步态、稳定性分析，随后又完成了 Isaac 竖直跳跃适配。四次正式地形运行均执行到预设时长；时长分别为 24、28、45、24 s，没有走完区域不能直接解释为失稳或通过能力不足。分析结论只覆盖已观测过程，延时后的表现仍需运行验证。`vertical_jump_run35` 是首次通过的开发运行；`vertical_jump_acceptance_run01` 与 `vertical_jump_acceptance_run02` 是相同最终配置、同一控制库下的两次独立重复验收。三次都在不施加外力的条件下完成准备、下蹲、蹬伸、上升腾空、下降腾空、落地、恢复和完成八阶段。独立分析器核对两次正式运行的原始 `run.json`、`jump_metrics.json` 与 `states.csv` 后均判定通过。地形日志仍没有足端 contact prim 或逐障碍接触证明，也不能外推到其他 seed、密度或高度。

## 正式证据目录

### 1. H1 模型更换

- `01_robot_swap/h1_fixstand_feedback_8s_final_v2/`：正式 8 s FixStand 站立；
- `01_robot_swap/h1_fixstand_feedback_8s_final_v2/analysis/metrics.json`：来源哈希、指标定义、保持阶段和最后记录样本的派生指标；
- `01_robot_swap/h1_fixstand_feedback_8s_final_v2/analysis/`：基座高度、Roll/Pitch、27 关节跟踪 RMSE、踝平衡反馈和关节力矩五张正式曲线；
- `01_robot_swap/L4-01_模型更换验收.md`：该项独立报告。

H1 运行使用 Unitree FixStand 关节目标/Kp/Kd 和骨盆到踝反馈。`run.json` 明确记录 `openloong_mpc_wbc_used: false`。100 Hz CSV 的保持阶段共 500 个样本，基座高度均值/总体标准差为 0.989677/0.000576 m，27 关节相对控制目标 RMSE 为 0.049752 rad；正式通过判定仍使用控制程序在 500 Hz 物理步生成的原始 `metrics.json`。

### 2. 路径跟踪

- `02_path_tracking/straight_run02/`：直线闭环；
- `02_path_tracking/turn_90/`：90° 转弯；
- `02_path_tracking/circle_open_loop_run02/`：圆形开环对照；
- `02_path_tracking/circle_closed_loop_run02/`：圆形闭环到点；
- `02_path_tracking/waypoints_run06/`：多路径点；
- `02_path_tracking/path_tracking_summary.json`：正式汇总；
- `02_path_tracking/path_tracking_summary.csv`：表格汇总。

当前代码会把参考路径作为无碰撞 USD 几何显示在平地上：青色带状线表示参考路径，绿色、黄色和红色圆盘分别表示起点、途经点和终点；闭合圆的起终点重合标记为洋红色。`02_path_tracking/path_visualization_smoke_20261031_v3/` 是 0.05 s Isaac 6.0.1 集成冒烟记录；其 `run.json.path_visualization` 和 `scene.usda` 证明几何已创建并导出。既有五组正式结果生成于该显示功能加入之前，历史 `scene.usda` 不会被追溯修改。重新运行相应配置时 GUI 会加载这些几何；请求录像或最终截图且视口捕获成功时，相应文件也会包含路径。

当前终点策略保持 Walk 状态，把速度参考降到零，即零速踏步。`waypoints_run06` 使用该策略；直线、90° 和圆形闭环到点目录生成于旧的到点后 Stand 策略，报告已说明版本边界。

### 3. 地形、障碍和跳跃

- `03_terrain_and_jump/slope_05deg/`：5° 坡道正式 24 s 行走观察；
- `03_terrain_and_jump/rough_medium_run02/`：粗糙地面正式 28 s 行走观察；
- `03_terrain_and_jump/random_obstacles_seed01_full/`：seed 1 随机障碍正式 45 s 行走观察，场景含 12 个障碍块；
- `03_terrain_and_jump/blind_step_up_run02/`：6 cm 盲踩上台阶正式 24 s 行走观察；
- 上述四个正式目录各自的 `analysis_gait/`：实际关节运动、姿态和速度的步态稳定性分析；
- `03_terrain_and_jump/gait_stability_analysis/summary.json`：统一入区过渡窗口和各段行走指标；
- `03_terrain_and_jump/terrain_summary.json`：原进入/穿越几何记录，作为观察范围说明，不作为四场景的稳定性排名；
- `03_terrain_and_jump/terrain_summary.csv`：对应覆盖范围表格；
- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_run35/`：首次通过的 13 s Isaac/PhysX 开发运行，不计入同二进制复现对；
- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run01/`：固定最终配置的第一次正式重复运行；
- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run02/`：同一控制库的第二次正式重复运行；
- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run01/analysis_final/acceptance.json` 与 `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run02/analysis_final/acceptance.json`：从原始 CSV 重算事件和高度的 21 项正式验收，两次均为 `pass: true`；
- 两个运行中原有的 `analysis/` 是加强原始 CSV 复核前的初版分析，继续保留但不作为当前主验收链接；
- [Pro2/output/L4/03_terrain_and_jump/JUMP_IMPLEMENTATION_REPORT.md](03_terrain_and_jump/JUMP_IMPLEMENTATION_REPORT.md)：跳跃算法映射、Isaac 适配差异和验收指标；
- `Pro2/output/L4/03_terrain_and_jump/vertical_jump_preflight/`：适配完成前的历史预检失败，保留为开发过程证据。

`03_terrain_and_jump/rough_low/` 和旧 `03_terrain_and_jump/rough_medium/` 使用修复前的 Approach 间隙几何，状态为 `invalidated_geometry`，不得作为算法成败证据。当前地形 manifest 会把实际生成的 `plan` 固化到 `run.json`。

跳跃不是通过向基座施加外力实现。蹬伸和落地阶段使用 OpenLoong Convex MPC；腾空阶段关闭接触力 MPC，按上游 `jump_mpc.cpp` 执行收腿和向前摆脚；PhysX 左右脚对地接触力驱动离地/落地事件，落地后的 WBC/QP 完成冲击吸收、姿态稳定和高度恢复。两次正式运行从 100 Hz `states.csv` 独立重算的保守起跳高度增量均为 0.060816 m，超过 0.060 m 阈值；1 kHz 事件摘要记录 0.070812 m，但两者都没有达到配置中的 0.12 m 目标。末 0.5 s 双足接触占比均为 100%，水平净漂移均为 0.022466 m，MPC/WBC QP 失败样本均为 0。run01/run02 的控制库 SHA-256 均为 `8e8d5f8c6328ef5521428e4da7920dc650d11ca5d36652365d2b7f13b6e1bc48`。该结论是仿真功能验收；右脚 1 kHz 接触力峰值约 8354 N、1 kHz 最大关节命令 396 N·m，100 Hz 日志中的力矩峰值为 376.453 N·m，仍需冲击和饱和分析，不能外推为实机可直接部署。

### 4. Convex MPC 与 NMPC

- `04_mpc_comparison/cpp_convex_walk_run03/`：正式 Convex 10 s；
- `04_mpc_comparison/nmpc_rti_walk_run04/`：正式 NMPC 10 s；
- `04_mpc_comparison/analysis_same_binary/`：同二进制分析与速度图；
- `04_mpc_comparison/python_convex_reference/`：Python Convex 数学自检；
- `04_mpc_comparison/nmpc_slsqp_offline/`：完整 NMPC 数学自检；
- `04_mpc_comparison/nmpc_rti_online/`：RTI NMPC 数学自检。

正式两个运行的控制器库 SHA-256 均为：

`96745fcdac054a8d755f61ad25d001edc38dee58f4b4455c2cb99a247ef60876`

每种算法只有一次运行。NMPC 有 350 次更新、314 次成功、36 次失败和 4 次超过 20 ms；外部 wrench 在全部日志样本中占 61.5%，其余阶段包含站立或保护回退。

### 5. 控制量分解

- `05_control_decomposition/azureloong_walk_same_binary/metrics.json`；
- `05_control_decomposition/azureloong_walk_same_binary/joint_metrics.csv`；
- `05_control_decomposition/azureloong_walk_same_binary/figures/`。

分析数据来自 `04_mpc_comparison/cpp_convex_walk_run03/states.csv`，因此该目录是派生分析，不是另一次独立仿真。

## 报告框架图

- [L4 总体适配架构](figures/L4总体适配架构.png)
- [L4 路径跟踪闭环](figures/L4路径跟踪闭环.png)
- [L4 Convex MPC 与 NMPC 对照](figures/L4凸MPC与NMPC对比.png)
- [L4 足底力与关节力矩分解](figures/L4力矩分解链路.png)

重新生成图片：

```text
python3 Pro2/tools/render_l4_diagrams.py
```

## 目录

- `00_baseline/`：AzureLoong 平地基线；
- `01_robot_swap/`：H1 模型更换；
- `02_path_tracking/`：路径跟踪；
- `03_terrain_and_jump/`：地形、障碍、跳跃实现、历史调试运行与独立验收；
- `04_mpc_comparison/`：Convex MPC 与 NMPC；
- `05_control_decomposition/`：MPC/WBC/PD/Isaac 力矩分解；
- `figures/`：四张竖向报告框架图。

