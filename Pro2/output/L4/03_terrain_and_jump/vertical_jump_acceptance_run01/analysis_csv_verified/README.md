# L4 竖直跳跃独立验收

结论：**通过**。分析器直接读取该次运行的 `run.json`、`jump_metrics.json` 和 `states.csv`，没有改写原始实验数据。

## 关键结果

- `states.csv` 独立重算起跳高度增量：`0.060815632343292236` m；
- `jump_metrics.json` 汇总高度增量：`0.07081210613250732` m；
- CSV 离地/落地时刻：`8.64` / `8.9` s；
- 落地后最长双脚同时离地：`0.0000` s；
- 末段双足接触占比：`1.000`；
- 末段最大水平/竖直速度：`0.0023` / `0.0007` m/s；
- 末段最大倾角：`0.0010` rad；
- 末段最大高度误差：`0.0042` m；
- 全程水平净漂移：`0.0225` m；
- MPC/WBC QP 失败样本：`0` / `0`。

## 验收项

| 检查 | 结果 | 实际值 | 判据 |
|---|---:|---|---|
| `run_completed` | 通过 | "completed" | run.json.status == completed |
| `complete_phase_sequence` | 通过 | {"states": ["prepare", "crouch", "thrust", "flight_up", "flight_down", "landing", "recovery", "complete"], "jump_metrics": ["prepare", "crouch", "thrust", "flight_up", "flight_down", "landing", "recovery", "complete"], "final_phase": "complete"} | 状态与汇总均依次包含 prepare 至 complete 八阶段 |
| `takeoff_and_touchdown` | 通过 | {"takeoff_time_s": 8.632, "touchdown_time_s": 8.901} | 起跳和落地时刻均存在，且落地晚于起跳 |
| `sustained_airborne_interval` | 通过 | {"start_time_s": 8.64, "end_time_s": 8.89, "samples": 26, "observed_duration_s": 0.25} | 配置起跳时刻后至少两个连续样本的双足力均不高于起跳阈值 |
| `bilateral_landing_after_airborne` | 通过 | {"landing_time_s": 8.9, "left_force_n": 623.4652099609375, "right_force_n": 620.1189575195312} | 持续腾空后出现双足力均不低于落地阈值的接触 |
| `event_timing_consistency` | 通过 | {"takeoff_delta_s": 0.008000000000000895, "touchdown_delta_s": -0.0009999999999994458, "tolerance_s": 0.010999999999999788} | CSV 独立事件与高频 jump_metrics 事件之差均不超过日志周期加确认时长 |
| `minimum_rise_threshold_consistency` | 通过 | {"recomputed_m": 0.06, "jump_metrics_m": 0.06, "source": "max(0.03, 0.5 * run.json.config.jump.target_height_m)"} | 配置可推导阈值时，jump_metrics 阈值必须与配置重算值一致 |
| `minimum_jump_rise` | 通过 | {"csv_actual_rise_m": 0.060815632343292236, "jump_metrics_actual_rise_m": 0.07081210613250732} | states.csv 独立重算高度增量 >= 0.060 m；JSON actual_rise 不单独决定结果 |
| `flight_mpc_disabled` | 通过 | {"flight_samples": 28, "enabled_samples": 0, "summary_flag": true} | 腾空阶段有日志且所有样本 controller:mpc_enabled == 0 |
| `external_force_assist_disabled` | 通过 | {"summary_flag": false, "peak_logged_force_n": 0.0} | 外力辅助标志为 false，且三轴外力日志均为 0 |
| `no_second_flight_after_touchdown` | 通过 | 0.0 | 落地后双脚同时失去接触不连续超过 0.05 s |
| `terminal_bilateral_contact` | 通过 | {"left": 1.0, "right": 1.0, "bilateral": 1.0} | 末 0.50 s 左、右和双足接触占比均 >= 95%（接触阈值 20 N） |
| `terminal_horizontal_speed` | 通过 | 0.002305975557524924 | 末段最大水平速度 <= 0.20 m/s |
| `terminal_vertical_speed` | 通过 | 0.0007388223137145075 | 末段最大竖直速度绝对值 <= 0.15 m/s |
| `terminal_posture` | 通过 | 0.0010271312326874877 | 末段最大 |roll|/|pitch| <= 0.20 rad |
| `terminal_height` | 通过 | 0.0041902828216553445 | 末段基座高度相对 1.080 m 的最大误差 <= 0.12 m |
| `planar_drift` | 通过 | 0.02246645328836961 | 全程水平净漂移 <= 0.30 m |
| `mpc_qp_success` | 通过 | {"active_samples": 83, "failure_samples": 0} | MPC 开启样本均为 QP status 0 |
| `wbc_qp_success` | 通过 | {"active_samples": 1272, "failure_samples": 0} | 非腾空 WBC 样本均为 QP status 0 |
| `builtin_jump_pass` | 通过 | true | 仿真内建 jump_metrics.pass == true |

## 曲线

- `height.png`：基座高度与高度参考；
- `velocity.png`：基座三轴速度；
- `contact_forces.png`：左右脚 PhysX 法向接触力；
- `attitude.png`：横滚角与俯仰角；
- `planar_trajectory.png`：基座水平轨迹。
