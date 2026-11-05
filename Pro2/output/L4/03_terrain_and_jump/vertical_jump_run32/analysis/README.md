# L4 竖直跳跃独立验收

结论：**未通过**。分析器直接读取该次运行的 `run.json`、`jump_metrics.json` 和 `states.csv`，没有改写原始实验数据。

## 关键结果

- 实际起跳高度增量：`0.07081210613250732` m；
- 落地后最长双脚同时离地：`0.0500` s；
- 末段双足接触占比：`0.804`；
- 末段最大水平/竖直速度：`1.6967` / `2.1250` m/s；
- 末段最大倾角：`0.4602` rad；
- 末段最大高度误差：`0.4605` m；
- 全程水平净漂移：`0.8583` m；
- MPC/WBC QP 失败样本：`0` / `0`。

## 验收项

| 检查 | 结果 | 实际值 | 判据 |
|---|---:|---|---|
| `run_completed` | 失败 | "failed" | run.json.status == completed |
| `complete_phase_sequence` | 失败 | {"states": ["prepare", "crouch", "thrust", "flight_up", "flight_down", "landing", "recovery"], "jump_metrics": ["prepare", "crouch", "thrust", "flight_up", "flight_down", "landing", "recovery"], "final_phase": "recovery"} | 状态与汇总均依次包含 prepare 至 complete 八阶段 |
| `takeoff_and_touchdown` | 通过 | {"takeoff_time_s": 8.632, "touchdown_time_s": 8.905} | 起跳和落地时刻均存在，且落地晚于起跳 |
| `minimum_jump_rise` | 通过 | 0.07081210613250732 | actual_rise_m >= 0.060 m |
| `flight_mpc_disabled` | 通过 | {"flight_samples": 28, "enabled_samples": 0, "summary_flag": true} | 腾空阶段有日志且所有样本 controller:mpc_enabled == 0 |
| `external_force_assist_disabled` | 通过 | {"summary_flag": false, "peak_logged_force_n": 0.0} | 外力辅助标志为 false，且三轴外力日志均为 0 |
| `no_second_flight_after_touchdown` | 通过 | 0.049999999999998934 | 落地后双脚同时失去接触不连续超过 0.05 s |
| `terminal_bilateral_contact` | 失败 | {"left": 0.803921568627451, "right": 0.9019607843137255, "bilateral": 0.803921568627451} | 末 0.50 s 左、右和双足接触占比均 >= 95%（接触阈值 20 N） |
| `terminal_horizontal_speed` | 失败 | 1.696663026729639 | 末段最大水平速度 <= 0.20 m/s |
| `terminal_vertical_speed` | 失败 | 2.1249549449845406 | 末段最大竖直速度绝对值 <= 0.15 m/s |
| `terminal_posture` | 失败 | 0.46018403179605805 | 末段最大 |roll|/|pitch| <= 0.20 rad |
| `terminal_height` | 失败 | 0.46050705194473274 | 末段基座高度相对 1.080 m 的最大误差 <= 0.12 m |
| `planar_drift` | 失败 | 0.8582645331613069 | 全程水平净漂移 <= 0.30 m |
| `mpc_qp_success` | 通过 | {"active_samples": 83, "failure_samples": 0} | MPC 开启样本均为 QP status 0 |
| `wbc_qp_success` | 通过 | {"active_samples": 1044, "failure_samples": 0} | 非腾空 WBC 样本均为 QP status 0 |
| `builtin_jump_pass` | 失败 | false | 仿真内建 jump_metrics.pass == true |

## 曲线

- `height.png`：基座高度与高度参考；
- `velocity.png`：基座三轴速度；
- `contact_forces.png`：左右脚 PhysX 法向接触力；
- `attitude.png`：横滚角与俯仰角；
- `planar_trajectory.png`：基座水平轨迹。
