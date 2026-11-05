# L4：Convex MPC、NMPC 与控制量分解

## 先通俗说明

这个目录增加了两套可以直接检查公式的 Python 控制器：

- `convex_mpc_reference.py` 把当前 OpenLoong 的线性单刚体预测过程写成容易阅读的凝聚 QP，由 OSQP 求解。
- `nmpc_srbd.py` 保留完整姿态旋转、世界坐标惯量和陀螺项，由 SciPy SLSQP 求解完整非线性有限时域问题，用作离线参考。
- `nmpc_rti.py` 沿非线性轨迹进行局部线性化，每周期执行一次 SQP-QP，作为 Isaac 在线对照控制器。

正式的 Convex MPC 基线仍是原来的 C++ OpenLoong 控制器。Python 版本用于检查公式与建立同口径对照，不把它冒充成 C++ 的逐元素复刻。NMPC 输出左右脚第一拍的 12 维接触 wrench，C++ 桥再把它交给原 WBC。NMPC 求解失败时，桥会自动使用内部 Convex MPC 的结果。

本机 Isaac Python 已有 NumPy、SciPy 和 OSQP，没有 CasADi。完整 NLP 使用 SLSQP；在线版本使用有限差分 SQP-RTI 与 OSQP。独立自检证明数学问题可以求解；是否在完整 Isaac 循环中按时完成，仍以闭环日志为准。

## 依赖与自检命令

以下入口都直接使用 Python，不需要启动脚本：

```bash
python3 Pro2/run_l4_control.py check-dependencies

python3 Pro2/run_l4_control.py self-check --controller convex

python3 Pro2/run_l4_control.py self-check --controller nmpc-slsqp

python3 Pro2/run_l4_control.py self-check --controller nmpc-rti

python3 Pro2/run_l4_control.py test
```

原生桥和控制量分解接口使用：

```bash
python3 Pro2/run_isaac.py \
  --build-only --mpc-horizon-steps 10

python3 Pro2/isaac/native/test_bridge.py
```

## 专业接口约定

两套 Python 控制器使用同一状态顺序：

```text
[roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz]
```

角速度、线速度、足端位置和 wrench 都在世界坐标系中。wrench 顺序为：

```text
左脚 [Fx, Fy, Fz, Mx, My, Mz]
右脚 [Fx, Fy, Fz, Mx, My, Mz]
```

法向力 `Fz` 向上为正。默认质量、惯量、摩擦系数和足底尺寸来自 `Pro2/OpenLoong-Dyn-Control/algorithm/mpc.cpp`。

共享参考和接触日程可以这样构造：

```python
from control.srbd_model import velocity_reference, predict_contact_schedule

reference = velocity_reference(
    state12, command_vx_vy_wz, dt=0.02, horizon=5, stand_height=1.08)
contacts = predict_contact_schedule(
    current_leg_state, next_leg_state, gait_phase,
    dt=0.02, horizon=5, swing_time=0.4)
```

NMPC 接到原 WBC 的最小调用方式为：

```python
from control.nmpc_rti import RtiNmpc
from control.runtime_adapter import ExternalMpcRuntime

solver = RtiNmpc(dt=0.02, horizon=3)
external_mpc = ExternalMpcRuntime(solver, update_period_s=0.02)

# 在 controller.step(...) 之前调用；foot_positions_world.shape == (2, 3)
update = external_mpc.update(
    controller, t, state12, reference, foot_positions_world, contacts)
```

`update_period_s=0.02` 表示 50 Hz SQP-RTI NMPC，`dt=0.02, horizon=3` 对应 60 ms 预测窗口。C++ WBC 仍在 1 kHz 运行，并在两次 NMPC 更新之间保持上一组有效 wrench。若 `result.success` 为假，`set_external_mpc_wrench` 会写入非零状态，C++ 桥在该周期改用内部 Convex MPC。

## 控制量分解接口

`Pro2/isaac/native.py` 的 `Controller.control_signals()` 返回 167 个量：

| 前缀 | 数量 | 含义 |
|---|---:|---|
| `mpc_wrench:` | 12 | 实际送入 WBC 的左右脚 wrench |
| `wbc_feedforward_Nm:` | 31 | 实际加入关节控制器的 WBC 前馈力矩 |
| `pd_raw_Nm:` | 31 | 低通滤波前的关节 PD 修正量 |
| `pd_filtered_Nm:` | 31 | 低通滤波后的关节 PD 修正量 |
| `prelimit_Nm:` | 31 | WBC 与滤波 PD 相加、限幅前的力矩 |
| `command_Nm:` | 31 | 限幅后发给 Isaac 的关节力矩 |

其中恒有：

```text
prelimit_Nm = wbc_feedforward_Nm + pd_filtered_Nm
```

Isaac 的 `get_dof_efforts()` 返回物理引擎中的 actuation effort；`get_dof_projected_joint_forces()` 返回包含约束反力影响的投影关节力。二者应分别记录，不能都写成“实际执行力矩”。

## 当前验收边界

- 数学模块、依赖检查、原生外部 wrench 接口和控制量分解接口已经实现并有自动测试。
- Python NMPC 自检使用静态双支撑工况，不等于 Isaac 人形闭环已经跑通。
- 外部 wrench 模式仍借用内部 MPC 生成的共同 WBC 运动参考，只替换足底 wrench 优化结果。这保证 WBC 接口一致，但报告中应称为“wrench 优化器对照”。
- 完整 SLSQP 只用于离线公式验证；动态小扰动下可能达到数百毫秒至秒级。SQP-RTI 的 N=3 独立自检约为十几毫秒，默认从 50 Hz 开始验收，并统计平均值、P95、最大值和超时次数。达到 200 Hz 之前不得声称实时性能等同于 C++ Convex MPC。
