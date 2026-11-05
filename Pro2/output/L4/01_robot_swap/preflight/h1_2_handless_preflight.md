# L4 模型更换：Unitree H1-2 预检报告

## 先看结论

- H1-2 USD 资产预检：**通过**。
- 当前 OpenLoong MPC/WBC 直接兼容：**未通过**。
- 这两个结论含义不同：资产通过表示 Isaac 能读到完整关节、质量、浮动基和足底碰撞信息；控制器兼容还要求动力学模型、关节维数和关节语义完全一致。

## 已核实的资产事实

- 来源：`/home/xqw/xqw-file/unitree_model/H1-2/h1_2_handless/h1_2_handless.usd`
- 关节数：27
- 刚体数：28
- 显式总质量：66.983999 kg
- 浮动基：True
- 关节根：`/h1_2/pelvis`
- 左右足底碰撞：均已在 composed USD 的 instance proxy 中检出。

## 为什么还不能直接替换模型运行

- H1-2 有 27 个执行关节（nv=33），而当前桥接器与 DataBus 合同固定为 31 个关节（nv=37）。
- H1 的关节和连杆名称与代码中固定的 AzureLoong motorName、踝、髋、手和腰部名称不同。
- 当前 Pinocchio/WBC 必须读取匹配的 URDF；本地 H1 包只有可组合的 USD，旁置配置指向另一台机器上的源 URDF。
- 必须为 H1 定义并验证 PVT 增益、力矩限制、初始姿态、任务选择矩阵和 WBC 维数。
- MPC 中的质量、质心惯量、髋宽、踝到足底距离和支撑面尺寸都是 AzureLoong 常量，必须替换为 H1 参数。

## 源码证据

| 相对路径 | 行号 | 固定假设 |
|---|---:|---|
| `Pro2/isaac/native/controller_bridge.cpp` | 26 | `constexpr int kJointCount = 31;` |
| `Pro2/isaac/native/controller_bridge.cpp` | 88 | `require(kin.model_nv == 37, "Expected the 31-joint AzureLoong model (nv=37)");` |
| `Pro2/OpenLoong-Dyn-Control/algorithm/pino_kin_dyn.cpp` | 43 | `r_ankle_joint = model_biped.getJointId("J_ankle_r_roll");` |
| `Pro2/OpenLoong-Dyn-Control/algorithm/pino_kin_dyn.h` | 29 | `const std::vector<std::string> motorName = {"J_arm_l_01", "J_arm_l_02", "J_arm_l_03", "J_arm_l_04", "J_arm_l_05",` |
| `Pro2/OpenLoong-Dyn-Control/common/PVT_ctrl.h` | 65 | `const std::vector<std::string> motorName={"J_arm_l_01","J_arm_l_02","J_arm_l_03", "J_arm_l_04", "J_arm_l_05",` |
| `Pro2/OpenLoong-Dyn-Control/algorithm/mpc.cpp` | 13 | `m = 77.35;` |
| `Pro2/OpenLoong-Dyn-Control/algorithm/mpc.cpp` | 191 | `Ic << 12.61, 0, 0.37, 0, 11.15, 0.01, 0.37, 0.01, 2.15;` |

## 资产检查

| 检查项 | 结果 |
|---|---|
| `default_prim` | 通过 |
| `up_axis` | 通过 |
| `meters_per_unit` | 通过 |
| `articulation_root` | 通过 |
| `floating_base` | 通过 |
| `joint_count` | 通过 |
| `joint_names` | 通过 |
| `all_joints_have_drives` | 通过 |
| `all_joint_body_relations_resolve` | 通过 |
| `rigid_body_count` | 通过 |
| `base_link_is_rigid` | 通过 |
| `left_foot_is_rigid` | 通过 |
| `right_foot_is_rigid` | 通过 |
| `all_rigid_bodies_have_explicit_mass` | 通过 |
| `total_mass_kg` | 通过 |
| `all_composition_layers_exist` | 通过 |
| `left_foot_has_collision` | 通过 |
| `right_foot_has_collision` | 通过 |

## 后续真正完成控制移植所需工作

1. 补齐与本地 USD 同版本的 H1-2 URDF，并验证 Pinocchio 关节顺序、质量和惯量与 USD 一致。
2. 将 `Pin_KinDyn`、`DataBus`、`PVT_Ctr` 和桥接 ABI 的 31 关节固定数组改成模型规格驱动。
3. 为 H1 重新定义足端、髋、手和腰部任务，并调整 WBC 选择矩阵与优先级任务维数。
4. 从 H1 模型计算总质量、质心惯量、髋宽和足底尺寸，替换 MPC 中的 AzureLoong 常量。
5. 建立 H1 站立初始姿态、每关节 PD 和力矩限制，先完成站立，再依次验收前进和转向。

完整机器可读证据保存在 `Pro2/output/L4/01_robot_swap/preflight/h1_2_handless_preflight.json`。
